"""
inventory_node.py
------------------
Inventaire probabiliste alimente par les lectures RFID de tous les robots
(rfid_model.InventoryBelief), et decisions adaptatives qui en decoulent.

Entrees :
  /rfid/scans          lectures d'articles (tag, RSSI) + pose du robot
  /rfid/ground_truth   enregistrements WMS (prior initial) ; la verite
                       terrain n'est utilisee QUE pour les metriques
  /warehouse/tasks/new taches emises (pour mesurer les erreurs de picking)

Sorties :
  /rfid/inventory          stock estime, stock WMS et incertitude par rack
  /rfid/inventory_metrics  taux d'erreur d'inventaire RFID vs WMS seul,
                           lectures, erreurs de picking
  /rfid/scan_requests      rack dont l'inventaire est le plus incertain ou
                           le plus ancien -> tache "rfid_scan" (decision
                           adaptative sous incertitude)
"""

import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from warehouse_marl.rfid_model import (
    InventoryBelief, RFIDReaderModel, inventory_error_breakdown, inventory_error_rate,
    rack_faces)


class InventoryNode(Node):

    def __init__(self):
        super().__init__('inventory_node')

        self.declare_parameter('map_origin_x', -1.5)
        self.declare_parameter('map_origin_y', -1.0)
        self.declare_parameter('stock_threshold', 0.7)
        self.declare_parameter('scan_request_period_sec', 15.0)
        self.declare_parameter('scan_request_threshold', 2.0)
        self.declare_parameter('scan_request_cooldown_sec', 90.0)
        self.declare_parameter('stale_after_sec', 600.0)
        # Doit correspondre au modele du capteur (rfid_sensor_node)
        self.declare_parameter('range_50_m', 2.0)
        self.declare_parameter('max_range_m', 3.5)

        ox = float(self.get_parameter('map_origin_x').value)
        oy = float(self.get_parameter('map_origin_y').value)
        self.faces = rack_faces(ox, oy)
        self.model = RFIDReaderModel(
            range_50=float(self.get_parameter('range_50_m').value),
            max_range=float(self.get_parameter('max_range_m').value))
        self.stock_threshold = float(self.get_parameter('stock_threshold').value)
        self.request_threshold = float(self.get_parameter('scan_request_threshold').value)
        self.request_cooldown = float(self.get_parameter('scan_request_cooldown_sec').value)
        self.stale_after = float(self.get_parameter('stale_after_sec').value)

        self.belief = None
        self.wms = {}
        self.truth = {}
        self.drift_events = 0
        self.scans = 0
        self.pick_tasks = 0
        self.pick_errors = 0
        self.scan_tasks = 0
        self._last_request = {}
        # Back-off for shelves whose scan missions fail (unreachable for now):
        # the cooldown doubles after each failure, up to 10 min.
        self._cooldown = {}
        self._scan_targets = {}  # task_id -> shelf

        self.create_subscription(String, '/rfid/ground_truth', self._on_truth, 10)
        self.create_subscription(String, '/rfid/scans', self._on_scan, 50)
        self.create_subscription(String, '/warehouse/tasks/new', self._on_task, 10)
        self.create_subscription(String, '/warehouse/tasks/status', self._on_status, 50)

        self.inventory_pub = self.create_publisher(String, '/rfid/inventory', 10)
        self.metrics_pub = self.create_publisher(String, '/rfid/inventory_metrics', 10)
        self.request_pub = self.create_publisher(String, '/rfid/scan_requests', 10)

        self.create_timer(2.0, self._publish)
        self.create_timer(
            float(self.get_parameter('scan_request_period_sec').value),
            self._request_scan)
        self.get_logger().info('InventoryNode pret ; attente des enregistrements WMS.')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_truth(self, msg: String):
        data = json.loads(msg.data)
        self.truth = data.get('truth', {})
        self.drift_events = int(data.get('drift_events', 0))
        if self.belief is None:
            self.wms = data.get('wms', {})
            self.belief = InventoryBelief(self.faces, self.model, self.wms)
            self.get_logger().info(
                f'Croyance initialisee depuis le WMS : {len(self.wms)} articles.')

    def _on_scan(self, msg: String):
        if self.belief is None:
            return
        data = json.loads(msg.data)
        reads = [(r['tag'], float(r['rssi'])) for r in data.get('reads', [])]
        self.belief.update((float(data['x']), float(data['y'])), reads, self._now())
        self.scans += 1

    def _on_task(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        task_type = data.get('task_type', 'pick')
        if task_type == 'rfid_scan':
            self.scan_tasks += 1
            self._scan_targets[data.get('task_id')] = data.get('target_node')
            return
        target = data.get('target_node')
        if target not in self.faces or not self.truth:
            return
        self.pick_tasks += 1
        if not any(shelf == target for shelf in self.truth.values()):
            self.pick_errors += 1
            self.get_logger().warn(
                f'[RFID] Erreur de picking : {data.get("task_id")} vise {target}, '
                f'vide en realite.')

    def _on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        shelf = self._scan_targets.get(data.get('task_id'))
        state = data.get('state')
        if shelf is None or state not in ('succeeded', 'failed', 'rejected', 'canceled'):
            return
        del self._scan_targets[data['task_id']]
        if state == 'succeeded':
            self._cooldown.pop(shelf, None)
        else:
            self._cooldown[shelf] = min(600.0, 2.0 * self._cooldown.get(shelf, self.request_cooldown))
            self.get_logger().info(
                f'[RFID] Scan de {shelf} echoue ; prochaine demande dans '
                f'{self._cooldown[shelf]:.0f} s au plus tot.')

    def _wms_stock(self):
        stock = {s: 0 for s in self.faces}
        for shelf in self.wms.values():
            if shelf in stock:
                stock[shelf] += 1
        return stock

    def _publish(self):
        if self.belief is None:
            return
        now = self._now()
        stock = self.belief.shelf_stock(self.stock_threshold)
        wms_stock = self._wms_stock()
        uncertainty = self.belief.shelf_uncertainty(now, self.stale_after)
        shelves = {}
        for s in sorted(self.faces):
            age = now - self.belief.last_observed[s]
            shelves[s] = {
                'stock': stock[s],
                'wms_stock': wms_stock[s],
                'uncertainty': round(uncertainty[s], 2),
                'last_scan_age_sec': None if math.isinf(age) else round(age, 1),
            }
        items = {tag: [shelf, round(p, 3)]
                 for tag, (shelf, p) in self.belief.item_estimates().items()}
        inv = String()
        inv.data = json.dumps({'stamp': now, 'shelves': shelves, 'items': items})
        self.inventory_pub.publish(inv)

        if not self.truth:
            return
        estimate = self.belief.estimate()
        rfid_error = inventory_error_rate(estimate, self.truth)
        breakdown = inventory_error_breakdown(estimate, self.truth)
        wms_error = inventory_error_rate(self.wms, self.truth)
        metrics = String()
        metrics.data = json.dumps({
            'stamp': now,
            'rfid_error_rate': round(rfid_error, 4),
            'wms_error_rate': round(wms_error, 4),
            'items_in_stock': sum(1 for s in self.truth.values() if s is not None),
            'drift_events': self.drift_events,
            'scans': self.scans,
            'reads': self.belief.reads,
            'pick_tasks': self.pick_tasks,
            'pick_errors': self.pick_errors,
            'scan_tasks': self.scan_tasks,
            'mean_uncertainty': round(sum(uncertainty.values()) / len(uncertainty), 3),
            'rfid_missed': breakdown['missed'],
            'rfid_phantom': breakdown['phantom'],
            'rfid_wrong_shelf': breakdown['wrong_shelf'],
        })
        self.metrics_pub.publish(metrics)

    def _request_scan(self):
        if self.belief is None:
            return
        now = self._now()
        scores = self.belief.shelf_uncertainty(now, self.stale_after)
        candidates = [
            (score, s) for s, score in scores.items()
            if score >= self.request_threshold
            and now - self._last_request.get(s, -math.inf) >= self._cooldown.get(s, self.request_cooldown)
        ]
        if not candidates:
            return
        score, shelf = max(candidates)
        self._last_request[shelf] = now
        msg = String()
        msg.data = json.dumps({'shelf': shelf, 'score': round(score, 2), 'stamp': now})
        self.request_pub.publish(msg)
        self.get_logger().info(
            f'[RFID] Inventaire incertain sur {shelf} (score {score:.1f}) -> demande de scan.')


def main(args=None):
    rclpy.init(args=args)
    node = InventoryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
