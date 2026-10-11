"""
task_generator_node.py
------------------------
Simule le flux de commandes / operations d'inventaire a traiter dans
l'entrepot (equivalent d'un WMS - Warehouse Management System - qui
emettrait des ordres de picking). Publie de nouvelles taches sur
/warehouse/tasks/new, consommees a la fois par consensus_node.py
(enchere distribuee) et par drl_allocator_node.py (politique apprise).

Chaque tache = "aller chercher / deposer un article a tel noeud du
graphe" (rack ou station). Permet de mesurer, comme demande dans le
sujet, le "temps de traitement d'inventaire" et le "taux d'erreur"
en comparant :
    - le mode heuristique (consensus seul, cout = plus court chemin),
    - le mode DRL (drl_allocator_node),
    - le mode hybride (consensus + biais DRL).
"""

import json
import random
import uuid

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from warehouse_marl.warehouse_graph import build_demo_warehouse


class TaskGeneratorNode(Node):

    def __init__(self):
        super().__init__('task_generator_node')

        self.declare_parameter('mean_period_sec', 4.0)
        self.declare_parameter('priority_min', 0.5)
        self.declare_parameter('priority_max', 2.0)
        self.declare_parameter('seed', 42)
        self.declare_parameter('map_origin_x', -1.5)
        self.declare_parameter('map_origin_y', -1.0)
        self.declare_parameter('map_topic', '/map_fused')
        self.declare_parameter('fallback_map_topic', '/map')
        self.declare_parameter('only_mapped_targets', True)
        # Nav2's circular robot radius is 0.22 m.  Requiring 0.45 m of raw
        # SLAM cells to be known around a target rejected safe goals merely
        # because a few cells at the outer fringe were still unknown.
        self.declare_parameter('target_clearance_m', 0.25)
        # Choix des racks de picking : croyance RFID (true) ou enregistrements
        # WMS seuls (false, reference pour l'evaluation experimentale).
        self.declare_parameter('use_rfid_inventory', True)
        # Demande de type Zipf par rack (0 = uniforme). Une demande
        # concentree laisse des racks rarement visites, dont l'inventaire
        # vieillit : c'est la que les missions de scan RFID sont utiles.
        self.declare_parameter('demand_skew', 0.0)
        # Liberation progressive des ordres (comme un WMS) : pas de nouvelle
        # tache de picking tant que ce nombre de taches est deja ouvert. Les
        # missions de scan RFID ne sont pas limitees.
        self.declare_parameter('max_open_tasks', 6)

        self.mean_period = self.get_parameter('mean_period_sec').value
        self.priority_min = self.get_parameter('priority_min').value
        self.priority_max = self.get_parameter('priority_max').value
        random.seed(self.get_parameter('seed').value)

        self.use_rfid_inventory = bool(
            self.get_parameter('use_rfid_inventory').value)
        self._inventory = {}
        self.max_open_tasks = int(self.get_parameter('max_open_tasks').value)
        self._open_tasks = {}  # task_id -> creation time
        self.only_mapped_targets = bool(
            self.get_parameter('only_mapped_targets').value)
        self.target_clearance = float(
            self.get_parameter('target_clearance_m').value)
        self.graph = build_demo_warehouse(
            float(self.get_parameter('map_origin_x').value),
            float(self.get_parameter('map_origin_y').value),
        )
        self.shelf_nodes = [
            n for n, d in self.graph.G.nodes(data=True) if d['node_type'] == 'shelf'
        ]
        skew = float(self.get_parameter('demand_skew').value)
        ranking = self.shelf_nodes[:]
        random.shuffle(ranking)
        self._demand_weight = {
            shelf: 1.0 / (rank + 1) ** skew for rank, shelf in enumerate(ranking)}

        self.task_pub = self.create_publisher(String, '/warehouse/tasks/new', 10)
        self._map = None
        self._primary_map_received = False
        self._map_topic = str(self.get_parameter('map_topic').value)
        fallback_map_topic = str(
            self.get_parameter('fallback_map_topic').value)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, self._map_topic,
            lambda msg: self._on_map(msg, primary=True), map_qos)
        if fallback_map_topic and fallback_map_topic != self._map_topic:
            self.create_subscription(
                OccupancyGrid, fallback_map_topic,
                lambda msg: self._on_map(msg, primary=False), map_qos)
        self.create_subscription(
            String, '/rfid/inventory', self._on_inventory, 10)
        self.create_subscription(
            String, '/rfid/scan_requests', self._on_scan_request, 10)
        self.create_subscription(
            String, '/warehouse/tasks/status', self._on_task_status, 50)
        self._task_timer = None
        self._last_wait_warning = -float('inf')
        self._schedule_next()

        self.get_logger().info(
            f"TaskGeneratorNode pret : {len(self.shelf_nodes)} emplacements possibles."
        )

    def _schedule_next(self):
        # Loi exponentielle -> arrivees de type processus de Poisson,
        # realiste pour un flux de commandes en entrepot.
        delay = random.expovariate(1.0 / self.mean_period)
        self._task_timer = self.create_timer(delay, self._emit_task_once)

    def _emit_task_once(self):
        # rclpy timers are periodic.  Destroy this instance before scheduling
        # the next exponential delay so the number of timers stays constant.
        timer = self._task_timer
        self._task_timer = None
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
        self._emit_task()
        self._schedule_next()

    def _on_map(self, msg: OccupancyGrid, primary: bool):
        if primary:
            if not self._primary_map_received:
                self.get_logger().info(
                    f'Carte principale recue sur {self._map_topic}.')
            self._primary_map_received = True
            self._map = msg
        elif not self._primary_map_received:
            self._map = msg

    def _on_inventory(self, msg: String):
        try:
            self._inventory = json.loads(msg.data).get('shelves', {})
        except json.JSONDecodeError:
            pass

    def _on_task_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get('state') in ('succeeded', 'failed', 'rejected', 'canceled'):
            self._open_tasks.pop(data.get('task_id'), None)

    def _stocked(self, shelf: str) -> bool:
        """Le rack contient-il des articles a prelever ? Sans donnees
        d'inventaire, tous les racks sont consideres approvisionnes."""
        info = self._inventory.get(shelf)
        if info is None:
            return True
        key = 'stock' if self.use_rfid_inventory else 'wms_stock'
        return info.get(key, 0) > 0

    def _on_scan_request(self, msg: String):
        """Decision adaptative : l'inventaire d'un rack est incertain ou
        ancien -> mission de scan RFID, attribuee par consensus comme les
        autres taches."""
        try:
            data = json.loads(msg.data)
            shelf = data['shelf']
        except (json.JSONDecodeError, KeyError):
            return
        if shelf not in self.shelf_nodes:
            return
        # An operator request (dashboard) is always honoured; automatic
        # requests wait until the shelf's service point is mapped and free.
        if data.get('source') != 'dashboard' and not self._mapped_free(shelf):
            self.get_logger().info(
                f'Scan RFID de {shelf} differe : point de service pas encore cartographie.')
            return
        score = float(data.get('score', 2.0))
        priority = round(min(self.priority_max, max(self.priority_min, score / 2.0)), 2)
        self._publish_task(shelf, priority, 'rfid_scan')

    def _mapped_free(self, node_id: str) -> bool:
        if not self.only_mapped_targets:
            return True
        if self._map is None:
            return False
        node = self.graph.G.nodes[node_id]
        info = self._map.info
        cx = int((node['x'] - info.origin.position.x) / info.resolution)
        cy = int((node['y'] - info.origin.position.y) / info.resolution)
        radius = max(1, int(self.target_clearance / info.resolution))
        if (cx - radius < 0 or cy - radius < 0 or
                cx + radius >= info.width or cy + radius >= info.height):
            return False
        radius_sq = radius * radius
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius_sq:
                    continue
                if self._map.data[(cy + dy) * info.width + cx + dx] != 0:
                    return False
        return True

    def _emit_task(self):
        now = self.get_clock().now().nanoseconds / 1e9
        # A task that never reports a final state must not block releases.
        self._open_tasks = {t: c for t, c in self._open_tasks.items() if now - c < 300.0}
        if len(self._open_tasks) >= self.max_open_tasks:
            return
        candidates = [n for n in self.shelf_nodes
                      if self._mapped_free(n) and self._stocked(n)]
        if not candidates:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_wait_warning >= 10.0:
                self.get_logger().warn(
                    'Aucun rack approvisionne et visible sur la carte; '
                    'generation de tache differee.')
                self._last_wait_warning = now
            return

        target = random.choices(
            candidates, weights=[self._demand_weight[c] for c in candidates])[0]
        priority = round(random.uniform(self.priority_min, self.priority_max), 2)
        self._publish_task(target, priority, 'pick')

    def _publish_task(self, target: str, priority: float, task_type: str):
        task_id = f"task_{uuid.uuid4().hex[:8]}"
        payload = {
            'task_id': task_id,
            'task_type': task_type,
            'target_node': target,
            'priority': priority,
            'created_at': self.get_clock().now().nanoseconds / 1e9,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.task_pub.publish(msg)
        self._open_tasks[task_id] = payload['created_at']
        self.get_logger().info(
            f"Nouvelle tache {task_id} ({task_type}) -> {target} (priorite {priority})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TaskGeneratorNode()
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
