"""
rfid_sensor_node.py
--------------------
Simule la COUCHE DE CAPTEURS RFID decrite dans le sujet :
    "The framework will integrate RFID-based sensing and IoT communication
    layers to enable adaptive decision-making under uncertain conditions."

Il n'existe pas de plugin Gazebo RFID standard ; ce noeud simule donc un
lecteur UHF embarque sur chaque robot, avec le modele physique de
rfid_model.RFIDReaderModel (lecture probabiliste selon la distance, RSSI
bruite, lectures fantomes). Les poses des robots viennent de TF
(map -> robotN/base_link).

Deux familles de tags :
  - tags de LOCALISATION, un par noeud du graphe (racks, stations) ->
    /rfid/detections, utilises par graph_publisher_node pour recaler la
    position logique des robots ;
  - tags d'ARTICLES poses sur les faces des racks (inventaire) ->
    /rfid/scans, un message par robot et par cycle de scan, consomme par
    inventory_node.

Le noeud porte aussi la VERITE TERRAIN de l'inventaire (rfid_model.
InventoryWorld) : toutes les ``drift_period_sec`` secondes, un evenement non
enregistre dans le WMS se produit (article deplace, retire, arrivee non
saisie). /rfid/ground_truth publie la verite et les enregistrements WMS,
uniquement pour l'evaluation (calcul du taux d'erreur d'inventaire).

Couche communication : topics ROS 2 (DDS publish/subscribe) en JSON.
"""

import json
import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import tf2_ros

from warehouse_marl.rfid_model import InventoryWorld, RFIDReaderModel, rack_faces
from warehouse_marl.warehouse_graph import build_demo_warehouse


class RFIDSensorNode(Node):

    def __init__(self):
        super().__init__('rfid_sensor_node')

        self.declare_parameter('robot_names', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter('map_origin_x', -1.5)
        self.declare_parameter('map_origin_y', -1.0)
        self.declare_parameter('scan_period_sec', 1.0)
        self.declare_parameter('min_republish_period_sec', 2.0)
        self.declare_parameter('items_per_shelf', 4)
        self.declare_parameter('drift_period_sec', 20.0)
        self.declare_parameter('seed', 7)
        # Modele du lecteur (voir rfid_model.RFIDReaderModel)
        self.declare_parameter('range_50_m', 2.0)
        self.declare_parameter('max_range_m', 3.5)
        self.declare_parameter('rssi_noise_db', 2.5)
        self.declare_parameter('ghost_rate', 0.01)

        self.robot_names = list(self.get_parameter('robot_names').value)
        self.min_period = float(self.get_parameter('min_republish_period_sec').value)
        ox = float(self.get_parameter('map_origin_x').value)
        oy = float(self.get_parameter('map_origin_y').value)
        seed = int(self.get_parameter('seed').value)

        self.model = RFIDReaderModel(
            range_50=float(self.get_parameter('range_50_m').value),
            max_range=float(self.get_parameter('max_range_m').value),
            rssi_noise_db=float(self.get_parameter('rssi_noise_db').value),
            ghost_rate=float(self.get_parameter('ghost_rate').value),
        )
        self.rng = random.Random(seed + 1)
        self.graph = build_demo_warehouse(ox, oy)
        self.world = InventoryWorld(
            rack_faces(ox, oy),
            items_per_shelf=int(self.get_parameter('items_per_shelf').value),
            seed=seed)

        self._last_detection_time = {}  # (robot, node) -> stamp
        self.drift_events = 0

        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)

        self.rfid_pub = self.create_publisher(String, '/rfid/detections', 10)
        self.scan_pub = self.create_publisher(String, '/rfid/scans', 50)
        self.truth_pub = self.create_publisher(String, '/rfid/ground_truth', 10)

        self.create_timer(float(self.get_parameter('scan_period_sec').value), self._scan)
        drift_period = float(self.get_parameter('drift_period_sec').value)
        if drift_period > 0.0:
            self.create_timer(drift_period, self._drift)
        self.create_timer(2.0, self._publish_truth)

        self.get_logger().info(
            f'RFIDSensorNode pret : {len(self.graph.G.nodes)} tags de localisation, '
            f'{len(self.world.items)} articles tagges, portee {self.model.max_range} m '
            f'(50% a {self.model.range_50} m), derive toutes les {drift_period:.0f} s.')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _scan(self):
        now = self._now()
        for robot_name in self.robot_names:
            try:
                transform = self._tf.lookup_transform(
                    'map', f'{robot_name}/base_link', rclpy.time.Time())
            except Exception:
                continue
            xy = (transform.transform.translation.x, transform.transform.translation.y)
            self._read_location_tags(robot_name, xy, now)

            reads = self.world.scan(xy, self.model, self.rng)
            msg = String()
            msg.data = json.dumps({
                'robot_id': robot_name,
                'stamp': now,
                'x': round(xy[0], 3),
                'y': round(xy[1], 3),
                'reads': [{'tag': tag, 'rssi': round(rssi, 1)} for tag, rssi in reads],
            })
            self.scan_pub.publish(msg)

    def _read_location_tags(self, robot_name, xy, now):
        for node_id, data in self.graph.G.nodes(data=True):
            tag = data.get('rfid_tag')
            if tag is None:
                continue
            dist = math.dist(xy, (data['x'], data['y']))
            if dist > self.model.max_range:
                continue
            key = (robot_name, node_id)
            if now - self._last_detection_time.get(key, -math.inf) < self.min_period:
                continue
            if self.rng.random() >= self.model.read_probability(dist):
                continue  # lecture manquee
            self._last_detection_time[key] = now
            msg = String()
            msg.data = json.dumps({
                'kind': 'location',
                'robot_id': robot_name,
                'node_id': node_id,
                'rfid_tag': tag,
                'distance_m': round(dist, 3),
                'rssi': round(self.model.rssi(dist, self.rng), 1),
                'stamp': now,
            })
            self.rfid_pub.publish(msg)

    def _drift(self):
        self.drift_events += 1
        event = self.world.drift()
        self.get_logger().info(f'[RFID][verite terrain] {event}')

    def _publish_truth(self):
        msg = String()
        msg.data = json.dumps({
            'stamp': self._now(),
            'truth': self.world.truth(),
            'wms': self.world.wms,
            'drift_events': self.drift_events,
        })
        self.truth_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RFIDSensorNode()
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
