"""
rfid_sensor_node.py
--------------------
Simule la COUCHE DE CAPTEURS RFID + COMMUNICATION IoT decrite dans le sujet :

    "The framework will integrate RFID-based sensing and IoT communication
    layers to enable adaptive decision-making under uncertain conditions."

Comme il n'existe pas de plugin Gazebo RFID standard, ce noeud simule le
comportement physique d'un lecteur RFID embarque sur chaque robot :
  - chaque noeud du graphe (rack, station) porte un tag RFID virtuel
    associe a une position (x, y) et un rayon de lecture ;
  - le noeud lit la pose TF de chaque robot dans la carte partagee ;
  - quand un robot entre dans le rayon de lecture d'un tag, une detection
    est publiee sur /rfid/detections (JSON), avec un bruit de detection
    configurable (taux de faux-negatifs) pour rester realiste et permettre
    de tester la robustesse du framework "sous incertitude" (cf. sujet).

Ce topic est consomme par graph_publisher_node.py (recalage de position)
et peut aussi etre logge/analyse pour tes resultats experimentaux
(reduction du taux d'erreur d'inventaire, cf. "Resultats attendus" du sujet).
"""

import json
import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import tf2_ros

from warehouse_marl.warehouse_graph import build_demo_warehouse


class RFIDSensorNode(Node):

    def __init__(self):
        super().__init__('rfid_sensor_node')

        self.declare_parameter('robot_names', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter('read_radius_m', 0.8)
        self.declare_parameter('detection_noise', 0.05)  # taux de faux-negatifs
        self.declare_parameter('min_republish_period_sec', 2.0)
        self.declare_parameter('pose_update_period_sec', 0.2)
        self.declare_parameter('map_origin_x', -1.5)
        self.declare_parameter('map_origin_y', -1.0)

        self.robot_names = self.get_parameter('robot_names').value
        self.read_radius = self.get_parameter('read_radius_m').value
        self.noise = self.get_parameter('detection_noise').value
        self.min_period = self.get_parameter('min_republish_period_sec').value

        # Reutilise la meme topologie que graph_publisher_node pour que les
        # tags RFID simules correspondent aux memes noeuds physiques.
        self.graph = build_demo_warehouse(
            float(self.get_parameter('map_origin_x').value),
            float(self.get_parameter('map_origin_y').value),
        )

        self._last_pose = {name: (0.0, 0.0) for name in self.robot_names}
        self._last_detection_time = {}  # (robot, node) -> stamp

        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)
        self.create_timer(
            float(self.get_parameter('pose_update_period_sec').value),
            self._update_robot_poses)

        self.rfid_pub = self.create_publisher(String, '/rfid/detections', 10)

        self.get_logger().info(
            f"RFIDSensorNode pret : {len(self.graph.G.nodes)} tags virtuels, "
            f"rayon de lecture {self.read_radius} m, bruit {self.noise*100:.0f}%."
        )

    def _update_robot_poses(self):
        for robot_name in self.robot_names:
            try:
                transform = self._tf.lookup_transform(
                    'map', f'{robot_name}/base_link', rclpy.time.Time())
            except Exception:
                continue
            self._on_pose(
                robot_name,
                transform.transform.translation.x,
                transform.transform.translation.y,
            )

    def _on_pose(self, robot_name: str, x: float, y: float):
        self._last_pose[robot_name] = (x, y)

        for node_id, data in self.graph.G.nodes(data=True):
            tag = data.get('rfid_tag')
            if tag is None:
                continue
            dist = math.hypot(data['x'] - x, data['y'] - y)
            if dist <= self.read_radius:
                self._maybe_publish_detection(robot_name, node_id, tag, dist)

    def _maybe_publish_detection(self, robot_name, node_id, tag, dist):
        now = self.get_clock().now().nanoseconds / 1e9
        key = (robot_name, node_id)
        last = self._last_detection_time.get(key, -math.inf)
        if now - last < self.min_period:
            return  # evite de spammer le meme tag en continu

        # Simulation d'un taux de non-detection (bruit capteur realiste)
        if random.random() < self.noise:
            self.get_logger().debug(
                f"[RFID] Detection manquee (bruit) {robot_name} @ {node_id}"
            )
            return

        self._last_detection_time[key] = now

        payload = {
            'robot_id': robot_name,
            'node_id': node_id,
            'rfid_tag': tag,
            'distance_m': round(dist, 3),
            'stamp': now,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.rfid_pub.publish(msg)
        self.get_logger().info(f"[RFID] {robot_name} a lu {tag} pres de {node_id}")


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
