"""
graph_publisher_node.py
------------------------
Noeud ROS2 qui :
  1) construit/maintient le WarehouseGraph (graphe dynamique) ;
  2) lit la pose TF ``map -> robotX/base_link`` pour mettre a jour
     l'occupation des noeuds et la congestion des aretes ;
  3) ecoute les detections RFID publiees par rfid_sensor_node sur
     /rfid/detections pour recaler la position logique d'un robot sur un
     noeud precis du graphe (RFID = verite terrain, l'odometrie derive) ;
  4) publie periodiquement l'etat complet du graphe (JSON) sur
     /warehouse/graph_state, consomme par consensus_node.py et
     drl_allocator_node.py.

Ce noeud fait le pont entre la simulation Gazebo/Nav2 (rosnav) et la
couche de coordination multi-agents (consensus + DRL).
"""

import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import tf2_ros

from warehouse_marl.warehouse_graph import build_demo_warehouse


class GraphPublisherNode(Node):

    def __init__(self):
        super().__init__('graph_publisher_node')

        self.declare_parameter('robot_names', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('pose_update_period_sec', 0.2)
        self.declare_parameter('node_snap_radius', 1.5)  # m
        self.declare_parameter('map_origin_x', -1.5)
        self.declare_parameter('map_origin_y', -1.0)

        self.robot_names = self.get_parameter('robot_names').value
        period = self.get_parameter('publish_period_sec').value
        self.snap_radius = self.get_parameter('node_snap_radius').value

        self.graph = build_demo_warehouse(
            float(self.get_parameter('map_origin_x').value),
            float(self.get_parameter('map_origin_y').value),
        )
        self._robot_last_node = {name: None for name in self.robot_names}
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)

        # Robot odometry is local to each spawn and cannot be compared in one
        # warehouse graph.  The shared map TF is the common source of truth.
        for name in self.robot_names:
            self.get_logger().info(f"Suit TF map -> {name}/base_link")

        # Abonnement aux detections RFID (recalage position logique)
        self.create_subscription(
            String, '/rfid/detections', self._on_rfid, 10
        )

        # Publication de l'etat du graphe
        self.graph_pub = self.create_publisher(String, '/warehouse/graph_state', 10)
        self.create_timer(
            float(self.get_parameter('pose_update_period_sec').value),
            self._update_robot_poses)
        self.create_timer(period, self._publish_graph)

        self.get_logger().info(
            f"GraphPublisherNode demarre avec {len(self.graph.G.nodes)} noeuds "
            f"et {len(self.graph.G.edges)} aretes."
        )

    # ------------------------------------------------------------------ #
    def _nearest_node(self, x: float, y: float):
        best, best_d = None, math.inf
        for nid, data in self.graph.G.nodes(data=True):
            d = math.hypot(data['x'] - x, data['y'] - y)
            if d < best_d:
                best, best_d = nid, d
        return best, best_d

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

        nearest, dist = self._nearest_node(x, y)
        if nearest is None or dist > self.snap_radius:
            return

        prev = self._robot_last_node[robot_name]
        if prev == nearest:
            return  # pas de changement de noeud

        # Libere le noeud precedent, occupe le nouveau
        if prev is not None:
            self.graph.set_node_occupancy(prev, robot_name, occupied=False)
            if self.graph.G.has_edge(prev, nearest):
                self.graph.update_congestion(prev, nearest, delta=1.0)
        self.graph.set_node_occupancy(nearest, robot_name, occupied=True)
        self._robot_last_node[robot_name] = nearest

        self.get_logger().debug(f"{robot_name} -> noeud {nearest} (d={dist:.2f}m)")

    def _on_rfid(self, msg: String):
        """Recale la position logique d'un robot quand un tag RFID est lu :
        c'est la verite terrain qui corrige la derive d'odometrie."""
        try:
            data = json.loads(msg.data)
            robot_name = data['robot_id']
            node_id = data['node_id']
            tag = data['rfid_tag']
        except (json.JSONDecodeError, KeyError):
            self.get_logger().warn("Message RFID mal forme, ignore.")
            return

        if data.get('kind', 'location') != 'location':
            return  # seuls les tags de localisation recalent la position
        if node_id not in self.graph.G.nodes:
            return

        self.graph.register_rfid_read(node_id, tag)

        prev = self._robot_last_node[robot_name] if robot_name in self._robot_last_node else None
        if prev != node_id:
            if prev is not None:
                self.graph.set_node_occupancy(prev, robot_name, occupied=False)
            self.graph.set_node_occupancy(node_id, robot_name, occupied=True)
            self._robot_last_node[robot_name] = node_id
            self.get_logger().info(
                f"[RFID] Recalage {robot_name} -> {node_id} (tag {tag})"
            )

    def _publish_graph(self):
        # Congestion is recent-use memory, so let it decay over time.
        for u, v in self.graph.G.edges:
            self.graph.update_congestion(u, v, delta=-0.1)
        msg = String()
        msg.data = self.graph.to_json()
        self.graph_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GraphPublisherNode()
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
