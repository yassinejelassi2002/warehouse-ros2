"""
rfid_metrics_logger.py
-----------------------
Enregistre /rfid/inventory_metrics dans un fichier CSV pendant une
simulation (une ligne toutes les 2 s), pour les figures du memoire.

    ros2 run warehouse_marl rfid_metrics_logger --ros-args -p output:=run1.csv
    ros2 run warehouse_marl plot_rfid_metrics run1.csv
"""

import csv
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

FIELDS = ['stamp', 'rfid_error_rate', 'wms_error_rate', 'items_in_stock',
          'drift_events', 'scans', 'reads', 'pick_tasks', 'pick_errors',
          'scan_tasks', 'mean_uncertainty', 'rfid_missed', 'rfid_phantom',
          'rfid_wrong_shelf']


class RFIDMetricsLogger(Node):

    def __init__(self):
        super().__init__('rfid_metrics_logger')
        self.declare_parameter('output', 'rfid_metrics.csv')
        self.path = str(self.get_parameter('output').value)
        self._file = open(self.path, 'w', newline='')
        self._writer = csv.DictWriter(self._file, fieldnames=FIELDS, extrasaction='ignore')
        self._writer.writeheader()
        self.rows = 0
        self.create_subscription(String, '/rfid/inventory_metrics', self._on_metrics, 10)
        self.get_logger().info(f'Enregistrement des metriques RFID dans {self.path}')

    def _on_metrics(self, msg: String):
        self._writer.writerow(json.loads(msg.data))
        self._file.flush()
        self.rows += 1

    def close(self):
        self._file.close()
        self.get_logger().info(f'{self.rows} lignes ecrites dans {self.path}')


def main(args=None):
    rclpy.init(args=args)
    node = RFIDMetricsLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
