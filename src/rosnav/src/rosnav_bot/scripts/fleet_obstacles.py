#!/usr/bin/env python3
"""
fleet_obstacles.py
------------------
Makes the robots visible to each other in Nav2.

The robots' lidars sit 0.25 m above the floor, while the robot bodies end at
0.225 m: each lidar's beams pass just over the other robots, so the costmaps
did not contain them and robots bumped into each other (pushes corrupt
odometry and make the estimated pose drift away from the real one).

This node shares positions across the fleet: for every robot it publishes a
point cloud of the *other* robots' footprints (``/<robot>/fleet_obstacles``,
``map`` frame), which each robot's costmap obstacle layers consume as an
extra observation source. Started by multi_robot.launch.py.
"""

import math
import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros

# Robot body in base_link (see robot_core.xacro): x -0.28..0.46, y +/-0.25.
BODY_X = (-0.28, 0.46)
BODY_Y = (-0.25, 0.25)
STEP = 0.08


def _footprint_points():
    pts = []
    nx = int(round((BODY_X[1] - BODY_X[0]) / STEP))
    ny = int(round((BODY_Y[1] - BODY_Y[0]) / STEP))
    for i in range(nx + 1):
        for j in range(ny + 1):
            pts.append((BODY_X[0] + i * (BODY_X[1] - BODY_X[0]) / nx,
                        BODY_Y[0] + j * (BODY_Y[1] - BODY_Y[0]) / ny))
    return pts


class FleetObstaclesNode(Node):

    def __init__(self):
        super().__init__('fleet_obstacles')
        self.declare_parameter('robot_names', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter('rate_hz', 10.0)
        # Same height as the lidar beams (0.175 m in base_link) so the laser
        # clears a robot's old position once it has moved away.
        self.declare_parameter('point_height', 0.175)
        self.robots = list(self.get_parameter('robot_names').value)
        self.z = float(self.get_parameter('point_height').value)
        self.body = _footprint_points()
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.pubs = {r: self.create_publisher(PointCloud2, f'/{r}/fleet_obstacles', 5)
                     for r in self.robots}
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)
        self.get_logger().info(f'Fleet obstacles for {self.robots}.')

    def _pose(self, robot):
        try:
            t = self.tf.lookup_transform('map', f'{robot}/base_link', rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw, t.header.stamp

    def _tick(self):
        poses = {r: self._pose(r) for r in self.robots}
        clouds = {}
        for r, p in poses.items():
            if p is None:
                continue
            x, y, yaw, _ = p
            c, s = math.cos(yaw), math.sin(yaw)
            clouds[r] = [(x + c * bx - s * by, y + s * bx + c * by) for bx, by in self.body]
        stamp = self.get_clock().now().to_msg()
        for r in self.robots:
            pts = [pt for other, cloud in clouds.items() if other != r for pt in cloud]
            self.pubs[r].publish(self._cloud(pts, stamp))

    def _cloud(self, pts, stamp):
        msg = PointCloud2()
        msg.header.frame_id = 'map'
        msg.header.stamp = stamp
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        msg.data = b''.join(struct.pack('<fff', px, py, self.z) for px, py in pts)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = FleetObstaclesNode()
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
