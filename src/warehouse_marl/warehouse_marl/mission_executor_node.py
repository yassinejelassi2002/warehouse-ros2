"""Execute warehouse task assignments through each robot's Nav2 action.

The allocation layer intentionally decides *who* should perform a task.  This
node is the missing physical bridge: it resolves the logical graph target to a
safe map-frame pose, queues one mission per robot, sends NavigateToPose goals,
and publishes lifecycle status used to release allocator capacity.
"""

from collections import deque
from dataclasses import dataclass
import json
from typing import Deque, Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from warehouse_marl.warehouse_graph import build_demo_warehouse


@dataclass
class Mission:
    task_id: str
    robot_id: str
    target_node: str
    created_at: Optional[float] = None
    retries: int = 0


class MissionExecutorNode(Node):

    def __init__(self):
        super().__init__('mission_executor_node')

        self.declare_parameter('robot_names', ['robot1', 'robot2'])
        self.declare_parameter('map_origin_x', -1.5)
        self.declare_parameter('map_origin_y', -1.0)
        self.declare_parameter('dispatch_period_sec', 0.25)
        self.declare_parameter('max_retries', 1)
        self.declare_parameter('pause_exploration', True)

        self.robot_names = list(self.get_parameter('robot_names').value)
        self.max_retries = int(self.get_parameter('max_retries').value)
        self.pause_exploration = bool(
            self.get_parameter('pause_exploration').value)
        self.graph = build_demo_warehouse(
            float(self.get_parameter('map_origin_x').value),
            float(self.get_parameter('map_origin_y').value),
        )

        # Avoid rclpy.Node's internal ``_clients`` attribute.
        self._nav_clients = {
            robot: ActionClient(
                self, NavigateToPose, f'/{robot}/navigate_to_pose')
            for robot in self.robot_names
        }
        self._queues: Dict[str, Deque[Mission]] = {
            robot: deque() for robot in self.robot_names
        }
        self._active: Dict[str, Optional[Mission]] = {
            robot: None for robot in self.robot_names
        }
        self._task_metadata = {}
        self._claimed_tasks = set()
        self._last_server_warning = {robot: -float('inf') for robot in self.robot_names}
        self._missions_active = None

        self.create_subscription(
            String, '/warehouse/tasks/new', self._on_new_task, 10)
        self.create_subscription(
            String, '/warehouse/consensus/assignments', self._on_assignment, 10)
        self.create_subscription(
            String, '/warehouse/tasks/direct_assignment', self._on_assignment, 10)
        self.status_pub = self.create_publisher(
            String, '/warehouse/tasks/status', 10)

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.mission_state_pub = self.create_publisher(
            Bool, '/warehouse/missions_active', state_qos)
        self._publish_mission_state(force=True)

        self.create_timer(
            float(self.get_parameter('dispatch_period_sec').value),
            self._dispatch)
        self.get_logger().info(
            f'MissionExecutorNode pret pour {self.robot_names}; '
            f'cibles Nav2 dans frame map.')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_new_task(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._task_metadata[data['task_id']] = data
        except (json.JSONDecodeError, KeyError):
            self.get_logger().warn('Tache JSON mal formee; ignoree.')

    def _on_assignment(self, msg: String):
        try:
            data = json.loads(msg.data)
            task_id = str(data['task_id'])
            robot_id = str(data['robot_id'])
            target_node = str(data['target_node'])
        except (json.JSONDecodeError, KeyError):
            self.get_logger().warn('Assignation JSON mal formee; ignoree.')
            return

        if task_id in self._claimed_tasks:
            return
        if robot_id not in self._queues:
            self.get_logger().error(
                f'Assignation {task_id}: robot inconnu {robot_id!r}.')
            return
        if target_node not in self.graph.G:
            self.get_logger().error(
                f'Assignation {task_id}: cible inconnue {target_node!r}.')
            self._emit_status_fields(
                task_id, robot_id, target_node, 'rejected',
                detail='unknown target node')
            return

        metadata = self._task_metadata.get(task_id, {})
        mission = Mission(
            task_id=task_id,
            robot_id=robot_id,
            target_node=target_node,
            created_at=metadata.get('created_at'),
        )
        self._claimed_tasks.add(task_id)
        self._queues[robot_id].append(mission)
        self._emit_status(mission, 'queued')
        self._publish_mission_state()
        self.get_logger().info(
            f'{task_id} en file pour {robot_id} -> {target_node}.')

    def _dispatch(self):
        for robot_id in self.robot_names:
            if self._active[robot_id] is not None or not self._queues[robot_id]:
                continue
            client = self._nav_clients[robot_id]
            if not client.server_is_ready():
                now = self._now()
                if now - self._last_server_warning[robot_id] >= 10.0:
                    self.get_logger().warn(
                        f'Attend /{robot_id}/navigate_to_pose.')
                    self._last_server_warning[robot_id] = now
                continue

            mission = self._queues[robot_id].popleft()
            self._active[robot_id] = mission
            node = self.graph.G.nodes[mission.target_node]
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(node['x'])
            goal.pose.pose.position.y = float(node['y'])
            goal.pose.pose.orientation.w = 1.0

            future = client.send_goal_async(goal)
            future.add_done_callback(
                lambda result, item=mission: self._on_goal_response(result, item))

    def _on_goal_response(self, future, mission: Mission):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'{mission.task_id}: erreur envoi Nav2: {exc}')
            self._finish_or_retry(mission, GoalStatus.STATUS_UNKNOWN)
            return

        if not handle.accepted:
            self.get_logger().warn(
                f'{mission.task_id}: objectif refuse par {mission.robot_id}.')
            self._finish(mission, 'rejected', GoalStatus.STATUS_UNKNOWN)
            return

        self._emit_status(mission, 'navigating')
        node = self.graph.G.nodes[mission.target_node]
        self.get_logger().info(
            f'{mission.robot_id} execute {mission.task_id} -> '
            f'({node["x"]:.2f}, {node["y"]:.2f}).')
        result = handle.get_result_async()
        result.add_done_callback(
            lambda response, item=mission: self._on_result(response, item))

    def _on_result(self, future, mission: Mission):
        try:
            status = int(future.result().status)
        except Exception as exc:
            self.get_logger().error(
                f'{mission.task_id}: resultat Nav2 indisponible: {exc}')
            status = GoalStatus.STATUS_UNKNOWN

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'{mission.robot_id} a termine {mission.task_id}.')
            self._finish(mission, 'succeeded', status)
        else:
            self._finish_or_retry(mission, status)

    def _finish_or_retry(self, mission: Mission, nav_status: int):
        if mission.retries < self.max_retries:
            mission.retries += 1
            self._active[mission.robot_id] = None
            self._queues[mission.robot_id].appendleft(mission)
            self._emit_status(
                mission, 'retrying', nav_status=nav_status,
                detail=f'retry {mission.retries}/{self.max_retries}')
            self.get_logger().warn(
                f'{mission.task_id} echoue (Nav2={nav_status}); '
                f'nouvel essai {mission.retries}/{self.max_retries}.')
            return

        state = ('canceled' if nav_status == GoalStatus.STATUS_CANCELED
                 else 'failed')
        self.get_logger().error(
            f'{mission.task_id} termine en echec (Nav2={nav_status}).')
        self._finish(mission, state, nav_status)

    def _finish(self, mission: Mission, state: str, nav_status: int):
        self._emit_status(mission, state, nav_status=nav_status)
        self._active[mission.robot_id] = None
        self._task_metadata.pop(mission.task_id, None)
        self._publish_mission_state()

    def _emit_status(
        self,
        mission: Mission,
        state: str,
        nav_status: Optional[int] = None,
        detail: str = '',
    ):
        self._emit_status_fields(
            mission.task_id, mission.robot_id, mission.target_node, state,
            created_at=mission.created_at, retries=mission.retries,
            nav_status=nav_status, detail=detail)

    def _emit_status_fields(
        self,
        task_id: str,
        robot_id: str,
        target_node: str,
        state: str,
        created_at: Optional[float] = None,
        retries: int = 0,
        nav_status: Optional[int] = None,
        detail: str = '',
    ):
        now = self._now()
        payload = {
            'task_id': task_id,
            'robot_id': robot_id,
            'target_node': target_node,
            'state': state,
            'stamp': now,
            'retries': retries,
        }
        if created_at is not None:
            payload['created_at'] = created_at
            payload['elapsed_sec'] = max(0.0, now - float(created_at))
        if nav_status is not None:
            payload['nav_status'] = nav_status
        if detail:
            payload['detail'] = detail
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def _publish_mission_state(self, force: bool = False):
        active = any(self._queues.values()) or any(
            mission is not None for mission in self._active.values())
        if not self.pause_exploration:
            active = False
        if not force and active == self._missions_active:
            return
        self._missions_active = active
        msg = Bool()
        msg.data = active
        self.mission_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutorNode()
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
