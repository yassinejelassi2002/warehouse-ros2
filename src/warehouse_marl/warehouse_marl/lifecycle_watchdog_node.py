"""Finish lifecycle bringups that a Nav2 lifecycle manager left stuck.

On ROS 2 Jazzy with Fast DDS, a lifecycle service reply is occasionally lost
during startup. The node completes its transition (for example
"Planner Server has GridBased planners available"), but the
nav2_lifecycle_manager never receives the reply and waits forever. That
robot then has no AMCL or no Nav2 and never moves.

This node watches every lifecycle node inside the robot namespaces, plus
the shared map providers (``extra_nodes``: the single-mode SLAM
``/slam_toolbox`` and the static ``/map_server``; without them no robot can
localize). If one
stays ``unconfigured`` or ``inactive`` longer than ``grace_sec``, it drives
the node to ``active`` itself, retrying until it succeeds.
"""

from typing import Dict, Optional

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class LifecycleWatchdogNode(Node):

    def __init__(self):
        super().__init__('lifecycle_watchdog')

        self.declare_parameter('robot_names', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter('grace_sec', 45.0)
        self.declare_parameter('check_period_sec', 3.0)
        self.declare_parameter('extra_nodes', ['/slam_toolbox', '/map_server'])

        self.robot_names = list(self.get_parameter('robot_names').value)
        self.grace = float(self.get_parameter('grace_sec').value)
        self.extra_nodes = set(self.get_parameter('extra_nodes').value)

        self._get_clients: Dict[str, object] = {}
        self._set_clients: Dict[str, object] = {}
        self._not_active_since: Dict[str, float] = {}
        self._pending: Dict[str, float] = {}  # fqn -> time of call in flight
        self._recovered = set()

        self.create_timer(
            float(self.get_parameter('check_period_sec').value), self._check)
        self.get_logger().info(
            f'Lifecycle watchdog for {self.robot_names} + {sorted(self.extra_nodes)} '
            f'(grace {self.grace:.0f}s).')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _lifecycle_nodes(self):
        services = {name for name, _ in self.get_service_names_and_types()}
        for name, ns in self.get_node_names_and_namespaces():
            fqn = f'{ns.rstrip("/")}/{name}'
            if ns.strip('/').split('/')[0] not in self.robot_names and fqn not in self.extra_nodes:
                continue
            if f'{fqn}/get_state' in services and f'{fqn}/change_state' in services:
                yield fqn

    def _check(self):
        now = self._now()
        for fqn in self._lifecycle_nodes():
            # A lost reply must not block this node either: retry after 10 s.
            if now - self._pending.get(fqn, -1e9) < 10.0:
                continue
            if fqn not in self._get_clients:
                self._get_clients[fqn] = self.create_client(GetState, f'{fqn}/get_state')
                self._set_clients[fqn] = self.create_client(ChangeState, f'{fqn}/change_state')
            self._pending[fqn] = now
            future = self._get_clients[fqn].call_async(GetState.Request())
            future.add_done_callback(
                lambda f, n=fqn, t=now: self._on_state(n, t, f))

    def _on_state(self, fqn: str, now: float, future):
        self._pending.pop(fqn, None)
        try:
            state = future.result().current_state.id
        except Exception:
            return

        if state == State.PRIMARY_STATE_ACTIVE:
            self._not_active_since.pop(fqn, None)
            if fqn in self._recovered:
                self._recovered.discard(fqn)
                self.get_logger().info(f'{fqn} is active.')
            return

        transition = self._next_transition(state)
        if transition is None:
            return
        since = self._not_active_since.setdefault(fqn, now)
        if now - since < self.grace:
            return

        self._recovered.add(fqn)
        self.get_logger().warn(
            f'{fqn} stuck in state {state} for {now - since:.0f}s; '
            f'sending transition {transition}.')
        request = ChangeState.Request()
        request.transition.id = transition
        self._pending[fqn] = now
        future = self._set_clients[fqn].call_async(request)
        future.add_done_callback(lambda f, n=fqn: self._pending.pop(n, None))

    @staticmethod
    def _next_transition(state: int) -> Optional[int]:
        if state == State.PRIMARY_STATE_UNCONFIGURED:
            return Transition.TRANSITION_CONFIGURE
        if state == State.PRIMARY_STATE_INACTIVE:
            return Transition.TRANSITION_ACTIVATE
        return None


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
