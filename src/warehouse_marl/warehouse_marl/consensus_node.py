"""
consensus_node.py
-------------------
Implemente le PROTOCOLE DE COORDINATION BASE SUR CONSENSUS demande par le
sujet :

    "Design of a distributed coordination protocol for heterogeneous
    robotic agents ... autonomous agents collaborate through
    consensus-based protocols."

Un noeud consensus_node est lance PAR ROBOT (un agent = un noeud, namespace
/robotX/consensus/*). Chaque agent :

  1) recoit les taches candidates diffusees par task_generator_node
     (/warehouse/tasks/new) ;
  2) calcule son propre "cout" pour realiser chaque tache (distance sur le
     graphe dynamique + charge actuelle), et publie une ENCHERE (bid) sur le
     topic partage /warehouse/consensus/bids ;
  3) ecoute les encheres des AUTRES agents sur ce meme topic ;
  4) applique une regle de consensus decentralisee (ici : "market-based /
     auction consensus", chaque agent attend un court round-trip time puis
     retient localement le gagnant = le cout minimal recu pour chaque
     tache) -> aucun controleur central n'arbitre, chaque agent arrive
     independamment a la MEME conclusion (propriete de consensus).
  5) si l'agent EST le gagnant pour une tache, il s'auto-assigne la tache
     et publie une confirmation sur /warehouse/consensus/assignments.

Ce protocole peut remplacer OU cooperer avec drl_allocator_node.py : le
cout utilise pour l'enchere peut provenir soit d'une heuristique simple
(voir _compute_bid_cost), soit de la Q-value / du score produit par la
politique DRL (voir drl_allocator_node.py -> publish sur
/warehouse/consensus/bid_bias).
"""

import json
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import tf2_ros

from warehouse_marl.warehouse_graph import build_demo_warehouse
from warehouse_marl.warehouse_graph import WarehouseGraph


@dataclass
class TaskState:
    task_id: str
    target_node: str
    priority: float = 1.0
    bids: Dict[str, float] = field(default_factory=dict)   # robot_id -> cost
    deadline: float = 0.0          # timestamp apres lequel on cloture le vote
    assigned_to: Optional[str] = None
    bid_at: float = 0.0


class ConsensusNode(Node):

    def __init__(self):
        super().__init__('consensus_node')

        self.declare_parameter('robot_id', 'robot1')
        self.declare_parameter('bidding_window_sec', 1.5)
        self.declare_parameter('max_concurrent_tasks', 1)
        self.declare_parameter('bid_delay_sec', 0.2)
        self.declare_parameter('map_origin_x', -1.5)
        self.declare_parameter('map_origin_y', -1.0)

        self.robot_id = self.get_parameter('robot_id').value
        self.bidding_window = self.get_parameter('bidding_window_sec').value
        self.max_concurrent = self.get_parameter('max_concurrent_tasks').value
        self.bid_delay = float(self.get_parameter('bid_delay_sec').value)

        self.graph = build_demo_warehouse(
            float(self.get_parameter('map_origin_x').value),
            float(self.get_parameter('map_origin_y').value),
        )
        self.current_pose = (0.0, 0.0)
        self.current_node = None
        self.active_tasks: List[str] = []
        self.tasks: Dict[str, TaskState] = {}

        # Biais optionnel fourni par la politique DRL (voir drl_allocator_node)
        self.drl_bias: Dict[str, float] = {}

        # --- Abonnements ---
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)
        self.create_subscription(
            String, '/warehouse/tasks/new', self._on_new_task, 10
        )
        self.create_subscription(
            String, '/warehouse/consensus/bids', self._on_bid, 10
        )
        self.create_subscription(
            String, '/warehouse/consensus/bid_bias', self._on_drl_bias, 10
        )
        self.create_subscription(
            String, '/warehouse/graph_state', self._on_graph_state, 10
        )
        self.create_subscription(
            String, '/warehouse/tasks/status', self._on_task_status, 10
        )

        # --- Publications ---
        self.bid_pub = self.create_publisher(String, '/warehouse/consensus/bids', 10)
        self.assign_pub = self.create_publisher(
            String, '/warehouse/consensus/assignments', 10
        )

        # Boucle de cloture des encheres (verifie les deadlines)
        self.create_timer(0.2, self._check_deadlines)
        self.create_timer(0.2, self._update_pose)

        self.get_logger().info(f"[{self.robot_id}] ConsensusNode demarre.")

    # ------------------------------------------------------------------ #
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _update_pose(self):
        try:
            transform = self._tf.lookup_transform(
                'map', f'{self.robot_id}/base_link', rclpy.time.Time())
        except Exception:
            return
        self.current_pose = (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )
        # Snap simplifie sur le noeud le plus proche
        best, best_d = None, float('inf')
        for nid, data in self.graph.G.nodes(data=True):
            d = ((data['x'] - self.current_pose[0]) ** 2 +
                 (data['y'] - self.current_pose[1]) ** 2) ** 0.5
            if d < best_d:
                best, best_d = nid, d
        self.current_node = best

    def _on_drl_bias(self, msg: String):
        """Recoit un biais de cout par (tache, agent) issu de la politique
        DRL (cle composite 'task_id:robot_id', cf. drl_allocator_node.py).
        Ne conserve que les entrees concernant CET agent -> permet
        d'hybrider heuristique + apprentissage tout en restant distribue
        (chaque agent ne voit/n'exploite que son propre biais)."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        suffix = f":{self.robot_id}"
        for key, value in data.items():
            if key.endswith(suffix):
                task_id = key[: -len(suffix)]
                self.drl_bias[task_id] = value

    def _on_graph_state(self, msg: String):
        try:
            self.graph = WarehouseGraph.from_json(msg.data)
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self.get_logger().warn('Etat du graphe mal forme; mise a jour ignoree.')

    def _on_task_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            terminal = data['state'] in (
                'succeeded', 'failed', 'rejected', 'canceled')
            task_id = data['task_id']
        except (json.JSONDecodeError, KeyError):
            return
        if not terminal:
            return
        if data['robot_id'] == self.robot_id:
            self.release_task(task_id)
        self.tasks.pop(task_id, None)
        self.drl_bias.pop(task_id, None)

    # ------------------------------------------------------------------ #
    def _on_new_task(self, msg: String):
        try:
            data = json.loads(msg.data)
            task_id = data['task_id']
            target_node = data['target_node']
            priority = data.get('priority', 1.0)
        except (json.JSONDecodeError, KeyError):
            return

        now = self._now()
        if task_id in self.tasks:
            # A bid from another participant can arrive before /tasks/new.
            task = self.tasks[task_id]
            if task.target_node != '?':
                return
            task.target_node = target_node
            task.priority = priority
            task.deadline = now + self.bidding_window
            task.bid_at = now + self.bid_delay
        else:
            self.tasks[task_id] = TaskState(
                task_id=task_id, target_node=target_node,
                priority=priority,
                deadline=now + self.bidding_window,
                bid_at=now + self.bid_delay,
            )

    def _compute_bid_cost(self, target_node: str, task_id: str) -> float:
        """Cout = longueur du plus court chemin dynamique (distance +
        congestion) sur le graphe partage, divise par la priorite de la
        tache, et ajuste par un biais optionnel issu du DRL."""
        if self.current_node is None:
            return float('inf')
        _, path_cost = self.graph.shortest_path(self.current_node, target_node)
        if path_cost == float('inf'):
            return float('inf')

        base_cost = path_cost / max(self.priority_of(task_id), 1e-3)
        bias = self.drl_bias.get(task_id, 0.0)
        return base_cost + bias

    def priority_of(self, task_id: str) -> float:
        t = self.tasks.get(task_id)
        return t.priority if t else 1.0

    def _publish_bid(self, task_id: str, cost: float):
        payload = {
            'task_id': task_id,
            'robot_id': self.robot_id,
            'cost': cost,
            'stamp': self._now(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.bid_pub.publish(msg)
        self.tasks[task_id].bids[self.robot_id] = cost

    def _on_bid(self, msg: String):
        """Chaque agent ecoute TOUTES les encheres (y compris les siennes)
        et met a jour son etat local -> c'est la propriete cle du consensus
        distribue : pas d'arbitre central, chaque agent construit la meme
        vue globale et en derive independamment la meme decision."""
        try:
            data = json.loads(msg.data)
            task_id, robot_id, cost = data['task_id'], data['robot_id'], data['cost']
        except (json.JSONDecodeError, KeyError):
            return

        if task_id not in self.tasks:
            # Tache encore inconnue localement (message arrive avant /new) ->
            # on cree une entree minimale, la deadline sera fixee a reception
            # de /warehouse/tasks/new.
            self.tasks[task_id] = TaskState(
                task_id=task_id, target_node='?',
                deadline=self._now() + self.bidding_window,
                bid_at=self._now() + self.bid_delay,
            )
        self.tasks[task_id].bids[robot_id] = cost

    # ------------------------------------------------------------------ #
    def _check_deadlines(self):
        now = self._now()
        for task_id, task in list(self.tasks.items()):
            if task.assigned_to is not None or task.deadline == 0.0:
                continue
            if (self.robot_id not in task.bids and task.target_node != '?' and
                    now >= task.bid_at and
                    len(self.active_tasks) < self.max_concurrent):
                cost = self._compute_bid_cost(task.target_node, task_id)
                if cost != float('inf'):
                    self._publish_bid(task_id, cost)
            if now < task.deadline:
                continue

            # Fenetre d'enchere close -> consensus local : le gagnant est
            # celui avec le cout minimal (egalite departagee par robot_id
            # pour garantir un resultat DETERMINISTE et IDENTIQUE chez
            # tous les agents, condition necessaire au consensus).
            if not task.bids:
                del self.tasks[task_id]
                continue

            winner = min(task.bids.items(), key=lambda kv: (kv[1], kv[0]))[0]
            task.assigned_to = winner

            if winner == self.robot_id:
                self.active_tasks.append(task_id)
                self._publish_assignment(task)
                self.get_logger().info(
                    f"[{self.robot_id}] GAGNE la tache {task_id} "
                    f"(cout={task.bids[winner]:.2f}, {len(task.bids)} enchereurs)"
                )

    def _publish_assignment(self, task: TaskState):
        payload = {
            'task_id': task.task_id,
            'robot_id': self.robot_id,
            'target_node': task.target_node,
            'stamp': self._now(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.assign_pub.publish(msg)

    def release_task(self, task_id: str):
        """A appeler (ex: depuis fleet_manager / mission_server) quand le
        robot a termine physiquement la tache -> libere le slot d'agent."""
        if task_id in self.active_tasks:
            self.active_tasks.remove(task_id)


def main(args=None):
    rclpy.init(args=args)
    node = ConsensusNode()
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
