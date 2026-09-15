"""
drl_allocator_node.py
-----------------------
Deploie la POLITIQUE DRL entrainee hors-ligne (drl/train_drl.py) dans la
boucle ROS2 temps-reel. Deux modes d'usage possibles (parametre 'mode') :

  - mode='direct'  : ce noeud agit comme un allocateur CENTRALISE qui
                      choisit lui-meme l'agent gagnant pour chaque tache
                      (via argmax des Q-values) et publie directement
                      l'assignation. Simple, mais reintroduit un point
                      unique de decision.

  - mode='hybrid'  : ce noeud NE DECIDE PAS ; il calcule les Q-values pour
                      la tache courante et publie un "biais" par tache sur
                      /warehouse/consensus/bid_bias, lu par CHAQUE
                      consensus_node.py (un par agent). La decision finale
                      reste prise de facon DISTRIBUEE par consensus, mais
                      orientee par ce que la politique apprise juge
                      favorable. C'est le mode recommande pour rester
                      fidele au sujet ("distributed... consensus-based").

Le modele attend l'observation definie dans drl/env.py : reconstruite ici
a partir de l'etat du graphe (/warehouse/graph_state) et de la charge de
chaque agent (suivie localement via les assignations de consensus).
"""

import json
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from warehouse_marl.warehouse_graph import WarehouseGraph
from warehouse_marl.drl.dqn_agent import DQNAgent


class DRLAllocatorNode(Node):

    def __init__(self):
        super().__init__('drl_allocator_node')

        self.declare_parameter('robot_names', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter('model_path', 'warehouse_dqn.pt')
        self.declare_parameter('mode', 'hybrid')  # 'hybrid' | 'direct'
        self.declare_parameter('max_active_tasks_per_agent', 3)

        self.robot_names = self.get_parameter('robot_names').value
        self.model_path = self.get_parameter('model_path').value
        self.mode = self.get_parameter('mode').value
        self.max_active = self.get_parameter('max_active_tasks_per_agent').value

        self.graph: WarehouseGraph = None
        self.agent_load = {name: 0 for name in self.robot_names}
        self.agent_node = {name: None for name in self.robot_names}

        obs_dim = len(self.robot_names) * 3 + 1
        self.agent = DQNAgent(obs_dim=obs_dim, n_actions=len(self.robot_names))

        self.model_loaded = os.path.isfile(self.model_path)
        if self.model_loaded:
            self.agent.load(self.model_path)
            self.get_logger().info(f"Modele DRL charge depuis {self.model_path}")
        else:
            message = (
                f"Modele {self.model_path} introuvable; aucun poids aleatoire "
                f"ne sera utilise. Entraine d abord via train_drl.")
            if self.mode == 'direct':
                raise FileNotFoundError(message)
            self.get_logger().warn(message + ' Consensus heuristique conserve.')

        self.create_subscription(
            String, '/warehouse/graph_state', self._on_graph_state, 10
        )
        self.create_subscription(
            String, '/warehouse/consensus/assignments', self._on_assignment, 10
        )
        self.create_subscription(
            String, '/warehouse/tasks/direct_assignment', self._on_assignment, 10
        )
        self.create_subscription(
            String, '/warehouse/tasks/status', self._on_task_status, 10
        )
        self.create_subscription(
            String, '/warehouse/tasks/new', self._on_new_task, 10
        )

        self.bias_pub = self.create_publisher(
            String, '/warehouse/consensus/bid_bias', 10
        )
        self.direct_assign_pub = self.create_publisher(
            String, '/warehouse/tasks/direct_assignment', 10
        )

        self.get_logger().info(f"DRLAllocatorNode demarre en mode '{self.mode}'.")

    # ------------------------------------------------------------------ #
    def _on_graph_state(self, msg: String):
        self.graph = WarehouseGraph.from_json(msg.data)
        for name in self.robot_names:
            for nid, data in self.graph.G.nodes(data=True):
                if name in data.get('occupied_by', []):
                    self.agent_node[name] = nid

    def _on_assignment(self, msg: String):
        try:
            data = json.loads(msg.data)
            robot_id = data['robot_id']
        except (json.JSONDecodeError, KeyError):
            return
        if robot_id in self.agent_load:
            self.agent_load[robot_id] += 1

    def _on_task_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            robot_id = data['robot_id']
            terminal = data['state'] in (
                'succeeded', 'failed', 'rejected', 'canceled')
        except (json.JSONDecodeError, KeyError):
            return
        if terminal and robot_id in self.agent_load:
            self.agent_load[robot_id] = max(0, self.agent_load[robot_id] - 1)

    def _on_new_task(self, msg: String):
        if self.graph is None or not self.model_loaded:
            return  # graphe pas encore recu

        try:
            data = json.loads(msg.data)
            task_id = data['task_id']
            target = data['target_node']
            priority = data.get('priority', 1.0)
        except (json.JSONDecodeError, KeyError):
            return

        obs = self._build_observation(target, priority)
        q_values = self.agent.q_values(obs)

        if self.mode == 'direct':
            self._publish_direct_assignment(task_id, target, q_values)
        else:
            self._publish_bias(task_id, q_values)

    # ------------------------------------------------------------------ #
    def _build_observation(self, target: str, priority: float) -> np.ndarray:
        """Reconstruit exactement le format d'observation utilise a
        l'entrainement (voir drl/env.py::_build_observation)."""
        feats = []
        for idx, name in enumerate(self.robot_names):
            src = self.agent_node.get(name)
            if src is not None and self.graph.G.has_node(src):
                _, cost = self.graph.shortest_path(src, target)
                cost = cost if cost != float('inf') else 50.0
            else:
                cost = 50.0
            cost_norm = cost / 20.0
            load_norm = self.agent_load.get(name, 0) / max(self.max_active, 1)
            feats.extend([cost_norm, load_norm, float(idx) / len(self.robot_names)])
        feats.append(priority / 2.0)
        return np.array(feats, dtype=np.float32)

    def _publish_bias(self, task_id: str, q_values: np.ndarray):
        """Convertit les Q-values (plus grand = meilleur) en un cout
        additif (plus petit = meilleur) pour rester homogene avec le cout
        utilise par consensus_node.py. On normalise par agent pour ne
        biaiser QUE le classement relatif, pas l'echelle absolue."""
        costs_bias = -q_values  # inversion : Q eleve -> biais negatif -> cout plus faible
        # Cle composite "task_id:robot_id" -> chaque consensus_node (un par
        # robot) ne retient que les cles qui le concernent (voir
        # consensus_node._on_drl_bias). Evite toute ambiguite d'agregation.
        payload = {
            f"{task_id}:{self.robot_names[i]}": float(costs_bias[i])
            for i in range(len(self.robot_names))
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.bias_pub.publish(msg)

    def _publish_direct_assignment(self, task_id: str, target: str, q_values: np.ndarray):
        best_idx = int(np.argmax(q_values))
        payload = {
            'task_id': task_id,
            'robot_id': self.robot_names[best_idx],
            'target_node': target,
            'q_values': q_values.tolist(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.direct_assign_pub.publish(msg)
        self.get_logger().info(
            f"[DRL direct] {task_id} -> {self.robot_names[best_idx]} "
            f"(Q={q_values.round(2).tolist()})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DRLAllocatorNode()
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
