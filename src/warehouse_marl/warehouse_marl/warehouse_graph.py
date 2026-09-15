"""
warehouse_graph.py
-------------------
Modelise l'entrepot comme un GRAPHE DYNAMIQUE, conformement a la description
du sujet de Master :

    "The warehouse will be modeled as a dynamic graph where autonomous
    agents collaborate through consensus-based protocols."

Un noeud represente soit :
    - une station de stockage / un rack (type='shelf')
    - une station de picking / un quai de chargement (type='station')
    - un point de passage (type='waypoint')

Une arete represente un chemin navigable entre deux noeuds, avec un poids
qui evolue dynamiquement en fonction :
    - de la distance euclidienne,
    - du niveau de congestion (nombre de robots presents / recents),
    - d'un etat bloque/libre (issu par ex. d'une detection RFID ou d'un
      obstacle detecte par le stack de navigation).

Cette classe est volontairement independante de ROS2 (aucune dependance a
rclpy) afin de pouvoir etre :
    1) utilisee telle quelle dans l'environnement Gymnasium d'entrainement
       DRL (drl/env.py), sans lancer de noeud ROS2 ;
    2) enveloppee par un noeud ROS2 (graph_publisher_node.py) qui la publie
       et la met a jour a partir des topics de simulation.
"""

from __future__ import annotations

import json
import heapq
import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


class _NodeView:
    """Small subset of NetworkX's NodeView used by this package."""

    def __init__(self, graph):
        self._graph = graph

    def __iter__(self):
        return iter(self._graph._nodes)

    def __len__(self):
        return len(self._graph._nodes)

    def __contains__(self, node_id):
        return node_id in self._graph._nodes

    def __getitem__(self, node_id):
        return self._graph._nodes[node_id]

    def __call__(self, data=False):
        if data:
            return self._graph._nodes.items()
        return self


class _EdgeView:
    """Iterable edge view compatible with the accesses used by ROS nodes."""

    def __init__(self, graph):
        self._graph = graph

    def __iter__(self):
        for source, targets in self._graph._adj.items():
            for target in targets:
                yield source, target

    def __len__(self):
        return sum(len(targets) for targets in self._graph._adj.values())


class _SimpleDiGraph:
    """Dependency-free directed attributed graph for the warehouse topology."""

    def __init__(self):
        self._nodes = {}
        self._adj = {}
        self.nodes = _NodeView(self)
        self.edges = _EdgeView(self)

    def __contains__(self, node_id):
        return node_id in self._nodes

    def __getitem__(self, source):
        return self._adj[source]

    def add_node(self, node_id, **attributes):
        self._nodes[node_id] = attributes
        self._adj.setdefault(node_id, {})

    def add_edge(self, source, target, **attributes):
        if source not in self._nodes or target not in self._nodes:
            raise KeyError(f'unknown edge endpoint: {source!r} -> {target!r}')
        self._adj[source][target] = attributes

    def has_node(self, node_id):
        return node_id in self._nodes

    def has_edge(self, source, target):
        return source in self._adj and target in self._adj[source]


@dataclass
class WarehouseNode:
    node_id: str
    x: float
    y: float
    node_type: str = "waypoint"      # 'shelf' | 'station' | 'waypoint'
    capacity: int = 1                # nb de robots simultanes autorises
    occupied_by: List[str] = field(default_factory=list)
    rfid_tag: Optional[str] = None   # identifiant du tag RFID associe


class WarehouseGraph:
    """Graphe dynamique pondere de l'entrepot."""

    def __init__(self):
        self.G = _SimpleDiGraph()
        self._last_update = time.time()

    # ------------------------------------------------------------------ #
    # Construction du graphe
    # ------------------------------------------------------------------ #
    def add_node(self, node: WarehouseNode) -> None:
        self.G.add_node(
            node.node_id,
            x=node.x,
            y=node.y,
            node_type=node.node_type,
            capacity=node.capacity,
            occupied_by=list(node.occupied_by),
            rfid_tag=node.rfid_tag,
        )

    def add_edge(self, u: str, v: str, bidirectional: bool = True) -> None:
        """Ajoute une arete ponderee par la distance euclidienne (poids de base)."""
        w = self._euclidean(u, v)
        self.G.add_edge(u, v, base_weight=w, congestion=0.0, blocked=False)
        if bidirectional:
            self.G.add_edge(v, u, base_weight=w, congestion=0.0, blocked=False)

    def _euclidean(self, u: str, v: str) -> float:
        xu, yu = self.G.nodes[u]["x"], self.G.nodes[u]["y"]
        xv, yv = self.G.nodes[v]["x"], self.G.nodes[v]["y"]
        return math.hypot(xv - xu, yv - yu)

    # ------------------------------------------------------------------ #
    # Mise a jour dynamique (appelee a chaque tick de simulation)
    # ------------------------------------------------------------------ #
    def update_congestion(self, u: str, v: str, delta: float) -> None:
        """Augmente/diminue le niveau de congestion d'une arete (ex: un robot
        vient de l'emprunter -> +delta ; un robot vient de la liberer -> -delta)."""
        if self.G.has_edge(u, v):
            c = self.G[u][v].get("congestion", 0.0) + delta
            self.G[u][v]["congestion"] = max(0.0, c)

    def set_blocked(self, u: str, v: str, blocked: bool) -> None:
        """Marque une arete comme bloquee (obstacle detecte, robot en panne...)."""
        if self.G.has_edge(u, v):
            self.G[u][v]["blocked"] = blocked

    def set_node_occupancy(self, node_id: str, robot_id: str, occupied: bool) -> None:
        occ = self.G.nodes[node_id]["occupied_by"]
        if occupied and robot_id not in occ:
            occ.append(robot_id)
        elif not occupied and robot_id in occ:
            occ.remove(robot_id)

    def register_rfid_read(self, node_id: str, rfid_tag: str) -> None:
        """Associe/confirme un tag RFID lu physiquement a un noeud du graphe.
        Permet de recaler la position logique d'un agent avec la realite
        physique (voir rfid_sensor_node.py)."""
        if node_id in self.G.nodes:
            self.G.nodes[node_id]["rfid_tag"] = rfid_tag

    def edge_cost(self, u: str, v: str, congestion_weight: float = 2.0) -> float:
        """Cout effectif d'une arete = distance + penalite de congestion,
        ou +infini si l'arete est bloquee. Utilise par le planificateur de
        chemin ET par l'environnement DRL comme composante de la fonction
        de recompense/etat."""
        if not self.G.has_edge(u, v):
            return math.inf
        data = self.G[u][v]
        if data.get("blocked", False):
            return math.inf
        return data["base_weight"] + congestion_weight * data.get("congestion", 0.0)

    def shortest_path(self, source: str, target: str) -> Tuple[List[str], float]:
        """Plus court chemin en tenant compte de la congestion courante
    (Dijkstra sur le cout dynamique edge_cost)."""
        if not self.G.has_node(source) or not self.G.has_node(target):
            return [], math.inf
        distances = {source: 0.0}
        previous = {}
        frontier = [(0.0, source)]
        while frontier:
            cost, node = heapq.heappop(frontier)
            if cost != distances.get(node):
                continue
            if node == target:
                path = [target]
                while path[-1] != source:
                    path.append(previous[path[-1]])
                path.reverse()
                return path, cost
            for neighbour in self.G._adj[node]:
                edge_cost = self.edge_cost(node, neighbour)
                candidate = cost + edge_cost
                if candidate < distances.get(neighbour, math.inf):
                    distances[neighbour] = candidate
                    previous[neighbour] = node
                    heapq.heappush(frontier, (candidate, neighbour))
        return [], math.inf

    # ------------------------------------------------------------------ #
    # (Se)Serialisation -> utilise pour publier l'etat sur un topic ROS2
    # ------------------------------------------------------------------ #
    def to_json(self) -> str:
        payload = {
            "timestamp": time.time(),
            "nodes": [
                {"id": n, **self.G.nodes[n]} for n in self.G.nodes
            ],
            "edges": [
                {"u": u, "v": v, **self.G[u][v]} for u, v in self.G.edges
            ],
        }
        return json.dumps(payload)

    @classmethod
    def from_json(cls, payload: str) -> "WarehouseGraph":
        data = json.loads(payload)
        g = cls()
        for n in data["nodes"]:
            node = WarehouseNode(
                node_id=n["id"], x=n["x"], y=n["y"],
                node_type=n.get("node_type", "waypoint"),
                capacity=n.get("capacity", 1),
                occupied_by=n.get("occupied_by", []),
                rfid_tag=n.get("rfid_tag"),
            )
            g.add_node(node)
        for e in data["edges"]:
            g.G.add_edge(e["u"], e["v"], base_weight=e["base_weight"],
                         congestion=e["congestion"], blocked=e["blocked"])
        return g


def build_demo_warehouse(
    map_origin_x: float = 0.0,
    map_origin_y: float = 0.0,
) -> WarehouseGraph:
    """Build the logical graph for ``rosnav_bot/worlds/warehouse.world``.

    Coordinates are *service points in navigable aisles*, not the centres of
    collision meshes.  ``map_origin_x/y`` is the Gazebo-world position of the
    SLAM map origin (normally robot1's spawn).  Subtracting it here keeps every
    graph consumer in Nav2's ``map`` frame while preserving the same edge
    lengths for offline training, where the default origin is sufficient.
    """
    g = WarehouseGraph()

    def map_xy(world_x: float, world_y: float) -> Tuple[float, float]:
        return world_x - map_origin_x, world_y - map_origin_y

    # One reachable picking point per physical shelf segment (5 rows x 3
    # segments).  Rows 4 and 5 share the wide drive aisle between them.
    # The other values are the centres of the documented drive aisles.
    shelf_x = (0.0, 3.5, 7.0)
    service_y = (5.30, 2.25, -1.50, -5.20, -5.20)
    for row in range(5):
        for col, world_x in enumerate(shelf_x):
            nid = f"shelf_{row}_{col}"
            x, y = map_xy(world_x, service_y[row])
            g.add_node(WarehouseNode(
                node_id=nid, x=x, y=y,
                node_type="shelf", capacity=1,
                rfid_tag=f"RFID-SHELF-{row}{col}",
            ))

    # Safe staging and transfer points.  The storage entrance is centred in
    # the separator-wall opening at y=0.
    station_a = map_xy(-7.0, 4.0)
    station_b = map_xy(-7.0, -4.0)
    dock = map_xy(-2.15, 0.0)
    g.add_node(WarehouseNode(node_id="station_A", x=station_a[0], y=station_a[1],
                              node_type="station", capacity=2,
                              rfid_tag="RFID-STATION-A"))
    g.add_node(WarehouseNode(node_id="station_B", x=station_b[0], y=station_b[1],
                              node_type="station", capacity=2,
                              rfid_tag="RFID-STATION-B"))
    g.add_node(WarehouseNode(node_id="charging_dock", x=dock[0], y=dock[1],
                              node_type="station", capacity=4,
                              rfid_tag="RFID-DOCK"))

    # Aretes : chaque rack relie a ses voisins directs + aux stations via
    # l'allee correspondante (topologie simplifiee en grille)
    for row in range(5):
        for col in range(len(shelf_x)):
            nid = f"shelf_{row}_{col}"
            if col < len(shelf_x) - 1:
                g.add_edge(nid, f"shelf_{row}_{col+1}")
            if row < 4:
                g.add_edge(nid, f"shelf_{row+1}_{col}")

    # The entrance connects to the central aisle; the two staging stations
    # connect through it.  Shelf-row connections model the east-side route
    # around the racks while keeping the high-level graph connected.
    g.add_edge("station_A", "charging_dock")
    g.add_edge("station_B", "charging_dock")
    g.add_edge("charging_dock", "shelf_2_0")

    return g
