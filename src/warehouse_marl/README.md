# warehouse_marl

Package ROS2 (Humble / Jazzy) implementant la couche **coordination
distribuee multi-agents + DRL + RFID/IoT** demandee par le sujet de Master
*"Intelligent Distributed Multi-Agent Coordination for Autonomous Smart
Warehousing in Industry 4.0"* (ARTI, ENIT/INSAT, 2025/2026).

Ce package est concu pour se greffer **par-dessus** le socle de simulation
et de navigation ROS2 [`rosnav`](https://github.com/darshmenon/rosnav)
(Nav2 + SLAM Toolbox + Gazebo Harmonic, monde `warehouse` fourni), qui
gere la couche bas niveau (perception, evitement d'obstacles, deplacement
physique). `warehouse_marl` ajoute la couche haut niveau : graphe
dynamique, RFID, consensus distribue, allocation par DRL.

## Correspondance avec le sujet de Master

| Exigence du sujet | Fichier(s) |
|---|---|
| Entrepot modelise comme un graphe dynamique | `warehouse_marl/warehouse_graph.py` |
| Couche de detection RFID | `warehouse_marl/rfid_sensor_node.py` |
| Couche de communication IoT | topics ROS2 `/rfid/detections`, `/warehouse/graph_state` |
| Protocole de coordination distribue base sur consensus | `warehouse_marl/consensus_node.py` |
| Framework DRL pour l'allocation de taches et le path planning | `warehouse_marl/drl/env.py`, `dqn_agent.py`, `train_drl.py`, `drl_allocator_node.py` |
| Validation par simulation a grande echelle | `launch/warehouse_marl.launch.py` + monde `warehouse` de rosnav |
| Reduction du temps de traitement / taux d'erreur vs heuristique | comparer les modes `heuristique seul` / `hybrid` / `direct` (voir plus bas) |

## Architecture

```
                     ┌────────────────────────┐
                     │   task_generator_node   │  (flux de commandes)
                     └────────────┬────────────┘
                                  │ /warehouse/tasks/new
                                  ▼
   ┌──────────────┐   ┌─────────────────────┐    ┌──────────────────┐
   │ rfid_sensor  │   │ graph_publisher_node │    │ drl_allocator_node│
   │    _node     │──▶│  (graphe dynamique)   │◀──│  (politique DQN)  │
   └──────────────┘   └──────────┬───────────┘    └─────────┬─────────┘
      /rfid/detections           │ /warehouse/graph_state    │ /warehouse/consensus/bid_bias
                                  ▼                            │
                  ┌───────────────────────────┐                │
                  │  consensus_node (x N)      │◀───────────────┘
                  │  un noeud PAR agent robot   │
                  └──────────────┬─────────────┘
                                  │ /warehouse/consensus/bids
                                  │ /warehouse/consensus/assignments
                                  ▼
                  ┌───────────────────────────┐
                  │ mission_executor_node      │
                  │ files + etat des taches   │
                  └──────────────┬────────────┘
                                 │ /robotN/navigate_to_pose
                                 ▼
                                Nav2
```

## Installation

```bash
# Dependances systeme ROS2 (Humble ou Jazzy)
sudo apt install ros-$ROS_DISTRO-rclpy ros-$ROS_DISTRO-std-msgs \
                  ros-$ROS_DISTRO-nav-msgs ros-$ROS_DISTRO-geometry-msgs

# Environnement Python partageant les paquets ROS systeme
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

# Dependances optionnelles pour entrainer/utiliser le DQN
python -m pip install gymnasium torch

# Construction du workspace
cd ~/chaimaws/src/warehouse-ros2
python -m colcon build --symlink-install --packages-select rosnav_bot warehouse_marl
source install/setup.bash
```

## Utilisation

### 1) Entrainer la politique DRL (hors-ligne, pas besoin de ROS2/Gazebo)

```bash
ros2 run warehouse_marl train_drl --episodes 500 --n-agents 3 --out warehouse_dqn.pt
```

Produit `warehouse_dqn.pt` (poids du reseau) et `training_log.csv`
(courbe de convergence reward / cout moyen par episode -> directement
exploitable pour les figures de ton memoire).

### 2) Lancer la simulation (terminal 1, package rosnav)

```bash
ros2 launch rosnav_bot multi_robot.launch.py \
    world:=warehouse robot_count:=2 explore:=true slam_mode:=single \
    merge_scans:=true fleet_mgmt:=false
```

### 3) Lancer la couche de coordination (terminal 2, ce package)

```bash
ros2 launch warehouse_marl warehouse_marl.launch.py \
    robot_names:="['robot1','robot2']" \
    drl_mode:=heuristic \
    map_origin_x:=-1.5 map_origin_y:=-1.0
```

Le mode `heuristic` est le point de depart valide : il ne depend pas d'un
modele appris. Les coordonnees `map_origin_x/y` doivent etre celles du spawn
Gazebo de `robot1`; dans le monde `warehouse`, la validation de collision
deplace le spawn demande `(-2.0, -1.0)` vers le point libre
`(-1.5, -1.0)`, d'ou les valeurs ci-dessus. Le generateur attend que des points de picking
soient reellement libres et connus dans `/map_fused` (`/map` sert de repli), puis les missions suspendent
temporairement les objectifs d'exploration pendant qu'elles utilisent Nav2.

Apres une modification/reconstruction de `rosnav_bot`, relancer une fois la
simulation de base afin que le `frontier_coordinator` charge ce mecanisme de
pause.

### 4) Observer le systeme

```bash
ros2 topic echo /warehouse/tasks/new
ros2 topic echo /warehouse/consensus/assignments
ros2 topic echo /warehouse/tasks/status
ros2 topic echo /rfid/detections
ros2 topic echo /warehouse/graph_state
```

## Modes d'allocation comparables (pour ton evaluation experimentale)

- **Heuristique pure** (`drl_mode:=heuristic`, recommande pour valider la
  simulation) : consensus par plus court chemin + priorite, sans modele.
- **DRL direct** : `drl_mode:=direct` -> allocation centralisee par
  argmax des Q-values (necessite un `model_path` valide).
- **Hybride** : `drl_mode:=hybrid` -> le DRL oriente les encheres, la
  decision finale reste distribuee. Sans modele valide, aucun poids aleatoire
  n'est utilise et le consensus reste heuristique.

Le topic `/warehouse/tasks/status` publie `queued`, `navigating`, `retrying`,
et un etat terminal (`succeeded`, `failed`, `rejected`, `canceled`) avec
`elapsed_sec`. Il permet de mesurer le temps physique creation -> fin, plus
pertinent que le seul temps d'allocation.

## Limites connues / pistes d'extension pour le memoire

- L'environnement DRL (`drl/env.py`) est un environnement **de haut
  niveau** (allocation), pas un controleur bas niveau : le DRL ne pilote
  pas directement les roues (delegue a Nav2/MPPI de rosnav). C'est un
  choix pedagogique raisonnable a justifier/discuter dans ton memoire.
- Le "capteur RFID" est simule par proximite geometrique (pas de plugin
  Gazebo RFID physique) : a mentionner comme hypothese de simulation.
- Le DQN peut etre remplace par un algorithme multi-agent plus avance
  (QMIX, MADDPG, PPO multi-agent) pour aller plus loin dans la partie
  recherche du memoire.
