# Nav2, Multi-Robot and MARL Integration Fixes

Date: 2026-09-15  
Validated platform: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic

This document records the changes made to turn the warehouse simulation,
multi-robot Nav2 stack and `warehouse_marl` package into one executable
end-to-end system.

## Result

The working data flow is:

```text
Gazebo robots and sensors
        |
        v
SLAM + merged scans + /map_fused
        |
        v
Nav2 navigation <--- mission_executor_node
                           ^
                           |
task generator -> heuristic consensus and/or DQN allocator
```

The following checks were completed successfully:

- `rosnav_bot` and `warehouse_marl` build with `colcon`.
- The three `warehouse_graph` tests pass with zero failures.
- Both `/robot1/navigate_to_pose` and `/robot2/navigate_to_pose` action
  servers become available.
- Both Nav2 stacks reach the active lifecycle state.
- The fused map is received by Nav2 and the MARL task generator.
- Generated tasks are assigned and converted into real Nav2 goals.
- Robot 2 completed multiple generated shelf missions during the live test.

Some distant missions can still fail while SLAM has only mapped part of the
warehouse. This is expected until the destination and a connecting path are
known.

## Navigation and simulation fixes

### Consistent simulation time

All Nav2, exploration, map fusion and MARL nodes now use Gazebo simulation
time. This prevents nodes from comparing wall-clock timestamps with `/clock`
timestamps.

Changed files:

- `src/rosnav/src/rosnav_bot/launch/multi_robot.launch.py`
- `src/rosnav/src/rosnav_bot/launch/nav2_navigation_global_tf.launch.py`

### RViz pose fluctuation and AMCL resets

The delayed command-line initial-pose publisher was removed. It previously
published the same initial pose repeatedly, causing AMCL resets and visible
robot jumps in RViz.

Initial poses are now published by the normal launch/localization flow only.

### Shared map for two robots

For `slam_mode:=single` and multiple robots:

- scan merging is enabled by default;
- `/map_fused` is used by the Nav2 global and local costmaps;
- the fused-map publisher uses transient-local durability, so nodes started
  later immediately receive the most recent map;
- one-robot operation continues to use `/map`.

Changed files:

- `src/rosnav/src/rosnav_bot/launch/multi_robot.launch.py`
- `src/rosnav/src/rosnav_bot/scripts/map_fusion.py`

### Startup ordering

Map fusion starts after the heavy Gazebo/Nav2 initialization, and frontier
exploration starts after map fusion. This reduces CPU contention and prevents
AMCL/Nav2 lifecycle configuration timeouts during startup.

Current two-robot startup timing is approximately:

- map fusion: 31 seconds;
- frontier exploration: 32 seconds.

### Costmap filter warnings

Keepout and speed-filter plugins are removed from the active costmap plugin
lists when no keepout-mask server is launched. This stops repeated
`Filter mask was not received` warnings without hiding a configured filter.

If keepout zones are required later, their mask and information servers must
be launched and the plugins can remain enabled.

### MPPI footprint configuration

`consider_footprint` was disabled for the MPPI cost critic because the robots
are configured with `robot_radius` rather than a polygon footprint.

Changed files:

- `src/rosnav/src/rosnav_bot/config/nav2_multirobot_params.yaml`
- `src/rosnav/src/rosnav_bot/config/nav2_multirobot_params_jazzy.yaml`

## MARL and task-execution fixes

### New mission executor

The mission executor was not an existing node that was modified. It is a new
node that was added because an essential connection was missing from the
original project.

Before this addition, the system stopped after choosing a robot:

```text
task generated -> consensus/DQN selects a robot -> assignment published
                                                        |
                                                        v
                                                no navigation command
```

The logs could therefore show valid bids, a consensus winner and a published
assignment while both robots remained stationary. Publishing an assignment is
only a high-level decision; Nav2 does not automatically understand a warehouse
task message.

The new execution flow is:

```text
task generated
      |
      v
consensus or DQN selects Robot N
      |
      v
mission_executor_node receives the assignment
      |
      v
sends NavigateToPose to /robotN/navigate_to_pose
      |
      v
Nav2 plans the path, avoids obstacles and moves the robot
      |
      v
mission_executor_node reports succeeded or failed
```

This separation is intentional:

- the heuristic consensus or DQN answers **which robot should do the task**;
- the mission executor answers **how to start and supervise that task**;
- Nav2 answers **how to physically reach the destination**.

Without the mission executor, the MARL layer is only an allocation
demonstration. With it, the allocation becomes an executable warehouse
mission and can be measured from task creation through navigation completion.

It now:

- subscribes to consensus and direct-DRL assignments;
- maintains a mission queue for every robot;
- sends goals to `/robotN/navigate_to_pose`;
- retries navigation when appropriate;
- publishes task states including `queued`, `navigating`, `retrying`,
  `succeeded`, `failed`, `rejected` and `canceled`;
- publishes task elapsed time for experimental evaluation.

The node also prevents two independent goal sources from fighting over the
same robot. When `explore:=true`, the frontier coordinator already sends Nav2
goals so the robot can build the map. A MARL assignment sends a second kind of
goal. Without arbitration, a frontier goal can cancel or replace the warehouse
mission, or the mission can interrupt exploration repeatedly. The mission
executor publishes `/warehouse/missions_active`, allowing the frontier
coordinator to pause during an active warehouse task and resume afterward.

New file:

- `src/warehouse_marl/warehouse_marl/mission_executor_node.py`

### Exploration and mission arbitration

Exploration and warehouse missions previously competed for the same Nav2
action server. The mission executor now publishes
`/warehouse/missions_active`. The frontier coordinator pauses exploration
goals while a warehouse mission is active and resumes afterward.

Important behavior:

- `explore:=true` means robots are allowed to move without MARL to map
  unexplored frontiers;
- `explore:=false` means robot movement comes only from manual or MARL Nav2
  goals;
- with exploration enabled, robots can resume exploring between MARL tasks.

Changed file:

- `src/rosnav/src/rosnav_bot/scripts/frontier_coordinator.py`

### Warehouse graph

The graph was rebuilt with safe aisle/service coordinates matching the
warehouse world. Its origin is configurable and defaults to the validated
Robot 1 spawn at `(-1.5, -1.0)`.

The graph implementation now uses the Python standard library instead of a
runtime NetworkX dependency. It contains its own shortest-path calculation.

Changed file:

- `src/warehouse_marl/warehouse_marl/warehouse_graph.py`

### Correct global robot positions

The graph publisher, RFID simulator and consensus agents now obtain robot
positions using the global `map -> robotN/base_link` transform instead of
interpreting local odometry coordinates as global coordinates.

Changed files:

- `src/warehouse_marl/warehouse_marl/graph_publisher_node.py`
- `src/warehouse_marl/warehouse_marl/rfid_sensor_node.py`
- `src/warehouse_marl/warehouse_marl/consensus_node.py`

### Valid task generation

The task generator now:

- uses `/map_fused` as its primary map and `/map` as fallback;
- waits for a target to be known and free before publishing it;
- uses clearance compatible with the configured robot radius;
- destroys completed one-shot timers instead of accumulating timers.

Changed file:

- `src/warehouse_marl/warehouse_marl/task_generator_node.py`

### Consensus bookkeeping

Consensus uses the dynamic graph and simulation time. Bidding is delayed
briefly so hybrid DQN bid information can arrive. Robot capacity and task
state are released when a task reaches a terminal state.

The shortest-path/load/priority heuristic existed in the original
`consensus_node`. The changes made it an explicit, complete and executable
`drl_mode:=heuristic` pipeline.

### Safe DRL behavior

The DQN allocator no longer allows untrained random weights to control task
allocation when a model file is missing:

- `hybrid` mode falls back to heuristic consensus;
- `direct` mode stops with a clear missing-model error;
- robot load is decremented after terminal task states.

Changed file:

- `src/warehouse_marl/warehouse_marl/drl_allocator_node.py`

### Launch modes

`warehouse_marl.launch.py` now supports:

- `heuristic`: distributed shortest-path/load/priority consensus, no model;
- `hybrid`: distributed consensus influenced by DQN bid scores;
- `direct`: centralized assignment from DQN output.

It launches the graph publisher, RFID simulator, task generator, mission
executor, the required consensus agents, and the optional DQN allocator.

Changed file:

- `src/warehouse_marl/launch/warehouse_marl.launch.py`

## What the learned component does

The current DQN is a high-level task allocator. Its observation includes
estimated path cost, robot load, robot index and task priority. Its action is
the selected robot.

It does not directly control robot velocity or replace Nav2. Nav2 remains
responsible for path planning, local obstacle avoidance and wheel commands.
The system should therefore be described as distributed multi-agent
coordination plus DRL task allocation, not as a learned motion controller.

The number used for `train_drl --n-agents` must equal the number of robots
passed in `robot_names` at runtime.

## Packaging and tests

The ROS dependencies and console entry points were updated to include Nav2
actions, TF, task status handling and the mission executor. Pytest selection
was also updated for current `setuptools` behavior.

Changed files:

- `src/warehouse_marl/package.xml`
- `src/warehouse_marl/setup.py`
- `src/warehouse_marl/setup.cfg`
- `src/warehouse_marl/README.md`

New tests:

- `src/warehouse_marl/test/test_warehouse_graph.py`

Local build environments must use `setuptools<80` because the installed
`colcon-core` requires it. Virtual environments, `build/`, `install/`, `log/`,
Python caches and editor settings are excluded by `.gitignore`.

## Standard run commands

Build:

```bash
cd ~/chaimaws/src/warehouse-ros2
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python -m pip install "setuptools<80"
python -m colcon build --symlink-install \
  --packages-select rosnav_bot warehouse_marl
source install/setup.bash
```

Simulation with exploration:

```bash
ros2 launch rosnav_bot multi_robot.launch.py \
  world:=warehouse robot_count:=2 explore:=true slam_mode:=single \
  merge_scans:=true fleet_mgmt:=false
```

Simulation where robots wait for MARL/manual goals:

```bash
ros2 launch rosnav_bot multi_robot.launch.py \
  world:=warehouse robot_count:=2 explore:=false slam_mode:=single \
  merge_scans:=true fleet_mgmt:=false
```

Heuristic coordination:

```bash
ros2 launch warehouse_marl warehouse_marl.launch.py \
  robot_names:="['robot1','robot2']" drl_mode:=heuristic
```

Train a two-robot DQN:

```bash
ros2 run warehouse_marl train_drl \
  --episodes 500 --n-agents 2 \
  --out warehouse_dqn_2robots.pt \
  --log-csv training_2robots.csv
```

Hybrid learned coordination:

```bash
ros2 launch warehouse_marl warehouse_marl.launch.py \
  robot_names:="['robot1','robot2']" drl_mode:=hybrid \
  model_path:="$PWD/warehouse_dqn_2robots.pt"
```

Task-status monitoring:

```bash
ros2 topic echo /warehouse/tasks/status
```

## Known limitations

- With `explore:=false` and no saved map, SLAM initially knows only the area
  visible from the spawn positions. The task generator may wait for a known
  reachable target.
- The RFID layer is geometric proximity simulation, not a Gazebo radio plugin.
- Completing a task currently means reaching a service point. No manipulator
  or physical box pickup is implemented.
- The DQN is one high-level allocator, not a full independent-policy algorithm
  such as QMIX, MADDPG or multi-agent PPO.
- A trained `.pt` model is not currently stored in the repository.
- The root `README.md` still describes Humble as the project status, while the
  configuration documented and validated here is Jazzy.


