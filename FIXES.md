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

## Scaling to 3 and 4 robots (2026-10-02)

`multi_robot.launch.py` already generates spawns, AMCL, Nav2 parameters,
map fusion and the frontier coordinator from `robot_count`. Running more
than two robots exposed two bugs. Some configuration was also still sized
for two robots.

### Frontier coordinator dropped TF for one robot

Symptom: with 4 robots, in 3 of 4 runs one robot (robot3 or robot4) was
never reported as `map -> robotN/base_link TF ready`. It stayed in
`waiting_tf` and never received an exploration goal, although it was
correctly localized (`tf2_echo map robotN/base_link` worked, and restarting
only the coordinator made it ready immediately).

Cause: the coordinator received `/tf` on its own node through a depth-100
queue. One frontier cycle (WFD over the whole map plus one costmap per
robot) blocks that node's executor for seconds. During that time `/tf`
carries about 250 messages/s with 4 robots, so the queue keeps under 0.5 s
of traffic. Low-rate transforms, in particular AMCL's
`map -> robotN/odom`, were almost always discarded. A diagnostic
subscriber inside the coordinator counted only 12 `map -> robot2/odom`
messages in 90 s, the newest being 70 s old. More robots means longer
cycles and more `/tf` traffic, which is why the bug appeared above two
robots.

Fix: the TF listener now runs on a dedicated node
(`frontier_coordinator_tf`) spun in its own thread, with a 1000-message
`/tf` queue. It no longer waits for frontier cycles. (Passing
`spin_thread=True` with the coordinator's own node was tried first and did
not fix it.)

Changed file:

- `src/rosnav/src/rosnav_bot/scripts/frontier_coordinator.py`

### Recovery behaviors used an unprefixed `odom` frame

On Jazzy, the behavior server (spin, backup, drive_on_heading) uses the
`local_frame` parameter, which defaults to `odom`. The template only set
`global_frame`, so recoveries failed with
`"odom" passed to lookupTransform argument target_frame does not exist`.
This also affected two-robot runs. `local_frame: "ROBOT_NS/odom"` was
added.

Changed file:

- `src/rosnav/src/rosnav_bot/config/nav2_multirobot_params_jazzy.yaml`

### Fleet-size configuration

- RViz (`rviz/multi_robot.rviz`) now shows Robot3 and Robot4 models.
- `warehouse_marl.launch.py` defaults `robot_names` to
  `['robot1','robot2','robot3','robot4']`. Pass the list that matches
  `robot_count`.
- The DQN must be trained with `--n-agents` equal to the number of robots.

### One launch file for the whole system

`warehouse_marl/launch/warehouse_full.launch.py` starts everything with
one command: Gazebo (warehouse world), RViz, the N robots (SLAM/AMCL, Nav2,
scan fusion, frontier exploration) and the full `warehouse_marl` layer.
Robot names `robot1..robotN` are generated from `robot_count` and passed to
MARL. MARL starts after map fusion, using the same delay formula as
`multi_robot.launch.py`.

```bash
ros2 launch warehouse_marl warehouse_full.launch.py                 # 3 robots
ros2 launch warehouse_marl warehouse_full.launch.py robot_count:=4
ros2 launch warehouse_marl warehouse_full.launch.py explore:=false  # missions only
ros2 launch warehouse_marl warehouse_full.launch.py marl:=false     # simulation only
ros2 launch warehouse_marl warehouse_full.launch.py headless:=true rviz:=false
```

Other arguments: `slam_mode` (`single` or `multi`), `drl_mode`,
`model_path`, `merge_scans`, `map_origin_x/y`. The launch also starts a
`lifecycle_watchdog` (see below). `warehouse_marl` now declares
`rosnav_bot` as an exec dependency.

### Robots were mislocalized by metres (root cause of failed missions)

Symptom: most missions ended with `Failed to create plan`, even for goals
1-2 m away. The robots' own global costmaps showed them sitting inside
obstacles, and two robots were sometimes 0.07 m apart in the map, which is
physically impossible for 0.5 m robots.

Measurement: each robot's estimated pose was compared with Gazebo's true
pose (`gz model -m robotN -p`). Errors reached 2-9 m within 2-3 minutes.
The IMU heading always matched the truth; the wheel-odometry heading was
off by 100-150 deg. Three faults combined:

1. Wrong wheel geometry in the drive plugin. `gazebo_control.xacro` used
   `wheel_radius 0.1` and `wheel_separation 0.35`, but the robot model has
   radius 0.075 and wheels at y = +/-0.225 (separation 0.45). Wheel
   odometry over-reported distance by 1.33x and rotation by about 1.7x, and
   the robots drove at 75 % of the commanded speed. Fixed to 0.075 / 0.45.
2. The EKF fused wheel x/y. `ekf.yaml` took absolute x/y from wheel
   odometry, which is integrated along the slipping wheel heading. It now
   fuses only the wheel velocities (vx, vy); the heading comes from the
   IMU, so the EKF integrates speed along the correct heading.
3. The Nav2 footprint was too small. Nav2 used `robot_radius: 0.22`, but
   the real body spans x -0.28..0.46 m and y +/-0.25 m (circumscribed
   radius about 0.53 m). Paths were planned for a robot half the real size,
   so the chassis front hit shelves. Stuck robots ran backup/spin
   recoveries with slipping wheels, which corrupted odometry further. The
   costmaps now use the real rectangle as `footprint`, MPPI
   `consider_footprint` is enabled, and `inflation_radius` is 0.70.

Result (3 robots, exploration + MARL, same scenario):

| Run                                      | Missions succeeded / finished |
|------------------------------------------|-------------------------------|
| Before any mission fix                   | 0 / 57 (115 attempts)         |
| Mission executor fix only                | 1 / 8                         |
| + wheel geometry, EKF, watchdog          | 7 / 15                        |
| + real footprint (current)               | 9 / 14                        |

With all fixes, pose errors mostly stayed under 0.3 m (single samples up
to about 1 m include measurement lag, because each `gz model` call takes
about a second while the robot moves). Remaining failures were plans
toward shelves in parts of the warehouse that were not mapped yet.

Changed files:

- `src/rosnav/src/rosnav_bot/urdf/gazebo_control.xacro`
- `src/rosnav/src/rosnav_bot/config/ekf.yaml`
- `src/rosnav/src/rosnav_bot/config/nav2_multirobot_params_jazzy.yaml`
- `src/rosnav/src/rosnav_bot/config/nav2_multirobot_params.yaml` (Humble,
  same footprint change; not run in this environment)

Note: the wheel-geometry and EKF fixes change odometry for every launch
that uses the diff-drive robot, including single-robot ones. That is
intended (they were wrong everywhere), but saved maps or tuned parameters
made with the old odometry may need to be redone.

### Missions could not interrupt exploration goals

Symptom: with `explore:=true` and MARL running, every mission ended with
Nav2 status 6 (aborted). One test produced 115 attempts, 0 successes and
90 `Preemption request was rejected since the requested BT XML file is
not the same as the one that the current goal is executing` warnings.

Cause: exploration goals use the `explore_nav` behavior tree; mission goals
use the default tree. Nav2 accepts a new goal while another is running,
then refuses the preemption because the trees differ, and aborts the new
goal. Pausing exploration (`/warehouse/missions_active`) stops *new*
frontier goals but did not end the one already running.

Fix: `mission_executor_node` watches each robot's
`navigate_to_pose/_action/status`. If the robot is still running another
client's goal, it sends one cancel-all request and waits until Nav2
reports the robot idle before sending the mission. In the final run
there were 0 preemption rejections.

Changed file:

- `src/warehouse_marl/warehouse_marl/mission_executor_node.py`

### Intermittent: one robot's AMCL or Nav2 never finished starting

Symptom: in about 2 of 10 runs, one robot never moved. Its lifecycle
manager logged `Configuring amcl` or `Configuring planner_server` and then
nothing else, although the node itself had finished configuring.

Cause: on Jazzy with Fast DDS, a lifecycle service reply is occasionally
lost during startup. `nav2_lifecycle_manager` has no timeout for that call
and waits forever.

Fix: new `lifecycle_watchdog_node` (started by `warehouse_full.launch.py`).
It watches every lifecycle node in `/robot1 .. /robotN`, plus the shared
map providers `/slam_toolbox` and `/map_server` (in one run the root SLAM
node's activation reply was lost: no map, so no robot could localize and
every Nav2 bringup aborted). If one stays
`unconfigured` or `inactive` for more than 45 s, it configures or activates
the node itself, and retries until the node is active. In a test run it
recovered robot3's stalled Nav2 bringup, and robot3 then navigated
normally.

New file:

- `src/warehouse_marl/warehouse_marl/lifecycle_watchdog_node.py`

### `slam_mode:=multi` never started SLAM on Jazzy

On Jazzy, `slam_toolbox` is a lifecycle node. In `slam_mode:=multi`,
`multi_robot.launch.py` started one per robot as a plain node, so each
stayed `unconfigured`: no map, no `robotN/map -> robotN/odom`, and every
Nav2 bringup aborted. Each robot now gets a `lifecycle_manager_slam` that
configures and activates its `slam_toolbox` (Jazzy and later only;
Humble's slam_toolbox is not a lifecycle node). Multi mode was tested to
bring up all SLAM and Nav2 stacks, but the mission comparison above was
done in the default `single` mode.

Changed file:

- `src/rosnav/src/rosnav_bot/launch/multi_robot.launch.py`

### Spawn positions (warehouse world, `robot_layout:=line`)

| Robot  | Spawn (x, y)   |
|--------|----------------|
| robot1 | (-1.50, -1.00) |
| robot2 | (-0.55, -1.00) |
| robot3 | (0.40, -1.00)  |
| robot4 | (1.60, -1.00)  |
| robot5 | (2.80, -1.00)  |

robot1's spawn is unchanged, so `map_origin_x:=-1.5 map_origin_y:=-1.0`
stays valid for any fleet size.

### Verification

On a clean machine (12 cores):

- 3 robots, full system: see the mission table above. All Nav2 stacks
  became active; all robots were TF-ready and received exploration goals.
- 4 robots: one run (before the localization fixes): all four became
  TF-ready and received goals. Not re-run with the latest fixes.
- 5 robots: not tested.

### How to test it yourself

1. Rebuild and source. Run colcon from the workspace root (the folder
   that contains `src/`), not from inside `src/`:

   ```bash
   cd ~/warehouse/warehouse-ros2
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --packages-select rosnav_bot warehouse_marl
   source install/setup.bash
   ```

   If `colcon` complains about setuptools, use the project virtualenv
   described in "Standard run commands" (`python3 -m colcon ...`).

2. Make sure no previous simulation is still running (leftover Gazebo or
   Nav2 processes publish stale `/tf` and distort the result):

   ```bash
   ps -eo pid,args | grep -E "gz sim|/opt/ros/jazzy/lib/" | grep -v grep
   ```

   If anything is listed, stop it (`pkill -f "gz sim"`, then kill the
   remaining PIDs) before continuing.

3. Start the whole system (use `robot_count:=3`, `4` or `5`):

   ```bash
   ros2 launch warehouse_marl warehouse_full.launch.py robot_count:=3
   ```

   In RViz, the robots should now drive around shelves without touching
   them, and their laser scans should stay aligned with the map walls.

4. After about a minute, check that every robot is up:

   ```bash
   for r in robot1 robot2 robot3; do ros2 lifecycle get /$r/bt_navigator; done
   ```

   All must say `active [3]`. If the watchdog had to finish a stuck
   bringup, the launch output shows
   `lifecycle_watchdog: /robotN/... stuck ... sending transition`.

5. Check localization against the simulator's true position (optional):

   ```bash
   gz model -m robot2 -p                          # true pose (world frame)
   ros2 run tf2_ros tf2_echo map robot2/base_link # estimate (map frame)
   ```

   The map frame is the world shifted by robot1's spawn: world x =
   map x - 1.5, world y = map y - 1.0. Run the two commands right after
   each other; they should agree within a few tens of centimetres.

6. In a second terminal, about 2 minutes after launch, check exploration:

   ```bash
   ros2 topic echo /exploration/stats --once --field data
   ```

   Pass: `"tf_waiting": []`, every robot in `"tf_ready"`, and
   `"succeeded_goals"` increasing over time.

7. Check the missions:

   ```bash
   ros2 topic echo /warehouse/tasks/status --field data
   ```

   Look for `"state": "navigating"` followed by `"succeeded"`. Missions to
   shelves in unexplored areas can still end in `"failed"`; their share
   drops as the map fills in. `Preemption request was rejected` should not
   appear in the launch output.

8. Run the test 2 or 3 times: several of these problems were intermittent.

## RFID layer (2026-10-02)

Each item on the racks has an RFID tag and each robot carries an RFID
reader. Details and topics are in `src/warehouse_marl/README.md`, section
"Couche RFID".

- `rfid_model.py`: the reader and the tags on the rack faces of the
  warehouse world.
- `rfid_sensor_node.py`: each robot reads the nearby tags as it drives.
- `inventory_node.py` (new): combines the reads into the stock of each
  rack and requests scans of racks that need checking.
- `task_generator_node.py`: turns scan requests into `rfid_scan` tasks,
  allocated by consensus like the other tasks.

Consensus fix found during the Gazebo test: a task whose auction closed
with no bid (all robots busy) was deleted, so most tasks were silently
dropped under load (38 picks created, 24 executed; 3 of 4 scan tasks
lost). Unassigned tasks now stay open; the first later bid restarts the
auction window on every agent. Robots bid on one open task at a time,
highest priority first. The task generator stops releasing picks while
`max_open_tasks` (6) are open; RFID scan requests are not limited.

Two more consensus fixes from the same Gazebo tests:

- Shelf capacity: two robots were often sent to the same popular shelf,
  blocked each other at its service point, and neither could plan (6 of
  33 missions succeeded). Agents now do not bid on a task whose shelf is
  already targeted by an active task (capacity from the graph). Next run:
  44 of 51 missions succeeded.
- Conflict resolution: with late bids, all three agents once claimed the
  same task, and the mission executor ran the first claim, not the best.
  Claims now carry their cost; every agent and the mission executor keep
  the best claim (lowest cost, then robot_id) and losers withdraw. The
  executor waits 0.5 s (`claim_window_sec`) before dispatching.

## Web dashboard (2026-10-05)

`warehouse_full.launch.py` now also starts a web dashboard at
http://localhost:8080. Its layout and world-from-SDF map follow the AMR-X
dashboard in `linda-ws`; it has its own colour theme (neutral graphite,
safety-orange accent, wood-coloured racks). It is a ROS 2 node
(`warehouse_marl/dashboard_server.py`) that serves static files from
`warehouse_marl/dashboard/` and streams state with Server-Sent Events, so it
needs no rosbridge, Node.js or database (neither Node.js nor rosbridge is
installed on this machine).

Views: overview (KPIs, map, RFID reads per second, fleet, activity), live
map (Gazebo world, robots, RFID tags lit when read, read beams, racks
coloured by last scan, mission targets; click a rack to see its tags, when
and by which robot they were read, request an RFID scan or create a pick),
missions and consensus, RFID reads (per rack, live read feed).

The dashboard shows RFID scanning only (reads, scanned racks, scan
missions).

Supporting changes: operator scan requests (`source: dashboard`) are always
turned into tasks, while automatic ones still wait until the shelf's service
point is mapped (and are now logged when deferred).

Verified with the full Gazebo system (3 robots): all four views render live
data, the pick and scan buttons create tasks that go through consensus, and
`/../` paths are refused. Screenshots: `src/warehouse_marl/doc/dashboard_*.png`.

## Robot position drift between Gazebo and RViz / dashboard (2026-10-11)

Symptom: a robot was in an aisle in Gazebo while RViz and the dashboard
(both show the `map -> robotN/base_link` estimate) showed it driving into
a wall or rack.

Measurement: a recorder compares, once per second, Gazebo's true pose
(`/world/warehouse/dynamic_pose/info`, with its sim timestamp) with the
estimated pose at the same sim time, plus the true speed, the wheel-odometry
speed and the commanded speed. Typical failure (robot3, sim 151-153 s): true
speed 0 while the wheels reported 0.39 -> 0.14 m/s. The robot had clipped a
rack corner while squeezing through a 0.7 m gap between two rack segments;
the wheels kept turning, odometry kept counting distance that never
happened, and the estimate "drove" on into the rack (map error 1.5-2.3 m).

Causes and fixes:

1. Robots were invisible to each other. The lidar beams are 0.25 m above
   the floor and the robot bodies end at 0.225 m, so each lidar passes over
   the other robots: costmaps did not contain them, robots drove into each
   other and pushed one another (a pushed robot moves without its wheels
   turning; its odometry, and so its pose, stays behind). New
   `scripts/fleet_obstacles.py`, started by `multi_robot.launch.py`,
   publishes for each robot the other robots' footprints as a point cloud
   (`/<robot>/fleet_obstacles`, map frame); it is an extra `fleet`
   observation source in the local and global costmaps.
2. Spawns too close (0.95 m) for the real 0.74 x 0.5 m body (it sweeps a
   0.53 m radius when turning in place): neighbours collided at start-up.
   New `robot_min_gap` argument (default 1.2 m), separate from the obstacle
   clearance, so robot1 stays at (-1.5, -1.0) (the map origin used by MARL
   and the dashboard): robot2 (-0.3, -1.0), robot3 (0.9, -1.0), robot4
   (2.1, -1.0), robot5 (3.55, -1.0).
3. Robots squeezed through the 0.7 m gaps between rack segments (dead-end
   slots between back-to-back racks) and clipped rack corners.
   `footprint_padding: 0.1` in the costmaps makes those gaps impassable;
   main aisles (2-2.5 m) and the separator opening (4 m) are unaffected.
4. No last-line safety stop: the `collision_monitor` was configured but
   never launched (the velocity smoother drove the wheels directly).
   `nav2_navigation_global_tf.launch.py` now runs it between the velocity
   smoother and the wheels (`cmd_vel_smoothed -> collision_monitor ->
   cmd_vel`), on the laser.
5. AMCL odometry noise `alpha1-4` 0.2 -> 0.1: wheel + IMU odometry is
   accurate to a few cm when the robot is not blocked, so AMCL relies on it
   more in feature-poor areas (it drifted 0.5-0.75 m in the empty staging
   zone).
6. `scripts/world_to_map.py` builds a complete map from the world SDF
   (geometry between 0 and 0.45 m; no floor markings, overhead catwalk or
   people), origin at robot1's spawn. `multi_robot.launch.py` uses it when
   `explore:=false` and no saved map exists (before, that case failed).

Tried and reverted (documented so they are not retried blindly):

- AMCL random-particle recovery (`recovery_alpha_slow/fast` 0.001/0.1):
  AMCL jumped 6-12 m to look-alike spots.
- Wider SLAM scan-matching window (`correlation_search_space_dimension`
  0.5 -> 1.2): recovered a simulated push in 9 s, but in normal runs let
  robot1 slide ~1 m along the long separator wall.
- Fleet cloud as a collision-monitor stop source: stopped pushes but
  deadlocked robots that met (missions 71 -> 11 succeeded).
- Stronger AMCL laser model (`max_beams` 60 -> 180, `z_hit` 0.8,
  `z_rand` 0.2): AMCL became overconfident and locked onto a wrong match
  (robot3 3.6 m off, missions 7 succeeded / 70 failed).

Changed files:

- `src/rosnav/src/rosnav_bot/scripts/fleet_obstacles.py` (new)
- `src/rosnav/src/rosnav_bot/scripts/world_to_map.py` (new), `CMakeLists.txt`
- `src/rosnav/src/rosnav_bot/launch/multi_robot.launch.py`
- `src/rosnav/src/rosnav_bot/launch/nav2_navigation_global_tf.launch.py`
- `src/rosnav/src/rosnav_bot/config/nav2_multirobot_params_jazzy.yaml`
- `src/rosnav/src/rosnav_bot/config/nav2_multirobot_params.yaml`
- `src/rosnav/src/rosnav_bot/config/mapper_params_*.yaml` (comment only;
  values back to the originals)

Verification (3 robots, ~9-10 simulated minutes each,
true pose from Gazebo vs estimate at the same sim time):

| Configuration | Max map error robot1 / robot2 / robot3 | Missions ok / failed |
|---|---|---|
| Before these fixes | 0.70 / 2.07 / 2.25 m (first 5 min) | - |
| Kept configuration (fixes 1-5, `explore:=true`) | 0.49 / 2.71 / 1.47 m | 71 / 21 |
| + fleet cloud in collision monitor (reverted) | 0.26 / 0.32 / 1.09 m | 11 / 28 |
| Generated map, `explore:=false` (fix 6) | 2.90 / 2.40 / 0.89 m | 6 / 20 |

Known limitation: the drift is reduced, not solved. In some runs a robot
is still pushed by another or gets stuck, and its estimated pose (RViz,
dashboard) then lags its real pose by 1-3 m until AMCL/SLAM pulls it back.
Results vary noticeably between runs with identical settings; the test
machine ran at a load average of about 40 (Gazebo slower than real time),
which may contribute. Next steps if more accuracy is needed: test with 2
robots / lower CPU load to separate load effects, then revisit
robot-robot avoidance (e.g. a collision-monitor fleet polygon that only
slows down, so it cannot deadlock).

Measurement tools used (not part of the packages): a recorder comparing
`/world/warehouse/dynamic_pose/info` with `map -> robotN/base_link`, plus
true / wheel / commanded speeds, and a controlled "push" test with
`gz service -s /world/warehouse/set_pose`.

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


