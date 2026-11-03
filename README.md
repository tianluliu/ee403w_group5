# Autonomous LiDAR Mapping Robot

Penn State EE403W Group 5 software for a battery-powered differential-drive robot that autonomously explores an unknown flat indoor environment, builds a 2D LiDAR map, avoids stationary obstacles, and stops when exploration is complete.

Target hardware:
- Raspberry Pi 4 (4 GB)
- RPLIDAR C1
- Arduino Uno
- Cytron MDD10A
- two 131:1 DC gearmotors
- 12 V battery system

## Current status

Implemented and exercised on the Raspberry Pi:
- dynamic differential-drive simulator
- simulated Arduino safety/control path
- moving `/scan` and `/odom`
- live RPLIDAR C1 `/scan`
- SLAM Toolbox online mapping
- Nav2 navigation and obstacle avoidance
- custom frontier-based exploration
- complex exploration world
- autonomous Nav2 goal validation/execution
- conservative completion detection
- Foxglove visualization
- 334 passing unit tests at the documented software baseline

The physical drivetrain is **not yet integrated**. Arduino firmware, real serial transport, calibrated drive/geometry values, physical odometry, and final hardware acceptance testing remain.

See [`docs/CURRENT_STATE.md`](docs/CURRENT_STATE.md) for exact implementation status.

## Agent / contributor start here

AI coding agents and contributors should read [`AGENTS.md`](AGENTS.md) first.

| File | Purpose |
|---|---|
| [`AGENTS.md`](AGENTS.md) | Durable repository instructions and safety/workflow rules |
| [`docs/CURRENT_STATE.md`](docs/CURRENT_STATE.md) | Current implemented vs. pending status |
| [`docs/design.md`](docs/design.md) | Current architecture and design decisions |
| [`docs/interfaces.md`](docs/interfaces.md) | Current ROS, TF, simulation, serial, and configuration interfaces |
| [`docs/EXPLORATION.md`](docs/EXPLORATION.md) | Frontier algorithm and state machine |
| [`docs/RUNBOOK.md`](docs/RUNBOOK.md) | Build/run/Foxglove/test commands |
| [`docs/tasks.md`](docs/tasks.md) | Current backlog and next steps |
| [`docs/open_questions.md`](docs/open_questions.md) | Only unresolved hardware/requirements questions |
| [`docs/requirements.md`](docs/requirements.md) | PDR requirements and proposed acceptance definitions |
| [`docs/test_procedures.md`](docs/test_procedures.md) | Formal acceptance procedures |
| [`docs/dev_workflow.md`](docs/dev_workflow.md) | Development/Git/Pi workflow |
| [`docs/AGENT_HANDOFF.md`](docs/AGENT_HANDOFF.md) | Cross-agent handoff format |
| [`docs/future_3dgs.md`](docs/future_3dgs.md) | Out-of-scope future 3DGS note |

## Autonomous simulation architecture

```text
custom frontier explorer
        |
        v
      Nav2
        |
     /cmd_vel
        |
        v
dynamic robot simulator
        |
        +----> /odom + TF
        |
        +----> /scan
                  |
                  v
             SLAM Toolbox
                  |
                 /map
                  |
                  +----> explorer
```

The simulated motor path deliberately exercises the project's safety abstractions:

```text
/cmd_vel
 -> MotorBridge
 -> LoopbackTransport
 -> MockMotorController
 -> wheel duty
 -> Simulator2D
```

## Exploration algorithm

Pure algorithm:

```text
src/lidar_robot/exploration.py
```

ROS supervisor:

```text
ros2_ws/src/robot_explorer/robot_explorer/explorer_node.py
```

Current ranking:

```text
score = information_gain_cells / (path_distance_m + 0.25)
```

See [`docs/EXPLORATION.md`](docs/EXPLORATION.md).

## Run the full simulation

Terminal 1:

```bash
cd ~/ee403w_group5
source ~/.bashrc
source ros2_ws/install/setup.bash
ros2 launch robot_bringup sim_nav.launch.py
```

Terminal 2:

```bash
cd ~/ee403w_group5
source ~/.bashrc
source ros2_ws/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Terminal 3:

```bash
cd ~/ee403w_group5
source ~/.bashrc
source ros2_ws/install/setup.bash
ros2 run robot_explorer explorer_node --ros-args -p autonomous:=true
```

In Foxglove use fixed frame `map`.

Useful topics:

```text
/map
/scan
/sim/world
/sim/path
/exploration/markers
/exploration/goal_preview
```

Full instructions: [`docs/RUNBOOK.md`](docs/RUNBOOK.md).

## Tests

```bash
cd ~/ee403w_group5
./scripts/test.sh
git diff --check
```

Documented software baseline:

```text
Ran 334 tests
OK
```

## Current next steps

Software:
1. benchmark the complex-world explorer
2. improve exploration speed and decisiveness
3. compare tuning against objective metrics

Physical integration:
1. confirm exact drivetrain/encoder details
2. establish physical odometry
3. implement Arduino firmware
4. implement real serial transport
5. calibrate geometry and drive constants
6. integrate and validate the physical robot safely

## Demo-only experiments

Handheld LiDAR/RF2O experiments are separate from the production capstone architecture unless the team explicitly adopts them.
