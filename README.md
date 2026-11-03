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
- SLAM Toolbox online mapping
- Nav2 navigation/obstacle avoidance
- custom frontier-based exploration
- complex exploration world
- autonomous goal validation/execution
- conservative completion detection
- Foxglove visualization
- 334 passing unit tests at the documented baseline

The physical drivetrain is **not yet integrated**. Arduino firmware, real serial transport, calibrated drive/geometry values, physical odometry, and final hardware acceptance testing remain.

See [`docs/CURRENT_STATE.md`](docs/CURRENT_STATE.md).

## Agent / contributor start here
AI coding agents and contributors should read [`AGENTS.md`](AGENTS.md) first.

| File | Purpose |
|---|---|
| [`AGENTS.md`](AGENTS.md) | Durable repository instructions |
| [`docs/CURRENT_STATE.md`](docs/CURRENT_STATE.md) | Current implemented/pending status |
| [`docs/EXPLORATION.md`](docs/EXPLORATION.md) | Frontier algorithm and state machine |
| [`docs/RUNBOOK.md`](docs/RUNBOOK.md) | Build/run/Foxglove/test commands |
| [`docs/AGENT_HANDOFF.md`](docs/AGENT_HANDOFF.md) | Cross-agent handoff format |
| [`docs/requirements.md`](docs/requirements.md) | Project requirements/acceptance definitions |
| [`docs/interfaces.md`](docs/interfaces.md) | ROS and serial interfaces |

Some older Phase 1 documents contain historical status statements. Use `docs/CURRENT_STATE.md` plus current code for present engineering status.

## Architecture
```text
frontier explorer
 -> Nav2
 -> /cmd_vel
 -> dynamic simulator
 -> /odom + /scan + TF
 -> SLAM Toolbox
 -> /map
 -> explorer
```

Simulated motor path:
```text
/cmd_vel
 -> MotorBridge
 -> LoopbackTransport
 -> MockMotorController
 -> wheel duty
 -> Simulator2D
```

## Exploration
Core:
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

## Run full simulation

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

Foxglove fixed frame: `map`.

Useful topics:
```text
/map
/scan
/sim/world
/sim/path
/exploration/markers
/exploration/goal_preview
```

## Tests
```bash
cd ~/ee403w_group5
./scripts/test.sh
git diff --check
```

Documented baseline:
```text
Ran 334 tests
OK
```

## Current next steps
Software:
1. benchmark complex-world exploration
2. improve speed/decisiveness
3. compare tuning against objective metrics

Hardware:
1. confirm drivetrain/encoder details
2. establish physical odometry
3. implement Arduino firmware
4. implement real serial transport
5. calibrate geometry/drive constants
6. integrate and validate physical robot

## Demo-only experiments
Handheld LiDAR/RF2O experiments are separate from the production architecture unless intentionally adopted.
