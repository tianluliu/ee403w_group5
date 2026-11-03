# AGENTS.md

## Repository purpose
Penn State EE403W Group 5 autonomous LiDAR mapping robot.

Target hardware: Raspberry Pi 4, RPLIDAR C1, Arduino Uno, Cytron MDD10A, two DC gearmotors, 12 V battery. The final robot should explore an unknown flat indoor environment, build a 2D map, avoid stationary obstacles, and stop automatically.

## Read first
Before substantive work, read:
1. `README.md`
2. `docs/CURRENT_STATE.md`
3. `docs/EXPLORATION.md`
4. `docs/RUNBOOK.md`
5. `docs/AGENT_HANDOFF.md`
6. task-specific source/tests
7. `docs/requirements.md` for project requirements
8. `docs/interfaces.md` for ROS/serial interfaces

## Current baseline
Stable branch: `main`.

Milestones:
- `f7d331c` reproducible RPLIDAR setup
- `343aa1c` dynamic 2D simulator
- `4f5d5d9` LiDAR scan visualization
- `37cdab2` ROS safety tests
- `a297e24` simulator physics tests
- `576d80c` SLAM Toolbox simulation mapping
- `d9ce6dc` Nav2 simulation navigation
- `24d7508` frontier exploration algorithms
- `d0dce8b` autonomous frontier exploration

At `d0dce8b`, `./scripts/test.sh` passed 334 unit tests on the Raspberry Pi.

## Implemented
- ROS 2 Jazzy workspace on Raspberry Pi.
- Live RPLIDAR C1 `/scan` operation.
- Dynamic differential-drive simulator.
- Simulated motor path: `MotorBridge -> LoopbackTransport -> MockMotorController`.
- Dynamic `/odom`, TF, and moving `/scan`.
- SLAM Toolbox online mapping.
- Nav2 path planning, obstacle avoidance, and repeated `NavigateToPose`.
- Complex deterministic world.
- Custom frontier-based exploration.
- Frontier clustering, clearance filtering, safe goal placement, information gain, path-cost ranking, candidate persistence, Nav2 path validation, failed-goal suppression, and completion detection.
- Foxglove visualization.

## Not yet implemented/validated on the physical drivetrain
- Real Arduino firmware.
- Real Pi-to-Arduino serial transport.
- Final motor wiring/calibration.
- Final physical dimensions and LiDAR mount transform.
- Trustworthy physical odometry/encoder integration.
- Full hardware launch.
- Full physical autonomous acceptance testing.

Never claim physical validation from simulation.

## Architecture

Simulation:
```text
robot_explorer
 -> Nav2
 -> /cmd_vel
 -> robot_mock
 -> MotorBridge
 -> LoopbackTransport
 -> MockMotorController
 -> Simulator2D
 -> /odom + /scan + TF
 -> SLAM Toolbox
 -> /map
```

Expected physical control path:
```text
robot_explorer
 -> Nav2
 -> /cmd_vel
 -> Pi motor bridge
 -> USB serial
 -> Arduino Uno
 -> PWM/DIR
 -> Cytron MDD10A
 -> motors
```

Expected TF/sensing path:
```text
RPLIDAR C1 -> /scan
odometry -> odom -> base_link
SLAM Toolbox -> map -> odom
static mount -> base_link -> laser
```

Only one publisher may own each TF edge.

## Exploration
Core: `src/lidar_robot/exploration.py`
ROS supervisor: `ros2_ws/src/robot_explorer/robot_explorer/explorer_node.py`

Current ranking:
```text
score = information_gain_cells / (path_distance_m + 0.25)
```

Current defaults:
- clearance 0.18 m
- min cluster 5 cells
- standoff 0.20 m
- information radius 0.50 m
- decision period 1.0 s
- persistence 3 checks
- candidate tolerance 0.20 m
- settle 2.0 s
- failure suppression 20 s
- suppression radius 0.35 m
- completion checks 5
- allowed known-space growth 5 cells/check

State machine:
```text
SEARCHING -> VALIDATING -> NAVIGATING -> SETTLING -> SEARCHING
                                                   -> COMPLETE
```

Known weakness: complex-world exploration works but is slower and more indecisive than desired. Benchmark before tuning.

## Simulator
`robot_mock/worlds.py` contains:
- `build_demo_world()` — original 4x4 m world
- `build_complex_world()` — 8x6 m world

Complex world intentionally contains:
- central obstacle
- sealed unreachable room
- reachable room with 0.8 m doorway
- occluding partial walls
- freestanding wall

`mock_robot_node.py` currently selects `build_complex_world()`.

## Safety
Preserve:
- DISARMED default
- command TTL/watchdog behavior
- stale-command stop
- no silent re-arm after watchdog failure during the same motion session
- mock-config refusal for hardware-dependent behavior
- `RobotConfig.require_complete(scope)`

Do not weaken safety just to make a test pass.

## Tests
Before committing substantive changes:
```bash
cd ~/ee403w_group5
./scripts/test.sh
git diff --check
```

## Git
Keep `main` stable. Use short-lived branches for substantial experiments, validate, merge, delete branch.

Do not commit:
- `build/`, `install/`, `log/`
- maps/bags/logs
- credentials/passwords
- local connection settings
- third-party source trees when a reproducible dependency reference is sufficient

## Demo-only experiments
Handheld LiDAR/RF2O experiments are not production architecture unless the team explicitly adopts them. Keep `~/handheld_lidar_demo` outside this repo by default.

## Agent handoff
After meaningful work, leave:
- branch + commit
- files changed
- behavior changed
- tests/run results
- reproduction commands
- known limitations
- next recommended step

Do not rely on chat memory as the only engineering record.
