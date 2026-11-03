# AGENTS.md

## Repository purpose

Penn State EE403W Group 5 autonomous LiDAR mapping robot.

Target hardware: Raspberry Pi 4, RPLIDAR C1, Arduino Uno, Cytron MDD10A, two DC gearmotors, and a 12 V battery system. The final robot should explore an unknown flat indoor environment, build a 2D map, avoid stationary obstacles, and stop automatically.

## Read first

Before substantive work, read:
1. `README.md`
2. `docs/CURRENT_STATE.md`
3. `docs/design.md`
4. `docs/interfaces.md`
5. `docs/EXPLORATION.md`
6. `docs/RUNBOOK.md`
7. `docs/tasks.md`
8. task-specific source and tests
9. `docs/requirements.md` when changing scope or acceptance behavior

## Source-of-truth order

For present implementation behavior:
1. current code and tests
2. `docs/CURRENT_STATE.md`
3. focused current docs (`design.md`, `interfaces.md`, `EXPLORATION.md`, `RUNBOOK.md`)
4. `README.md`

For project requirements, `docs/requirements.md` and the PDR-derived requirements remain authoritative unless the team explicitly changes them.

## Current software baseline

Stable branch: `main`.

Important milestones:
- `f7d331c` reproducible RPLIDAR dependency setup
- `343aa1c` dynamic 2D simulator
- `4f5d5d9` LiDAR scan visualization
- `37cdab2` ROS safety tests
- `a297e24` simulator physics tests
- `576d80c` SLAM Toolbox simulation mapping
- `d9ce6dc` Nav2 simulation navigation
- `24d7508` frontier exploration algorithms
- `d0dce8b` autonomous frontier exploration

At the documented software baseline, `./scripts/test.sh` passed 334 unit tests on the Raspberry Pi.

## Implemented

- ROS 2 Jazzy on Raspberry Pi.
- Live RPLIDAR C1 `/scan`.
- Dynamic differential-drive simulator.
- Simulated motor path through `MotorBridge -> LoopbackTransport -> MockMotorController`.
- Dynamic `/odom`, TF, and moving `/scan`.
- SLAM Toolbox online mapping.
- Nav2 path planning, obstacle avoidance, and repeated `NavigateToPose`.
- Complex deterministic exploration world.
- Custom frontier-based exploration.
- Frontier clustering, clearance filtering, safe goal placement, information gain, path-cost ranking, candidate persistence, Nav2 path validation, failed-goal suppression, and completion detection.
- Foxglove visualization.

## Not yet implemented/validated on the physical drivetrain

- Real Arduino firmware.
- Real Pi-to-Arduino serial transport.
- Final motor wiring/calibration.
- Final physical dimensions and LiDAR mount transform.
- Trustworthy physical odometry/encoder integration.
- Full physical launch profile.
- Full autonomous hardware acceptance testing.

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

Expected physical command path:

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

Known weakness: the complex-world explorer works but is slower and more indecisive than desired. Add benchmarks before tuning.

## Simulator

`robot_mock/worlds.py` contains:
- `build_demo_world()` — original 4 m x 4 m world
- `build_complex_world()` — 8 m x 6 m world

The complex world intentionally contains:
- central obstacle
- sealed unreachable room
- reachable room with 0.8 m doorway
- occluding partial walls
- freestanding wall

`mock_robot_node.py` currently uses `build_complex_world()`.

## Safety

Preserve:
- DISARMED default
- command TTL and link watchdog
- stale-command stop
- no silent re-arm after watchdog failure during the same simulated motion session
- mock-config refusal for hardware-dependent behavior
- `RobotConfig.require_complete(scope)`
- bounded speed/duty behavior

Do not weaken safety to make a test pass.

## Tests

Before committing substantive changes:

```bash
cd ~/ee403w_group5
./scripts/test.sh
git diff --check
```

Do not equate unit-test success with physical acceptance.

## Git

Keep `main` stable. Use short-lived branches for substantial experiments, validate, merge, then delete the branch.

Do not commit:
- ROS build output
- generated maps/bags/logs
- credentials/passwords
- local connection settings
- third-party source trees when a reproducible dependency reference is sufficient

## Demo-only experiments

Handheld LiDAR/RF2O experiments are not production architecture unless the team explicitly adopts them. Keep `~/handheld_lidar_demo` outside this repo by default.

## Documentation discipline

When architecture, interfaces, exploration behavior, verified status, or run commands change:
1. update code
2. update/add tests
3. update the relevant focused document
4. update `CURRENT_STATE.md` if project status changed
5. update README if the user-facing overview changed
6. report exactly what was tested

## Agent handoff

After meaningful work, leave:
- branch + commit
- files changed
- behavior changed
- tests/runtime checks
- reproduction commands
- known limitations
- next recommended step

Do not rely on chat memory as the only engineering record.
