# Design

Status: current engineering design for the software baseline through `d0dce8b`. Physical drivetrain integration and formal acceptance testing are still pending.

## 1. Scope

The system is a battery-powered differential-drive robot that uses a 2D LiDAR to explore and map an unknown flat indoor environment, avoid stationary obstacles, and stop when exploration is complete.

The repository does **not** implement SLAM or low-level navigation from scratch. It uses SLAM Toolbox and Nav2. The repository does contain a custom frontier-based exploration policy that decides which reachable unexplored region Nav2 should visit next.

## 2. System architecture

Expected physical architecture:

```text
12 V battery
  -> motor power path -> Cytron MDD10A -> left/right motors
  -> regulated 5 V -> Raspberry Pi 4

RPLIDAR C1 -> USB -> Raspberry Pi 4
Raspberry Pi 4 -> USB serial -> Arduino Uno
Arduino Uno -> PWM/DIR -> Cytron MDD10A
```

Responsibilities:

| Unit | Responsibilities |
|---|---|
| Raspberry Pi 4 | ROS 2, LiDAR, SLAM Toolbox, Nav2, exploration, map/status output, future serial bridge |
| Arduino Uno | motor protocol, duty/TTL/link safety, MDD10A PWM/DIR, optional encoder reporting |
| Development computer | editing, Git, documentation, remote Foxglove visualization, optional unit tests |

Exact physical wiring, final dimensions, motor calibration, and physical odometry remain unresolved.

## 3. Current software stack

| Layer | Choice | Current status |
|---|---|---|
| OS | Ubuntu Server 24.04 ARM64 | installed and used on Pi |
| ROS | ROS 2 Jazzy | installed and used |
| LiDAR | Slamtec `sllidar_ros2`, RPLIDAR C1 | live `/scan` demonstrated |
| Simulation | custom deterministic 2D simulator | implemented/exercised |
| SLAM | SLAM Toolbox | integrated/exercised in simulation |
| Navigation | Nav2 | integrated/exercised in simulation |
| Exploration | custom frontier-based explorer | integrated/exercised in simulation |
| Visualization | Foxglove bridge/client | used |
| Physical odometry | pending hardware decision | not integrated |
| Physical motor bridge | protocol/model exists; firmware/real transport pending | not integrated |

## 4. Simulation architecture

```text
robot_explorer
 -> Nav2
 -> /cmd_vel
 -> robot_mock/mock_robot_node
 -> MotorBridge
 -> LoopbackTransport
 -> MockMotorController
 -> wheel duty
 -> Simulator2D
 -> /odom + /scan + TF
 -> SLAM Toolbox
 -> /map
```

The simulator intentionally exercises the same host-side command/safety abstractions intended for hardware.

## 5. TF ownership

Required chain:

```text
map -> odom -> base_link -> laser
```

Simulation with SLAM:

| Edge | Owner |
|---|---|
| `map -> odom` | SLAM Toolbox |
| `odom -> base_link` | `mock_robot_node` |
| `base_link -> laser` | `mock_robot_node` static broadcaster |

`sim.launch.py` intentionally has no SLAM and publishes a simulation-only identity `map -> odom` for visualization.

Physical deployment keeps the same frame names but replaces simulation publishers with measured hardware/odometry sources.

Exactly one publisher must own each TF edge.

## 6. Command path and safety

Intended physical path:

```text
Nav2
 -> velocity smoother
 -> collision monitor
 -> /cmd_vel
 -> Pi bridge
 -> serial protocol v1
 -> Arduino Uno
 -> MDD10A
 -> motors
```

Implemented safety behavior in the core/mock path:
- motors disabled by default in real configuration
- mock MCU boots DISARMED
- explicit ARM before motion
- command TTL
- link timeout
- independent duty cap
- fail-safe STOP
- mock config rejected for non-simulated transport
- hardware-dependent configuration must pass `RobotConfig.require_complete(scope)`
- stale simulated commands stop motion
- no silent re-arm after watchdog failure during the same simulated motion session

Physical firmware must reproduce the serial/watchdog semantics in `interfaces.md`.

A physical kill switch/fuse remains a hardware requirement/open item.

## 7. Configuration

- `config/robot.toml`: real robot values; unknown values stay unset.
- `config/robot.mock.toml`: simulation-only values with `[meta] mock = true`.
- `config/slam_toolbox_sim.yaml`: simulation SLAM.
- `config/nav2_sim.yaml`: simulation Nav2.

Never copy mock dimensions or drive constants into the physical profile.

## 8. Physical odometry strategy

Preferred order:
1. wheel encoders if the installed motors provide them
2. an approved alternative such as laser odometry if encoders are unavailable
3. open-loop duty dead reckoning only as a fallback

The handheld RF2O experiment is demo-only unless explicitly adopted.

## 9. Exploration design

Custom frontier exploration lives in `src/lidar_robot/exploration.py`; ROS orchestration lives in `robot_explorer/explorer_node.py`.

```text
/map
 -> reachable frontier detection
 -> clustering/filtering
 -> safe frontier goal
 -> information-gain/path-distance ranking
 -> persistence
 -> Nav2 path validation
 -> NavigateToPose
 -> settle/repeat
 -> stable completion
```

Current score:

```text
score = information_gain_cells / (path_distance_m + 0.25)
```

See `EXPLORATION.md`.

## 10. Verification status

At the documented software baseline:
- 334 unit tests pass
- dynamic simulation has run on the Pi
- SLAM Toolbox has run with the simulator
- Nav2 has driven repeated goals
- autonomous frontier exploration has run in the complex world
- RPLIDAR C1 has produced live scans
- full physical drivetrain/autonomous validation has **not** been completed

Formal acceptance criteria remain unclaimed until `test_procedures.md` is executed and recorded.

## 11. Current technical risks

- physical odometry source unresolved
- exact motor/encoder details and chassis geometry unresolved
- Arduino firmware and real serial transport missing
- physical power/wiring/kill-switch details need verification
- exploration efficiency needs tuning
- planar LiDAR cannot detect obstacles outside its scan plane
- final CPU/memory/runtime margin must be measured under physical workload

## Observational benchmarking

The explorer optionally writes JSONL through the ROS-independent
`lidar_robot.benchmark` accumulator and subscribes to `/odom` only when enabled.
No additional control or TF publisher is introduced. See `BENCHMARKING.md` for
definitions, provenance requirements and deferred offline coverage analysis.

Focused controller TF diagnosis is separately opt-in and read-only: zero-wait
probes of the explorer's TF buffer plus Nav2 rosout/path observations. It never
gates action requests or changes TF/control settings; see `TF_DIAGNOSTICS.md`
for why observer-side availability must be correlated with controller errors.

Simulator control diagnosis uses bounded in-memory timing windows and MCU
transition events, drained to an opt-in 1 Hz topic. Events are captured inside
the existing watchdog/command operations; no extra protocol polls, threads or
safety decisions are introduced. See `SIM_DIAGNOSTICS.md` for state timing and
map-quality capture limitations.
