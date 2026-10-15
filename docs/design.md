# Design

Status: DRAFT for team confirmation. Phase 1. Sections marked **(exists)** are implemented and
unit-tested on the Mac; everything marked **(planned)** has not been built or run.

## 1. Scope

This design realizes the PDR (see requirements.md) without changing it. It adds the one thing the
PDR block diagram leaves out, the Arduino, as an explicit proposal for team confirmation (PDR-1).

## 2. Proposed system architecture (planned, needs confirmation: HQ-01)

PDR slide 6 draws Pi -> MDD10A directly. Slides 3 and 9 name the Arduino Uno as the motor
controller. Proposed wiring of responsibilities:

```
 Power
   12 V battery ──► [fuse + kill switch]* ──► MDD10A ──► left motor
        │                                        └─────► right motor
        └──► 5 V regulator ──► Raspberry Pi 4 (4 GB)

 Signals
   RPLIDAR C1 ──USB──► Raspberry Pi 4 ──USB serial (protocol v1)──► Arduino Uno
   Arduino Uno ──PWM + DIR (logic level), common GND──► MDD10A

 * not in the PDR; recommended (HQ-08)
```

The Pi never drives the MDD10A directly and never powers motors. The Uno's own supply is an open
question (HQ-06).

## 3. Responsibilities

| Unit | Does | Does not |
|---|---|---|
| Raspberry Pi 4 | LiDAR acquisition, ROS 2 graph, SLAM, Nav2, exploration supervision, map saving, experiment logging, serial bridge to the Arduino | drive motor power, run RViz/Gazebo or a desktop |
| Arduino Uno | Parse protocol v1, enforce duty cap, command TTL and link timeout, drive MDD10A PWM/DIR, encoder counting only if encoders exist | SLAM, navigation, any decision about where to go |
| MacBook Air | Editing, Git, documentation, platform-independent unit tests, SSH deployment, log and map inspection, remote visualization client | run the robot stack; ROS 2 is not assumed on macOS |

## 4. Software stack (planned; each item to be verified in its phase)

| Layer | Choice | Status |
|---|---|---|
| OS | Ubuntu 24.04 Server ARM64 on the Pi | preferred, unverified on this unit (HQ-11) |
| ROS | ROS 2 Jazzy | preferred, unverified on this unit |
| LiDAR driver | `sllidar_ros2` (Slamtec); its README lists the C1 and ships a C1 launch file | Jazzy build unverified (HQ-13) |
| SLAM | SLAM Toolbox, online asynchronous mode, 0.05 m resolution | not run |
| Navigation | Nav2, with its collision monitor in front of `/cmd_vel` | not run |
| Exploration | an existing frontier package (candidate `explore_lite`) plus a thin supervisor for the start trigger, termination, timeouts, map save and logs | Jazzy build unverified (HQ-15) |
| Odometry | see section 7 | undecided (HQ-02) |

No SLAM, mapping or navigation algorithm is written in this repository. The Pi 4 with 4 GB is
a tight fit for SLAM plus Nav2; CPU and memory headroom must be measured in Phase 3 and 4.

## 5. Command path and safety layers

```
Nav2 planner/controller ─► velocity smoother ─► collision monitor ─► /cmd_vel
   ─► bridge node ─► [serial, protocol v1] ─► Arduino ─► MDD10A ─► motors
```

| Layer | Defends against | Status |
|---|---|---|
| Hardware kill switch + fuse | hung software, stuck motor, short | proposed, not in PDR (HQ-08) |
| Arduino link timeout | Pi crash or hang, USB cable unplugged | modelled and tested in the mock; firmware not written |
| Arduino command TTL | stalled bridge, stale command | modelled and tested in the mock |
| Arduino duty cap, range checks | host bug, corrupt value | modelled and tested in the mock |
| Boot DISARMED, ARM required | accidental motion at reset | modelled and tested in the mock |
| Host `motors_enabled` gate | running tests against hardware by accident | **(exists)** `MotorBridge` refuses `arm`/`send_twist` before writing any byte; default `false` |
| Host fail-safe STOP | a bad value or lost reply on the drive path | **(exists)** `MotorBridge.send_twist` sends STOP, then re-raises |
| Mock-config refusal | invented dimensions driving a real robot | **(exists)** `MotorBridge` rejects a mock config on a non-simulated transport |
| Kinematics limits and duty cap | commanding more than the bring-up envelope | **(exists)** `kinematics.twist_to_drive` |
| Config completeness check | firmware/launch from guessed numbers | **(exists)** `RobotConfig.require_complete` |
| Collision monitor (scan-based) | obstacles that Nav2 misjudges | planned (Phase 4) |

Fail-safe rule used everywhere: if a value is missing, non-finite, out of range or stale, the
answer is "stop", never "use a default".

## 6. Configuration design (exists)

- `config/robot.toml` is the single source of hardware values. Unknown values are omitted.
- `config/robot.mock.toml` has invented values and `[meta] mock = true`. `require_complete` refuses
  it for every hardware-dependent scope; the library never loads it by name.
- Scopes: `sim`, `navigation`, `odometry`, `firmware`. `encoders.present` must be `yes` or `no`
  (not `unknown`) before `firmware` or `odometry` scopes pass.
- TOML is read with the standard library (`tomllib`, Python 3.11+), so the same file works on the
  Mac (3.14 here) and the Pi (3.12 on Ubuntu 24.04). ROS parameter YAML will be generated from it
  in a later task and is not hand-maintained in parallel.

## 7. Odometry strategy (decision pending: HQ-02)

| Option | When | Trade-off |
|---|---|---|
| A. Wheel encoders in the bridge node (diff-drive odometry) | motors have encoders, or the team approves adding them | best short-term accuracy; needs Arduino counting and a protocol extension |
| B. Laser odometry from `/scan` (candidate: an existing ROS 2 laser-odometry package) | no encoders | no extra hardware; quality depends on scene structure; ROS 2 Jazzy availability unverified |
| C. Open-loop dead reckoning from commanded duty | fallback only | unreliable under load and slip; if used at all, publish with large covariance |

The software must work with no encoders. The mock MCU advertises encoders through the HELLO
`caps` byte (`0x01`); the real answer comes from hardware inspection.

## 8. Mac to Raspberry Pi workflow (scripts planned; commands below work today)

1. Edit and test on the Mac. No ROS, Docker or extra packages are needed:
   `python3 -m unittest discover -s tests/unit -t tests/unit`
2. Sync the repository to the Pi over SSH with `rsync` (planned `scripts/deploy.sh`, dry-run by
   default).
3. Run the same unit tests on the Pi to catch Ubuntu/ARM64 differences, then ROS 2 integration tests
   there (planned `scripts/remote.sh`).
4. Visualize from the Mac without installing ROS: a WebSocket bridge on the Pi (candidate:
   `foxglove_bridge`) viewed in a Mac app through an SSH tunnel. Unverified; decided in Phase 3.
5. Docker is **not** used: nothing in Phase 1 needs it, and ROS 2 integration is tested on the real
   target (the Pi) instead.

Nothing in this workflow touches hardware until a task explicitly says so and is approved.

## 9. Repository layout

Exists:

```
config/                  robot.toml, robot.mock.toml
src/lidar_robot/         clock, config, protocol, kinematics, transport, bridge, mock_mcu,
                         world, mock_lidar, frames, metrics            (stdlib only)
tests/unit/              platform-independent tests (unittest)
tests/ros2_integration/  ROS 2 tests: UNVERIFIED, skipped without rclpy
ros2_ws/src/             ROS 2 Jazzy skeletons, never built or run (below)
scripts/                 test.sh, check_env.sh, deploy.sh, remote.sh
docs/                    this folder
```

Not created: `firmware/` (specification only until approved).

| ROS package | State | Role |
|---|---|---|
| `robot_base_bridge` | skeleton, heartbeat node only | will host the `/cmd_vel` to serial bridge node in Phase 2 |
| `robot_mock` | skeleton, mock `/scan` node (mock configs only) | exercise the ROS graph without hardware |
| `robot_bringup` | `heartbeat.launch.py`, `mock.launch.py`; no hardware launch file | launch files; the hardware profile arrives in Phase 2 |
| `robot_description` | planned | static `base_link -> laser` from `robot.toml` |
| `robot_exploration` | planned | start trigger, termination criterion, timeouts, map save, logging |

The ROS nodes are thin by design. They contain no motor-command path (a unit test checks this
statically); the logic they will call lives in `lidar_robot`, which is tested on the Mac.

## 10. Test strategy

| Level | Where | Contents | Status |
|---|---|---|---|
| Platform-independent unit | Mac and Pi | protocol, config, kinematics, mock MCU, bridge, world, mock LiDAR, TF validator, coverage metric, static ROS package structure, script safety | 307 tests, passing on the Mac; not yet run on the Pi |
| ROS 2 integration | Pi only | mock-LaserScan publisher, heartbeat, refusal of real config | written, **UNVERIFIED** (never run; skipped on the Mac) |
| Hardware, wheels elevated | Pi + robot | bridge, watchdog stop, LiDAR scan check | Phase 2, needs approval |
| System | Pi + robot | coverage, obstacle, termination, runtime | Phases 3-6 |

## 11. Risks

- Hardware questions HQ-01 to HQ-08 gate Phase 2. The Gantt shows CDR on 3-Nov.
- No encoders would make odometry the main technical risk.
- Pi 4 (4 GB) CPU and memory under SLAM Toolbox plus Nav2 is unmeasured.
- Jazzy compatibility of the LiDAR driver and the exploration package is unverified.
- A planar LiDAR misses obstacles that do not cross its scan plane; this limits R-05 and must be
  stated in the demo scope.
- Open-loop speed (duty to m/s) depends on load and battery voltage; calibrate in Phase 2.
