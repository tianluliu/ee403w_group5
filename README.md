# Autonomous LiDAR Mapping Robot

Software foundation for the team's PDR project: a battery-powered, differential-drive robot
(Raspberry Pi 4, RPLIDAR C1, Arduino Uno, Cytron MDD10A) that explores an unknown flat indoor
room, builds a 2D map, avoids stationary obstacles and stops by itself.

## Status: Phase 1 (software foundation). Nothing has run on hardware.

- No acceptance criterion from the PDR has been tested. See `docs/requirements.md`.
- The robot's real dimensions, motor details and wiring are **unknown** and left unset in
  `config/robot.toml`; invented values live only in `config/robot.mock.toml`.
- **Motors are disabled by default** (`safety.motors_enabled = false`), and the Arduino model
  boots DISARMED. There is no serial-port code and no Arduino firmware yet.
- ROS 2 packages exist as skeletons but are **UNVERIFIED**: they have never been built or run.

## Quick start (MacBook, no installs needed)

```bash
scripts/check_env.sh      # read-only: python3 >= 3.11, ssh, rsync present?
scripts/test.sh           # platform-independent unit tests (stdlib only)
scripts/test.sh -v        # verbose
```

The same command works on the Pi (Ubuntu 24.04 ships Python 3.12). ROS 2 is not needed on the
Mac. Mac-to-Pi deployment over SSH is described in `docs/dev_workflow.md`.

## Layout

```
config/            robot.toml (real, unknowns unset), robot.mock.toml (tests only)
src/lidar_robot/   pure-Python core: protocol, config, kinematics, bridge, transport,
                   mock MCU, mock LiDAR/world, TF validator, coverage metric
tests/unit/        platform-independent tests (unittest; run on Mac and Pi)
tests/ros2_integration/   ROS 2 tests: UNVERIFIED, skipped without rclpy
ros2_ws/src/       ROS 2 Jazzy package skeletons: robot_base_bridge (heartbeat),
                   robot_mock (mock /scan), robot_bringup (launch files)
scripts/           test.sh, check_env.sh, deploy.sh, remote.sh
docs/              requirements, design, tasks, interfaces, open questions,
                   test procedures, dev workflow, future 3DGS note
```

## Documents

| File | Contents |
|---|---|
| `docs/requirements.md` | PDR requirements, safety and software requirements, proposed acceptance definitions |
| `docs/design.md` | architecture, safety layers, odometry options, workflow, risks |
| `docs/tasks.md` | what is done and verified, what remains, approval gates |
| `docs/interfaces.md` | ROS topics and TF chain, Pi to Arduino serial protocol v1, config interface |
| `docs/open_questions.md` | hardware questions and PDR inconsistencies for the team |
| `docs/test_procedures.md` | step-by-step acceptance test procedures (none executed yet) |
| `docs/dev_workflow.md` | Mac to Pi SSH workflow, ROS 2 build and run notes |
| `docs/future_3dgs.md` | design-only note for the optional 3D Gaussian Splatting extension |

## Rules of the road

- Do not connect to the Pi, flash the Arduino, install packages or power the motors without the
  team's explicit approval for that step (`docs/tasks.md` marks these).
- Hardware-dependent output (firmware constants, real launch parameters) must call
  `RobotConfig.require_complete(scope)` first; it refuses mock configs and reports every missing
  value instead of guessing.
- First powered motion happens with the wheels off the ground, behind a hardware kill switch
  (open question HQ-08).
