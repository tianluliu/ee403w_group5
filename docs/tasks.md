# Current Tasks

This file is the active backlog. Completed historical software milestones are summarized first; remaining work follows in priority order.

## Completed software milestones

- [x] PDR requirements captured
- [x] real/mock configuration separation and safety validation
- [x] Pi/Arduino protocol framing and mock MCU behavior
- [x] differential-drive kinematics
- [x] `MotorBridge` safety/fail-safe behavior
- [x] deterministic 2D world and LiDAR model
- [x] dynamic differential-drive simulator
- [x] simulated Arduino control path
- [x] dynamic `/odom`, `/scan`, and TF
- [x] RPLIDAR C1 dependency setup
- [x] live RPLIDAR C1 scan demonstration
- [x] SLAM Toolbox simulation integration
- [x] Nav2 simulation integration
- [x] repeated Nav2 goal execution
- [x] complex exploration world
- [x] pure frontier exploration algorithms
- [x] ROS autonomous explorer state machine
- [x] Nav2 path validation before exploration navigation
- [x] failed-frontier suppression
- [x] conservative exploration-completion detection
- [x] Foxglove visualization
- [x] 334 passing unit tests at documented software baseline
- [x] current project/agent documentation reconciled with implementation

## Next software task: benchmark exploration

Before changing the frontier policy:
- [x] log total exploration time
- [x] log simulated path length
- [x] count navigation goals
- [x] count failed goals
- [x] count suppressed/retried goals
- [x] record completion result
- [ ] compute or record accessible-area coverage where practical
- [x] exercise recorder on ROS 2 Jazzy/Pi (exploratory interrupted run)
- [ ] measure recorder/evaluator overhead
- [x] audit TF source/configuration and add opt-in controller diagnostics (TF_DIAGNOSTICS.md)
- [x] reproduce/diagnose Nav2 future extrapolation from stale map/odom timestamps (`tf-02`)
- [x] enable simulation-only `restamp_tf: true` as a single-variable experiment
- [x] validate restamping comparison on Pi (`tf-03`: COMPLETE 549.97 s, zero MPPI code 102)
- [ ] repeat before claiming general TF reliability; assess map integrity/coverage
- [x] add bounded opt-in simulator timer/watchdog instrumentation
- [ ] capture and diagnose control/watchdog timing and NOT_ARMED periods on Pi
- [ ] examine recorded maps around NO_VALID_PATH failures and large SLAM corrections
- [ ] add aligned offline truth-grid coverage analysis (see BENCHMARKING.md)
- [ ] save a baseline result for `build_complex_world()`

## Exploration tuning

After baseline metrics exist:
- [ ] reduce goal indecision/churn
- [ ] improve ranking/hysteresis
- [ ] reduce unnecessary waiting without destabilizing SLAM
- [ ] compare tuned metrics against baseline
- [ ] preserve sealed-room completion behavior
- [ ] preserve traversable-doorway behavior
- [ ] keep all tests passing

## Physical drivetrain integration

- [ ] confirm exact motor part number
- [ ] confirm whether encoders exist
- [ ] measure wheel radius, track width, body footprint, and LiDAR mount
- [ ] confirm battery/regulator/Uno power arrangement
- [ ] confirm MDD10A PWM/DIR wiring and common ground
- [ ] confirm hardware fuse/kill switch
- [ ] choose physical odometry source
- [ ] fill `config/robot.toml` with measured values
- [ ] implement Arduino protocol-v1 firmware
- [ ] implement real serial `Transport`
- [ ] implement/connect the physical `/cmd_vel` bridge
- [ ] wheels-up watchdog/stop test
- [ ] low-speed controlled motor test

## Physical ROS integration

- [x] Ubuntu 24.04 ARM64 + ROS 2 Jazzy available on Pi
- [x] RPLIDAR C1 can publish live scans
- [ ] create measured `base_link -> laser` transform
- [ ] publish trustworthy physical `odom -> base_link`
- [ ] run SLAM with real LiDAR + physical odometry
- [ ] tune physical Nav2 footprint, limits, costmaps, and controller
- [ ] integrate physical collision monitor
- [ ] run frontier explorer on the real robot

## Formal acceptance

The following remain unclaimed until the procedures in `test_procedures.md` are executed and recorded:
- [ ] T-AUTO autonomous operation
- [ ] T-NOMAP start with no preexisting map
- [ ] T-COV >= 90% accessible-floor coverage
- [ ] T-OBS stationary obstacle avoidance/clearance
- [ ] T-STOP automatic termination
- [ ] T-LOC localization/map quality
- [ ] T-BAT >= 30 minute runtime
- [ ] T-WD physical watchdog timing

## Project hygiene / decisions

- [ ] ratify proposed acceptance definitions (HQ-14)
- [ ] decide physical start command (HQ-16)
- [ ] choose final test environments (HQ-17)
- [ ] choose battery-runtime load profile (HQ-18)
- [ ] choose repository/ROS package license and consistent maintainer metadata (HQ-19)

3D Gaussian Splatting remains outside the committed project scope.
