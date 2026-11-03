# Current Project State

Software baseline: `d0dce8b` (`Add autonomous frontier exploration`).

This document summarizes what the repository currently does and what remains for physical integration.

## Executive summary

The Raspberry Pi can run a complete autonomous simulation pipeline:

```text
dynamic robot simulation
 -> simulated LiDAR + odometry
 -> SLAM Toolbox
 -> Nav2
 -> custom frontier explorer
 -> autonomous NavigateToPose goals
```

The physical drivetrain is not yet integrated.

## Exercised on Raspberry Pi

- ROS 2 Jazzy package builds.
- Dynamic differential-drive simulation.
- Simulated Arduino safety/control path.
- Moving `/scan`.
- Dynamic `/odom`.
- Simulated TF chain.
- SLAM Toolbox online mapping.
- Nav2 planning/navigation.
- Consecutive Nav2 goals.
- Simulated obstacle avoidance.
- Custom frontier detection/ranking.
- Autonomous goal selection and path validation.
- Failed-goal suppression.
- Stable exploration-completion detection.
- Foxglove visualization.
- Live RPLIDAR C1 scans.
- 334 passing unit tests at the documented software baseline.

## Not yet verified on the final physical robot

- Arduino firmware.
- Real Pi-to-Arduino serial transport.
- MDD10A control from the real Uno.
- Motor direction/duty calibration.
- Final chassis dimensions and footprint.
- Final LiDAR mounting transform.
- Physical odometry source.
- Full physical `map -> odom -> base_link -> laser` chain.
- Full autonomous hardware run.
- Formal 90% coverage acceptance test.
- Formal obstacle-clearance acceptance test.
- 30-minute battery runtime test.
- Formal automatic-stop acceptance test.

## Major components

### Pure Python core

`src/lidar_robot/`

Important modules:
- `protocol.py`
- `config.py`
- `kinematics.py`
- `bridge.py`
- `transport.py`
- `mock_mcu.py`
- `world.py`
- `mock_lidar.py`
- `simulator.py`
- `exploration.py`

### ROS 2 workspace

`ros2_ws/src/`

Important packages:
- `robot_base_bridge`
- `robot_mock`
- `robot_bringup`
- `robot_explorer`

### Configuration

- `config/robot.toml` — real hardware values; unknowns remain unset
- `config/robot.mock.toml` — simulation-only values
- `config/slam_toolbox_sim.yaml`
- `config/nav2_sim.yaml`

## Complex world

The current simulator uses `build_complex_world()`.

The 8 m x 6 m world contains:
1. a central obstacle
2. a sealed room whose interior should remain unknown but unreachable
3. a reachable room with a 0.8 m doorway
4. partial walls creating occluded but reachable regions
5. a freestanding wall

This environment is intentionally designed to catch bad exploration assumptions.

## Current exploration state

The explorer:
- reads `/map`
- finds/ranks reachable frontiers
- waits for candidate persistence
- validates with `ComputePathToPose`
- sends `NavigateToPose`
- waits for navigation result
- lets the map settle
- suppresses failed areas temporarily
- publishes completion when no persistent reachable frontier remains and known map area is stable

Known weakness: larger-world exploration can be slow and indecisive.

## Recommended next software task

Add objective benchmark logging before tuning:
- completion time
- path length
- number of goals
- failed goals
- suppressed/retried goals
- coverage
- final reachable unknown area

Then tune exploration and compare against the baseline.

## Physical integration priority

1. confirm exact motors and encoder availability
2. choose physical odometry source
3. implement Arduino firmware
4. implement real serial transport
5. calibrate geometry and drivetrain
6. wheels-up safety test
7. low-speed physical motion test
8. integrate real LiDAR + odometry + SLAM
9. integrate Nav2
10. integrate explorer

## Demo-only work

A local RF2O handheld-LiDAR workspace may exist under `~/handheld_lidar_demo`. It is not part of the production architecture unless explicitly adopted.

## Benchmark instrumentation

Opt-in explorer JSONL logging now records timing, sampled odometry distance,
action outcomes, suppression/retry events and completion. See `BENCHMARKING.md`.
Coverage alignment and formal completed baseline trials remain pending; no
exploration policy or physical validation status changed.

Branch verification: `./scripts/test.sh all` on macOS passed 341 unit tests;
all 5 ROS integration tests skipped because ROS packages are unavailable.
`git diff --check` passed. These results do not establish ROS runtime validity.

User-reported Pi verification: 341 unit tests and all 5 ROS integration tests
passed; `git diff --check` passed; `robot_explorer`, `robot_mock`, and
`robot_bringup` built with colcon. The first recorder run was interrupted after
594.079 s measured exploration time. See `BENCHMARK_DIAGNOSTIC_01.md` for the
observed navigation/TF failure and retry loop; this is exploratory evidence,
not a formal completed baseline. Recorder overhead remains unmeasured.
