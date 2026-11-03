# Current Project State

Last documented baseline: `d0dce8b` (`Add autonomous frontier exploration`).

This is the current engineering-status summary. Older Phase 1 documents remain useful for requirements and early design intent, but some of their status statements are outdated.

## Executive summary
The repository now supports a complete autonomous simulation pipeline:

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
- Simulated TF.
- SLAM Toolbox online mapping.
- Nav2 planning/navigation.
- Consecutive Nav2 goals.
- Simulated obstacle avoidance.
- Custom frontier detection/ranking.
- Autonomous goal selection and path validation.
- Failed-goal suppression.
- Stable exploration-completion detection.
- Foxglove visualization.
- 334 passing unit tests at baseline.

The real RPLIDAR C1 has also produced live `/scan` data. This is not equivalent to final robot integration.

## Pending physical integration
- Arduino firmware.
- Real serial transport.
- MDD10A control from the real Uno.
- Motor direction/duty calibration.
- Final dimensions/footprint.
- Final LiDAR mount transform.
- Physical odometry source.
- Full physical TF chain.
- Full autonomous hardware run.
- 90% accessible-area coverage acceptance test.
- obstacle-clearance test.
- 30-minute runtime test.
- final automatic-stop test.

## Major components

### Pure Python core
`src/lidar_robot/`
- protocol/config/kinematics
- bridge/transport/mock MCU
- world/mock LiDAR
- simulator
- exploration

### ROS 2
`ros2_ws/src/`
- `robot_base_bridge`
- `robot_mock`
- `robot_bringup`
- `robot_explorer`

### Configuration
- `config/robot.toml` — real values, unknowns stay unset
- `config/robot.mock.toml` — simulation-only
- `config/slam_toolbox_sim.yaml`
- `config/nav2_sim.yaml`

## Complex world
Current simulator uses `build_complex_world()`.

The 8x6 m world contains a central obstacle, a sealed room, a reachable room with a 0.8 m doorway, partial walls creating occlusion, and a freestanding wall.

The sealed room is a deliberate test: unknown space behind a continuous wall may remain unknown without blocking completion. The room with a doorway must remain explorable.

## Current exploration state
The explorer:
- reads `/map`
- finds/ranks reachable frontiers
- waits for candidate persistence
- validates using `ComputePathToPose`
- sends `NavigateToPose`
- waits for result
- lets SLAM settle
- suppresses failed areas temporarily
- publishes completion when no persistent reachable frontier remains and known map area is stable

Known weakness: larger-world exploration can be slow and indecisive.

## Recommended next software task
Add objective benchmark logging before tuning:
- completion time
- path length
- number of goals
- failures
- suppressed/retried goals
- coverage
- final reachable unknown area

Then tune exploration and compare against the baseline.

## Demo-only work
A local RF2O handheld-LiDAR workspace may exist under `~/handheld_lidar_demo`. It is not part of the production architecture unless explicitly adopted.
