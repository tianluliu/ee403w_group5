# Interfaces

Status: current software interfaces for the documented baseline. Simulation/ROS interfaces below are implemented and exercised unless explicitly marked physical/pending.

## 1. Conventions

- Units: metres, seconds, radians.
- Frames follow REP-103/REP-105 conventions: x forward, y left, z up, positive yaw counter-clockwise.
- `Twist.linear.x` is forward velocity of `base_link`.
- `Twist.angular.z` is yaw rate.
- Hardware-dependent physical values live in `config/robot.toml`; unknown values stay unset.
- `config/robot.mock.toml` contains simulation-only values.

## 2. ROS 2 topics

### Core runtime topics

| Topic | Type | Current publisher | Main consumers | Status |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `mock_robot_node` in simulation; `sllidar_node` on real C1 | SLAM Toolbox, Nav2 costmaps/collision monitor | implemented |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 collision-monitor output | `mock_robot_node`; future physical bridge | implemented in simulation |
| `/odom` | `nav_msgs/Odometry` | `mock_robot_node` in simulation; physical source TBD | Nav2/TF consumers | implemented in simulation |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox | Nav2, explorer, visualization | implemented |
| `/tf` | `tf2_msgs/TFMessage` | SLAM/odometry publishers | all TF consumers | implemented |
| `/tf_static` | `tf2_msgs/TFMessage` | LiDAR mount publisher | all TF consumers | implemented in simulation |

### Simulation topics

| Topic | Type | Publisher | Purpose |
|---|---|---|---|
| `/sim/path` | `nav_msgs/Path` | `mock_robot_node` | bounded simulated trajectory |
| `/sim/world` | `visualization_msgs/MarkerArray` | `mock_robot_node` | ground-truth world/robot markers |

### Exploration topics

| Topic | Type | Publisher | Purpose |
|---|---|---|---|
| `/exploration/goal_preview` | `geometry_msgs/PoseStamped` | `robot_explorer` | best current frontier goal |
| `/exploration/markers` | `visualization_msgs/MarkerArray` | `robot_explorer` | frontier/goal visualization |
| `/exploration/complete` | `std_msgs/Bool` | `robot_explorer` | `true` on completion |

### Other project topic

| Topic | Type | Publisher | Purpose |
|---|---|---|---|
| `/robot/heartbeat` | `std_msgs/Header` | `robot_base_bridge/heartbeat_node` | graph liveness only; not motor safety |

A final product start service and software `/estop` interface are not yet finalized.

## 3. Nav2 actions used by exploration

| Action | Type | Use |
|---|---|---|
| `/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | validate a frontier goal |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | execute the validated goal |

The explorer chooses the destination; Nav2 owns path planning/control.

## 4. TF chain and ownership

Required chain:

```text
map -> odom -> base_link -> laser
```

### Simulation with SLAM (`sim_slam` / `sim_nav`)

| Edge | Owner |
|---|---|
| `map -> odom` | SLAM Toolbox |
| `odom -> base_link` | `mock_robot_node` |
| `base_link -> laser` | `mock_robot_node` static broadcaster |

### Standalone simulator (`sim.launch.py`)

SLAM is intentionally absent. `robot_bringup` publishes a simulation-only identity `map -> odom` so the scene can be visualized in a map frame.

### Physical robot

| Edge | Intended owner |
|---|---|
| `map -> odom` | SLAM Toolbox |
| `odom -> base_link` | selected physical odometry source |
| `base_link -> laser` | static transform from measured mount |

Exactly one publisher may own each edge.

## 5. LiDAR interface

Real sensor: RPLIDAR C1 through `sllidar_ros2`.

Development settings:
- serial baud: 460800
- scan frame: `laser`
- scan mode: Standard
- prefer `/dev/serial/by-id/...` over `/dev/ttyUSB*`

Live C1 `/scan` data has been demonstrated on ROS 2 Jazzy.

The production mount transform is still pending physical measurement.

## 6. Raspberry Pi to Arduino serial protocol, version 1

Framing and mock behavior are implemented in:
- `src/lidar_robot/protocol.py`
- `src/lidar_robot/mock_mcu.py`
- `src/lidar_robot/bridge.py`

Real Arduino firmware and a real serial `Transport` are not implemented yet. The mock is the executable reference.

### 6.1 Framing

```text
$<CMD>,<field>,<field>,...*<HH>\n
```

- ASCII only.
- `HH` is XOR of bytes between `$` and `*`, as two uppercase hex digits.
- Entire frame is at most 64 bytes.
- `$` restarts framing.
- Bytes outside a frame are ignored.
- configured Arduino serial baud: 115200, 8N1

### 6.2 Host commands

| Command | Fields | Effect |
|---|---|---|
| `HELLO` | `seq` | protocol/firmware/capability/state reply |
| `ARM` | `seq` | zero outputs, clear fault, enter ARMED |
| `DISARM` | `seq` | zero outputs, enter DISARMED |
| `DRV` | `seq,left,right,ttl_ms` | set open-loop duty while ARMED |
| `STOP` | `seq` | zero outputs immediately |
| `PING` | `seq` | liveness |
| `STAT` | `seq` | state/output/status report |

`left`/`right` are duty permille bounded by the MCU cap. `ttl_ms` must be 1..1000.

### 6.3 Replies

| Reply | Meaning |
|---|---|
| `ACK,seq,state` | accepted |
| `NAK,seq,reason` | rejected |
| `HELLO,seq,proto,fw_id,caps,state` | capability/state report |
| `STAT,seq,state,left,right,ttl_remaining_ms,fault,uptime_ms` | status |

NAK reasons: `BAD_FRAME`, `BAD_ARGS`, `UNKNOWN_CMD`, `NOT_ARMED`, `RANGE`, `TTL`.

Capability bit `0x01` indicates encoders.

### 6.4 Safety behavior required of physical firmware

1. reset/power-up/brown-out/watchdog reset -> DISARMED, outputs zero
2. ARM is required to leave DISARMED
3. command TTL expiry -> outputs zero
4. link timeout while ARMED -> outputs zero, DISARMED, `LINK_TIMEOUT`
5. evaluate time before processing newly received bytes
6. rejected `DRV` while ARMED -> outputs zero
7. corrupt/unknown input never creates motion
8. MCU independently enforces duty cap

Current mock/default timing:
- command period 100 ms
- command TTL 250 ms
- link timeout 500 ms

### 6.5 Host obligations

`MotorBridge` currently:
- connects/HELLO
- arms
- sends twist through kinematics/duty conversion
- STOP/DISARM
- PING/STAT
- validates sequence/replies
- gates motion on `safety.motors_enabled`
- rejects unsafe mock-to-real configuration
- fails safe to STOP

A real byte transport and physical ROS bridge remain to be implemented.

Protocol v1 has no encoder telemetry; adding it requires an intentional protocol version change.

## 7. Simulation/control interfaces

| Module | Purpose |
|---|---|
| `lidar_robot.kinematics` | differential-drive Twist/wheel/duty conversion |
| `lidar_robot.mock_mcu.MockMotorController` | simulated Arduino safety/controller |
| `lidar_robot.transport.LoopbackTransport` | in-memory byte transport + fault injection |
| `lidar_robot.bridge.MotorBridge` | host protocol client and safety gate |
| `lidar_robot.simulator.Simulator2D` | differential-drive physics/collision |
| `lidar_robot.world.World` | 2D geometry/raycast world |
| `lidar_robot.mock_lidar.MockLidar` | simulated LaserScan data |
| `lidar_robot.exploration` | ROS-independent frontier algorithms |
| `robot_mock/mock_robot_node` | ROS simulator integration |
| `robot_explorer/explorer_node` | ROS exploration state machine |

## 8. Configuration interface

- `config/robot.toml`: real robot; unknown physical values omitted; motors disabled by default.
- `config/robot.mock.toml`: simulation-only; `[meta] mock = true`.
- `load_config(path)` returns immutable `RobotConfig`.
- unknown keys/types/non-finite/out-of-range values are errors.
- `cfg.missing(scope)` reports missing values.
- `cfg.require_complete(scope)` rejects incomplete or mock config for hardware-dependent scopes.

## 9. Physical dimensions still required

Before final physical integration, measure/confirm:
- wheel radius
- track width
- footprint/body dimensions
- LiDAR x/y/z/yaw mount
- motor output speed/calibration
- encoder availability/details

## Optional benchmark output

Explorer parameters `benchmark_path` (default empty/disabled) and
`benchmark_label` (default empty) select exclusive-create JSONL output and trial
provenance. Recording requires `autonomous=true` and adds a `/odom` subscription
(depth 100). No topic/action/TF publishers change. Schema version 1 and metric
semantics are defined in `BENCHMARKING.md`.

Optional `benchmark_tf_diagnostics=false` enables read-only TF probes and
`/plan` (`nav_msgs/Path`, depth 1) / `/rosout` (`rcl_interfaces/Log`, depth 100)
subscriptions when benchmark output is configured. Additive JSONL fields and
events are defined in `TF_DIAGNOSTICS.md`; no ROS publishers or TF owners change.

## Simulator control diagnostics (optional)

`sim_nav`/`sim_slam` launch argument `control_diagnostics=false` forwards a bool
to `mock_robot`. When true, `/sim/control_diagnostics` publishes 1 Hz JSON in
`std_msgs/String` (reliable/volatile, depth 10), schema documented in
`SIM_DIAGNOSTICS.md`. No control/TF topic ownership changes. The mock MCU now
accepts an optional observation callback and exposes read-only
`diagnostic_state()`; neither sends frames nor ticks/refreshes deadlines.
