# Interfaces

Status: DRAFT for team confirmation. Sections 3 (serial protocol) and 5 (configuration) describe
code that exists and is unit-tested. Sections 1, 2 and 4 describe ROS 2 interfaces that are
**planned and not yet implemented or run**; nothing in this file has been tested against real
hardware or a real ROS 2 install.

## 1. Conventions

- Units: metres, seconds, radians. Frames follow REP-103 (x forward, y left, z up; yaw
  counter-clockwise positive) and REP-105 naming.
- `Twist.linear.x` is forward speed of `base_link`; `Twist.angular.z` is CCW yaw rate.
- LaserScan out-of-range values follow REP-117: closer than `range_min` is `-inf`, no return is
  `+inf`, invalid is `NaN` (the mock LiDAR in `src/lidar_robot/mock_lidar.py` never emits NaN).
- All hardware-dependent numbers live in `config/robot.toml`. Unknown values are left unset, never
  guessed (section 5).

## 2. ROS 2 topics (planned)

| Topic | Type | Publisher | Subscriber | Notes |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `sllidar_node` (Slamtec driver) | SLAM Toolbox, Nav2 costmaps, collision monitor | Frame `laser`. Datasheet scan rate 8-12 Hz (10 typical). QoS expected to be sensor-data (best effort); verify the driver default in Phase 2. |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 chain (final stage: collision monitor) | base bridge node | Nav2 on Jazzy defaults to `Twist`; it has an optional `enable_stamped_cmd_vel` switch and newer releases move toward `TwistStamped`. Decide once, set it consistently in every Nav2 node and the bridge, and test it. Absence of messages for longer than the command TTL must stop the robot. |
| `/odom` | `nav_msgs/Odometry` | odometry source (see design.md section 7) | SLAM Toolbox (via TF), Nav2 | `header.frame_id=odom`, `child_frame_id=base_link`. Source depends on open question HQ-02 (encoders). |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox | Nav2, exploration supervisor, map saver | Transient-local, reliable. Frame `map`. Proposed resolution 0.05 m. |
| `/tf` | `tf2_msgs/TFMessage` | see section 2.1 | all | Dynamic transforms. |
| `/tf_static` | `tf2_msgs/TFMessage` | `base_link -> laser` publisher | all | Transient-local. |

Project-specific interfaces:

| Interface | Type | State |
|---|---|---|
| `/robot/heartbeat` | `std_msgs/Header` (`frame_id` = source id, `stamp` = ROS time) | skeleton node written (`robot_base_bridge/heartbeat_node`), **UNVERIFIED**. It shows the software graph is alive; it is **not** the motor safety mechanism (that is the Arduino watchdog). |
| `/scan` from `mock_scan_node` | `sensor_msgs/LaserScan` | skeleton written, **UNVERIFIED**; publishes fake obstacles and refuses non-mock configs |
| emergency-stop flag | `std_msgs/Bool`, latched | proposed name only |
| start command | `std_srvs/Trigger` | proposed, see HQ-16 |
| exploration status | topic | proposed only |

### 2.1 Required TF chain and frame owners

```
map ──► odom ──► base_link ──► laser
```

| Edge | Owner (sole publisher) | Rate / kind | Source of values |
|---|---|---|---|
| `map -> odom` | SLAM Toolbox | dynamic, tied to scan processing | scan matching |
| `odom -> base_link` | odometry source | dynamic, proposed >= 20 Hz | encoders in bridge, or laser odometry (HQ-02) |
| `base_link -> laser` | static publisher | static | `lidar.mount_x_m`, `mount_y_m`, `mount_z_m`, `mount_yaw_rad` in `robot.toml` |

Rules: exactly one publisher per edge; every frame has one parent; no extra frame is inserted
between the four above without a team decision (Nav2 `robot_base_frame` is `base_link`).

### 2.2 Timestamp behaviour

- Hardware runs use the Pi's ROS time (`use_sim_time: false`). The Pi has no battery-backed
  clock, so enable NTP/chrony before relying on wall-clock log timestamps.
- A scan's stamp must be the acquisition time of that scan. SLAM Toolbox needs the
  `odom -> base_link` transform to be available at that stamp. Verify the driver's stamping and
  the transform latency in Phase 3; this is unverified.
- The Mac is not a ROS 2 node in this design, so there is no cross-machine clock-sync
  requirement for the robot to function.

### 2.3 Robot dimensions

| Quantity | Config key | Value |
|---|---|---|
| Wheel radius | `drivetrain.wheel_radius_m` | **unknown** (HQ-04) |
| Track width | `drivetrain.track_width_m` | **unknown** (HQ-04) |
| Footprint (circumscribed radius) | `body.footprint_radius_m` | **unknown** (HQ-04) |
| Body length / width | `body.length_m`, `body.width_m` | **unknown** (HQ-04) |
| LiDAR pose in `base_link` | `lidar.mount_x_m`, `mount_y_m`, `mount_z_m`, `mount_yaw_rad` | **unknown** (HQ-04) |

`config/robot.mock.toml` contains made-up values for software tests only.

## 3. Raspberry Pi to Arduino serial protocol, version 1

Implemented in `src/lidar_robot/protocol.py` (framing) and `src/lidar_robot/mock_mcu.py`
(behaviour). The real firmware does not exist yet; it must match this section, and the mock is its
executable reference.

### 3.1 Framing

```
$<CMD>,<field>,<field>,...*<HH>\n
```

- ASCII only. `HH` is the XOR of every byte between `$` and `*`, written as exactly **two
  upper-case hex digits**.
- Tokens are non-empty and contain only printable ASCII excluding space, `$`, `*`, `,`.
- A whole frame, counting `$`, `*`, the checksum, an optional `CR` and the final `LF`, is at most
  **64 bytes**. Longer input is an error (`encode`, `decode` and the stream assembler all enforce
  this identically).
- A `$` always starts a new frame; a `$` inside an unfinished frame discards the partial frame.
- Bytes outside a frame are ignored. Serial settings: 115200 baud, 8N1 (config
  `serial.arduino_baud`; confirm on hardware).

### 3.2 Host to MCU commands

`seq` is a decimal integer 0..65535 chosen by the host and echoed in the reply.

| Command | Fields | Effect |
|---|---|---|
| `HELLO` | `seq` | Reply with protocol version, firmware id, capability byte, state. No state change. |
| `ARM` | `seq` | Zero outputs, clear any latched fault, enter `ARMED`. Never starts motion. |
| `DISARM` | `seq` | Zero outputs, enter `DISARMED`. |
| `DRV` | `seq,left,right,ttl_ms` | Set open-loop duty. `left`,`right` in permille, -cap..+cap (cap is the MCU's `max_duty_permille`, 500 in the bring-up config). `ttl_ms` in 1..1000. Only accepted while `ARMED`. |
| `STOP` | `seq` | Zero outputs immediately; state unchanged. |
| `PING` | `seq` | Liveness only. |
| `STAT` | `seq` | Report state and outputs. |

### 3.3 MCU replies

| Reply | Meaning |
|---|---|
| `ACK,seq,state` | Accepted. `state` is `ARMED` or `DISARMED`. |
| `NAK,seq,reason` | Rejected. `seq` is 0 when the sequence number itself was unusable. |
| `HELLO,seq,proto,fw_id,caps,state` | `caps` is two hex digits; bit 0x01 = encoders available. |
| `STAT,seq,state,left,right,ttl_remaining_ms,fault,uptime_ms` | `fault` is `NONE` or `LINK_TIMEOUT`; `uptime_ms` wraps at 2^32. |

NAK reasons: `BAD_FRAME` (framing, checksum or size error), `BAD_ARGS`, `UNKNOWN_CMD`,
`NOT_ARMED`, `RANGE`, `TTL`. For `DRV` the check order is `BAD_ARGS`, `NOT_ARMED`, `RANGE`, `TTL`.

### 3.4 Safety behaviour the firmware must reproduce

1. Reset, power-up, brown-out and watchdog reset all start `DISARMED` with zero outputs.
2. `ARM` is the only way to leave `DISARMED`. A latched `LINK_TIMEOUT` fault clears only on `ARM`.
3. **Command TTL:** the outputs go to zero when the last accepted `DRV` is `ttl_ms` old
   (inclusive: expired when elapsed >= ttl). The MCU stays `ARMED`.
4. **Link timeout:** while `ARMED`, if no complete, checksum-valid frame arrives for
   `link_timeout_ms` (500 in the bring-up config), zero the outputs, go `DISARMED`, latch
   `LINK_TIMEOUT`. A valid frame that is NAK'd still counts as link activity. Corrupt, partial or
   oversized frames do not.
5. **Time is evaluated before newly received bytes are processed**, so a late frame cannot revive
   an expired watchdog. Use unsigned subtraction for `millis()` rollover safety.
6. A rejected `DRV` while `ARMED` (any reason) also zeroes the outputs.
7. Unknown commands and corrupt frames are NAK'd and do not otherwise change state or motion.
8. The MCU enforces its own duty cap independently of the host.

Recommended hardware companion (not yet confirmed, HQ-08): pull-down resistors on the MDD10A PWM
inputs so the outputs stay low while the Uno is resetting or its pins are high impedance.

### 3.5 Host obligations (implemented in `MotorBridge`, except the periodic scheduling)

- Send `DRV` at `limits.cmd_period_ms` (100 ms) with `ttl_ms = limits.cmd_ttl_ms` (250 ms). When
  idle, send `STOP` or `PING` at no more than half of `link_timeout_ms`. The bridge class sends
  one command per call; the periodic loop belongs to the Phase 2 bridge node.
- Sequence numbers run 1..65535 and skip 0 (0 is reserved for replies about unusable frames).
  `DRV` is never retried automatically; `STOP`/`DISARM` are retried up to 3 times.
- A reply is accepted only if its sequence number matches; other replies are counted as stale.
- Refuse `ARM` and every non-zero `DRV` unless `safety.motors_enabled = true` in the loaded
  config (`false` by default).
- Treat any exception in the velocity-to-duty conversion as "stop".
- Encoders: protocol v1 has no encoder messages. Adding them requires a protocol version bump and
  the `caps` bit.

### 3.6 Example frames (generated by the encoder)

```
$HELLO,1*5F                              host -> MCU
$HELLO,1,1,mock-0.1,00,DISARMED*67       MCU  -> host   (proto 1, no encoders)
$ARM,2*40                                host -> MCU
$ACK,2,ARMED*24                          MCU  -> host
$DRV,3,300,-300,250*69                   host -> MCU
$ACK,3,ARMED*25                          MCU  -> host
$DRV,4,900,0,250*4A                      host -> MCU    (above the duty cap)
$NAK,4,RANGE*2F                          MCU  -> host   (outputs also zeroed)
$STOP,5*01   $PING,6*0A   $STAT,7*09   $DISARM,8*14
$STAT,7,ARMED,300,-300,200,NONE,150*77   MCU  -> host
$NAK,0,BAD_FRAME*31                      MCU  -> host   (corrupt input)
```

## 4. Mock and simulation interfaces (implemented)

| Module | Purpose |
|---|---|
| `lidar_robot.kinematics` | `Twist` to wheel speeds to capped open-loop duty (`twist_to_drive(cfg, twist)`). |
| `lidar_robot.mock_mcu.MockMotorController` | Arduino model; `feed(bytes) -> bytes`, `tick()`, `outputs`, `state`, `fault`. |
| `lidar_robot.transport` | `Transport` interface (`write`, `read`, `close`, `is_simulated`) and `LoopbackTransport` with fault injection (disconnect, dropped writes/replies, corrupted replies, noise, fragmented reads). No serial implementation exists yet. |
| `lidar_robot.bridge.MotorBridge` | Host client: `connect`, `arm`, `send_twist`, `stop`, `disarm`, `ping`, `stat`, `close`. Errors: `MotorsDisabledError`, `NotArmedError`, `LinkError`/`TransportDown`, `ProtocolMismatch`, `MotorRejected`, `ConfigSafetyError`. `stop()`/`disarm()` never raise and return whether the MCU acknowledged. |
| `lidar_robot.frames` | `Transform`, `TfTree` (one parent, no cycles, one publisher per edge), `validate_required_chain`, `laser_mount_transform`, `mock_tf_edges` (mock configs only), `static_publisher_args`. |
| `lidar_robot.metrics` | `Grid` (OccupancyGrid semantics), `accessible_cells`, `coverage(robot_map, truth, start, inflate_cells)` returning a `CoverageResult` with `.fraction` and `.meets(0.90)`. |
| `lidar_robot.world.World` | 2D walls/boxes with `raycast`. |
| `lidar_robot.mock_lidar.MockLidar` | `scan(Pose2D, stamp_ms) -> ScanData` with LaserScan-shaped fields. |
| `lidar_robot.clock` | `SystemClock`, `FakeClock`, `Deadline` for deterministic timing tests. |

## 5. Configuration interface

- `config/robot.toml`: real robot. Unknown physical values are omitted. `safety.motors_enabled`
  is `false`.
- `config/robot.mock.toml`: software tests only. Marked `[meta] mock = true`.
- `load_config(path)` returns an immutable `RobotConfig`. Unknown sections or keys, wrong types,
  non-finite or out-of-range values are errors.
- `cfg.missing(scope)` lists unknown values for a scope: `sim`, `navigation`, `odometry`,
  `firmware`, or `all`.
- `cfg.require_complete(scope)` raises `IncompleteHardwareConfig` if anything is missing **or**
  the config is a mock. Anything that generates firmware constants or real launch parameters must
  call it first.
