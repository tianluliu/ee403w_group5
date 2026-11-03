# Controller TF diagnostic run

## Source audit and hypotheses

The current `sim_nav` includes `sim_slam`, not standalone `sim` or `mock`:
SLAM owns `map -> odom`; `mock_robot_node` owns dynamic `odom -> base_link`
and static `base_link -> laser`. No duplicate edge owner is launched by this
profile. `amcl.tf_broadcast=true` appears in the Nav2 YAML but this navigation
launch does not start AMCL. Running `mock`, standalone `sim`, another simulator,
or localization simultaneously could introduce duplicates; check the live graph.

Simulator physics/odometry/TF nominal rate is 50 Hz. Odometry and TF sample the
same pose sequentially using separate `get_clock().now()` calls: compatible ROS
clock, slightly different timestamps, not identical stamps. Scans use the same
clock at 10 Hz. The static mount is time-independent. No `/clock` is published;
SLAM/Nav2 launch explicitly uses `use_sim_time=false`, simulator/explorer default
false. Live overrides and clock jumps still need checking.

SLAM config requests `transform_publish_period=0.02`, `transform_timeout=0.2`,
`tf_buffer_duration=30`, `scan_queue_size=1`. A 50 Hz broadcast rate does not
prove fresh transform timestamps. Upstream Jazzy SLAM defaults `restamp_tf=false`
and stamps `map -> odom` at the last callback scan stamp plus transform_timeout.
Its async callback updates that scan header before motion filtering, so being
stationary alone does not imply stale timestamps. Delayed/missing scans, TF
message-filter drops, CPU contention or processing delay could leave the latest
map transform behind odometry. The installed Pi version must be recorded.

Sources:
- [SLAM transform loop and parameters](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/src/slam_toolbox_common.cpp)
- [SLAM async scan callback](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/src/slam_toolbox_async.cpp)

Local costmap uses `odom`; global plan uses `map`. Controller/MPPI must transform
robot/path/goal between them during execution. Upstream Jazzy controller can
throw TF_ERROR when robot pose acquisition or end-pose transformation fails;
MPPI can throw it when transforming robot pose into the global-plan frame or
goal into the costmap frame. MPPI uses the robot-pose timestamp for path
transforms, not simply the original exploration-goal timestamp. Planning success
at a different time/in a different buffer does not validate these lookups.

Sources:
- [Controller TF exception sites](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_controller/src/controller_server.cpp)
- [MPPI path timestamp handling](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_mppi_controller/src/path_handler.cpp)

Ranked hypotheses (none established yet):
1. Missing `map -> odom` timestamp coverage at controller robot/path timestamps,
   even though a latest/common-time lookup and standalone planning succeed.
2. Simulator/TF reception delays or CPU starvation cause stale robot transforms.
   Prior log's maximum odometry gap was 1.192 s. That is supporting context, not
   proof of controller-side TF latency.
3. Stale path endpoint timestamp outside a buffer's history, clock mismatch/jump,
   or an unexpected live duplicate publisher. These require specific exception
   text and live graph evidence.

Relevant unchanged config: controller 10 Hz; MPPI transform tolerance 0.3 s;
local/global costmap transform tolerance 0.3 s; behavior server 0.1 s;
controller costmap update timeout 0.3 s. These settings serve different purposes;
MPPI's transform tolerance is passed as a lookup wait timeout, not blanket
permission to use old data. SLAM's 1 s map update interval is separate from its
TF publication loop. No tolerances or control parameters changed for diagnosis.

## Enable the minimum diagnostics

Build/source as in RUNBOOK. Use one fresh simulator/SLAM/Nav2 stack. Record all
Nav2 launch console output as well as JSONL. Before starting, capture provenance:

```bash
mkdir -p data/benchmarks
ros2 node list > data/benchmarks/tf-nodes.txt
ros2 topic info /tf --verbose > data/benchmarks/tf-publishers.txt
ros2 topic info /tf_static --verbose > data/benchmarks/tf-static-publishers.txt
dpkg-query -W ros-jazzy-nav2-controller ros-jazzy-nav2-mppi-controller ros-jazzy-slam-toolbox > data/benchmarks/tf-versions.txt
ros2 param dump /slam_toolbox > data/benchmarks/tf-slam-params.yaml
ros2 param dump /controller_server > data/benchmarks/tf-controller-params.yaml
ros2 param get /slam_toolbox use_sim_time
ros2 param get /mock_robot use_sim_time
ros2 param get /controller_server use_sim_time
ros2 param get /bt_navigator use_sim_time
```

Inspect node/publisher names for duplicate launches; `/tf` having several
publishers is normal and does not itself establish duplicate edge ownership.
Select a 180 s diagnostic limit in advance; stop earlier after a representative
TF failure if captured. If it does not reproduce, a separately declared longer
run may be needed (the prior first code-102 failure occurred after 5 minutes).
Start the explorer with a fresh output filename:

```bash
ros2 run robot_explorer explorer_node --ros-args -p autonomous:=true \
  -p benchmark_path:="$PWD/data/benchmarks/tf-01.jsonl" \
  -p benchmark_label:="tf-01 commit=<commit> machine=lidarbot limit=180s" \
  -p benchmark_tf_diagnostics:=true
```

Use the existing operator stop procedure at the declared limit. Stopping the
explorer does not itself cancel an active Nav2 goal; ensure the simulator/Nav2
stack is stopped as well. This diagnostic option does not change that behavior.
A short low-volume TF/scan/odom bag, if console/JSONL cannot resolve timing:

```bash
ros2 bag record -o data/benchmarks/tf-01-bag /tf /tf_static /odom /scan /plan /rosout
```

Bag recording is optional and adds load; note whether enabled. Generated logs,
parameter captures and bags stay ignored under `data/`.

## Fields and interpretation

`benchmark_tf_diagnostics` defaults false, requires `benchmark_path`, adds no
control/TF publisher and leaves decision parameters/state semantics unchanged.

- Diagnostic-mode events include `attempt_id` (increments once per latched goal,
  including a latch unable to reach a server) and `ros_now_ns`. Periodic/rosout
  IDs mean the observer's current attempt, not an authoritative Nav2 goal UUID.
- `navigation_start` contains full requested goal frame, stamp, XYZ/quaternion.
  Its immediately following `tf_snapshot` with reason `navigation_start`
  captures the robot in map coordinates, stamp and availability. A missing robot
  transform is recorded, not substituted. Results preserve exact `error_code`
  and `error_msg`; absent fields are null, empty strings remain empty. No lower
  level exception message is invented if Nav2 omits it.
- `tf_snapshot`: once per second and at navigation start/result. Dynamic edges
  include latest stamp, signed age (`ROS now - stamp`; negative is future-dated),
  pose and zero-wait lookup availability at ROS now. Static mount has no age
  metric. Additional probes test map/odom at the latest odom/base stamp, requested
  goal stamp at start, and observed `/plan` endpoint stamp. Last `/odom` stamp,
  age and difference from latest odom/base TF stamp expose reception/stamp skew;
  these are independently received streams, not matched physics-tick pairs.
- `nav2_log`: warning/error/fatal `/rosout` messages from controller, navigator,
  SLAM and costmap loggers, with source ROS stamp and unaltered text. This is
  necessary because action results can omit the detailed exception. Preserve
  console logs too, since rosout delivery is not guaranteed.
- `evaluator_timing`: monotonic duration for every evaluator call, inter-start
  interval and estimated lateness relative to a nominal periodic schedule that
  skips elapsed periods. Lateness is an estimate, not rclpy executor TimerInfo.
- `state_transition`: old/new state, reason, attempt ID and ROS/monotonic times.

Lookups never wait and never gate navigation. A failure at exact ROS now alone
is normal for a stream stamped just behind now; controller lookups may wait up
to their configured timeout. The observer has its own buffer/executor and can
lag during evaluation; its failures do not prove that Nav2's buffer failed.
Correlate controller exception requested/latest/earliest timestamps with these
snapshots, ages and evaluator durations. One-second sampling can miss short
outages. A 50 Hz stream repeating a stale stamp needs raw TF/bag inspection.

Evidence that distinguishes the causes:
- `map -> odom` old while odom/base is fresh, plus controller future-extrapolation
  text at the missing map timestamp: SLAM scan/transform timestamp coverage.
- stale odom/base with controller robot-pose failures: simulator publication or
  reception/scheduling delay; inspect bag/console for CPU and watchdog events.
- past extrapolation at endpoint stamp: path/buffer-history mismatch.
- clock jumps, incompatible `use_sim_time`, or multiple live edge authorities:
  deployment/clock issue. Do not mask these by relaxing tolerances.

Tests here cover serialization, failed-vs-latest lookups, static/future stamps,
state observation and completion guards. The optional ROS smoke test exercises
timer/subscription/output wiring. Pi reproduction and root cause remain pending.
