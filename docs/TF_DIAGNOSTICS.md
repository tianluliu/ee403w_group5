# Controller TF diagnostic run

## Source audit before the controlled fix

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

Initial hypotheses before `tf-02` (see confirmed mechanism below):
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
timer/subscription/output wiring. The `tf-02` reproduction identifies the
timestamp failure below; validation of the controlled fix remains pending.


## tf-02 evidence and single-variable restamp experiment

The predefined 600 s follow-up used unchanged commit `576cfcb`, Nav2 controller/
MPPI 1.3.13 and SLAM Toolbox 2.8.5. Artifacts are ignored under
`data/benchmarks/tf-02*`. The JSONL contains four terminal navigation aborts with
code 102 (attempts 4--7), code/message fields `102`/empty string. Console and
bagged rosout identify six MPPI goal-to-costmap transform exceptions: attempt 3
recovered, attempts 4--6 aborted, and attempt 7 encountered two exceptions before
aborting. All are future extrapolation from `map` to `odom` at robot-pose
execution timestamps, not missing frames or old exploration-goal timestamps.

| Attempt | Requested ROS time | Latest map/odom TF time | First covering TF receipt after request |
|---|---|---|---|
| 3 (recovered) | 1793745089.486026 | 1793745089.447247 | 0.510 s |
| 4 | 1793745101.048666 | 1793745101.007151 | 0.466 s |
| 5 | 1793745112.443648 | 1793745112.405277 | 0.344 s |
| 6 | 1793745189.690554 | 1793745189.621387 | 0.597 s |
| 7 | 1793745221.819196 | 1793745221.800695 | 0.400 s |
| 7 | 1793745230.781191 | 1793745230.755916 | 0.616 s |

Raw bag receipts corroborate each controller-reported stale horizon; these are
not just explorer-buffer observations. Odometry/base TF contains the requested
robot timestamps. Map/odom broadcast continues near 50 Hz but repeats an old
scan-derived timestamp; for attempt 4 the same timestamp repeats 34 times over
about 0.655 s. Covering data arrives after MPPI's 0.3 s lookup deadline. SLAM
scan-filter drops and irregular sensor timing support callback/processing delay,
but the capture does not isolate its exact CPU/executor/filter contribution.
All captured `use_sim_time` settings are false; no raw dynamic stamp backsteps
or conflicting active TF edge ownership were established.

The only runtime change being tested is `restamp_tf: true` in
`config/slam_toolbox_sim.yaml`. Effective `tf-02` parameters had it false, so the
old horizon was last callback scan time + 0.2 s. With this option, SLAM Toolbox
stamps its latest map/odom correction at current ROS time + the unchanged 0.2 s
transform_timeout, decoupling TF timestamp advancement from scan callbacks.
See the [installed-version source](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp).
This simulation-only experiment targets that demonstrated timestamp horizon;
it does not make delayed SLAM processing fresh or establish physical validity.

MPPI and local/global costmap transform tolerances intentionally remain 0.3 s.
All exploration parameters, speed limits, completion semantics, TF owners and
motor/safety behavior remain unchanged. Separately observed simulated motor
"not armed" errors remain unresolved and outside this experiment.

### Exact Pi verification procedure (runtime validation pending)

Transfer this commit using the normal team workflow, then check its SHA on the
Pi. Stop previous simulation/Nav2/explorer/bag processes and use one fresh stack,
no prior map and the same visualization/load conditions as `tf-02`. Do not run
standalone `sim`/`mock` alongside `sim_nav`. Use new output names below; never
overwrite prior evidence. Declare the 600 s exploration limit before starting.

Terminal 1:

```bash
cd ~/ee403w_group5
git branch --show-current
git rev-parse HEAD
source /opt/ros/jazzy/setup.bash
export LIDAR_ROBOT_ROOT="$PWD"
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
cd ros2_ws
colcon build --packages-select robot_mock robot_explorer robot_bringup --symlink-install
source install/setup.bash
cd ..
mkdir -p data/benchmarks
ros2 launch robot_bringup sim_nav.launch.py 2>&1 | tee data/benchmarks/tf-03-nav2-console.log
```

Terminal 2 (after stack activation, before exploration):

```bash
cd ~/ee403w_group5
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 param get /slam_toolbox restamp_tf
ros2 param get /controller_server FollowPath.transform_tolerance
ros2 param get /local_costmap/local_costmap transform_tolerance
ros2 param get /global_costmap/global_costmap transform_tolerance
ros2 param dump /slam_toolbox > data/benchmarks/tf-03-slam-params.yaml
ros2 param dump /controller_server > data/benchmarks/tf-03-controller-params.yaml
ros2 node list > data/benchmarks/tf-03-nodes.txt
ros2 topic info /tf --verbose > data/benchmarks/tf-03-tf-publishers.txt
ros2 topic info /tf_static --verbose > data/benchmarks/tf-03-tf-static-publishers.txt
ros2 bag record -o data/benchmarks/tf-03-bag /tf /tf_static /odom /scan /plan /rosout
```

The parameter reads must return true and 0.3/0.3/0.3 respectively. Do not run the
comparison if the effective values differ.

Terminal 3:

```bash
cd ~/ee403w_group5
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
./scripts/test.sh all
git diff --check
ros2 topic echo /rosout > data/benchmarks/tf-03-rosout.txt
```

Terminal 4:

```bash
cd ~/ee403w_group5
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
timeout --signal=INT --kill-after=10s 600s ros2 run robot_explorer explorer_node --ros-args \
  -p autonomous:=true \
  -p benchmark_path:="$PWD/data/benchmarks/tf-03.jsonl" \
  -p benchmark_label:="tf-03 commit=$(git rev-parse --short HEAD) machine=lidarbot limit=600s restamp=true" \
  -p benchmark_tf_diagnostics:=true
```

Timeout status 124 is expected at the limit. If exploration completes sooner,
stop the run and record completion. At the limit/early completion, stop the
simulation/Nav2 launch with Ctrl-C as well: explorer shutdown alone does not
cancel an active Nav2 goal. Then Ctrl-C the bag recorder and rosout capture to
flush artifacts. A forced kill or missing summary must be reported explicitly.

Compare against `tf-02` using action errors, console exceptions and raw TF:

- Map/odom timestamps should advance near broadcast rate even when scan callbacks
  pause, with signed age near -0.2 s absent transport/scheduling delay.
- There should be no recurring MPPI goal-to-costmap future-extrapolation/code-102
  failures from the demonstrated horizon. Check internal recovered exceptions
  as well as terminal navigation outcomes.
- Verify scan/map alignment visually in fixed frame `map`, including moving
  scans against walls, and retain the bag for timestamp/transform checks.
- Report navigation results, map progress, interruptions and separate motor
  errors. Zero TF errors with stalled motion is not proof of exploration success.

One successful comparison is initial runtime evidence, not full acceptance.
Repeat before claiming reliability; keep sealed-room/doorway and safety tests
passing. Do not tune other variables during this experiment.
