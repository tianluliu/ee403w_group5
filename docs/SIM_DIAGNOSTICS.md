# Simulator control/watchdog and map-quality capture

This is instrumentation only. `control_diagnostics` defaults false; pass
`control_diagnostics:=true` to `sim_nav.launch.py` to enable it on the existing
mock simulator. It is forwarded through `sim_slam`; no additional simulator or
TF owner is launched. No watchdog/TTL, re-arm, controller, SLAM, exploration,
speed or completion decision changes. Keep tested `restamp_tf=true`.

## Diagnostic topic

`/sim/control_diagnostics`: `std_msgs/String`, JSON schema version 1, reliable,
volatile, depth 10, nominal 1 Hz. It contains:

- `ros_ns`, `monotonic_ms`, `publication_id` for correlation/drop detection.
- Command receipt monotonic time/age, commanded linear/angular velocities,
  motion-session-active flag and cached bridge MCU state.
- `mcu`: actual state, fault, TTL remaining, link age/remaining, output duty
  pair (permille), last checksum-valid command/time and cumulative command counts.
  Command counts include validly framed but subsequently rejected commands.
- `timers`: callback start intervals and durations for physics/control/scan,
  window counts, mean/min/max/last intervals, and monotonic-nanosecond timestamps
  bracketing the maximum interval/duration. Windows drain at publication;
  previous callback-start times persist across windows. These are actual callback
  observations, not ROS executor TimerInfo. Physics still caps integration dt
  at 0.1 s, as before; measured elapsed intervals are not capped.
- `events`: ARM, explicit DISARM, first accepted STOP after a different command,
  command TTL expiry, watchdog disarm, actual MCU NOT_ARMED rejection, host
  errors and control decision changes (drive/fresh-zero STOP/stale STOP).
  Event source timestamps are taken at the operation/transition, not the later
  publication. Watchdog events capture link age before the subsequent frame can
  refresh it. Bridge state can still precede ACK processing at an MCU event.

Events buffer in memory and publish only at 1 Hz; no new per-tick console output
or extra STAT/PING frames are added. At most 64 event records are retained per
window; `dropped_events` is cumulative. Adjacent identical host/rejection errors
coalesce with repeat count and exact first/last times. ARM/STOP/watchdog edges
never coalesce. Repeated idle STOP frames are represented by cumulative STOP
counts and last command time rather than one event per frame. No per-frame DRV
trace is published. Event counts include repeats. Command arrival is a local
receipt timestamp because Twist has no header.

Reading state does not tick the MCU or refresh deadlines. An overdue-but-not-yet
processed deadline can show zero remaining while still ARMED; the later
`watchdog_disarm` marks when the existing MCU tick actually disarmed it.
Observer exceptions cannot suppress protocol/watchdog actions. Publishing errors
disable diagnostics with an error log. `record_errors`, `observer_errors`, missing publication
IDs and event drops must be reported. A shutdown/crash may lose the last partial
window; retain console logs and keep recording briefly after exploration ends.

This extension has local tests, but its Pi overhead and runtime operation still
need verification. Existing repetitive motor-error logs remain unchanged.

## Pi build and fresh-run setup

Transfer the committed branch through the normal team workflow. Verify the SHA
against the handoff. Stop old simulation/Nav2/explorer/recorders; do not run
standalone `sim`/`mock` alongside this stack. Use the same Pi/load/Foxglove setup
as tf-03 and note any differences. Declare a 600 s exploration limit. Each
terminal starts in `~/ee403w_group5`; create a fresh `sim-04` directory (rename
it everywhere if it already exists).

Terminal 1 (build, validate, capture provenance, launch):

```bash
cd ~/ee403w_group5
source /opt/ros/jazzy/setup.bash
export LIDAR_ROBOT_ROOT="$PWD"
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
cd ros2_ws
colcon build --packages-select robot_mock robot_explorer robot_bringup --symlink-install
source install/setup.bash
cd ..
mkdir data/benchmarks/sim-04
./scripts/test.sh all > data/benchmarks/sim-04/tests.txt 2>&1
git diff --check > data/benchmarks/sim-04/diff-check.txt
git rev-parse HEAD > data/benchmarks/sim-04/commit.txt
git branch --show-current > data/benchmarks/sim-04/branch.txt
git status --short > data/benchmarks/sim-04/git-status.txt
dpkg-query -W ros-jazzy-nav2-controller ros-jazzy-nav2-mppi-controller ros-jazzy-slam-toolbox > data/benchmarks/sim-04/versions.txt
uname -a > data/benchmarks/sim-04/system.txt
sha256sum config/slam_toolbox_sim.yaml config/nav2_sim.yaml config/robot.mock.toml > data/benchmarks/sim-04/config-hashes.txt
ros2 launch robot_bringup sim_nav.launch.py control_diagnostics:=true 2>&1 | tee data/benchmarks/sim-04/nav2-console.log
```

Inspect the test result before launching; skips are not passes. Stop if checks
fail. After stack activation, Terminal 2 (effective values and bag):

```bash
cd ~/ee403w_group5
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 param get /mock_robot control_diagnostics
ros2 param get /slam_toolbox restamp_tf
ros2 param get /controller_server FollowPath.transform_tolerance
ros2 param get /local_costmap/local_costmap transform_tolerance
ros2 param get /global_costmap/global_costmap transform_tolerance
ros2 param dump /mock_robot > data/benchmarks/sim-04/mock-params.yaml
ros2 param dump /slam_toolbox > data/benchmarks/sim-04/slam-params.yaml
ros2 param dump /controller_server > data/benchmarks/sim-04/controller-params.yaml
ros2 param dump /collision_monitor > data/benchmarks/sim-04/collision-params.yaml
ros2 param dump /bt_navigator > data/benchmarks/sim-04/navigator-params.yaml
ros2 param dump /local_costmap/local_costmap > data/benchmarks/sim-04/local-costmap-params.yaml
ros2 param dump /global_costmap/global_costmap > data/benchmarks/sim-04/global-costmap-params.yaml
ros2 node list > data/benchmarks/sim-04/nodes.txt
ros2 topic info /tf --verbose > data/benchmarks/sim-04/tf-publishers.txt
ros2 topic info /tf_static --verbose > data/benchmarks/sim-04/tf-static-publishers.txt
ros2 topic info /map --verbose > data/benchmarks/sim-04/map-publishers.txt
ros2 bag record -o data/benchmarks/sim-04/bag /cmd_vel /collision_monitor_state /map /tf /tf_static /odom /scan /plan /rosout /sim/control_diagnostics
```

Values above must be true/true/0.3/0.3/0.3. The existing collision monitor config
uses `/collision_monitor_state`; verify it appears in `ros2 topic list`.
The diagnostic publisher starts before exploration; verify one snapshot (Ctrl-C
after receiving one message) before starting the explorer:

```bash
ros2 topic echo /sim/control_diagnostics --once
```

Terminal 3 (unfiltered rosout backup):

```bash
cd ~/ee403w_group5
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 topic echo /rosout > data/benchmarks/sim-04/rosout.txt
```

Terminal 4 (explorer):

```bash
cd ~/ee403w_group5
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
timeout --signal=INT --kill-after=10s 600s ros2 run robot_explorer explorer_node --ros-args \
  -p autonomous:=true \
  -p benchmark_path:="$PWD/data/benchmarks/sim-04/explorer.jsonl" \
  -p benchmark_label:="sim-04 commit=$(git rev-parse --short HEAD) machine=lidarbot limit=600s control_diagnostics=true" \
  -p benchmark_tf_diagnostics:=true
```

Timeout exit 124 is expected at the limit; report any forced kill/missing summary.
On COMPLETE or timeout, stop simulator/Nav2 with Ctrl-C in Terminal 1 (explorer
shutdown alone does not cancel active Nav2 navigation). Leave bag/rosout running
for a few seconds, then Ctrl-C them to flush. Do not keep a failed goal moving
while waiting for captures. Record observed stalls, wall-clock times, completion
or interruption, CPU/load changes, and bag/diagnostic warnings in
`data/benchmarks/sim-04/notes.txt` using an editor. Bag recording adds load; do
not claim equal scheduling overhead without measuring the timer data.

## Analysis and copy-back contract

Watchdog question: correlate event source monotonic/ROS times, control gaps,
last command age, cached vs actual state, fault and last valid link time. A
watchdog disarm must stay latched during the same motion session; zero remaining
TTL alone does not imply disarm. `/cmd_vel` plus collision-monitor state separates
upstream stops from a simulator unable to execute fresh motion commands. No
automatic recovery is introduced.

Map question: retain every raw `/map` (origin, resolution, data, stamp) and
correlate occupancy/extent changes with map/odom corrections, scan/odometry and
planner errors. Compare maps in a consistent ground-truth frame; do not equate
known-cell growth or COMPLETE with coverage. Check whether the robot cell stays
free/reachable and whether previously accessible regions become occupied or
unknown. This run supplies evidence; it does not change SLAM or completion.

Copy the **entire** `data/benchmarks/sim-04/` directory back to the Mac: JSONL,
console, rosout, all parameter/provenance/check files, notes, and the complete bag
folder including metadata and every MCAP/DB3 segment. Do not copy only metadata
or grep excerpts. On the Mac, replace the SSH placeholders with your connection:

```bash
cd /Users/brendan/Documents/ee403w_group5
mkdir -p data/benchmarks/sim-04
rsync -av '<pi-user>@<pi-host>:~/ee403w_group5/data/benchmarks/sim-04/' data/benchmarks/sim-04/
```

Generated artifacts remain ignored. No hardware acceptance claim follows from
these simulation checks.
