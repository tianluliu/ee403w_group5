# Offline sim-05 SLAM loop-closure experiment

Runtime is **Raspberry Pi / ROS 2 Jazzy only**. The Mac can test the pure Python
measurements/gates and inspect JSON reports. This harness has not yet run on the
Pi; unit tests do not establish working playback or a loop-closure diagnosis.
No live configuration, launch, controller, simulator, watchdog or physical files
are changed. All generated parameters, bags and results stay under ignored data/.

## Question and original event

sim-05 had a 1.52038 m map/odom translation correction at recorded receipt time
1793829384677301288 ns (TF header 1793829384875050937 ns). Yaw changed only
0.16909 degrees. The next raw map changed from 161x121 to 173x148 cells;
occupied cells increased 1167 to 3135. Using occupied >=65, fixed identity world
alignment and cell centers, the fraction >0.15 m from real walls changed from
zero to about 45.23%. Continuous simulator odometry, unchanged world geometry
and correct raw scan geometry implicate SLAM correction, but do not establish
which constraint caused it. Completion after the corrupted map is not coverage
acceptance. This experiment tests loop closure, rather than tuning SLAM.

Crucial limitation: the bag starts about 292 s after the original SLAM process
started. It does not contain that process's initial pose graph or accepted-scan
history. Fresh SLAM may never reproduce the bad constraint. Failure to reproduce
is inconclusive about loop closure; stop rather than proceed to B.

## Architecture, timestamps and ownership

`prepare_slam_replay.py` reads the original MCAP bag without changing it, hashes
its files before/after, and writes a separate input bag containing `/scan`,
`/odom`, `/tf` and `/tf_static`. Within each TFMessage only `odom -> base_link`
is retained on /tf and `base_link -> laser` on /tf_static. Empty TFMessages are
dropped. Recorded SLAM `map -> odom` is never replayed. Retained transform objects
and header stamps are unchanged; scan/odom serialized payloads are copied.
Source receive order and timestamps are preserved, including equal-time order;
a reopened output sequence hash must equal the expected retained sequence.
Original offered QoS metadata is copied. All `/sim/world` wall markers must be
constant in odom; their segments are saved for offline measurements.

Each trial uses an empty localhost-only ROS domain (default 83), one paused
rosbag2 player, a fresh lifecycle async SLAM Toolbox instance and one recorder.
Do not run other nodes in this domain during a trial. The player alone owns
simulator inputs; SLAM alone creates map/odom. The harness checks initial graph
emptiness, input publisher ownership and effective parameters before resuming.
No Nav2, explorer, mock robot or static transform publisher is launched.

Player rate is 1.0, read-ahead 1000, `/clock` 100 Hz. Original message header
stamps are not rebased. SLAM uses `use_sim_time=true`; the recorder also uses
bag clock so output receive times are comparable to the original epoch.
This is essential for historical TF lookups and `restamp_tf` (which stamps using
current ROS time). Clock at EOF is held while processes drain for 5 wall seconds.
This procedure and QoS are identical in A/B. Restamp may put TF headers 0.2 s
ahead; comparisons use bag receipt time, and report TF headers separately.
Playback preserves the input schedule, not DDS cross-topic callback execution
order or bitwise determinism of asynchronous scan acceptance/optimization.

The recorder captures `/clock`, `/map`, `/tf`, `/tf_static`, `/rosout`, `/pose`
and `/slam_toolbox/graph_visualization`; console stdout/stderr is captured too.
The latter topics/logs depend on what the installed SLAM version actually emits.
Debug logging is not enabled as an extra parameter change. Absence of an explicit
loop-closure message is not evidence of absence of optimization.

## Pi procedure

Stop live launch processes first. Use a fresh output path for each experiment;
existing trial directories are refused. From the repository on the Pi:

```bash
cd ~/ee403w_group5
git switch codex/exploration-benchmark
source /opt/ros/jazzy/setup.bash
./scripts/test.sh all
cd ros2_ws
colcon build --symlink-install --packages-select robot_mock robot_bringup robot_explorer
source install/setup.bash
cd ..
git diff --check
```

The harness uses system `slam_toolbox`, `rosbag2_py`, MCAP storage, rclpy and
PyYAML from the Jazzy environment. It requires no new ROS package/build target.
The build above checks the existing workspace; replay does not launch it.

Prepare once (the original bag directory contains metadata.yaml):

```bash
python3 scripts/prepare_slam_replay.py \
  --source data/benchmarks/sim-05/bag \
  --output data/benchmarks/slam-replay/prepared
```

A1/A2 use captured current simulation parameters, including restamp_tf=true and
do_loop_closing=true. The sole common replay override is use_sim_time=true.
The runner captures ALL declared effective parameters, installed Jazzy package
versions and relevant middleware environment settings. Analysis runs after recording closes.

```bash
python3 scripts/run_slam_replay.py --prepared data/benchmarks/slam-replay/prepared --runs data/benchmarks/slam-replay/runs --trial A1
python3 scripts/run_slam_replay.py --prepared data/benchmarks/slam-replay/prepared --runs data/benchmarks/slam-replay/runs --trial A2
python3 scripts/analyze_slam_replay.py gate --runs data/benchmarks/slam-replay/runs
```

Hard gate: BOTH completed A trials must have a >1.0 m TF translation jump within
±2 s of original event time, followed within 2 s by a raw map whose occupied
bad-wall fraction is >=25% and increases >=20 percentage points from the latest
pre-jump map. This tests the same sequence and abrupt damage, not merely an
unrelated later failure. A1/A2 effective parameters, versions, input digest and
runner settings/harness hash must match. Thresholds are predefined, not selected
after looking at results. Gate failure exits nonzero. **Stop there. Do not run B
or tune SLAM.** Retain all artifacts. If missing history prevents reproduction,
next live capture needs original-start scans/TF plus timestamped accepted-scan
IDs/poses, candidate/accepted loop constraints and pre/post optimization graph
snapshots; an occupancy map cannot reconstruct the missing graph.

Only after PASS, execute:

```bash
python3 scripts/run_slam_replay.py --prepared data/benchmarks/slam-replay/prepared --runs data/benchmarks/slam-replay/runs --trial B1
python3 scripts/run_slam_replay.py --prepared data/benchmarks/slam-replay/prepared --runs data/benchmarks/slam-replay/runs --trial B2
python3 scripts/analyze_slam_replay.py compare --runs data/benchmarks/slam-replay/runs
```

Each B command independently recomputes the A gate. Before B playback it compares
its effective parameter tree with BOTH A trees: the ONLY difference must be
`do_loop_closing: true -> false`. Input hash, installed versions, timing/domain
settings and harness hash must also match. Parameters are generated separately
under prepared/A.yaml and prepared/B.yaml; live YAML is never edited. This is
an experiment, not adoption of disabled loop closure as a fix.

## Reports and interpretation

Each analysis.json contains every map's ROS/bag/header time, width, height,
resolution, origin, occupied-cell count, median and p90 wall distance and
fraction >0.15 m. World alignment is fixed identity from the original initial
map/world frame convention; each map uses its full origin transform. There is
NO fitted translation/rotation or independent realignment per map. Distances
are from occupied cell centers (>=65) to deterministic /sim/world line segments,
not wall-marker rendering thickness. Empty occupied sets report null quality.

Reports include all consecutive map/odom translation/wrapped-yaw jumps, largest
jump of each kind, first map with bad-wall fraction >=25%, and reproduction
boolean. Map resizing/distortion timing is visible in the full map evolution;
rosout SLAM records and console loop/optimization/Ceres/drop/queue/filter lines
are retained. Comparison.json collects A1/A2/B1/B2 metrics/evolution. Compare both
B runs against both reproducible A runs, especially the original event sequence;
a smaller maximum alone does not prove a valid map. Read full console logs for
warnings, missing scans and recorder loss. No automatic claim of a fix is made.

Copy the **entire** `data/benchmarks/slam-replay/` tree back to the Mac, including
prepared manifest/source-config/A/B YAML and filtered input metadata+MCAP, all
four trial directories that actually ran (run.json, effective parameters,
parameters.yaml, record QoS, console logs, output bag metadata+MCAP, analysis.json),
gate.json and comparison.json if generated. Keep original sim-05 artifacts too.
To reevaluate the gate/compare on the Mac (no ROS required), use the same gate
or compare commands. Raw-bag analysis/preparation/runtime require the Pi.
