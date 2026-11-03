# Exploration benchmarking

The optional recorder observes the existing explorer; it does not change ranking,
parameters, completion gates, TF ownership, or motor safety. The first Pi run
produced an exploratory interrupted diagnostic (`BENCHMARK_DIAGNOSTIC_01.md`).
No formal completed complex-world baseline is claimed yet.

## Run protocol

Build/source the workspace as in RUNBOOK.md. Start a fresh `sim_nav.launch.py` for
each trial, with no loaded map or manual steering. Use the current complex world
and initial pose (1, 2, 0). Record commit, machine/OS, ROS/Nav2/SLAM versions and
configuration hashes alongside the results. Keep the same startup procedure and
machine workload across trials. Start the explorer once the stack is ready:

```bash
mkdir -p data/benchmarks
ros2 run robot_explorer explorer_node --ros-args -p autonomous:=true \
  -p benchmark_path:="$PWD/data/benchmarks/complex-01.jsonl" \
  -p benchmark_label:="complex-01 commit=<commit> machine=<machine>"
```

The output parent must exist; an existing output file is refused. Empty
`benchmark_path` (default) disables recording. Recording requires autonomous mode.
`benchmark_label` is free-form provenance. Output is JSONL, schema version 1:
metadata, timestamped events, then a summary on completion or orderly shutdown.
Elapsed timestamps use `time.monotonic()`. Parameters are captured in metadata.
I/O errors are logged and disable recording without changing explorer decisions;
such output is incomplete and must not be treated as a successful benchmark.

Run at least three fresh trials. Choose and record a timeout before running;
the recorder does not impose one or cancel motion. On a timed-out trial use the
existing operator stop procedure, preserve the partial log and label the trial
as timed out externally (orderly shutdown records `interrupted`). Never discard
failed trials. Report individual outcomes, median/range for completed trials,
and completion count over all trials. Geometry is deterministic; asynchronous
SLAM/Nav2 timing is not. Compare instrumented and uninstrumented runs for overhead.

## Definitions

- Startup: recorder creation to first autonomous evaluation with map, TF and an
  in-bounds robot cell. This does not assert that Nav2 action servers are ready.
- Exploration time: that first evaluation to the unchanged COMPLETE transition.
  Partial runs report elapsed time and an interrupted/error outcome separately.
- Distance: sum of successive finite, strictly increasing-stamp `/odom` XY
  samples after readiness and before completion. First sample seeds the sum.
  This is sampled simulated odometry, not physical distance. Samples, invalid or
  nonincreasing samples, and maximum sample gap expose data quality. Dropped
  samples can underestimate curved travel; resets/jumps can overestimate it.
  `/sim/path` is deliberately unused because its history is bounded.
- Goals: `navigation_request` counts requests sent; `navigation_accepted` counts
  acceptance. `navigation_result_status_N` counts terminal ROS action statuses
  (4 succeeded, 5 canceled, 6 aborted). Planner request/results are separate.
- `failure_VALIDATING` and `failure_NAVIGATING`: supervisor failures by phase;
  event reasons distinguish rejection, unavailable servers, transport errors,
  and unusable paths. These are not all aborted navigation goals.
- Suppression: entries created by `_fail_latched_goal`. `suppressed_skip` counts
  candidate exclusions per evaluation, not unique frontier regions.
- Retry: a latched goal within `suppression_radius_m` of any previously failed
  position whose suppression expired. This spatial heuristic is not stable
  frontier identity; SLAM can move coordinates. Each latching counts at most once.
- Candidate replacement: a pre-latch candidate fails the existing proximity
  check. Evaluation events record cluster/usable/known-cell counts and state.
- Final summary: supervisor state, outcome, counters, elapsed/startup time,
  sampled path length and sample quality. Completion is the existing heuristic,
  not formal physical T-STOP acceptance. Abrupt process death may leave no summary.

## Coverage limitation and follow-up

Coverage is explicitly null with an unavailable reason. Known map size and
frontier counts are diagnostics, not coverage percentages or proof of no unknown
reachable area. `lidar_robot.metrics.coverage()` requires equal grid geometry;
SLAM maps can change origin/extent and need alignment to world coordinates.

Next, build/test an offline truth-grid rasterizer from `build_complex_world()`
and sample the SLAM map into that grid using an explicit map/odom alignment,
origin rotation and resolution. Inflate walls with a declared footprint and
flood-fill from the start. Exclude the sealed interior; verify the 0.8 m doorway
room remains reachable. Report known-free coverage, accessible unknown area and
incorrectly occupied accessible area separately. Keep this calculation outside
the policy loop. Existing coverage classification thresholds differ from the
explorer's; document them rather than silently changing either implementation.
