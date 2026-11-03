# complex-01 exploratory diagnostic

Input: ignored `data/benchmarks/complex-01.jsonl`, metadata commit `f8e4efb`,
machine `lidarbot`. This is an operator-interrupted exploratory run, not a formal
completed baseline or a predefined-timeout trial. No policy changes accompany
this analysis. Raw logs remain outside version control.

## Observed results

425 valid JSONL records; event counts agree with the final summary. Readiness at
1.015 s, interruption at 595.094 s, measured interval 594.079 s. Final state
NAVIGATING; no completion event. Sampled path 17.468 m over 12,117 odometry samples;
maximum sample gap 1.192 s. Coverage unavailable.

| Event | Count |
|---|---:|
| evaluations | 87 |
| candidate replacements | 0 |
| latched goals | 29 |
| planner requests / successful results | 28 / 28 |
| navigation requests / accepted | 28 / 27 |
| navigation successes / aborts / rejections | 3 / 23 / 1 |
| terminal cancellations | 0 |
| accepted navigation outstanding at interruption | 1 |
| suppressions / candidate exclusion observations | 24 / 120 |
| spatial retries (summary counter) | 18 |

Abort reasons: 22 `error_code=102`, one `error_code=105`; the other failure is
`NavigateToPose rejected goal`. Nav2 Jazzy's FollowPath action defines 102 as
TF_ERROR and 105 as FAILED_TO_MAKE_PROGRESS:
https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_msgs/action/FollowPath.action
The installed Pi package version and detailed Nav2 errors should be captured to
confirm provenance and identify the failing transform/timestamp. The log does
not establish which TF edge failed or why.

The 28 planner request/result latencies sum to 1.860 s (median 0.060 s,
maximum 0.139 s). All yielded a path usable by the supervisor. Three successful
navigation requests consumed 93.808, 114.886 and 31.055 s (239.749 s total).
The 23 aborted requests consumed 146.024 s, including 46.838 s for code 105
and 99.186 s for code 102. Navigation duration includes server/recovery/waiting
and is not equivalent to motion time.

## Ranked findings

1. **Navigation/TF failures dominate the failure to finish.** 23 of 26 terminal
   navigation results aborted; 22 were code 102. Successful standalone planning
   does not establish controller-time TF availability. At 428.59--592.53 s,
   18 latches cycle exactly through approximately (0.598, 4.864), (6.098, 0.264),
   (7.798, 2.864), six each. Seventeen abort before interruption, the last is
   outstanding. Known cells grow only 17,524 to 17,525. These are distant
   repeated attempts, not nearby successful micro-goals. Correct completion
   remains blocked: four frontier clusters and typically two usable candidates.
2. **Fixed retry suppression perpetuates infrastructure failures.** The same
   locations recur every 27--31 s, after the 20 s suppression expires. Suppression
   indiscriminately treats TF failure as a frontier failure. This explains the
   late retry loop; it does not prove those frontiers are geometrically bad.
3. **Decision/settle overhead is substantial but secondary.** Every latch follows
   exactly three evaluations (87/29), with no replacements. First-to-third
   evaluation spans sum to 88.576 s; within-triplet spacing median 1.501 s despite
   a 1 s configured period. Repeated computation/scheduling needs profiling.
   Successful and failed completed navigation attempts have about 3.62--4.73 s
   from result/rejection to the next evaluation record, despite a 2 s settle.
   These intervals include scheduling and frontier computation, not just sleep.
4. **Score short-sightedness/map churn are unproven here.** Zero logged candidate
   replacements, identical known-cell counts within each three-check group, and
   wide goal jumps argue against churn as the observed bottleneck. Neither full
   map changes nor scores/alternatives/robot poses are logged. Known-cell count
   cannot establish unchanged map content. The score's distance denominator and
   non-occlusion-aware radial gain are possible weaknesses, not demonstrated
   causes of this run's failure.

## Approximate state accounting

Reconstruct transitions from planner/navigation requests, result/failure events,
and first subsequent evaluation; infer one unavailable-planner SETTLING branch
from a latch at 57.717 s with no planner request, followed by a new latch at
63.811 s. Existing code takes this branch when the server is unavailable.

| State proxy | Seconds | Fraction |
|---|---:|---:|
| SEARCHING | 89.66 | 15.1% |
| VALIDATING | 2.17 | 0.4% |
| NAVIGATING | 388.18 | 65.3% |
| SETTLING | 114.07 | 19.2% |

These partition the measured interval but are not exact state residency:
evaluation records occur after expensive frontier work, so SETTLING absorbs
some actual SEARCHING computation. Nominal settle deadlines alone account for
about 55 s (27 completed/rejected requests times 2 s, plus the inferred 1 s
unavailable-planner retry). They cannot explain the full 114 s proxy. The
additional persistence spans similarly include work and scheduling. Do not add
these overlapping approximations as independent idle-time measurements.

## Next experiments and minimum instrumentation

First capture detailed Nav2 error messages, installed package versions, TF ages
and failed transform lookup diagnostics; profile evaluator wall time and timer
lateness. Record explicit state transitions with reasons, action/attempt IDs,
server-unavailable events, selected goal pose/yaw, robot pose, score/gain/grid
path distance and top few alternatives. Map receipt/version and content-change
counts separate content changes from size changes. Low-rate navigation feedback
(distance remaining/recovery count) and per-goal odometry distance separate slow
travel from recovery/waiting. Avoid full-map logging in the decision callback.

Only after diagnosing TF failures, test changes independently:

1. Treat transient infrastructure failures as navigation health failures rather
   than repeatedly penalizing frontier locations; use bounded backoff and resume
   only when health recovers. Expect fewer repeated 102 attempts and retries,
   with more successes and known-area growth. Never declare completion while
   frontiers remain merely because navigation is unhealthy.
2. If repeated frontier computation causes missed decision periods, reuse
   reachability/inflation work within an evaluation before changing policy.
   Reduce persistence 3 to 2 as a separate experiment only after measuring it.
   Expect lower evaluation latency/search-to-latch time; compare navigation and
   map progress to ensure that faster latching is useful.

Do not first change scoring, clearance, safety, TF ownership, or completion.
Do not first increase speed or relax transform tolerances. Reassess ranking with
logged alternatives after navigation is reliable. Keep settle at 2 s for the
initial comparison; indiscriminate shorter waits may just accelerate failures.
