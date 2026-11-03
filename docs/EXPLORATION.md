# Frontier Exploration Design

## Name
The simulation uses **custom frontier-based exploration with information-gain/path-cost ranking and Nav2 goal execution**.

Nav2 plans and drives. The project code decides **where to explore next**.

## Occupancy-grid idea
SLAM Toolbox publishes `/map`. Cells are treated as unknown, known-free, or occupied.

A frontier is reachable known-free space touching unknown space.

## Algorithm pipeline
```text
/map
 -> obstacle inflation / clearance
 -> reachable known-free cells
 -> frontier detection
 -> frontier clustering
 -> small-cluster filtering
 -> safe goal on known side
 -> yaw toward unknown
 -> information-gain estimate
 -> known-space grid path distance
 -> ranking
 -> stable candidate
 -> Nav2 path validation
 -> NavigateToPose
 -> map settles
 -> repeat
```

## Core file
`src/lidar_robot/exploration.py`

Important functions/types:
- `GridMap`
- `reachable_free_cells()`
- `find_frontier_cells()`
- `cluster_frontier_cells()`
- `find_frontier_clusters()`
- `choose_frontier_goal()`
- `frontier_information_gain()`
- `rank_frontiers()`

## Frontier detection
Frontier adjacency to unknown uses cardinal neighbors so a diagonal corner alone does not create a false opening.

Obstacle inflation accounts for robot clearance. This is what allows narrow gaps to be rejected while wider doorways remain reachable.

## Clustering
Connected frontier cells become a `FrontierCluster`. Tiny clusters are filtered.

Default:
```text
min_cluster_size = 5
```

At 0.05 m resolution this helps reject small SLAM artifacts.

## Goal placement
Goals are placed on the known/reachable side of the frontier rather than in unknown space.

Default standoff:
```text
0.20 m
```

Yaw faces the unknown region.

## Information gain
`frontier_information_gain()` counts unique unknown cells around a frontier.

Default radius:
```text
0.50 m
```

## Path cost
The algorithm uses grid distance through known-free space, not only Euclidean distance. A wall-forced detour is therefore more expensive.

## Ranking
Current formula:
```text
score = information_gain_cells / (path_distance_m + 0.25)
```

Sort preference:
1. higher score
2. shorter path
3. larger cluster
4. deterministic cell ordering

## ROS supervisor
`ros2_ws/src/robot_explorer/robot_explorer/explorer_node.py`

States:
```text
SEARCHING
VALIDATING
NAVIGATING
SETTLING
COMPLETE
```

### SEARCHING
Build/rank frontiers, ignore currently suppressed candidates, and require the best candidate to persist.

Defaults:
```text
decision_period_s = 1.0
persistence_checks = 3
candidate_tolerance_m = 0.20
```

### VALIDATING
Submit the latched goal to Nav2 `/compute_path_to_pose`. Require a successful non-empty path.

### NAVIGATING
Send the validated goal to `/navigate_to_pose`. Do not continuously replace it while Nav2 is executing.

### SETTLING
Wait for map/costmap updates.

Default:
```text
settle_time_s = 2.0
```

### Failed-goal suppression
Defaults:
```text
failure_suppression_s = 20
suppression_radius_m = 0.35
```

A failed area may be reconsidered later because the map can change.

## Completion
Completion is intentionally conservative.

Do not complete if:
- frontier clusters still exist but are currently unusable
- failed candidates are still under suppression

When there are no clusters, require stable known-map size over multiple checks.

Defaults:
```text
completion_checks = 5
completion_max_known_growth_cells = 5
```

Completion publishes:
```text
/exploration/complete = true
```

## Important environment semantics
Unknown interior space behind a sealed wall is allowed to remain unknown because it is unreachable.

Unknown space behind a traversable doorway is not allowed to be ignored.

This distinction must be preserved in future tuning.

## Current weakness
The algorithm works but was observed to be too slow/indecisive in the complex world.

Benchmark before tuning. Useful metrics:
- completion time
- total distance
- number of goals
- failed goals
- retries/suppressions
- coverage
- major goal changes

Potential tuning areas:
- scoring formula
- hysteresis
- larger-frontier preference
- map-change-aware reevaluation
- settle timing
- path-cost estimation

Do not sacrifice doorway correctness, clearance, or completion correctness for speed.
