# Test Procedures

Status: formal acceptance procedures. They have **not** yet been executed and recorded as the final project acceptance set.

Development checks have already demonstrated live LiDAR scans and autonomous simulation, but those are not substitutes for controlled physical tests.

Before final testing, ratify the proposed thresholds in `requirements.md` (HQ-14).

Common rules:
- record date and git commit
- record `config/robot.toml` version
- record environment and operator
- keep raw logs/maps
- an operator-stopped run is aborted, not passed
- powered tests require verified power/wiring and an immediately reachable hardware stop

## T-WD: physical motor watchdog

1. Secure robot with wheels off the ground.
2. Connect/ARM and stream low-speed commands.
3. Stop sending drive commands while keeping the link alive; measure output-stop latency.
4. Repeat by breaking the serial link; confirm output goes to zero and state requires fresh ARM.
5. Reset the Arduino during the test; confirm reset never creates motion.

Pass criteria: use the ratified TTL/link-timeout margin from `requirements.md`.

## T-LID: LiDAR bring-up

A live RPLIDAR C1 `/scan` has already been demonstrated during development. For formal acceptance, repeat and record:
1. start the real C1 driver
2. verify stable scan rate and frame `laser`
3. place a known flat target at measured positions
4. verify zero angle/scan direction against the physical mount
5. record device path, scan mode, and result

## T-NOMAP

1. inspect the physical launch profile for map loading
2. start from a clean mapping session
3. verify no prior occupancy map is loaded
4. record the initial `/map` behavior

## T-AUTO

1. place robot at the marked start
2. issue exactly one agreed start command
3. provide no steering/navigation input
4. allow robot to complete or fail
5. log every operator action

Pass only if the robot completes the agreed mission with no steering input.

## T-COV: coverage

1. create/record an agreed ground-truth floor plan
2. run autonomous exploration from the agreed start
3. save the final robot map
4. align/compare according to the ratified procedure
5. repeat for the agreed trial set

The repository contains `lidar_robot.metrics` tooling for accessible-area/coverage calculations.

Pass: ratified threshold, currently proposed as >= 90% accessible area.

## T-OBS: obstacle avoidance

1. use stationary obstacles that intersect the 2D LiDAR scan plane
2. record obstacle geometry/placement
3. run the physical robot autonomously
4. record contacts and minimum footprint clearance
5. repeat across the agreed trial set

A 2D LiDAR cannot guarantee detection of obstacles entirely above/below its scan plane; that limitation must be stated.

## T-STOP: exploration termination

1. fully reachable enclosed room: expect autonomous completion
2. environment with a sealed/unreachable pocket: expect completion without entering the pocket
3. environment with a reachable room through a valid doorway: expect that room to be explored before completion
4. create a controlled navigation-failure condition and verify the agreed failure/completion semantics
5. compute coverage to catch premature completion

The current simulator completion heuristic is development behavior and is not automatically identical to the final ratified T-STOP definition.

## T-LOC: localization/map quality

Record:
- known wall dimensions vs. map dimensions
- final return-to-start position/heading error if required by the ratified procedure
- TF lookup health
- scan rate

Use ratified limits before declaring pass.

## T-BAT: battery runtime

1. fully charge battery
2. record starting voltage
3. run the agreed representative autonomous load profile
4. log runtime, supply voltage/current where possible, and Pi undervoltage/brownout events
5. record end condition

Pass: >= 30 minutes continuous operation with no disqualifying brownout/undervoltage failure.

## Evidence storage

For every formal run, record:
- git commit
- configuration
- room/test setup
- map/log filenames
- pass/fail result
- anomalies

Generated logs/maps/bags should normally remain out of Git unless intentionally curated as small test artifacts.
