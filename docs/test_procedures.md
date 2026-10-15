# Test procedures

Status: DRAFT. **None of these procedures has been executed.** The thresholds are PROPOSED in
requirements.md section 5 and need team ratification (HQ-14). Do not report any acceptance
criterion as met until its procedure has been run and the data saved.

Common rules: record date, git commit, `config/robot.toml` version, battery voltage at start,
environment description and operator. Keep raw logs. A run that is stopped by the operator
(stop/disarm) is recorded as aborted, not passed. Powered tests need the approvals listed in
tasks.md and a hardware kill switch within reach.

## T-WD: motor watchdog (Phase 2, wheels elevated)

Purpose: prove motors stop when commands or the link stop.
1. Robot secured so wheels spin freely. Kill switch reachable. Bring-up limits in `robot.toml`.
2. `connect`, then `arm`, then stream `send_twist` at 10 Hz for 5 s at a low speed.
3. Stop sending but keep the link alive (PING every 250 ms). Measure time from the last DRV to
   zero PWM (logic analyzer, scope, or MCU log).
4. Resume, then unplug the USB cable. Measure time to zero PWM and confirm the MCU stays
   DISARMED after replugging until ARM is sent.
5. Reset the Arduino while streaming: confirm outputs stay zero and the MCU boots DISARMED.
Pass: stop within command TTL + 50 ms (step 3) and link timeout + 50 ms (step 4); steps 4 and 5
never produce motion without a fresh ARM.

## T-LID: LiDAR bring-up (Phase 2)
1. Start the driver; confirm `/scan` rate between 8 and 12 Hz and `frame_id` = `laser`.
2. Place a flat object at 0.5 m directly ahead, then to the left. Confirm the range at the
   expected angle and that the scan direction and zero angle match `lidar.mount_yaw_rad` (HQ-12).
3. Confirm closer-than-0.05 m returns are reported as `-inf` or invalid and that nothing else
   produces spurious points.

## T-NOMAP and T-AUTO
- T-NOMAP: inspect launch files for any map loading; confirm the first `/map` is empty.
- T-AUTO: one start command, then no operator input until completion; log every input.

## T-COV: coverage
1. Build ground truth: tape-measure the room; draw the floor plan as an occupancy grid at 0.05 m
   (or save a manually driven reference map), with the start pose as the map origin.
2. Run the autonomous exploration; save the final map from the same origin.
3. Compare with the helper:
```python
from lidar_robot.metrics import Grid, coverage
result = coverage(robot_map, truth_map, start_cell=truth_map.world_to_cell(0.0, 0.0),
                  inflate_cells=footprint_radius_m / truth_map.resolution)
print(result.fraction, result.meets(0.90))
```
4. Repeat for the trial set (3 environments x 3 runs).
Pass: `result.meets(0.90)` in every run.

## T-OBS: obstacle avoidance
1. Place stationary obstacles that cross the LiDAR scan plane (record size and height); keep a
   clear start area. Motors enabled only in a controlled, clear area.
2. Run; mark the footprint outline and obstacle positions with tape; photograph.
3. Record contacts and the minimum footprint-to-obstacle distance (tape measure plus the logged
   minimum scan range).
Pass: zero contacts and minimum clearance >= 0.05 m in every run. Note obstacles below or above
the scan plane are invisible to this sensor and are out of the test.

## T-STOP: termination
1. Room fully enclosed and reachable: run to completion. Expect status COMPLETE, robot
   stationary within 5 s, map saved.
2. Room with a pocket blocked by an obstacle (unreachable): expect termination with the pocket
   reported unexplored.
3. Force a navigation failure (block the only route): expect ABORTED, never COMPLETE.
4. Compute T-COV on run 1 to detect premature termination.
Pass: as stated in requirements.md section 5.

## T-LOC: localization quality
1. Measure three wall lengths with a tape; compare with the saved map (cell-edge to cell-edge).
2. Mark the start pose; end the run back at the mark; measure position and heading error.
3. From the log, compute the fraction of scans with a successful TF lookup and the `/scan` rate.
Pass: wall error <= 5 % or 5 cm (larger), return error <= 0.10 m and 5 degrees, TF success
>= 99 %, scan rate near nominal.

## T-BAT: battery runtime
1. Full charge; record voltage. Fit an inline current meter or logger.
2. Run the autonomous mission (or a scripted equivalent load) continuously.
3. Log supply voltage and current; record time to end of run or cut-off, and any Pi undervoltage
   warning (kernel log).
Pass: >= 30 minutes with no brown-out. Report the margin. A short pre-check: estimated runtime
= usable capacity (Ah) / average current (A) must exceed 0.5 h with margin before the full test.
