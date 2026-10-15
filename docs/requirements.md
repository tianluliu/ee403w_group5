# Requirements: Autonomous LiDAR Mapping Robot

Status: DRAFT for team confirmation. Phase 1 (requirements, architecture, mock foundation) only.

**Source of truth:** `PDR Presentation.pdf` (14 slides, read in full; slide 6 and slide 11 are
images and were viewed directly). This document does not change the PDR's scope, hardware or
success criteria. Items that come from the project brief and not from the PDR are labelled
**Brief**. Numeric thresholds that the PDR leaves undefined are labelled **PROPOSED** and need team
ratification (open question HQ-14).

**Nothing below has been verified on hardware.** No acceptance criterion is claimed as met.

## 1. Purpose (PDR slides 2-3)

A battery-powered, differential-drive robot that autonomously explores and maps an unknown
indoor room with a 2D LiDAR, avoids stationary obstacles, and stops by itself, so that people
do not need to enter unknown or hazardous spaces. LiDAR is chosen over camera and ultrasonic
sensing for direct distance measurement and operation in low light (slides 2, 10).

## 2. PDR success criteria (slide 9)

| ID | Requirement (EARS form) | PDR source | Verification |
|---|---|---|---|
| R-01 | WHEN the operator issues the initial start command, THE robot SHALL operate autonomously with no further human steering. | s9, s8, s13 | T-AUTO (section 5) |
| R-02 | THE robot SHALL generate a two-dimensional map of the room. | s9 | T-MAP |
| R-03 | THE robot SHALL be able to begin operation with no preexisting map of the environment. | s9 | T-NOMAP |
| R-04 | THE robot SHALL map at least 90 % of the accessible floor area. | s9 (s13 writes "> 90 %", see PDR-3) | T-COV |
| R-05 | THE robot SHALL detect and avoid stationary obstacles encountered during normal operation. | s9 | T-OBS |
| R-06 | THE robot SHALL operate continuously for at least 30 minutes on battery power. | s9 | T-BAT |
| R-07 | WHEN no significant reachable unexplored area remains, THE robot SHALL stop automatically. | s9, s3 | T-STOP |

## 3. PDR system I/O, constraints and hardware

| ID | Requirement | PDR source |
|---|---|---|
| R-10 | Inputs: LiDAR distance measurements, an initial start command, battery power. | s8 |
| R-11 | Outputs: left/right motor-control commands, robot movement, estimated robot position, a 2D room map, mapping/exploration completion status. | s8 |
| R-12 | THE robot SHALL operate only in flat, single-level indoor environments with stationary major obstacles. | s13, s4 |
| R-13 | Confirmed hardware: Raspberry Pi 4 (4 GB), RPLIDAR C1 (360 degrees), Arduino Uno (7-12 V input), Cytron MDD10A dual-channel 10 A driver, two 131:1 DC gearmotors, 12 V battery supply, 5 V regulated supply for the Pi, Wi-Fi. | s3, s9 |
| R-14 | Roles: the Pi performs SLAM, navigation and exploration; the Arduino Uno with the MDD10A performs motor control. | s3, s5 |
| R-15 | Budget: $496 total; estimated parts $412.76; $83.24 contingency. Hardware beyond the confirmed list needs team approval. | s12 |
| R-16 | Camera or ultrasonic sensing is not part of the committed design. | s10 (trade-off slide) |

## 4. Out of scope

From the PDR (slide 4): custom SLAM, mapping or navigation software written from scratch;
stairs, rough terrain or multi-floor environments; object recognition, manipulation, and remote
teleoperation.

From the brief: 3D Gaussian Splatting is a documented future extension only (no implementation,
no hardware purchase without team approval).

## 5. Proposed measurable definitions (PROPOSED, need ratification)

These make the PDR criteria testable. They are not agreed values.

| Test | Definition and pass criterion |
|---|---|
| T-AUTO | After one start command, a full run completes with zero operator inputs other than stop/disarm. Logged. |
| T-NOMAP | Mapping starts from an empty map; no map file is loaded by any launch file. Checked by inspecting the launch configuration and the first `/map` message. |
| T-COV | **Accessible area** = free cells of a ground-truth map that a robot of the configured footprint can reach from the start cell (obstacles inflated by `body.footprint_radius_m`). **Coverage** = accessible cells that are known-free in the robot's saved map, divided by all accessible cells. Maps are compared in the same frame (origin = start pose). Pass: coverage >= 0.90 in every run of the trial set. Ground truth: tape-measured floor plan, or a manually driven reference map. |
| T-OBS | Stationary obstacles that cross the LiDAR scan plane (a planar LiDAR cannot see anything above or below it). Pass: zero contacts, and minimum footprint-to-obstacle clearance >= 0.05 m, over the trial set. Clearance measured from a logged scan plus tape reference. |
| T-STOP | **Significant unexplored area** = a reachable frontier of at least 0.5 m in length (about 10 cells at 0.05 m resolution), reachable meaning Nav2 can plan to it. Exploration is complete when none exists for three consecutive evaluations at least 5 s apart. Pass: within 5 s of completion the robot is stationary, the map is saved, status = COMPLETE; in a room with an unreachable pocket it still terminates and reports it; a failure or timeout ends as ABORTED, never COMPLETE. |
| T-LOC | Localization quality: (a) wall lengths in the saved map within 5 % or 5 cm (larger of the two) of tape measurements; (b) final pose within 0.10 m and 5 degrees of the marked start after a return-to-start; (c) at least 99 % of scans have a successful TF lookup; (d) `/scan` sustained near its nominal rate. |
| T-BAT | From a full charge, one continuous autonomous run lasts at least 30 minutes with no brown-out or undervoltage warning on the Pi. Log supply voltage and average current. Report the margin over 30 minutes. |
| T-WD | (Phase 2, wheels elevated) The motors reach zero output within command TTL + 50 ms of the last command, and within link timeout + 50 ms of the cable being unplugged, then stay stopped until re-armed. |
| Trial set | At least 3 distinct environments x 3 runs each. All runs must pass. |

## 6. Safety requirements (Brief)

| ID | Requirement |
|---|---|
| S-01 | Motors SHALL be disabled by default in software (`safety.motors_enabled = false`) and the Arduino SHALL boot DISARMED. |
| S-02 | THE Arduino SHALL zero the motors when a command's TTL expires and when no valid frame arrives within the link timeout. |
| S-03 | THE system SHALL provide an emergency stop and a manual-override path (stop/disarm; see PDR-4). |
| S-04 | Speeds and duty SHALL be bounded by configuration (bring-up: 0.15 m/s, 0.6 rad/s, 500 permille duty cap) and re-enforced independently on the Arduino. |
| S-05 | Obstacle clearances SHALL be conservative; a scan-based collision monitor sits in front of the motor bridge. |
| S-06 | Motors SHALL never be powered from Pi or Arduino GPIO; the MDD10A is fed from the battery. |
| S-07 | Voltage and current limits of every connection SHALL be validated before powering motors (open questions HQ-03, HQ-05, HQ-06, HQ-07). |
| S-08 | First motion tests are done with wheels off the ground, then in a controlled clear area. |
| S-09 | No hardware-dependent firmware or launch parameters are generated from incomplete or mock configuration. |

## 7. Software and development requirements (Brief)

| ID | Requirement |
|---|---|
| A-01 | Preferred stack: Ubuntu 24.04 ARM64 and ROS 2 Jazzy on the Pi, subject to verification (HQ-11, HQ-13). |
| A-02 | Use existing packages: Slamtec RPLIDAR driver, SLAM Toolbox, Nav2, standard messages and TF. Do not implement SLAM or navigation from scratch (also PDR s4). |
| A-03 | ROS interfaces: `/scan`, `/cmd_vel`, `/odom`, `/map`, `/tf`, `/tf_static`; TF chain `map -> odom -> base_link -> laser` (docs/interfaces.md). |
| A-04 | Hardware-dependent values are in configuration files; unknown values are reported, never guessed. |
| A-05 | Do not assume motor encoders exist. |
| D-01 | The development machine is a Mac: Bash/Zsh scripts, no ROS 2 assumed on macOS, no RViz2 or Gazebo required on the Mac, Docker only if clearly beneficial. |
| D-02 | Platform-independent tests (stdlib only, run on macOS and Ubuntu ARM64) are separate from ROS 2 integration tests (run on the Pi). |
| D-03 | Deployment and remote testing use SSH. |
| D-04 | Keep the Pi light: no full graphical stack on the Pi; heavier visualization on the development machine. |
| D-05 | No physical motor actuation during initial software testing. |

## 8. Traceability to automated tests (current state)

Only software behaviour is covered. No acceptance criterion R-01 to R-07 is yet verifiable.

| Requirement | Automated checks that exist today (`tests/unit`) |
|---|---|
| S-01, S-02 | `test_mock_mcu` (boot state, TTL, link timeout, late frame, fail-safe, fuzz of invariants); `test_config` (motors disabled in both configs); `test_bridge` (no arm/drive byte is written while disabled; fault-injection fuzz) |
| S-03 | `test_bridge` (`stop`/`disarm` always available and never raise). The physical kill switch and `/estop` are not built (HQ-08, HQ-09). |
| A-03 | `test_frames` (required chain, one parent, one publisher, static laser mount); `test_ros_package_structure` (static metadata only; ROS runtime UNVERIFIED) |
| T-COV tooling | `test_metrics` (coverage and accessible-area computation on hand-checked grids). The test procedure itself has not been run. |
| S-04 | `test_kinematics` (limits, duty cap, curvature preserved); `test_mock_mcu` (MCU-side cap) |
| S-09, A-04, A-05 | `test_config` (incomplete values reported, mock refused); `test_kinematics`, `test_world_and_lidar` (missing hardware values refused) |
| Protocol | `test_protocol` (framing, checksum, 64-byte limit, fragmentation, recovery) |
| Timing | `test_clock` (FakeClock, inclusive deadline boundary) |
