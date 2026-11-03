# Requirements: Autonomous LiDAR Mapping Robot

Status: requirements baseline derived from the PDR/project brief. Implementation status is tracked separately in `CURRENT_STATE.md`.

This document does not claim that physical acceptance criteria have been met. Proposed numerical definitions still require team ratification where marked.

## 1. Purpose

A battery-powered differential-drive robot that autonomously explores and maps an unknown indoor room with a 2D LiDAR, avoids stationary obstacles, and stops by itself.

## 2. PDR success criteria

| ID | Requirement | Verification |
|---|---|---|
| R-01 | After the initial start command, the robot operates autonomously with no further human steering. | T-AUTO |
| R-02 | The robot generates a two-dimensional map of the room. | T-MAP |
| R-03 | The robot can begin with no preexisting map. | T-NOMAP |
| R-04 | The robot maps at least 90% of accessible floor area. | T-COV |
| R-05 | The robot detects and avoids stationary obstacles encountered during normal operation. | T-OBS |
| R-06 | The robot operates continuously for at least 30 minutes on battery power. | T-BAT |
| R-07 | When no significant reachable unexplored area remains, the robot stops automatically. | T-STOP |

## 3. System constraints and hardware

- Raspberry Pi 4 (4 GB)
- RPLIDAR C1
- Arduino Uno
- Cytron MDD10A
- two 131:1 DC gearmotors
- 12 V battery supply
- regulated 5 V Pi supply
- Wi-Fi
- flat, single-level indoor environment
- major obstacles expected to be stationary

Responsibility split:
- Pi: LiDAR, SLAM, Nav2, exploration
- Arduino/MDD10A: motor control and motor-side safety

Camera/ultrasonic sensing is not part of the committed sensing design.

## 4. Out of scope

From the PDR:
- custom SLAM implementation
- custom low-level navigation/path-planning implementation
- stairs, rough terrain, or multi-floor environments
- object recognition/manipulation
- product remote teleoperation

The repository's custom frontier explorer is a supervisory **goal-selection policy** on top of SLAM Toolbox and Nav2. Team/instructor confirmation of this scope interpretation remains HQ-15.

3D Gaussian Splatting is a future extension only.

## 5. Proposed measurable acceptance definitions

These definitions make the PDR criteria testable. They are **proposed** until ratified (HQ-14).

| Test | Proposed definition/pass criterion |
|---|---|
| T-AUTO | After one start command, a full physical run completes with zero steering/operator inputs except emergency stop/disarm. |
| T-NOMAP | Mapping starts from no loaded prior map. |
| T-COV | Accessible area = ground-truth free cells reachable from the start after footprint inflation. Coverage = accessible ground-truth cells represented as known-free in the final map. Pass >= 0.90 for the agreed trial set. |
| T-OBS | Zero contacts with stationary obstacles intersecting the LiDAR scan plane; minimum agreed footprint clearance maintained. |
| T-STOP | No significant reachable unexplored area remains for the agreed persistence interval; robot stops and reports completion. Unreachable pockets must not prevent termination. |
| T-LOC | Map dimensions/return-to-start/TF/scan-rate quality meet agreed limits. |
| T-BAT | Continuous physical operation >= 30 minutes with no Pi brownout/undervoltage failure. |
| T-WD | Physical motor outputs stop within the agreed command-TTL/link-timeout margin and remain safely stopped until re-armed. |

Detailed procedures are in `test_procedures.md`.

### Development completion heuristic vs. T-STOP

The current simulator explorer uses:
- minimum cluster size 5 cells
- 5 stable completion checks
- maximum known-map growth 5 cells/check
- temporary failed-goal suppression

That is a development heuristic, not proof that formal T-STOP has passed. Align the final implementation with the ratified T-STOP definition, or update the ratified definition before final testing.

## 6. Safety requirements

| ID | Requirement |
|---|---|
| S-01 | Motors are disabled by default in software and the Arduino boots DISARMED. |
| S-02 | Arduino zeros outputs on command TTL expiry and link timeout. |
| S-03 | System provides emergency stop/manual stop-disarm capability. |
| S-04 | Speed/duty are bounded and independently enforced at the motor controller. |
| S-05 | Navigation uses conservative obstacle clearance and a scan-based collision layer/monitor. |
| S-06 | Motors are never powered from Pi/Arduino GPIO. |
| S-07 | Voltage/current limits are verified before powered motor testing. |
| S-08 | First powered motion is wheels-up, then low-speed in a controlled clear area. |
| S-09 | Incomplete/mock configuration is never used to generate real hardware-dependent output. |

## 7. Software/development requirements

- Ubuntu 24.04 ARM64 and ROS 2 Jazzy are the working Pi stack.
- Use existing RPLIDAR driver, SLAM Toolbox, Nav2, standard ROS messages, and TF.
- Core interfaces include `/scan`, `/cmd_vel`, `/odom`, `/map`, `/tf`, `/tf_static`.
- TF chain: `map -> odom -> base_link -> laser`.
- Unknown physical values are not guessed.
- Do not assume motor encoders exist.
- Keep heavyweight visualization off the Pi when practical; Foxglove is the current remote visualization method.
- Separate pure algorithm tests from ROS/hardware acceptance.

## 8. Current traceability

Automated/software evidence exists for:

| Area | Evidence |
|---|---|
| protocol framing/checksum/recovery | protocol unit tests |
| DISARMED/TTL/link timeout | mock MCU tests |
| host fail-safe/gating | bridge tests |
| kinematics/duty caps | kinematics tests |
| config/mock refusal | config tests |
| TF structural rules | frame tests |
| coverage computation tooling | metrics tests |
| simulator motion/collision | simulator tests |
| frontier reachability/clustering/goals/ranking | exploration tests |
| end-to-end simulated mapping/navigation/exploration | manual Raspberry Pi runtime exercises |

Documented software baseline: 334 unit tests passing.

## 9. Acceptance status

No final physical acceptance criterion R-01 through R-07 is currently claimed as passed.

Simulation demonstrates intended software architecture and behavior, but it does not replace formal physical acceptance testing.
