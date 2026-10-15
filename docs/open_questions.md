# Open hardware and requirements questions

Status: DRAFT for team confirmation. Nothing here has been checked against the physical robot.
"Blocks" says what software work must wait for the answer. Proposed defaults are suggestions, not
decisions.

## A. Inconsistencies found in the PDR

| ID | Finding | Source | Proposal |
|---|---|---|---|
| PDR-1 | The block diagram shows the Pi sending "Motor Command" straight to the MDD10A. The Arduino Uno is listed as the motor-control part elsewhere and is absent from the diagram. | slide 6 vs slides 3 and 9 | Confirm Pi -> USB serial -> Arduino -> MDD10A -> motors (HQ-01). |
| PDR-2 | The Arduino Uno is specified for a 7-12 V input, while the supply is a 12 V battery system. A charged "12 V" pack can exceed 12 V. | slide 9 | Decide how the Uno is powered (HQ-06). |
| PDR-3 | Coverage target is "at least 90 %" on one slide and "> 90 %" on another. | slides 9, 13 | Use "at least 90 %" (the success-criteria slide) unless the team says otherwise. |
| PDR-4 | "Remote teleoperation" is listed as out of scope, yet the project brief asks for a manual-override mechanism. | slide 4 | Treat manual override as stop/disarm/pause only. Any drive-by-hand tool is a bring-up aid, not a product feature (HQ-09). |
| PDR-5 | "Significant" unexplored area, "accessible floor area", "normal operation" and the test environments are not defined. | slides 3, 9 | Adopt the measurable definitions in requirements.md section 5, after team review (HQ-14). |

## B. Hardware questions

| ID | Question | Why it matters | Blocks | Proposed default |
|---|---|---|---|---|
| HQ-01 | Confirm the architecture Pi -> Arduino -> MDD10A -> motors. | Determines who owns PWM/DIR and the watchdog. | Phase 2 firmware and bridge | Arduino as motor controller, USB serial to the Pi. |
| HQ-02 | Do the 131:1 gearmotors have encoders? Exact part number? | Odometry quality, firmware encoder code, `/odom` source. | Odometry, firmware pin map | Treat as **no encoders** until verified. If none, evaluate laser odometry; if budget allows, ask the team about adding encoders (remaining contingency is $83.24). |
| HQ-03 | Gearmotor part number: rated voltage, no-load speed at the output shaft, stall current. | Open-loop speed model, current limits, fuse size. | Kinematics calibration, power budget | None. `drivetrain.output_free_speed_rpm` stays unset. |
| HQ-04 | Chassis measurements: wheel radius, track width, footprint, LiDAR position, height and yaw. | URDF/TF, Nav2 footprint, odometry. | Navigation, TF, odometry | None. Measure; keys are in `robot.toml`. |
| HQ-05 | Battery chemistry, capacity, protection (fuse/BMS), charger. | The 30 minute runtime requirement and safe voltage range. | Runtime test plan, power validation | None. |
| HQ-06 | 5 V regulator current rating, and how the Uno is powered (Pi USB or 12 V into VIN). | Pi 4 plus LiDAR plus Uno load; undervoltage resets. | Power validation | Power the Uno from the Pi's USB port if the regulator has the headroom, since that also carries the serial link. Verify with a meter under load. |
| HQ-07 | Wiring: Uno pins used for MDD10A PWM/DIR, common ground, logic levels. | Firmware pin map, `[arduino_pins]` in config. | Firmware | None. Uno PWM-capable pins are 3, 5, 6, 9, 10, 11; D0/D1 are the USB serial port. |
| HQ-08 | Emergency stop: is there a hardware kill switch on the motor supply, and a fuse? | A software stop cannot protect against a hung Pi or Arduino. | Any powered motor test | Recommend a latching switch plus an inline fuse on the 12 V motor feed, and pull-downs on the PWM inputs. Not in the PDR budget list. |
| HQ-09 | What counts as the "manual override"? | PDR-4. | Safety design | Physical kill switch, a software `/estop`, and disarm. |
| HQ-10 | USB topology: how the C1 adapter board and the Uno enumerate on the Pi (`/dev/ttyUSB*`, `/dev/ttyACM*`). | Stable device names. | Phase 2 | Use udev rules for fixed symlinks. Installing a rule needs sudo on the Pi, so it needs approval. |
| HQ-11 | What is installed on the Pi now (OS, SD card size, Wi-Fi setup)? | Ubuntu 24.04 plus Jazzy is the preferred stack but unverified for this unit. | Phase 2 setup | Ubuntu 24.04 Server ARM64 (no desktop). |
| HQ-12 | RPLIDAR C1 scan direction, zero angle and mounting orientation as published by the driver. | The mock assumes REP-103 (CCW from +x). | Laser TF yaw | Verify with a known obstacle in Phase 2. |

## C. Software and requirements questions

| ID | Question | Notes |
|---|---|---|
| HQ-13 | Is `sllidar_ros2` usable on Jazzy? | Its README lists the RPLIDAR C1 as supported but names older ROS distros (Humble, Foxy, Galactic, Rolling) for installation. Jazzy is unverified. |
| HQ-14 | Ratify the proposed acceptance definitions (coverage ground truth, "significant" frontier size, clearance, drift limits, trial counts). | See requirements.md section 5. |
| HQ-15 | Is a frontier-exploration package plus a thin supervisor acceptable under "no custom navigation from scratch"? | Candidate: `explore_lite` (m-explore-ros2), which targets Humble and newer. Jazzy is unverified. |
| HQ-16 | How is the "initial start command" given? | Proposed: a ROS service call over SSH, or a physical button read by the Arduino. Needs a choice. |
| HQ-17 | Test environment(s): size, obstacle types, surfaces. | A planar LiDAR sees only obstacles that cross its scan plane; glass and very dark or reflective surfaces may behave badly. Test obstacles must cross the scan plane. |
| HQ-18 | Battery runtime test load profile and start/stop criteria. | requirements.md section 5 proposes one. |

## D. Schedule note

As read from the Gantt image on slide 11 (marked tentative, revised 10/01/2026): hardware assembly
runs in weeks 8-10, "Software Pi Implementation" in week 11, and the CDR is on 3-Nov. Phase 2
hardware bring-up cannot start until HQ-01 to HQ-08 are answered, so those are the critical path.
