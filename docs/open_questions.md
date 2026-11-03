# Open Questions and Pending Decisions

This file contains only issues that remain unresolved or require physical/team confirmation. Software questions already answered by implementation are recorded first so agents do not reopen them.

## 1. Resolved software facts

These are no longer open:
- Raspberry Pi is running Ubuntu Server 24.04 ARM64 with ROS 2 Jazzy.
- `sllidar_ros2` works with the RPLIDAR C1 on this Pi; live `/scan` has been demonstrated.
- Foxglove bridge/client is a working visualization path.
- Simulation uses SLAM Toolbox.
- Simulation uses Nav2.
- Exploration is implemented as a custom frontier policy; the previously considered external frontier package was not adopted.
- Software architecture assumes Pi -> USB serial -> Arduino -> MDD10A -> motors.
- TF naming is `map -> odom -> base_link -> laser`.

These do not prove the corresponding physical wiring/geometry.

## 2. PDR interpretation items

| ID | Issue | Current interpretation |
|---|---|---|
| PDR-1 | PDR diagram shows Pi -> MDD10A while other slides name the Uno | software architecture uses Pi -> Arduino -> MDD10A; physically verify wiring |
| PDR-2 | Uno 7-12 V input vs nominal-12 V battery | decide actual Uno power path before powered integration |
| PDR-3 | coverage says `>= 90%` in one place and `> 90%` in another | use at least 90% unless team changes it |
| PDR-4 | teleoperation out of scope vs manual override | manual override = stop/disarm/pause, not product teleoperation |
| PDR-5 | “significant”, “accessible”, and test environments undefined | use proposed definitions only after ratification |

## 3. Open hardware questions

| ID | Question | Why it matters |
|---|---|---|
| HQ-02 | Exact motor part number; do motors have encoders? | physical odometry and firmware |
| HQ-03 | Rated voltage, no-load output speed, stall current | calibration and power/fuse sizing |
| HQ-04 | Wheel radius, track width, footprint, body size, LiDAR mount x/y/z/yaw | TF, Nav2, odometry |
| HQ-05 | Battery chemistry/capacity/BMS/fuse/charger | runtime and power safety |
| HQ-06 | 5 V regulator rating and Uno power source | brownout/power safety |
| HQ-07 | Actual Uno-to-MDD10A PWM/DIR pin map, logic levels, common ground | firmware/wiring |
| HQ-08 | Hardware kill switch and fuse implementation | safe powered tests |
| HQ-09 | Final manual-override mechanism | safety/operations |
| HQ-10 | Final USB topology/device naming for C1 and Uno | stable physical launch |
| HQ-12 | Final LiDAR orientation/zero-angle/mount yaw | correct TF and scan geometry |

Do not guess these values in `config/robot.toml`.

## 4. Open requirements/project questions

| ID | Question | Notes |
|---|---|---|
| HQ-14 | Ratify proposed acceptance definitions | see `requirements.md` |
| HQ-15 | Confirm custom frontier **goal-selection policy** is acceptable under “no custom navigation from scratch” | SLAM and low-level navigation remain off-the-shelf |
| HQ-16 | How is the final physical initial start command issued? | ROS service/SSH/physical control still needs decision |
| HQ-17 | What exact final test environments/surfaces/obstacles are used? | obstacles must intersect the 2D scan plane |
| HQ-18 | What exact battery-runtime load profile/start-stop criteria are used? | formal T-BAT needs a repeatable profile |
| HQ-19 | What license and consistent maintainer metadata should ROS packages use? | package metadata still has TODO/placeholder values |

## 5. Physical critical path

Before powered autonomous testing:
1. resolve motor/encoder details
2. measure chassis/LiDAR geometry
3. verify power/wiring/kill switch
4. implement firmware/serial bridge
5. establish physical odometry
6. calibrate and perform wheels-up safety testing

Software tuning can continue in simulation in parallel.
