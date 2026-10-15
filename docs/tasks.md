# Tasks

Status legend: `[x]` done and verified locally, `[ ]` not done. "Approval" marks work that needs the
user's explicit go-ahead because it touches hardware, the Pi, or installs software.

Test counts below are from `python3 -m unittest discover -s tests/unit -t tests/unit` on the
MacBook Air (Python 3.14.8, arm64). Python 3.12 / Ubuntu ARM64 has **not** been run.

## Phase 1: Requirements and architecture (in progress)

Done:
- [x] Read the PDR (all 14 slides, including the block diagram and Gantt images)
- [x] Hardware configuration with unknowns left unset, plus a separate mock config (`config/`)
- [x] Config loader: validation, unknown-key rejection, per-scope completeness, mock refusal
- [x] Serial protocol v1 framing (64-byte limit, strict checksum, fragmentation, resync)
- [x] Injectable clocks and an inclusive-boundary `Deadline`
- [x] Differential-drive kinematics with limits and duty cap (`kinematics.py`)
- [x] Mock Arduino with DISARMED default, ARM/DISARM/STOP, command TTL, link timeout (`mock_mcu.py`)
- [x] Minimal 2D world and mock LiDAR with LaserScan-shaped output (`world.py`, `mock_lidar.py`)
- [x] Unit tests for all of the above: 178 passing, 0 failed, 0 skipped
- [x] Mutation check of the safety behaviours (deliberate faults in a temp copy): 28 of 28 caught
- [x] requirements.md, design.md, tasks.md, interfaces.md, open_questions.md

Remaining Phase 1 (each needs authorization to start):
- [ ] Host-side bridge class with a transport interface and a loopback transport to the mock MCU
      (no serial port code yet)
- [ ] TF-chain validator and a coverage-metric helper (pure Python) with tests
- [ ] ROS 2 package skeletons, heartbeat node, mock LaserScan publisher, launch files (cannot be run
      on the Mac; syntax check only until tested on the Pi)
- [ ] `tests/ros2_integration/` that skips itself when `rclpy` is absent
- [ ] Mac scripts: `scripts/test.sh`, environment check, SSH `deploy.sh` (dry-run default),
      `remote.sh`; `README.md`, `.gitignore`; `git init` if the team wants version control here
- [ ] `docs/test_procedures.md` (step-by-step versions of requirements.md section 5) and
      `docs/dev_workflow.md` (SSH deployment and remote testing instructions)
- [ ] `docs/future_3dgs.md` (design-only note for the optional extension; nothing implemented)
- [ ] Run the unit suite on the Pi under Python 3.12 (Approval: connects to the Pi)

Team decisions needed before Phase 2: open questions HQ-01 to HQ-08 (docs/open_questions.md).

## Phase 2: Motor and sensor integration (Approval for every hardware step)

- [ ] Resolve HQ-01 to HQ-12; fill `robot.toml`; `require_complete("firmware")` must pass
- [ ] Verify power: regulator rating, battery, fuse, kill switch, common ground, logic levels
- [ ] Arduino firmware from protocol v1 (Approval: flashing). Reproduce section 3.4 of interfaces.md
- [ ] Serial transport and bridge node; test with wheels elevated; verify T-WD
- [ ] Install Ubuntu 24.04 / ROS 2 Jazzy / `sllidar_ros2` on the Pi (Approval: installs); verify scans
- [ ] Confirm encoders or choose the odometry option (design.md section 7)

## Phase 3: Mapping
- [ ] Validate TF and odometry; integrate SLAM Toolbox; save a room map; repeatable experiments (T-LOC)

## Phase 4: Autonomous navigation
- [ ] Integrate Nav2 and the collision monitor; verify clearance and recovery; reach known goals (T-OBS)

## Phase 5: Autonomous exploration
- [ ] Frontier exploration with an existing package; handle unreachable frontiers; termination (T-STOP); evaluate T-COV

## Phase 6: Final validation
- [ ] Coverage, obstacle avoidance, termination, 30 minute runtime (T-BAT), reproducible demo

Not scheduled: 3D Gaussian Splatting (outside the PDR scope; needs team approval and separate hardware).
