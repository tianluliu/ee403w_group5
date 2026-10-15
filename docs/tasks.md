# Tasks

Status legend: `[x]` done and verified locally, `[ ]` not done. "Approval" marks work that needs the
user's explicit go-ahead because it touches hardware, the Pi, or installs software.

Test counts are from `scripts/test.sh` (`python3 -m unittest discover -s tests/unit -t tests/unit`)
on the MacBook Air (Python 3.14.8, arm64): **307 tests, 307 passed, 0 failed, 0 skipped.**
Python 3.12 / Ubuntu ARM64 has **not** been run (syntax-parse only). ROS 2 code has **not** been
built or run: the ROS integration tests are UNVERIFIED and skip on the Mac.

## Phase 1: Requirements, architecture, mock foundation (software complete; review pending)

Done and verified on the Mac:
- [x] Read the PDR (all 14 slides, including the block diagram and Gantt images)
- [x] Hardware config with unknowns unset, plus a separate mock config; loader with validation,
      per-scope completeness check and mock refusal (`config.py`)
- [x] Serial protocol v1 framing: 64-byte limit enforced identically in encode/decode/assembler,
      strict checksum, fragmentation and resync (`protocol.py`)
- [x] Injectable clocks and an inclusive-boundary `Deadline` (`clock.py`)
- [x] Differential-drive kinematics with limits and duty cap (`kinematics.py`)
- [x] Mock Arduino: DISARMED default, ARM/DISARM/STOP, command TTL, link timeout (`mock_mcu.py`)
- [x] Minimal 2D world and mock LiDAR (`world.py`, `mock_lidar.py`)
- [x] Host-side bridge and fault-injecting loopback transport; no serial code (`bridge.py`,
      `transport.py`): motors-disabled gate, fail-safe STOP paths, mock-config refusal on real
      transports, retry policy, sequence handling
- [x] TF-chain validator (`frames.py`) and coverage metric (`metrics.py`)
- [x] ROS 2 Jazzy package skeletons: `robot_base_bridge` (heartbeat node), `robot_mock` (mock
      `/scan` node), `robot_bringup` (`heartbeat.launch.py`, `mock.launch.py`). Static structure
      is tested; **never built or run**
- [x] `tests/ros2_integration/` (UNVERIFIED; skips without rclpy)
- [x] Scripts: `test.sh`, `check_env.sh`, `deploy.sh` (dry-run default), `remote.sh`;
      `pi.env.example`, `.gitignore` (scripts tested via `--help`/`--show-command`/validation only)
- [x] Documents: requirements, design, tasks, interfaces, open_questions, test_procedures,
      dev_workflow, future_3dgs, README
- [x] Mutation check of the safety-relevant behaviour in a temp copy: 37 of 37 deliberate faults
      caught by the suite (plus 28 of 28 in the previous iteration and 11 of 11 before that)

Not done in Phase 1 (each needs authorization):
- [ ] Run the unit suite on the Pi under Python 3.12 (Approval: connects to the Pi)
- [ ] Build and run the ROS 2 packages and integration tests on a Jazzy machine (Approval: install)
- [x] Version control: the repository was initialised and the first commit was made by the user
      (not by me). Everything added since that commit is uncommitted; I have not committed anything
- [ ] Choose a license (placeholder `TODO` in every `package.xml`/`setup.py`) and a real maintainer
      contact (placeholder `team@example.invalid`)

Team decisions needed before Phase 2: open questions HQ-01 to HQ-08 (docs/open_questions.md).

## Phase 2: Motor and sensor integration (Approval for every hardware step)

- [ ] Resolve HQ-01 to HQ-12; fill `robot.toml`; `require_complete("firmware")` must pass
- [ ] Verify power: regulator rating, battery, fuse, kill switch, common ground, logic levels
- [ ] Arduino firmware from protocol v1 (Approval: flashing). Reproduce section 3.4 of interfaces.md
- [ ] Serial transport (the only missing piece of the bridge) and the `/cmd_vel` bridge node;
      test with wheels elevated; verify T-WD
- [ ] Install Ubuntu 24.04 / ROS 2 Jazzy / `sllidar_ros2` on the Pi (Approval: installs); verify scans (T-LID)
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
