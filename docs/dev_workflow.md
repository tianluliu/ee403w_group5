# Development workflow: Mac to Raspberry Pi

Status: DRAFT. Section 1 and 2 are verified on the MacBook. Sections 3 to 5 are written but
**have not been run against a real Pi**. Section 6 (ROS 2) is **UNVERIFIED**. Commands in
sections 3 to 6 are for you to run when you decide to; nothing here is run automatically.

Conventions: Bash/Zsh on macOS. No Windows PowerShell. No ROS 2, RViz2, Gazebo or Docker is
needed on the Mac.

## 1. Check the Mac (read-only)

```bash
scripts/check_env.sh
```

Needs `python3` 3.11 or newer (for `tomllib`), `ssh` and `rsync`. It installs nothing.

## 2. Test on the Mac

```bash
scripts/test.sh          # unit tests: protocol, config, kinematics, mock MCU, bridge, TF, metrics,
                         # ROS package structure (static), script safety
scripts/test.sh -v       # verbose
```

These tests never touch hardware, the network or a serial port.

## 3. One-time SSH setup (you do this; I have not)

```bash
ssh-keygen -t ed25519 -C "robot-dev"          # skip if you already have a key
ssh-copy-id <pi-user>@<pi-host>               # asks for the Pi password once
ssh <pi-user>@<pi-host> true                  # should now work without a password
```

Then create the local connection file (git-ignored, parsed not executed):

```bash
cp scripts/pi.env.example scripts/pi.env
# edit PI_HOST, PI_USER, PI_REMOTE_DIR, PI_SSH_PORT
```

Host, user and directory are validated against a strict character set, so a typo cannot inject
ssh options or shell syntax.

## 4. Deploy

```bash
scripts/deploy.sh --show-command    # print the rsync command; connects to nothing
scripts/deploy.sh                   # DRY RUN: lists what would be copied (opens an SSH connection)
scripts/deploy.sh --apply           # really copy
```

`deploy.sh` never deletes files on the Pi and runs nothing remotely. It skips `.git/`,
`__pycache__/`, `scripts/pi.env` and ROS build output.

## 5. Remote checks

```bash
scripts/remote.sh info              # CPU arch, OS release, Python version, names of ttyUSB*/ttyACM*
scripts/remote.sh test              # run the same unit tests on the Pi (Ubuntu ARM64, Python 3.12)
scripts/remote.sh test --show-command   # print the ssh command without connecting
```

Only these two fixed commands exist. Neither uses sudo, starts ROS, opens a serial port or
sends a motor command. Running the unit tests on the Pi is how Python 3.12 and ARM64
compatibility will finally be confirmed; until you do, they are checked only by syntax parsing.

## 6. ROS 2 on the Pi (UNVERIFIED, Phase 2, needs approval)

Installing ROS 2 Jazzy on the Pi is an install step and needs your approval; follow the official
ROS 2 Jazzy installation guide for Ubuntu 24.04. Afterwards, on the Pi:

```bash
export LIDAR_ROBOT_ROOT=$HOME/lidar-mapping-robot
export PYTHONPATH=$LIDAR_ROBOT_ROOT/src:$PYTHONPATH
source /opt/ros/jazzy/setup.bash
cd $LIDAR_ROBOT_ROOT/ros2_ws && colcon build --symlink-install
source install/setup.bash

ros2 launch robot_bringup heartbeat.launch.py       # heartbeat only
ros2 launch robot_bringup mock.launch.py            # heartbeat + mock /scan + static TF chain
cd $LIDAR_ROBOT_ROOT && scripts/test.sh ros         # ROS 2 integration tests
```

Things to confirm the first time, because they are assumptions today:
- the `static_transform_publisher` named flags (`--x`, `--frame-id`, ...) on Jazzy;
- the package builds with `colcon` and the entry points resolve;
- `ros2 topic echo /scan` and `ros2 run tf2_tools view_frames` show the expected data;
- `scripts/test.sh ros` reports tests as passed, not skipped.

`mock.launch.py` is hard-wired to `config/robot.mock.toml` and publishes fake obstacles. Never
run it on the robot while the real LiDAR driver is up; both would publish `/scan`.

## 7. Visualizing without ROS on the Mac

RViz2 is not required. Candidate (UNVERIFIED, decided in Phase 3): run a WebSocket bridge on the
Pi (such as `foxglove_bridge`) and view it in a Mac application through an SSH tunnel
(`ssh -L 8765:localhost:8765 <pi-user>@<pi-host>`). Keep heavy visualization off the Pi.

## 8. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `error: missing .../scripts/pi.env` | copy `scripts/pi.env.example` first |
| `PI_HOST is missing or contains unsupported characters` | only letters, digits, `.`, `_`, `-`; must start with a letter or digit |
| `Permission denied (publickey)` with BatchMode | run `ssh-copy-id`; the scripts never prompt for passwords |
| `ImportError: lidar_robot is not importable` | `PYTHONPATH` must contain `<repo>/src` |
| `RuntimeError: Set LIDAR_ROBOT_ROOT` | export the repo root before launching |
| ROS tests all skipped | rclpy missing or workspace not sourced; skipped is not passed |
