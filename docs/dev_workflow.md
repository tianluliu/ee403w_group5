# Development Workflow

Current target: Raspberry Pi 4 running Ubuntu Server 24.04 ARM64 and ROS 2 Jazzy. Repository path used in development: `~/ee403w_group5`.

## 1. Git workflow

Keep `main` stable.

Start substantial work:

```bash
cd ~/ee403w_group5
git switch main
git pull --ff-only origin main
git switch -c <purpose-branch>
```

Before committing:

```bash
./scripts/test.sh
git diff --check
git status
```

Stage intentional files only:

```bash
git add <files>
git commit -m "<concise message>"
git push -u origin <purpose-branch>
```

After validation, merge into `main` and delete the short-lived branch.

## 2. Unit tests

```bash
cd ~/ee403w_group5
./scripts/test.sh
./scripts/test.sh -v
```

Documented software baseline:

```text
Ran 334 tests
OK
```

Unit tests do not prove physical acceptance.

## 3. ROS 2 build on the Pi

```bash
cd ~/ee403w_group5/ros2_ws
source /opt/ros/jazzy/setup.bash

colcon build   --packages-select robot_mock robot_explorer robot_bringup robot_base_bridge   --symlink-install

source install/setup.bash
```

For a stale Python package install:

```bash
rm -rf build/<package> install/<package>
colcon build --packages-select <package> --symlink-install
source install/setup.bash
```

## 4. Shell environment

Launch files expect `LIDAR_ROBOT_ROOT` to point at the repository. Development has generally sourced `~/.bashrc` before the workspace:

```bash
cd ~/ee403w_group5
source ~/.bashrc
source ros2_ws/install/setup.bash
```

Verify when debugging:

```bash
echo "$LIDAR_ROBOT_ROOT"
python3 -c "import lidar_robot; print(lidar_robot.__file__)"
```

## 5. Run the stack

Use `docs/RUNBOOK.md` for the full simulator/SLAM/Nav2/explorer/Foxglove sequence.

Main launch profiles:
- `sim.launch.py` — simulator only
- `sim_slam.launch.py` — simulator + SLAM Toolbox
- `sim_nav.launch.py` — simulator + SLAM Toolbox + Nav2

## 6. Foxglove

Foxglove is the working remote visualization method.

On the Pi:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
hostname -I
```

Connect the client to:

```text
ws://<PI-IP>:8765
```

For the autonomous simulation, use fixed frame `map`.

## 7. Real LiDAR development

The RPLIDAR C1 has been used successfully on ROS 2 Jazzy.

Prefer a stable `/dev/serial/by-id/...` path instead of `/dev/ttyUSB0`.

C1 settings used in development:
- serial baud 460800
- frame `laser`
- Standard scan mode

Do not run the real driver and a mock publisher on `/scan` simultaneously.

## 8. Optional helper scripts

The repository still contains helper scripts such as `deploy.sh` and `remote.sh`. They are optional; direct Git/SSH development on the Pi is also valid.

Local connection files such as `scripts/pi.env` remain git-ignored.

## 9. What not to commit

Do not commit:
- `ros2_ws/build/`
- `ros2_ws/install/`
- `ros2_ws/log/`
- generated maps
- ROS bags/MCAP/DB3 files
- credentials
- local IP/connection files
- third-party source trees when dependency metadata is sufficient

## 10. Hardware changes

Physical motor/firmware work requires conservative bring-up:
1. verify power and wiring
2. keep wheels off the ground for first motion
3. verify watchdog/stop behavior
4. use low limits
5. only then proceed to floor testing

Never substitute mock physical values for measured hardware values.
