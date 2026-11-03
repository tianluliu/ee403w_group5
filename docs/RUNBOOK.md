# Simulation and Foxglove Runbook

## Full autonomous simulation

### Terminal 1 — simulator + SLAM + Nav2
```bash
cd ~/ee403w_group5
source ~/.bashrc
source ros2_ws/install/setup.bash
ros2 launch robot_bringup sim_nav.launch.py
```

### Terminal 2 — Foxglove bridge
```bash
cd ~/ee403w_group5
source ~/.bashrc
source ros2_ws/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Find Pi IP:
```bash
hostname -I
```

Connect Foxglove to:
```text
ws://<PI-IP>:8765
```

### Terminal 3 — autonomous explorer
```bash
cd ~/ee403w_group5
source ~/.bashrc
source ros2_ws/install/setup.bash

ros2 run robot_explorer explorer_node   --ros-args   -p autonomous:=true
```

### Optional completion monitor
```bash
ros2 topic echo /exploration/complete
```

## Foxglove
Use fixed frame:
```text
map
```

Useful topics:
```text
/map
/scan
/sim/world
/sim/path
/exploration/markers
/exploration/goal_preview
```

`/sim/world` is ground-truth geometry. `/map` is SLAM output.

## Build after changes
```bash
cd ~/ee403w_group5/ros2_ws
source /opt/ros/jazzy/setup.bash

colcon build   --packages-select robot_mock robot_explorer robot_bringup   --symlink-install

source install/setup.bash
```

## Diagnostics
```bash
ros2 node list
ros2 topic list
timeout 6 ros2 topic hz /scan
timeout 6 ros2 topic hz /odom
ros2 topic info /map
ros2 run tf2_ros tf2_echo odom base_link
ros2 pkg executables robot_explorer
```

Expected explorer executable:
```text
robot_explorer explorer_node
```

## If `robot_explorer` is not found
```bash
cd ~/ee403w_group5/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon list | grep robot_explorer

rm -rf build/robot_explorer install/robot_explorer
colcon build --packages-select robot_explorer --symlink-install
source install/setup.bash
```

## If `odom` does not exist
Check earlier launch output for a `mock_robot_node` crash. The simulator owns `odom -> base_link`; repeated Nav2 TF complaints can be downstream symptoms.

## Verify complex world
```bash
grep -n "build_.*world"   ~/ee403w_group5/ros2_ws/src/robot_mock/robot_mock/mock_robot_node.py   ~/ee403w_group5/ros2_ws/src/robot_mock/robot_mock/worlds.py
```

## Tests before push
```bash
cd ~/ee403w_group5
./scripts/test.sh
git diff --check
git status
```

Documented baseline:
```text
Ran 334 tests
OK
```

## Git pattern
```bash
git switch main
git pull --ff-only origin main
git switch -c <purpose-branch>
```

After successful work:
```bash
./scripts/test.sh
git diff --check
git add <intentional-files>
git commit -m "<message>"
git push -u origin <purpose-branch>
```

Avoid `git add .` when generated or third-party files may be present.
