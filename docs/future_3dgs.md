# Future extension (design only): 3D Gaussian Splatting

Status: **outside the committed PDR scope. Nothing is implemented and nothing is planned for
Phases 1 to 6.** This note exists so the idea is written down without influencing the PDR design.

## Why 2D LiDAR scans are not enough

The RPLIDAR C1 gives a single horizontal slice of ranges. Gaussian Splatting reconstructs
photorealistic appearance from many overlapping images with accurate camera poses. A 2D scan
has no appearance information and no vertical structure, so it cannot be the input. It can
only supply a coarse planar pose prior.

## What would be needed

| Piece | Notes |
|---|---|
| RGB or RGB-D camera | new hardware; not in the PDR parts list. Needs team approval and must not consume the PDR contingency (the PDR reserves $83.24 for replacements and surprises) without a decision. The PDR trade-off slide notes that cameras need more processing power and a well-lit environment. |
| Camera calibration | intrinsics (checkerboard), distortion model, camera-to-`base_link` extrinsics measured or calibrated against the LiDAR frame |
| Timestamped image collection | record images with ROS time alongside `/scan`, `/odom`, `/tf`, preferably in a rosbag; verify frame timestamps against the pose source |
| Camera poses | estimated offline by structure-from-motion (for example COLMAP) or a visual-inertial pipeline; LiDAR/SLAM poses may serve as priors but are not accurate enough alone |
| Transfer | copy recordings from the Pi to the Mac, then to a GPU workstation (rsync over SSH) |
| Reconstruction | offline 3D Gaussian Splatting training on a GPU workstation, not on the Pi or the MacBook Air |

## Constraints on the main project

- Keep the main stack unchanged: no camera node, no extra load on the Pi 4 during exploration.
- If approved later, treat it as a separate offline data-collection mode run after the mapping
  mission, with its own power and storage budget.
- Any claim about reconstruction quality needs its own acceptance criteria; none exist.
