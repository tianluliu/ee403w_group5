"""Platform-independent core library for the Autonomous LiDAR Mapping Robot.

Pure Python 3.11+ standard library only. Runs unchanged on the Mac development
machine and on the Raspberry Pi (Ubuntu 24.04 ARM64, Python 3.12). It must never
import rclpy or any hardware library at module level; ROS 2 and serial adapters
live in separate packages/modules.
"""

__version__ = "0.1.0"
