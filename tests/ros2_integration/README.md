# ROS 2 integration tests: UNVERIFIED

These tests have **never been executed**. They need a ROS 2 Jazzy environment (rclpy, the
built `robot_*` packages, and `<repo>/src` on `PYTHONPATH`; see `docs/dev_workflow.md`
section 6).

On the MacBook every test is skipped. A skip is not a pass. Nothing here counts as validated
until it has run green on the Pi or another Jazzy machine.

```bash
scripts/test.sh ros
```

The platform-independent suite (`tests/unit`) is separate and runs anywhere.
