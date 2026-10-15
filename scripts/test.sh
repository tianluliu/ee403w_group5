#!/usr/bin/env bash
# Run the test suites. Works on macOS and on the Pi; needs only python3 (3.11+).
set -euo pipefail
# shellcheck source=scripts/_common.sh
. "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

usage() {
  cat <<'EOF'
Usage: scripts/test.sh [unit|ros|all] [-v]

  unit   platform-independent tests in tests/unit (stdlib only). Default.
  ros    ROS 2 integration tests in tests/ros2_integration. Needs a sourced ROS 2 Jazzy
         workspace. Without rclpy every test is SKIPPED; a skip is NOT a pass, and these
         tests are UNVERIFIED until they have run green on a ROS 2 machine.
  all    unit, then ros
  -v     verbose output
EOF
}

mode="unit"
vflag=""
for arg in "$@"; do
  case "$arg" in
    unit | ros | all) mode="$arg" ;;
    -v | --verbose) vflag="-v" ;;
    -h | --help) usage; exit 0 ;;
    *) usage >&2; die "unknown option: $arg" ;;
  esac
done

cd "$REPO_ROOT"
export PYTHONDONTWRITEBYTECODE=1

run_unit() {
  echo "== unit tests (tests/unit) =="
  # shellcheck disable=SC2086
  python3 -m unittest discover -s tests/unit -t tests/unit $vflag
}

run_ros() {
  echo "== ROS 2 integration tests (tests/ros2_integration) - UNVERIFIED until run on a ROS 2 machine =="
  # shellcheck disable=SC2086
  python3 -m unittest discover -s tests/ros2_integration -t tests/ros2_integration $vflag
}

case "$mode" in
  unit) run_unit ;;
  ros) run_ros ;;
  all) run_unit; run_ros ;;
esac
