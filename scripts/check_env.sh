#!/usr/bin/env bash
# Read-only check of the development machine (the Mac). Installs nothing, connects to nothing.
set -uo pipefail

status=0

report_required() {
  if command -v "$1" > /dev/null 2>&1; then
    printf '  ok        %-8s %s\n' "$1" "$(command -v "$1")"
  else
    printf '  MISSING   %-8s (required)\n' "$1"
    status=1
  fi
}

report_optional() {
  if command -v "$1" > /dev/null 2>&1; then
    printf '  present   %-8s %s\n' "$1" "$(command -v "$1")"
  else
    printf '  absent    %-8s (%s)\n' "$1" "$2"
  fi
}

echo "Required for development and deployment:"
report_required python3
report_required ssh
report_required rsync

if command -v python3 > /dev/null 2>&1; then
  if python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 11) else 1)'; then
    printf '  ok        python3 version %s (>= 3.11, needed for tomllib)\n' \
      "$(python3 -c 'import platform; print(platform.python_version())')"
  else
    printf '  TOO OLD   python3 version %s (need >= 3.11)\n' \
      "$(python3 -c 'import platform; print(platform.python_version())')"
    status=1
  fi
  if python3 -c 'import tomllib' > /dev/null 2>&1; then
    echo "  ok        tomllib importable"
  else
    echo "  MISSING   tomllib (Python >= 3.11 needed)"
    status=1
  fi
fi

echo "Optional:"
report_optional git "not needed to run tests; useful for version control"
report_optional docker "not used by this project"
report_optional ros2 "not expected on the Mac; ROS 2 runs on the Pi"
report_optional shellcheck "optional lint for the scripts"

echo "Platform: $(uname -s) $(uname -m)"
if [ "$status" -eq 0 ]; then
  echo "Result: OK"
else
  echo "Result: problems found"
fi
exit "$status"
