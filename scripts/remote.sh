#!/usr/bin/env bash
# Run a FIXED, read-only/test command on the Raspberry Pi over SSH.
# Only two subcommands exist. Neither starts ROS nodes, launches anything, uses sudo, opens a
# serial port, or sends a motor command. Connection settings: scripts/pi.env.
set -euo pipefail
# shellcheck source=scripts/_common.sh
. "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

usage() {
  cat <<'EOF'
Usage: scripts/remote.sh <info|test> [--show-command]

  info   print CPU architecture, OS release, Python version, and the NAMES of /dev/ttyUSB*
         and /dev/ttyACM* devices (lists directory entries only; opens nothing)
  test   run the platform-independent unit tests on the Pi (python3 -m unittest)
  --show-command   print the ssh command and exit WITHOUT connecting to anything
EOF
}

sub=""
show=0
for arg in "$@"; do
  case "$arg" in
    info | test) sub="$arg" ;;
    --show-command) show=1 ;;
    -h | --help) usage; exit 0 ;;
    *) usage >&2; die "unknown option: $arg" ;;
  esac
done
if [ -z "$sub" ]; then
  usage >&2
  exit 2
fi

load_pi_env

case "$sub" in
  info)
    remote='uname -m; grep -E "^(PRETTY_NAME|VERSION_ID)=" /etc/os-release; python3 --version; ls -1 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "no ttyUSB/ttyACM devices listed"'
    ;;
  test)
    remote="cd '$PI_REMOTE_DIR' && PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests/unit -t tests/unit"
    ;;
esac

cmd=(ssh -p "$PI_SSH_PORT" -o BatchMode=yes -- "$PI_USER@$PI_HOST" "$remote")

if [ "$show" -eq 1 ]; then
  print_cmd "${cmd[@]}"
  exit 0
fi
"${cmd[@]}"
