#!/usr/bin/env bash
# Copy the repository to the Raspberry Pi with rsync over SSH.
# DRY RUN BY DEFAULT: nothing is transferred unless --apply is given. Never deletes files on
# the Pi and never runs anything remotely. Connection settings: scripts/pi.env (see
# scripts/pi.env.example). Writing this script does not connect to any host; running it with
# --apply does.
set -euo pipefail
# shellcheck source=scripts/_common.sh
. "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

usage() {
  cat <<'EOF'
Usage: scripts/deploy.sh [--apply] [--show-command]

  (no flags)       rsync --dry-run: list what WOULD be copied (still opens an SSH connection)
  --apply          really copy the files
  --show-command   print the rsync command and exit WITHOUT connecting to anything
EOF
}

apply=0
show=0
for arg in "$@"; do
  case "$arg" in
    --apply) apply=1 ;;
    --show-command) show=1 ;;
    -h | --help) usage; exit 0 ;;
    *) usage >&2; die "unknown option: $arg" ;;
  esac
done

load_pi_env

cmd=(rsync -az --itemize-changes
  --exclude=.git/ --exclude=__pycache__/ --exclude='*.pyc' --exclude=.DS_Store
  --exclude=scripts/pi.env
  --exclude=ros2_ws/build/ --exclude=ros2_ws/install/ --exclude=ros2_ws/log/)
if [ "$apply" -eq 0 ]; then
  cmd+=(--dry-run)
fi
cmd+=(-e "ssh -p $PI_SSH_PORT -o BatchMode=yes" -- "$REPO_ROOT/" "$PI_USER@$PI_HOST:$PI_REMOTE_DIR/")

if [ "$show" -eq 1 ]; then
  print_cmd "${cmd[@]}"
  exit 0
fi

if [ "$apply" -eq 0 ]; then
  echo "Dry run (no files will be copied). Re-run with --apply to deploy."
else
  echo "Deploying to $PI_USER@$PI_HOST:$PI_REMOTE_DIR/"
fi
"${cmd[@]}"
