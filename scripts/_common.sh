#!/usr/bin/env bash
# Shared helpers. Source this file; do not execute it.
# Compatible with macOS bash 3.2 and Ubuntu bash 5.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

# Print a command line with shell quoting, without running it.
print_cmd() {
  printf '%q ' "$@"
  printf '\n'
}

# Load Pi connection settings. The file is PARSED, never executed: only the four keys below
# are accepted, and every value is validated so it cannot smuggle ssh/rsync options or shell
# syntax into the commands built from it.
load_pi_env() {
  local file="${PI_ENV_FILE:-$REPO_ROOT/scripts/pi.env}"
  [ -f "$file" ] || die "missing $file (copy scripts/pi.env.example to scripts/pi.env and edit it)"

  PI_HOST=""
  PI_USER=""
  PI_REMOTE_DIR=""
  PI_SSH_PORT="22"

  local line key value
  local line_pattern='^(PI_HOST|PI_USER|PI_REMOTE_DIR|PI_SSH_PORT)=(.*)$'
  while IFS= read -r line || [ -n "$line" ]; do
    case "$line" in
      '' | '#'*) continue ;;
    esac
    if [[ $line =~ $line_pattern ]]; then
      key="${BASH_REMATCH[1]}"
      value="${BASH_REMATCH[2]}"
      value="${value%\"}"
      value="${value#\"}"
      printf -v "$key" '%s' "$value"
    else
      die "unsupported line in $file (only PI_HOST, PI_USER, PI_REMOTE_DIR, PI_SSH_PORT are allowed): $line"
    fi
  done < "$file"

  local host_pattern='^[A-Za-z0-9][A-Za-z0-9._-]*$'
  local user_pattern='^[a-z_][a-z0-9_-]*$'
  local dir_pattern='^[A-Za-z0-9./][A-Za-z0-9._/-]*$'
  local port_pattern='^[0-9]{1,5}$'
  [[ $PI_HOST =~ $host_pattern ]] || die "PI_HOST is missing or contains unsupported characters"
  [[ $PI_USER =~ $user_pattern ]] || die "PI_USER is missing or contains unsupported characters"
  [[ $PI_REMOTE_DIR =~ $dir_pattern ]] || die "PI_REMOTE_DIR is missing or contains unsupported characters"
  case "$PI_REMOTE_DIR" in
    *..*) die "PI_REMOTE_DIR must not contain '..'" ;;
  esac
  [[ $PI_SSH_PORT =~ $port_pattern ]] || die "PI_SSH_PORT must be a number"
  if [ "$PI_SSH_PORT" -lt 1 ] || [ "$PI_SSH_PORT" -gt 65535 ]; then
    die "PI_SSH_PORT must be within 1..65535"
  fi
}
