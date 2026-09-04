#!/usr/bin/env bash
# Configure the fixed PiPER and Bunker USB-CAN interfaces used by this robot.

set -Eeo pipefail
export LC_ALL=C

usage() {
  cat <<'EOF'
Usage: configure_can.sh [FRONT_PIPER_CAN_INTERFACE] [BUNKER_CAN_INTERFACE] [REAR_PIPER_CAN_INTERFACE]

Defaults:
  Both interfaces are resolved from their USB-adapter serial numbers.
  Front PiPER: pass can2 explicitly on this robot, at 1,000,000 bit/s
  Rear PiPER:  pass can3 explicitly on this robot, at 1,000,000 bit/s
  Bunker:      serial 001D00255443570A20393433 at   500,000 bit/s

The script validates both interfaces before changing either one. Existing links
already UP at the requested bitrate and in ERROR-ACTIVE state are left running.
Otherwise the selected link is cycled DOWN, configured with restart-ms=100 and
txqueuelen=1000, then brought UP.

Run `sudo -v` before launch when passwordless sudo is not configured. The
script deliberately uses non-interactive sudo so a launch cannot hang at a
hidden password prompt.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -gt 3 ]]; then
  usage >&2
  exit 2
fi

arm_interface="${1:-auto}"
bunker_interface="${2:-auto}"
rear_piper_interface="${3:-}"
arm_bitrate=1000000
bunker_bitrate=500000

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
resolver="${script_dir}/resolve_can_interfaces.sh"
if [[ "$arm_interface" == auto || "$bunker_interface" == auto ]]; then
  if [[ ! -x "$resolver" ]]; then
    echo "error: CAN resolver is missing or not executable: $resolver" >&2
    exit 1
  fi
  resolved="$($resolver --shell)"
  eval "$resolved"
  if [[ "$arm_interface" == auto ]]; then
    arm_interface="$ARM_CAN_INTERFACE"
  fi
  if [[ "$bunker_interface" == auto ]]; then
    bunker_interface="$BUNKER_CAN_INTERFACE"
  fi
fi

printf 'Resolved CAN roles: front PiPER=%s, Bunker=%s\n' \
  "$arm_interface" "$bunker_interface"

if [[ "$arm_interface" == "$bunker_interface" ]] \
    || [[ -n "$rear_piper_interface" \
    && "$rear_piper_interface" == "$bunker_interface" ]]; then
  echo "error: PiPER and Bunker must use distinct CAN interfaces" >&2
  exit 2
fi
if ! command -v ip >/dev/null 2>&1; then
  echo "error: the ip command is required" >&2
  exit 1
fi

validate_interface() {
  local interface="$1"
  local details
  if ! details="$(ip -details link show dev "$interface" 2>&1)"; then
    echo "error: CAN interface '$interface' does not exist: $details" >&2
    return 1
  fi
  if ! grep -q 'link/can' <<<"$details"; then
    echo "error: interface '$interface' is not a CAN link" >&2
    return 1
  fi
}

# Validate the complete target set before making either state change.
validate_interface "$arm_interface"
validate_interface "$bunker_interface"
if [[ -n "$rear_piper_interface" ]]; then
  validate_interface "$rear_piper_interface"
fi

privileged=()
if [[ "$(id -u)" -ne 0 ]]; then
  if ! command -v sudo >/dev/null 2>&1 || ! sudo -n true 2>/dev/null; then
    echo "error: CAN configuration needs root; run 'sudo -v' first" >&2
    exit 1
  fi
  privileged=(sudo -n)
fi

configure_interface() {
  local role="$1"
  local interface="$2"
  local bitrate="$3"
  local details current_bitrate current_can_state

  details="$(ip -details link show dev "$interface")"
  current_bitrate="$(sed -n 's/.* bitrate \([0-9][0-9]*\).*/\1/p' \
    <<<"$details" | head -n 1)"
  current_can_state="$(sed -n 's/.* can state \([A-Z-][A-Z-]*\).*/\1/p' \
    <<<"$details" | head -n 1)"
  if grep -q '<[^>]*UP[^>]*>' <<<"$details" \
      && [[ "$current_bitrate" == "$bitrate" ]] \
      && [[ "$current_can_state" == "ERROR-ACTIVE" ]]; then
    echo "$role CAN $interface is already UP at $bitrate bit/s and ERROR-ACTIVE"
    return
  fi

  echo "Configuring $role CAN $interface at $bitrate bit/s"
  if grep -q '<[^>]*UP[^>]*>' <<<"$details" \
      && [[ "$current_bitrate" == "$bitrate" ]] \
      && [[ -n "$current_can_state" ]]; then
    echo "$role CAN $interface was $current_can_state; cycling the link to clear stale CAN state"
  fi
  "${privileged[@]}" ip link set dev "$interface" down
  "${privileged[@]}" ip link set dev "$interface" type can \
    bitrate "$bitrate" restart-ms 100
  "${privileged[@]}" ip link set dev "$interface" txqueuelen 1000
  "${privileged[@]}" ip link set dev "$interface" up

  details="$(ip -details link show dev "$interface")"
  if ! grep -q '<[^>]*UP[^>]*>' <<<"$details" \
      || ! grep -q " bitrate $bitrate " <<<"$details"; then
    echo "error: $role CAN $interface did not reach the requested state" >&2
    return 1
  fi
  echo "$role CAN $interface is UP at $bitrate bit/s"
}

configure_interface "Front PiPER arm" "$arm_interface" "$arm_bitrate"
if [[ -n "$rear_piper_interface" && "$rear_piper_interface" != "$arm_interface" ]]; then
  configure_interface "Rear PiPER arm" "$rear_piper_interface" "$arm_bitrate"
fi
configure_interface "Bunker" "$bunker_interface" "$bunker_bitrate"
