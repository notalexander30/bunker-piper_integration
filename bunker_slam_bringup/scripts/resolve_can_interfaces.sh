#!/usr/bin/env bash
# Resolve the robot USB-CAN adapters by immutable USB serial, not canN order.

set -Eeuo pipefail
export LC_ALL=C

arm_serial="${PIPER_CAN_USB_SERIAL:-004E002B4148570A20343133}"
bunker_serial="${BUNKER_CAN_USB_SERIAL:-001D00255443570A20393433}"
output="plain"

usage() {
  cat <<'EOF'
Usage: resolve_can_interfaces.sh [--shell]

Finds the PiPER and Bunker SocketCAN interface names by USB-adapter serial.
The canN names may change whenever adapters are moved or reconnected.

Environment overrides:
  PIPER_CAN_USB_SERIAL
  BUNKER_CAN_USB_SERIAL

With --shell, prints safely quoted ARM_CAN_INTERFACE and
BUNKER_CAN_INTERFACE assignments suitable for eval.
EOF
}

case "${1:-}" in
  "") ;;
  --shell) output="shell" ;;
  -h|--help) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac

serial_for_interface() {
  local interface="$1" path
  path="$(readlink -f "/sys/class/net/${interface}/device")"
  while [[ "$path" != / ]]; do
    if [[ -r "$path/serial" ]]; then
      sed -n '1p' "$path/serial"
      return 0
    fi
    path="$(dirname "$path")"
  done
  return 1
}

find_by_serial() {
  local wanted="$1" role="$2" interface serial
  local -a matches=()
  shopt -s nullglob
  for path in /sys/class/net/can*; do
    interface="${path##*/}"
    serial="$(serial_for_interface "$interface" 2>/dev/null || true)"
    if [[ "$serial" == "$wanted" ]]; then
      matches+=("$interface")
    fi
  done
  shopt -u nullglob

  if (( ${#matches[@]} != 1 )); then
    printf 'error: expected exactly one %s USB-CAN adapter with serial %s; found %d\n' \
      "$role" "$wanted" "${#matches[@]}" >&2
    return 1
  fi
  printf '%s\n' "${matches[0]}"
}

arm_interface="$(find_by_serial "$arm_serial" PiPER)"
bunker_interface="$(find_by_serial "$bunker_serial" Bunker)"

if [[ "$arm_interface" == "$bunker_interface" ]]; then
  echo 'error: PiPER and Bunker resolved to the same CAN interface' >&2
  exit 1
fi

if [[ "$output" == shell ]]; then
  printf 'ARM_CAN_INTERFACE=%q\n' "$arm_interface"
  printf 'BUNKER_CAN_INTERFACE=%q\n' "$bunker_interface"
else
  printf 'PiPER  serial=%s interface=%s bitrate=1000000\n' \
    "$arm_serial" "$arm_interface"
  printf 'Bunker serial=%s interface=%s bitrate=500000\n' \
    "$bunker_serial" "$bunker_interface"
fi
