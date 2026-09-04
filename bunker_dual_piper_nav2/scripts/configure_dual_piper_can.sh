#!/usr/bin/env bash
# Configure the SocketCAN interfaces used by the front and rear PiPER arms.

set -Eeo pipefail
export LC_ALL=C

usage() {
  cat <<'EOF'
Usage: configure_dual_piper_can.sh FRONT_PIPER_CAN REAR_PIPER_CAN

Each selected interface is configured at 1,000,000 bit/s with restart-ms=100
and txqueuelen=1000. If both arms are intentionally on one shared CAN bus, pass
the same interface twice; the script configures it once.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -ne 2 ]]; then
  usage >&2
  exit 2
fi

front_interface="$1"
rear_interface="$2"
bitrate=1000000

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
  local details current_bitrate current_can_state

  validate_interface "$interface"
  details="$(ip -details link show dev "$interface")"
  current_bitrate="$(sed -n 's/.* bitrate \([0-9][0-9]*\).*/\1/p' \
    <<<"$details" | head -n 1)"
  current_can_state="$(sed -n 's/.* can state \([A-Z-][A-Z-]*\).*/\1/p' \
    <<<"$details" | head -n 1)"
  if grep -q '<[^>]*UP[^>]*>' <<<"$details" \
      && [[ "$current_bitrate" == "$bitrate" ]] \
      && [[ "$current_can_state" == "ERROR-ACTIVE" ]]; then
    echo "$role PiPER CAN $interface is already UP at $bitrate bit/s and ERROR-ACTIVE"
    return
  fi

  echo "Configuring $role PiPER CAN $interface at $bitrate bit/s"
  if grep -q '<[^>]*UP[^>]*>' <<<"$details" \
      && [[ "$current_bitrate" == "$bitrate" ]] \
      && [[ -n "$current_can_state" ]]; then
    echo "$role PiPER CAN $interface was $current_can_state; cycling the link"
  fi
  "${privileged[@]}" ip link set dev "$interface" down
  "${privileged[@]}" ip link set dev "$interface" type can \
    bitrate "$bitrate" restart-ms 100
  "${privileged[@]}" ip link set dev "$interface" txqueuelen 1000
  "${privileged[@]}" ip link set dev "$interface" up

  details="$(ip -details link show dev "$interface")"
  if ! grep -q '<[^>]*UP[^>]*>' <<<"$details" \
      || ! grep -q " bitrate $bitrate " <<<"$details"; then
    echo "error: $role PiPER CAN $interface did not reach the requested state" >&2
    return 1
  fi
  echo "$role PiPER CAN $interface is UP at $bitrate bit/s"
}

echo "Dual PiPER CAN roles: front=$front_interface, rear=$rear_interface"
configure_interface "front" "$front_interface"
if [[ "$rear_interface" == "$front_interface" ]]; then
  echo "Rear PiPER shares $rear_interface; leaving the already configured bus up"
else
  configure_interface "rear" "$rear_interface"
fi
