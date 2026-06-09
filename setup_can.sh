#!/usr/bin/env bash
set -euo pipefail

# Optional: explicit device override as first arg, e.g. ./setup_can.sh /dev/ttyUSB0
DEVICE1="${1:-}"

# Substring (case-insensitive) used to identify the CAN adapter via udev's ID_MODEL.
# Override if your adapter reports a different model name.
CAN_MODEL_MATCH="${CAN_MODEL_MATCH:-CAN}"

if [ -z "$DEVICE1" ]; then
  if [ ! -d /dev/serial/by-id ]; then
    echo "/dev/serial/by-id does not exist. Are the usbserial/ftdi_sio modules loaded?" >&2
    exit 1
  fi

  echo "Auto-detecting CAN USB device (ID_MODEL contains '${CAN_MODEL_MATCH}')..."

  matches=()
  match_details=()

  while IFS= read -r -d '' link; do
    dev_node="$(readlink -f "$link" || true)"
    case "$dev_node" in
      /dev/ttyUSB*) ;;
      *) continue ;;
    esac

    props="$(udevadm info -q property -n "$dev_node" 2>/dev/null || true)"
    model="$(printf '%s\n' "$props" | awk -F= '$1=="ID_MODEL"{print $2; exit}')"
    serial_short="$(printf '%s\n' "$props" | awk -F= '$1=="ID_SERIAL_SHORT"{print $2; exit}')"

    if printf '%s' "$model" | grep -qi -- "$CAN_MODEL_MATCH"; then
      already=false
      for u in "${matches[@]:-}"; do
        if [ "$u" = "$dev_node" ]; then already=true; break; fi
      done
      if [ "$already" = false ]; then
        matches+=("$dev_node")
        match_details+=("by-id=$(basename "$link") serial=${serial_short:-?} model=${model:-?}")
      fi
    fi
  done < <(find /dev/serial/by-id -maxdepth 1 -type l -print0 2>/dev/null)

  count="${#matches[@]}"
  if [ "$count" -eq 0 ]; then
    echo "No CAN device found (no /dev/serial/by-id entry whose ID_MODEL matches '$CAN_MODEL_MATCH')." >&2
    echo "Tip: 'ls /dev/serial/by-id' and 'udevadm info -q property -n /dev/ttyUSBX' to inspect." >&2
    exit 2
  fi
  if [ "$count" -gt 1 ]; then
    echo "Multiple CAN devices matched; pass the desired one explicitly: $0 /dev/ttyUSBX" >&2
    for i in "${!matches[@]}"; do
      echo "  - ${matches[$i]} (${match_details[$i]})" >&2
    done
    exit 2
  fi

  DEVICE1="${matches[0]}"
  echo "Using $DEVICE1  (${match_details[0]})"
fi

echo "loading slcan module"
sudo modprobe slcan
echo "attaching CAN on $DEVICE1"
sudo slcand -o -s8 -t hw -S 3000000 "$DEVICE1" can0
echo "bringing up interface"
sudo ip link set up can0
echo "done"
