#!/usr/bin/env bash
# Give the two RoArms fixed names on the Pi, by the USB port they're plugged into:
#   /dev/roarm_game  (table arm, plays tic-tac-toe)
#   /dev/roarm_ugv   (bucket arm on the UGV)
#
# Why: /dev/ttyUSB0 and /dev/ttyUSB1 can swap after a reboot, and both arms'
# USB chips usually report the same (or no) serial number. The physical port
# is the only reliable difference, so ALWAYS PLUG EACH ARM INTO THE SAME PORT
# (label the ports on the Pi).
#
# Run on the Pi:   bash tools/setup_arm_ports.sh
set -euo pipefail

RULES=/etc/udev/rules.d/99-roarm.rules

find_new_tty() {
  # the ttyUSB/ttyACM device that appeared since the snapshot in $1
  comm -13 <(echo "$1") <(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sort) | head -1
}

ask_arm() {
  local name="$1" label="$2"
  echo
  echo "== $label"
  echo "   1. UNPLUG the USB cable of this arm (leave the other arm as it is)."
  read -rp "   Press Enter when it is unplugged... "
  local before
  before=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sort || true)
  echo "   2. Now PLUG it into the USB port it will always use."
  local dev=""
  for _ in $(seq 1 30); do
    sleep 1
    dev=$(find_new_tty "$before")
    [ -n "$dev" ] && break
  done
  if [ -z "$dev" ]; then
    echo "   No new serial device appeared. Check the cable and try again." >&2
    exit 1
  fi
  local path
  path=$(udevadm info -q property -n "$dev" | sed -n 's/^ID_PATH=//p')
  if [ -z "$path" ]; then
    echo "   Could not read the USB port path of $dev." >&2
    exit 1
  fi
  echo "   Found $dev on port $path"
  echo "SUBSYSTEM==\"tty\", ENV{ID_PATH}==\"$path\", SYMLINK+=\"$name\", MODE=\"0666\"" >> "$TMP"
}

TMP=$(mktemp)
echo "# RoArm fixed names by physical USB port - written by setup_arm_ports.sh" > "$TMP"
ask_arm roarm_game "TABLE ARM (tic-tac-toe)"
ask_arm roarm_ugv  "BUCKET ARM (on the UGV)"

if [ "$(grep -c 'ID_PATH' "$TMP")" -eq 2 ] && \
   [ "$(sed -n 's/.*ID_PATH}=="\([^"]*\)".*/\1/p' "$TMP" | sort -u | wc -l)" -ne 2 ]; then
  echo "Both arms were found on the SAME port. Plug them into different ports and run again." >&2
  exit 1
fi

echo
echo "Writing $RULES (needs sudo):"
cat "$TMP"
sudo cp "$TMP" "$RULES"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
rm -f "$TMP"
sleep 1
echo
ls -l /dev/roarm_game /dev/roarm_ugv 2>/dev/null || echo "Links not visible yet: unplug and replug both arms once."
echo "Done. Use /dev/roarm_game and /dev/roarm_ugv from now on."
