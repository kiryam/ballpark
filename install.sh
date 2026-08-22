#!/usr/bin/env bash
# Ballpark installer.
#
# Modes:
#   ./install.sh               on the klipper host (moonraker update_manager
#                              runs it this way after clone/update)
#   ./install.sh <ssh-host>    from a workstation: pushes the repo to
#                              ~/ballpark on the host, installs and restarts
#
# Local mode paths can be overridden via env: CONFIG_DIR=... KLIPPER_DIR=...
set -e
cd "$(dirname "$0")"

HOST="${1:-}"
if [ -n "$HOST" ]; then
  ssh "$HOST" 'mkdir -p ~/ballpark'
  scp -q tool_offset_sphere.py tool_offset_sphere.cfg install.sh "$HOST:~/ballpark/"
  ssh "$HOST" 'cd ~/ballpark && ./install.sh && systemctl restart klipper'
  sleep 12
  echo -n "klipper: "
  ssh "$HOST" "curl -s --max-time 6 http://localhost:7125/printer/info" \
    | grep -o '"state":"[a-z]*"' || echo "unknown"
  exit 0
fi

# ---- on-host install ----
# detect the config dir and klipper checkout (root-over-ssh, kiryam-style
# homes, standard layouts); override via CONFIG_DIR= / KLIPPER_DIR=
CONFIG_DIR="${CONFIG_DIR:-}"
if [ -z "$CONFIG_DIR" ]; then
  for d in "$HOME/printer_data/config" "$HOME/klipper_config" /home/*/printer_data/config; do
    [ -d "$d" ] && CONFIG_DIR="$d" && break
  done
fi
KLIPPER_DIR="${KLIPPER_DIR:-}"
if [ -z "$KLIPPER_DIR" ]; then
  for d in "$HOME/klipper" /home/klipper /home/*/klipper; do
    [ -d "$d/klippy/extras" ] && KLIPPER_DIR="$d" && break
  done
fi
if [ -z "$CONFIG_DIR" ] || [ -z "$KLIPPER_DIR" ]; then
  echo "!! klipper layout not found (set CONFIG_DIR=... KLIPPER_DIR=...)" >&2
  exit 1
fi

DEST="$CONFIG_DIR/ballpark"
mkdir -p "$DEST"
cp tool_offset_sphere.py tool_offset_sphere.cfg "$DEST/"
ln -sfn "$DEST/tool_offset_sphere.py" "$KLIPPER_DIR/klippy/extras/tool_offset_sphere.py"
echo "==> plugin installed: $DEST + extras symlink"

# migrate away the pre-flat-layout copy (avoids a duplicate section)
if [ -f "$CONFIG_DIR/tool_offset/tool_offset_sphere.cfg" ]; then
  rm -f "$CONFIG_DIR/tool_offset/tool_offset_sphere.cfg"
  echo "==> migrated: removed old tool_offset/tool_offset_sphere.cfg"
fi

# ensure [include ballpark/*.cfg] in printer.cfg - inserted BEFORE the
# klipper autosave block (#*#), appending after it discards autosave data
PCFG="$CONFIG_DIR/printer.cfg"
if [ -f "$PCFG" ] && ! grep -qE '^\[include ballpark/\*\.cfg\]' "$PCFG"; then
  python3 - "$PCFG" <<'PY'
import sys, shutil, time
p = sys.argv[1]
lines = open(p).read().splitlines(keepends=True)
for i, l in enumerate(lines):
    if l.startswith('#*#'):
        shutil.copy(p, "%s.bak.%s" % (p, time.strftime('%Y%m%d%H%M%S')))
        lines.insert(i, '[include ballpark/*.cfg]\n\n')
        open(p, 'w').writelines(lines)
        print("==> added [include ballpark/*.cfg] before the autosave block (backup saved)")
        break
else:
    shutil.copy(p, "%s.bak.%s" % (p, time.strftime('%Y%m%d%H%M%S')))
    with open(p, 'a') as f:
        f.write('\n[include ballpark/*.cfg]\n')
    print("==> appended [include ballpark/*.cfg] (backup saved)")
PY
fi
echo "==> done. Restart klipper to load (moonraker restarts it automatically)."
