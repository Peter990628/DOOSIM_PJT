#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="/mnt/isaac45/isaacsim_5.1/kit/cache"
pkill -f 'isaac_amr_ros_tray_backup_precoupled.py' 2>/dev/null || true
pkill -f 'RUN_BACKUP_2_GUARANTEED_V2_15.sh' 2>/dev/null || true
sleep 1
if mountpoint -q "$TARGET"; then
  sudo umount "$TARGET" || true
fi
echo '[DONE] backup demo processes stopped; external storage itself remains mounted.'
