#!/usr/bin/env bash
set -euo pipefail

# Non-destructive preparation of the user's existing external Isaac layout.
# Seagate NTFS remains read-only; the 300 GiB ext4 image remains read-only.
# NO fsck, ntfsfix, formatting, copying, or relocation is performed here.

NTFS_DEV="${ISAAC_NTFS_DEV:-/dev/sdb1}"
NTFS_MNT="${ISAAC_NTFS_MNT:-/IsaacSimStorage}"
IMAGE="${ISAAC_IMAGE:-/IsaacSimStorage/IsaacSimStorage/isaac45_ext4.img}"
ISAAC_MNT="${ISAAC_MNT:-/mnt/isaac45}"
PY="$ISAAC_MNT/isaacsim_5.1/python.sh"
SIM="$ISAAC_MNT/isaacsim_5.1/isaac-sim.sh"

echo '================================================================='
echo '[EXTERNAL ISAAC45 SAFE PREPARE]'
echo "NTFS_DEV=$NTFS_DEV"
echo "IMAGE=$IMAGE"
echo 'Mode: read-only storage + writable cache overlay at runtime'
echo '================================================================='

[[ -b "$NTFS_DEV" ]] || { echo "[ERROR] external partition not found: $NTFS_DEV"; exit 10; }
sudo mkdir -p "$NTFS_MNT" "$ISAAC_MNT"

# Mount the Seagate volume read-only if needed.
if ! findmnt -rn "$NTFS_MNT" >/dev/null 2>&1; then
  echo '[PREP] mounting Seagate NTFS read-only'
  sudo mount -t ntfs3 -o ro "$NTFS_DEV" "$NTFS_MNT"
fi

[[ -f "$IMAGE" ]] || { echo "[ERROR] 300GiB image not found: $IMAGE"; exit 11; }
SIZE=$(stat -c '%s' "$IMAGE")
echo "[PASS] image found: $SIZE bytes"

# If /mnt/isaac45 is already healthy, retain it.
if findmnt -rn "$ISAAC_MNT" >/dev/null 2>&1 && dd if="$PY" of=/dev/null bs=64K count=1 status=none 2>/dev/null; then
  echo '[PASS] existing /mnt/isaac45 is readable; keeping current loop mount'
else
  if findmnt -rn "$ISAAC_MNT" >/dev/null 2>&1; then
    echo '[PREP] removing stale /mnt/isaac45 mount'
    sudo umount "$ISAAC_MNT"
  fi

  # Detach stale loop devices that point to the old/wrong path or the same image.
  while IFS= read -r loop; do
    [[ -z "$loop" ]] && continue
    echo "[PREP] detaching stale loop: $loop"
    sudo losetup -d "$loop" || true
  done < <(sudo losetup -a | awk -v img="$IMAGE" '
    /isaac45_ext4\.img/ {
      dev=$1; sub(/:$/, "", dev);
      if (index($0, img)==0 || index($0, img)>0) print dev
    }' | sort -u)

  echo '[PREP] attaching correct image read-only'
  LOOP=$(sudo losetup --read-only --find --show "$IMAGE")
  echo "[PREP] LOOP=$LOOP"
  sudo mount -t ext4 -o ro,noload "$LOOP" "$ISAAC_MNT"
fi

findmnt "$NTFS_MNT"
findmnt "$ISAAC_MNT"

dd if="$PY" of=/dev/null bs=64K count=1 status=none 2>/dev/null || { echo '[ERROR] real read failed: python.sh'; exit 12; }
dd if="$SIM" of=/dev/null bs=64K count=1 status=none 2>/dev/null || { echo '[ERROR] real read failed: isaac-sim.sh'; exit 13; }

echo '[PASS] ISAAC PYTHON READ OK'
echo '[PASS] ISAAC SIM READ OK'
echo '[READY] external Isaac storage is usable in read-only mode.'
