#!/usr/bin/env bash
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:---check}"
YES=0
for a in "$@"; do [[ "$a" == "--yes" ]] && YES=1; done

OUTER_UUID="${ISAAC_OUTER_UUID:-0C54D12F54D11BF0}"
OUTER_MOUNT="${ISAAC_OUTER_MOUNT:-/mnt/isaac_outer}"
INNER_MOUNT="${ISAAC_INNER_MOUNT:-/mnt/isaac45}"
IMAGE_REL="${ISAAC_IMAGE_REL:-IsaacSimStorage/isaac45_ext4.img}"

fail(){ echo "[FAIL] $*" >&2; exit 2; }
section(){ echo; echo "===== $* ====="; }

section 'CURRENT STATE'
findmnt "$INNER_MOUNT" 2>/dev/null || true
findmnt "$OUTER_MOUNT" 2>/dev/null || true
sudo losetup -l 2>/dev/null | grep -F 'isaac45_ext4.img' || true

if [[ "$MODE" == "--check" ]]; then
  "$ROOT/CHECK_ISAAC_STORAGE.sh"
  exit $?
fi
[[ "$MODE" == "--repair" ]] || fail "usage: $0 --check | --repair [--yes]"

cat <<'WARN'
[WARNING]
This repair mode unmounts /mnt/isaac45, detaches the loop device, checks the
backing image, and only then runs e2fsck on the UNMOUNTED inner ext4 image.
It never runs e2fsck if sampled image reads show an I/O error.
WARN
if (( YES == 0 )); then
  read -r -p 'Type REPAIR to continue: ' ans
  [[ "$ans" == "REPAIR" ]] || fail 'cancelled'
fi

section 'STOP ISAAC USING THE INNER VOLUME'
pkill -TERM -f "$INNER_MOUNT/isaacsim_5.1" 2>/dev/null || true
sleep 2
sudo systemctl stop isaac45-automount.service 2>/dev/null || true

section 'UNMOUNT INNER + DETACH STALE LOOPS'
if mountpoint -q "$INNER_MOUNT"; then
  sudo umount "$INNER_MOUNT" || fail "cannot unmount $INNER_MOUNT"
fi
while read -r loop back; do
  [[ -n "$loop" ]] || continue
  if [[ "$back" == *isaac45_ext4.img* ]]; then
    echo "[DETACH] $loop <- $back"
    sudo losetup -d "$loop" || fail "cannot detach $loop"
  fi
done < <(sudo losetup -l -n -O NAME,BACK-FILE 2>/dev/null || true)

section 'LOCATE OUTER DEVICE'
DEV="$(blkid -U "$OUTER_UUID" 2>/dev/null || true)"
if [[ -z "$DEV" ]]; then
  DEV="$(findmnt -n -o SOURCE "$OUTER_MOUNT" 2>/dev/null | head -n1 || true)"
fi
[[ -n "$DEV" ]] || fail "cannot find outer device UUID=$OUTER_UUID"
echo "[OUTER DEVICE] $DEV"

section 'REMOUNT OUTER RW'
# V2.4: unmount by DEVICE, not by findmnt's escaped target string.
# A desktop mount such as "/media/.../Seagate Expansion Drive" can be emitted by
# findmnt -r as Seagate\x20Expansion\x20Drive; passing that escaped text to umount
# leaves the NTFS volume mounted and the following mount is rejected as exclusively open.
if findmnt -rn -S "$DEV" >/dev/null 2>&1; then
  echo "[UNMOUNT OUTER DEVICE] $DEV"
  sudo umount "$DEV" || fail "cannot unmount existing outer mount for $DEV"
fi
sudo mkdir -p "$OUTER_MOUNT"
sudo mount -o rw "$DEV" "$OUTER_MOUNT" || fail "cannot mount $DEV rw at $OUTER_MOUNT"
findmnt "$OUTER_MOUNT" || true
probe="$OUTER_MOUNT/.isaac_outer_rw_probe_$$"
sudo touch "$probe" || fail 'outer filesystem write probe failed'
sudo rm -f "$probe"

IMG="$OUTER_MOUNT/$IMAGE_REL"
[[ -f "$IMG" ]] || fail "backing image missing: $IMG"
ls -lh "$IMG"

section 'SAMPLED BACKING-IMAGE READ TEST'
size="$(stat -c '%s' "$IMG")"
(( size > 64*1024*1024 )) || fail "unexpected image size: $size"
# Read 4 MiB at several points. Use MiB offsets to stay aligned and portable.
mb=$((size / 1024 / 1024))
for pct in 0 10 50 85 95; do
  skip=$(( mb * pct / 100 ))
  (( skip > 4 )) && skip=$((skip - 2))
  echo "[READ] ${pct}% (skip=${skip}MiB)"
  if ! sudo dd if="$IMG" of=/dev/null bs=1M skip="$skip" count=4 status=none; then
    fail "backing image I/O failed at ${pct}%; DO NOT fsck. Check USB cable/port/outer drive/NTFS first."
  fi
  echo "[PASS] ${pct}%"
done

section 'ATTACH FRESH LOOP'
LOOPDEV="$(sudo losetup -fP --show "$IMG")" || fail 'losetup failed'
echo "[LOOP] $LOOPDEV"
sudo losetup -l "$LOOPDEV" || true

cleanup_on_fail(){
  if ! mountpoint -q "$INNER_MOUNT" 2>/dev/null; then sudo losetup -d "$LOOPDEV" 2>/dev/null || true; fi
}
trap cleanup_on_fail EXIT

section 'READ-ONLY EXT4 CHECK'
TMPLOG="$(mktemp)"
set +e
sudo e2fsck -f -n "$LOOPDEV" 2>&1 | tee "$TMPLOG"
fsck_rc=${PIPESTATUS[0]}
set -e
if grep -Eqi 'Input/output error|I/O error|Buffer I/O|short read' "$TMPLOG"; then
  rm -f "$TMPLOG"
  fail 'e2fsck encountered storage I/O; repair aborted before modifying ext4'
fi
rm -f "$TMPLOG"
echo "[CHECK RC] $fsck_rc (nonzero can simply mean filesystem errors were found)"

section 'EXT4 REPAIR'
sudo e2fsck -f -y "$LOOPDEV" || {
  rc=$?
  # e2fsck status 1/2 can still mean corrected/reboot-required; >2 is considered failure here.
  (( rc <= 2 )) || fail "e2fsck repair failed rc=$rc"
}

section 'MOUNT INNER RW'
sudo mkdir -p "$INNER_MOUNT"
sudo mount -o rw "$LOOPDEV" "$INNER_MOUNT" || fail 'inner ext4 mount failed'
trap - EXIT
findmnt "$INNER_MOUNT" || true

section 'FINAL REAL I/O + ISAAC PYTHON TEST'
source "$ROOT/scripts/isaac_storage_guard.sh"
ISAAC_ROOT="$INNER_MOUNT/isaacsim_5.1"
if ! isaac_probe_root "$ISAAC_ROOT"; then
  fail "$isaac_storage_last_error"
fi
echo '[PASS] /mnt/isaac45 real read/write + Isaac python execution'
echo '[READY] Run ./RUN_TRAY_1_ISAAC_TOTAL_360.sh'
