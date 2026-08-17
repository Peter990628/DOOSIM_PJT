#!/usr/bin/env bash
# Safe Isaac Sim storage preflight helpers.
# This file NEVER repairs a filesystem. It only performs real read/write/execute probes.

isaac_storage_last_error=""

_isaac_fail() {
  isaac_storage_last_error="$1"
  return 1
}

isaac_probe_root() {
  local root="${1:-}"
  local py="$root/python.sh"
  local mount_target="" probe="" out=""
  isaac_storage_last_error=""

  [[ -n "$root" ]] || _isaac_fail "empty Isaac Sim root"
  [[ -f "$py" ]] || _isaac_fail "python.sh not found: $py"

  # Metadata can remain cached even when the filesystem is broken, so do a real data read.
  if ! dd if="$py" of=/dev/null bs=64K count=1 status=none 2>/dev/null; then
    _isaac_fail "real read failed (I/O): $py"
    return 1
  fi
  if ! bash -n "$py" >/dev/null 2>&1; then
    _isaac_fail "python.sh could not be fully read/parsed: $py"
    return 1
  fi

  # For the user's loop-mounted external Isaac volume, require an actual write too.
  # This catches the exact state where findmnt says rw but ext4 is already I/O-dead.
  if [[ "$root" == /mnt/isaac45/* || "$root" == /mnt/isaac45 ]]; then
    mount_target="$(findmnt -T "$py" -n -o TARGET 2>/dev/null | head -n1 || true)"
    [[ -n "$mount_target" ]] || {
      _isaac_fail "no mounted filesystem contains $py"
      return 1
    }
    probe="$mount_target/.isaac_storage_probe_${UID}_$$"
    if ! ( : > "$probe" ) 2>/dev/null; then
      _isaac_fail "write probe failed although mount may report rw: $mount_target"
      return 1
    fi
    rm -f "$probe" 2>/dev/null || true
  fi

  # Execute the bundled interpreter. This catches unreadable libraries beyond python.sh itself.
  if ! out="$(timeout 35 "$py" -c 'print("ISAAC_STORAGE_PYTHON_OK")' 2>&1)"; then
    _isaac_fail "Isaac python execution failed: ${out//$'\n'/ }"
    return 1
  fi
  if [[ "$out" != *"ISAAC_STORAGE_PYTHON_OK"* ]]; then
    _isaac_fail "Isaac python ran but health token was not observed"
    return 1
  fi
  return 0
}

isaac_select_root() {
  local explicit="${ISAAC_SIM_DIR:-}"
  local candidate

  if [[ -n "$explicit" ]]; then
    if isaac_probe_root "$explicit"; then
      printf '%s\n' "$explicit"
      return 0
    fi
    echo "[ISAAC STORAGE FAIL] explicit ISAAC_SIM_DIR=$explicit" >&2
    echo "[REASON] $isaac_storage_last_error" >&2
    return 1
  fi

  for candidate in \
    /home/peter-msi/isaacsim-5.1.0 \
    /mnt/isaac45/isaacsim_5.1
  do
    [[ -e "$candidate/python.sh" ]] || continue
    if isaac_probe_root "$candidate"; then
      printf '%s\n' "$candidate"
      return 0
    fi
    echo "[ISAAC STORAGE SKIP] $candidate" >&2
    echo "[REASON] $isaac_storage_last_error" >&2
  done
  return 1
}
