from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from pathlib import Path

PATIENT_TO_ARG = {"김서울": "1", "박인천": "2", "서수원": "3"}
AMR_TO_SCRIPT = {"AMR-01": "04_run_ocr_mission_1.sh", "AMR-02": "04_run_ocr_mission_2.sh"}


class MissionRuntime:
    """GUI에서 실제 OCR mission shell을 직접 실행하고 stdout 상태를 추적한다."""

    def __init__(self, root: str | Path, service):
        self.root = Path(root).resolve()
        self.service = service
        self._lock = threading.RLock()
        self._procs = {}
        self._status = {
            amr: {
                "running": False,
                "pid": None,
                "patient": None,
                "started_at": None,
                "returncode": None,
                "last_log": None,
                "last_log_at": None,
                "error": None,
            }
            for amr in AMR_TO_SCRIPT
        }

    def start(self, amr_name: str, patient_name: str):
        if amr_name not in AMR_TO_SCRIPT:
            raise RuntimeError(f"지원하지 않는 AMR입니다: {amr_name}")
        patient_arg = PATIENT_TO_ARG.get(str(patient_name).strip())
        if not patient_arg:
            raise RuntimeError(f"지원하지 않는 환자입니다: {patient_name}")
        with self._lock:
            existing = self._procs.get(amr_name)
            if existing and existing.poll() is None:
                raise RuntimeError(f"{amr_name} 실제 미션이 이미 실행 중입니다.")

            script = self.root / AMR_TO_SCRIPT[amr_name]
            if not script.exists():
                raise RuntimeError(f"실제 미션 스크립트를 찾을 수 없습니다: {script.name}")
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            proc = subprocess.Popen(
                [str(script), patient_arg],
                cwd=str(self.root),
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            self._procs[amr_name] = proc
            st = self._status[amr_name]
            st.update({
                "running": True,
                "pid": proc.pid,
                "patient": patient_name,
                "started_at": time.time(),
                "returncode": None,
                "last_log": None,
                "last_log_at": None,
                "error": None,
            })
            threading.Thread(target=self._reader, args=(amr_name, proc), daemon=True).start()
            # 중복 환자 lock/환경 오류처럼 시작 직후 종료되는 실패를 GUI에 즉시 돌려준다.
            time.sleep(0.35)
            rc = proc.poll()
            if rc is not None and rc != 0:
                detail = self._status[amr_name].get("error") or self._status[amr_name].get("last_log") or f"returncode={rc}"
                raise RuntimeError(f"{amr_name} 실제 미션 프로세스가 시작 직후 종료되었습니다. returncode={rc}. {detail}")
            return proc.pid

    def _reader(self, amr_name, proc):
        tail = []
        try:
            assert proc.stdout is not None
            for raw in proc.stdout:
                line = raw.rstrip("\n")
                if not line:
                    continue
                tail.append(line)
                tail = tail[-30:]
                with self._lock:
                    st = self._status[amr_name]
                    st["last_log"] = line
                    st["last_log_at"] = time.time()
                try:
                    self.service.apply_mission_log(amr_name, line)
                except Exception as exc:
                    with self._lock:
                        self._status[amr_name]["error"] = f"GUI 상태 반영 실패: {exc}"
            rc = proc.wait()
        except Exception as exc:
            rc = proc.poll()
            with self._lock:
                self._status[amr_name]["error"] = str(exc)
        with self._lock:
            st = self._status[amr_name]
            st["running"] = False
            st["returncode"] = rc
            if rc not in (0, None):
                st["error"] = st.get("error") or ("\n".join(tail[-8:]) if tail else f"returncode={rc}")
        try:
            self.service.mission_process_finished(amr_name, rc, tail[-8:])
        except Exception:
            pass

    def status(self):
        with self._lock:
            return {name: dict(value) for name, value in self._status.items()}

    def stop_all(self):
        with self._lock:
            items = list(self._procs.items())
        for _, proc in items:
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except Exception:
                    pass
