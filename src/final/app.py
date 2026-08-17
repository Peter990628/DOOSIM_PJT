from __future__ import annotations

import atexit
from uuid import uuid4

from flask import Flask, jsonify, render_template, request

from db import init_db
from ros_bridge import RosManager
from service import GuiError, HospitalService


def create_app(*, start_ros=True, service_instance=None, ros_manager=None):
    flask_app = Flask(__name__)
    init_db(reset=True)

    hospital_service = service_instance or HospitalService()
    manager = ros_manager or RosManager(hospital_service)
    hospital_service.set_navigator(manager)
    server_session = uuid4().hex
    flask_app.extensions["hospital_service"] = hospital_service
    flask_app.extensions["ros_manager"] = manager
    flask_app.extensions["server_session"] = server_session

    def current_state():
        state = hospital_service.state()
        state["server_session"] = server_session
        return state

    if start_ros:
        manager.start()
        atexit.register(manager.stop)

    @flask_app.get("/")
    def index():
        return render_template("index.html")

    @flask_app.get("/api/state")
    def api_state():
        return jsonify(current_state())

    @flask_app.post("/api/command")
    def api_command():
        data = request.get_json(silent=True)
        if not isinstance(data, dict):
            return jsonify({"ok": False, "error": "JSON 형식의 명령이 필요합니다."}), 400

        action = data.get("action")

        def required(name):
            value = data.get(name)
            if value is None or value == "":
                raise GuiError(f"필수 항목이 없습니다: {name}")
            return value

        def integer(name):
            value = required(name)
            if isinstance(value, bool):
                raise GuiError(f"{name}은 정수여야 합니다.")
            try:
                return int(value)
            except (TypeError, ValueError) as exc:
                raise GuiError(f"{name}은 정수여야 합니다.") from exc

        def text(name):
            value = required(name)
            if not isinstance(value, str):
                raise GuiError(f"{name}은 문자열이어야 합니다.")
            value = value.strip()
            if not value:
                raise GuiError(f"필수 항목이 없습니다: {name}")
            return value

        try:
            if not isinstance(action, str):
                raise GuiError("action은 문자열이어야 합니다.")

            # 실제 ROS 2 연동 전용:
            # - MOVE: GUI가 04_run_ocr_mission_1.sh / 2.sh를 실제 프로세스로 실행
            # - RETURN: /amr1|amr2/inspection/complete Trigger 서비스 호출
            # - RX: /amr1/world_pose, /amr2/world_pose
            # 웹 좌표 디버그 이동은 지원하지 않습니다.
            if not getattr(manager, "is_scenario_mode", lambda: False)():
                raise GuiError("실제 ROS 2 연동 모드에서만 명령을 사용할 수 있습니다. run_ros_gui.sh로 실행하세요.")

            if action in {"start_mri_mission", "start_return"} and not manager.status().get("connected"):
                raise GuiError("ROS 2 GUI 노드가 연결되지 않아 명령을 발행할 수 없습니다.")

            if action == "start_mri_mission":
                amr_name = text("amr")
                bed_id = integer("bed_id")
                floor = integer("floor")
                room = text("room")
                hospital_service.start_mri_mission(amr_name, bed_id, floor, room)
                bed = next((item for item in current_state()["beds"] if item["id"] == bed_id), None)
                if not bed:
                    raise GuiError("선택한 환자 정보를 찾을 수 없습니다.")
                patient_name = str(bed.get("patient_name") or bed.get("label") or "").strip()
                try:
                    pid = manager.start_mission(amr_name, patient_name)
                except Exception as exc:
                    hospital_service.abort_mission_start(amr_name, str(exc))
                    raise GuiError(f"실제 미션 실행 실패: {exc}") from exc
                return jsonify({"ok": True, "accepted": True, "mission_pid": pid, "state": current_state()})

            if action == "start_return":
                amr_name = text("amr")
                job = hospital_service.active_job(amr_name) or {}
                if job.get("phase") not in {"waiting_exam", "return_ready"}:
                    raise GuiError("MRI 11m 후진 완료 후 검사 대기 상태에서만 복귀할 수 있습니다.")
                try:
                    message = manager.trigger_return(amr_name)
                except Exception as exc:
                    raise GuiError(f"검사완료/복귀 서비스 호출 실패: {exc}") from exc
                return jsonify({"ok": True, "accepted": True, "message": message, "state": current_state()})

            raise GuiError("지원하지 않는 명령입니다. GUI는 MRI 이동/복귀 명령만 ROS로 발행합니다.")
        except (GuiError, KeyError, TypeError, ValueError, RuntimeError) as exc:
            return jsonify({"ok": False, "error": str(exc), "state": current_state()}), 400

    return flask_app


app = create_app()
service = app.extensions["hospital_service"]
ros = app.extensions["ros_manager"]


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
