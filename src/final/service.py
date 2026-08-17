from __future__ import annotations

import threading
from datetime import datetime, timedelta
from functools import wraps

from db import add_event, connect, now_text, row, rows
import map_config as map_config_module
from map_config import (
    FLOORS,
    PRIORITY_RULES,
    TRAFFIC_CORRIDORS,
    nav_point_catalog,
    nearest_nav_point,
)
from pose_config import (
    DEFAULT_ARRIVAL_TOLERANCE_M,
    floor_point_poses,
)


class GuiError(RuntimeError):
    pass


STORAGE_LOCATION = (1, "보관실")
MRI_LOCATION = (2, "MRI실")
MRI_FRONT_LOCATION = (2, "MRI실 앞")
ELEVATOR_1F = (1, "엘리베이터 앞")
ELEVATOR_2F = (2, "엘리베이터 앞")
EXAM_READY_SECONDS = 0

MOVING_PHASES = {
    "moving_to_patient",
    "moving_to_elevator_1f",
    "moving_to_mri",
    "backing_out_after_drop",
    "moving_to_repickup",
    "backing_out_after_pickup",
    "moving_to_elevator_2f",
    "returning_to_ward",
    "returning_to_storage",
}

WAIT_PHASES = {
    "ward_attach_wait",
    "elevator_transfer_to_2f",
    "unloading_wait",
    "boarding_wait",
    "elevator_transfer_to_1f",
    "ward_detach_wait",
}

PHASE_INFO = {
    "moving_to_patient": (2, "보관실 → 환자 병실 이동 중"),
    "ward_docking_ready": (3, "병실 도착 · OCR/ArUco 환자 확인 및 도킹 준비"),
    "ward_attach_wait": (3, "OCR/ArUco 확인 · 침상 도킹 · Lift 상승 후 안정화"),
    "moving_to_elevator_1f": (4, "침상 결합 유지 · 1층 엘리베이터 이동 중"),
    "elevator_transfer_to_2f": (5, "엘리베이터 탑승 · 1층 → 2층 이동 · Map 전환"),
    "moving_to_mri": (6, "2층 엘리베이터 앞 → MRI실 이동 중"),
    "unloading_wait": (7, "PatientTransfer · 환자 침상 → MRI 이송 중"),
    "backing_out_after_drop": (8, "환자 MRI 인계 후 빈 침상과 MRI 검사 대기 위치로 이동 중"),
    "waiting_exam": (8, "MRI 검사 중 · AMR/빈 침상 대기"),
    "return_ready": (8, "검사 완료 · 복귀 명령 대기"),
    "moving_to_repickup": (9, "복귀 명령 수신 · MRI실 재진입 중"),
    "boarding_wait": (9, "PatientTransfer · 환자 MRI → 침상 회수 중"),
    "backing_out_after_pickup": (10, "환자 회수 완료 · 2층 엘리베이터 이동 준비"),
    "moving_to_elevator_2f": (10, "MRI실 → 2층 엘리베이터 이동 중"),
    "elevator_transfer_to_1f": (11, "엘리베이터 탑승 · 2층 → 1층 이동 · Map 전환"),
    "returning_to_ward": (12, "1층 엘리베이터 앞 → 환자 병실 복귀 중"),
    "ward_storage_ready": (12, "침상 원위치 반환 · AMR/침상 언도킹 준비"),
    "ward_detach_wait": (12, "Magnet Unlock · Lift 하강 · 침상 반환 중"),
    "returning_to_storage": (13, "AMR 단독 보관실 복귀 중"),
    "failed_navigation": (0, "이동 실패 · 시나리오/ROS 상태 확인 필요"),
}

ATTACHED_PHASES = {
    "ward_attach_wait",
    "moving_to_elevator_1f",
    "elevator_transfer_to_2f",
    "moving_to_mri",
    "boarding_wait",
    "backing_out_after_pickup",
    "moving_to_elevator_2f",
    "elevator_transfer_to_1f",
    "returning_to_ward",
}

# MRI실 방향 이송으로 간주하는 구간입니다.
# 주요 구역 사이에서 50% 이상 진행한 AMR은 아래 기본 우선순위보다 먼저 처리됩니다.
MRI_BOUND_PHASES = {
    "moving_to_elevator_1f",
    "moving_to_mri",
}

SCENARIO_STATE_DISPLAY = {
    "IDLE": "대기",
    "MOVING": "이동 중",
    "DOCKING": "환자 인식 · 도킹 중",
    "UNDOCKING": "침상 언도킹 중",
    "EXAM": "MRI 검사 중",
    "RETURN_READY": "복귀 대기",
    "TRAFFIC_WAIT": "교행 대기",
    "ERROR": "오류",
}


def synchronized(method):
    @wraps(method)
    def wrapper(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)

    return wrapper


class HospitalService:
    def __init__(self):
        self.navigator = None
        self._lock = threading.RLock()
        # 현재 주요 구역 간 이동 진행률(0.0~1.0).
        # 실제 /world_pose 위치를 기준으로 주요 구역 간 진행 상태를 계산합니다.
        self._route_progress = {}
        # 교행 양보로 빨간 대기 포인트에 빠진 AMR 상태를 유지합니다.
        # 내부 교행 대기 상태 저장소입니다. 지도 위치 자체에는 사용하지 않습니다.
        self._traffic_waiting = {}
        # 웹 시뮬레이션 이동 기능은 제거되었습니다. 레거시 내부 메서드 호환용 저장소만 유지합니다.
        self._debug_positions = {}
        # AMR별 /world_pose의 최신 실제 좌표와 tolerance 판정 결과를 보관합니다.
        # tolerance 안에서는 등록 포인트 중심에 스냅하고, 포인트 사이에서는 raw world_pose를
        # 지도 경로에 연속 투영하여 1초 주기로 이동 마커가 자연스럽게 갱신되도록 합니다.
        self._live_positions = {}
        # 디버깅 맵 버튼용 층별 스냅 기록. 실제 AMR floor/DB/시나리오 상태와 완전히 분리됩니다.
        # 같은 /world_pose x/y를 1층 또는 2층 좌표표에 각각 대입해 화면에서 확인할 수 있습니다.
        self._floor_debug_positions = {1: {}, 2: {}}

    def set_navigator(self, navigator):
        self.navigator = navigator

    @synchronized
    def apply_scenario_status(self, robot_status):
        """Scenario -> GUI 최소 상태를 AMR의 층/표시 상태에 반영합니다.

        세부 phase는 건드리지 않습니다. Scenario가 보낸 floor는 /world_pose를 어떤 층 좌표표로
        해석할지 결정하는 데 사용하고, state는 GUI용 간단 상태 문구로만 반영합니다.
        """
        if not isinstance(robot_status, dict):
            return False
        amr_name = str(robot_status.get("amr") or "").strip()
        if not amr_name:
            return False
        amr = row("SELECT * FROM amrs WHERE name=?", (amr_name,))
        if not amr:
            return False

        state = str(robot_status.get("state") or "").strip().upper()
        status_text = SCENARIO_STATE_DISPLAY.get(state, amr.get("status") or "대기")
        # Scenario status의 floor는 목적지 층을 미리 넣는 구현도 있어 실제 위치가
        # 1층에 있는데 GUI 마커가 2층으로 사라지는 원인이 될 수 있습니다.
        # GUI의 floor/room은 world_pose 좌표 판정과 로컬 엘리베이터 전환 단계만 소유하고,
        # scenario_status에서는 간단 상태 문구만 반영합니다.
        timestamp = now_text()
        with connect() as con:
            con.execute(
                "UPDATE amrs SET status=?, updated_at=? WHERE name=?",
                (status_text, timestamp, amr_name),
            )
        return True

    def _ros_enabled(self):
        return bool(self.navigator and self.navigator.status().get("enabled"))

    def active_job(self, amr_name):
        return row(
            "SELECT * FROM jobs WHERE amr_name=? "
            "AND phase NOT IN ('complete','cancelled') ORDER BY id DESC LIMIT 1",
            (amr_name,),
        )

    def _ensure_destination(self, floor, room):
        if floor not in FLOORS or room not in FLOORS[floor]["pois"]:
            raise GuiError(f"존재하지 않는 목적지입니다: {floor}층 {room}")

    def _job_target(self, job):
        if not job:
            return None
        phase = job["phase"]
        if phase == "failed_navigation":
            phase = job.get("resume_phase")
        if phase == "moving_to_patient":
            return (job.get("origin_floor") or 1, job.get("origin_room") or "병실")
        if phase == "moving_to_elevator_1f":
            return ELEVATOR_1F
        if phase == "moving_to_mri":
            return MRI_LOCATION
        if phase == "backing_out_after_drop":
            return MRI_FRONT_LOCATION
        if phase == "moving_to_repickup":
            return MRI_LOCATION
        if phase == "backing_out_after_pickup":
            return MRI_FRONT_LOCATION
        if phase == "moving_to_elevator_2f":
            return ELEVATOR_2F
        if phase == "returning_to_ward":
            return (job.get("origin_floor") or 1, job.get("origin_room") or "병실")
        if phase == "returning_to_storage":
            return STORAGE_LOCATION
        return None

    def _route_segments(self, floor, from_room, to_room):
        cfg = FLOORS.get(floor)
        if not cfg:
            return set()
        route = cfg.get("route", [])
        try:
            start = route.index(from_room)
            end = route.index(to_room)
        except ValueError:
            return set()
        step = 1 if end > start else -1
        result = set()
        for index in range(start, end, step):
            result.add((route[index], route[index + step]))
        return {tuple(sorted(item)) for item in result}

    def _route_direction(self, floor, from_room, to_room):
        cfg = FLOORS.get(int(floor), {})
        route = cfg.get("route", [])
        try:
            start = route.index(from_room)
            end = route.index(to_room)
        except ValueError:
            return 0
        if end > start:
            return 1
        if end < start:
            return -1
        return 0

    def _ordered_path_infos(self, floor, from_room, to_room):
        """등록된 실제 경로 순서의 물리 포인트 목록을 반환합니다."""
        floor = int(floor)
        cfg = FLOORS.get(floor, {})
        route = cfg.get("route", [])
        try:
            start = route.index(from_room)
            end = route.index(to_room)
        except ValueError:
            return []

        result = []
        start_info = self._point_info(floor, f"poi:{floor}:{from_room}")
        if start_info:
            result.append(start_info)
        if start == end:
            return result

        step = 1 if end > start else -1
        for index in range(start, end, step):
            left = route[index]
            right = route[index + step]
            route_cfg = next(
                (item for item in cfg.get("waypoint_routes", []) if item.get("from") == left and item.get("to") == right),
                None,
            )
            reversed_route = False
            if route_cfg is None:
                route_cfg = next(
                    (item for item in cfg.get("waypoint_routes", []) if item.get("from") == right and item.get("to") == left),
                    None,
                )
                reversed_route = route_cfg is not None
            if route_cfg is None:
                return []
            waypoints = list(route_cfg.get("waypoints", []))
            if reversed_route:
                waypoints.reverse()
            for wp in waypoints:
                info = self._point_info(floor, f"{route_cfg.get('id')}:{wp.get('id')}")
                if info:
                    result.append(info)

        end_info = self._point_info(floor, f"poi:{floor}:{to_room}")
        if end_info:
            result.append(end_info)
        return result

    def _remaining_path_infos(self, planned, position_records):
        path = self._ordered_path_infos(planned["floor"], planned["from_room"], planned["to_room"])
        current = position_records.get(planned["amr"]["name"])
        current_id = current.get("canonical_id") if current else None
        if not current_id:
            return path
        for index, info in enumerate(path):
            if info.get("canonical_id") == current_id:
                return path[index:]
        return path

    def _yield_resolution(self, priority, yielding, wait, position_records):
        """양보 AMR이 현 위치에서 기다릴지, 대기 포인트로 빠질지 상황별로 판단합니다.

        - 동일 방향 추종은 이 함수에 들어오기 전에 교행 충돌에서 제외합니다.
        - 현재 위치가 우선 AMR의 앞으로 지나갈 물리 경로와 겹치지 않으면 그 자리에서 정지합니다.
        - 현재 위치가 우선 AMR의 진행 경로를 실제로 막는 경우에만 빨간 대기 포인트로 이탈합니다.
        """
        yielding_name = yielding["amr"]["name"]
        if yielding_name in self._traffic_waiting:
            return "wait_at_waiting_point", "이미 교행 대기 포인트에 진입해 있어 현재 위치를 유지합니다."

        current = position_records.get(yielding_name)
        priority_path = self._remaining_path_infos(priority, position_records)
        priority_path_ids = {item.get("canonical_id") for item in priority_path if item.get("canonical_id")}
        current_id = current.get("canonical_id") if current else None

        if current_id and current_id not in priority_path_ids:
            return "hold_current", "현재 위치가 우선 AMR의 진행 경로 밖이므로 불필요한 대피 이동 없이 현 위치에서 대기합니다."

        if wait:
            return "divert_to_wait", "현재 위치가 우선 AMR의 진행 경로를 막을 수 있어 가장 가까운 비어 있는 대기 포인트로 이탈합니다."

        return "hold_current", "진행 경로를 비울 필요가 있지만 사용 가능한 대기 포인트가 없어 현 위치에서 정지합니다."

    def _is_mri_bound(self, job):
        phase = job.get("phase")
        if phase == "failed_navigation":
            phase = job.get("resume_phase")
        return phase in MRI_BOUND_PHASES

    def _waiting_point_for_direction(
        self, floor, from_room, to_room, yielding_amr=None, position_records=None
    ):
        """양보 AMR에서 가장 가까운 사용 가능한 빨간 대기 포인트를 고릅니다.

        실제 위치 판정에서는 현재 표시 좌표(display)의
        직선거리를 사용합니다. 현재 위치를 알 수 없는 경우에만 기존 이동 방향 순서를
        fallback으로 사용합니다. 다른 AMR이 점유한 대기 포인트는 후보에서 제외합니다.
        """
        floor = int(floor)
        cfg = FLOORS.get(floor, {})
        points = list(cfg.get("waiting_points", []))
        if not points:
            return None

        # 먼저 점유되지 않은 대기 포인트만 남깁니다.
        available = [
            point
            for point in points
            if not self._occupied_by(
                floor, point.get("display"), exclude_amr=yielding_amr
            )
        ]
        if not available:
            return None

        records = position_records if position_records is not None else self._position_records()
        current = records.get(yielding_amr) if yielding_amr else None
        current_display = current.get("display") if current else None
        if current_display and len(current_display) >= 2:
            cx, cy = float(current_display[0]), float(current_display[1])

            def distance_sq(point):
                display = point.get("display") or ()
                if len(display) < 2:
                    return float("inf")
                dx = float(display[0]) - cx
                dy = float(display[1]) - cy
                return dx * dx + dy * dy

            return min(available, key=distance_sq)

        # 현재 위치 정보가 없는 예외 상황에서는 기존 경로 방향을 fallback으로 사용합니다.
        route = cfg.get("route", [])
        try:
            start = route.index(from_room)
            end = route.index(to_room)
            ordered = available if end >= start else list(reversed(available))
        except ValueError:
            ordered = available
        return ordered[0] if ordered else None

    def _waiting_point_by_id(self, floor, point_id):
        cfg = FLOORS.get(int(floor), {})
        return next(
            (point for point in cfg.get("waiting_points", []) if point.get("id") == point_id),
            None,
        )

    def _home_slot_for(self, amr_name):
        return FLOORS.get(1, {}).get("home_slots", {}).get(amr_name)

    def _canonical_point_id(self, floor, display):
        if display is None or len(display) < 2:
            return None
        return f"{int(floor)}:{float(display[0]):.6f}:{float(display[1]):.6f}"

    def _point_catalog(self, floor):
        floor = int(floor)
        cfg = FLOORS.get(floor, {})
        result = {}
        for room, poi in cfg.get("pois", {}).items():
            result[f"poi:{floor}:{room}"] = {
                "floor": floor,
                "point_key": f"poi:{floor}:{room}",
                "point_id": poi.get("point_id"),
                "room": room,
                "display": list(poi.get("display", ())),
                "label": poi.get("display_name", room),
                "source": "poi",
            }
        for patient in cfg.get("patient_points", []):
            key = f"patient:{patient.get('patient_name')}"
            result[key] = {
                "floor": floor,
                "point_key": key,
                "point_id": patient.get("id"),
                "room": "병실",
                "patient_name": patient.get("patient_name"),
                "display": list(patient.get("display", ())),
                "label": patient.get("display_name", patient.get("patient_name", key)),
                "source": "patient",
            }
        for route in cfg.get("waypoint_routes", []):
            for wp in route.get("waypoints", []):
                key = f"{route.get('id')}:{wp.get('id')}"
                result[key] = {
                    "floor": floor,
                    "point_key": key,
                    "point_id": wp.get("id"),
                    "display": list(wp.get("display", ())),
                    "label": wp.get("id", key),
                    "source": "waypoint",
                }
        for point in cfg.get("waiting_points", []):
            key = f"wait:{floor}:{point.get('id')}"
            result[key] = {
                "floor": floor,
                "point_key": key,
                "point_id": point.get("id"),
                "display": list(point.get("display", ())),
                "label": point.get("display_name", point.get("id", key)),
                "source": "waiting",
            }
        for amr_name, slot in cfg.get("home_slots", {}).items():
            key = f"home:{amr_name}"
            result[key] = {
                "floor": floor,
                "point_key": key,
                "point_id": slot.get("id"),
                "display": list(slot.get("display", ())),
                "label": slot.get("display_name", slot.get("id", key)),
                "source": "home",
                "amr_name": amr_name,
            }
        for item in result.values():
            item["canonical_id"] = self._canonical_point_id(item["floor"], item["display"])
        return result

    def _point_info(self, floor, point_key):
        return self._point_catalog(int(floor)).get(point_key)

    def _update_floor_debug_position(self, amr_name, floor, x, y, yaw, timestamp):
        """같은 world_pose를 지정한 층의 고정 포인트 좌표표로만 판정합니다.

        이 기록은 GUI 디버깅 표시 전용이며 DB의 AMR floor/room, 실제 도착 판정,
        교행/미션 상태에는 영향을 주지 않습니다. 포인트 사이에서는 해당 층에서
        마지막으로 확인된 포인트를 계속 유지합니다.
        """
        floor = int(floor)
        floor_records = self._floor_debug_positions.setdefault(floor, {})
        previous = floor_records.get(amr_name)
        snapped = map_config_module.nearest_nav_point(floor, x, y, amr_name=amr_name)

        if snapped is None:
            if not previous or not previous.get("display"):
                return None
            held = dict(previous)
            held["raw_pose"] = {"x": float(x), "y": float(y), "yaw": float(yaw)}
            held["in_tolerance"] = False
            held["holding_previous_point"] = True
            held["updated_at"] = timestamp
            held["debug_floor_basis"] = floor
            floor_records[amr_name] = held
            return held

        snapped = dict(snapped)
        canonical_id = self._canonical_point_id(floor, snapped.get("display"))
        record = {
            "amr_name": amr_name,
            "floor": floor,
            "display": list(snapped.get("display") or ()),
            "canonical_id": canonical_id,
            "snapped_canonical_id": canonical_id,
            "source": "world_pose_debug",
            "position_source": "world_pose",
            "display_mode": "fixed_point_snap_debug_floor",
            "raw_pose": {"x": float(x), "y": float(y), "yaw": float(yaw)},
            "in_tolerance": True,
            "holding_previous_point": False,
            "debug_floor_basis": floor,
            "snapped_point_key": snapped.get("point_key"),
            "snapped_point_id": snapped.get("point_id"),
            "snapped_source": snapped.get("source"),
            "snapped_label": snapped.get("label"),
            "distance_m": snapped.get("distance_m"),
            "updated_at": timestamp,
        }
        for key in ("point_key", "point_id", "room", "label"):
            if key in snapped:
                record[key] = snapped.get(key)
        floor_records[amr_name] = record
        return record

    def _amr_position_record(self, amr):
        """지도 AMR 위치는 /world_pose로 확정된 이산 포인트만 사용합니다."""
        amr_name = amr["name"]
        live = self._live_positions.get(amr_name)
        if live and live.get("display"):
            result = dict(live)
            result["amr_name"] = amr_name
            return result
        return None

    def _position_records(self, amrs=None):
        amrs = amrs if amrs is not None else rows("SELECT * FROM amrs ORDER BY name")
        records = {}
        for amr in amrs:
            record = self._amr_position_record(amr)
            if record:
                records[amr["name"]] = record
        return records

    def _occupied_by(self, floor, display, exclude_amr=None, amrs=None):
        canonical_id = self._canonical_point_id(floor, display)
        if not canonical_id:
            return None
        for amr_name, record in self._position_records(amrs).items():
            if amr_name == exclude_amr:
                continue
            if record.get("canonical_id") == canonical_id:
                return amr_name
        return None

    def _ensure_display_free(self, amr_name, floor, display, label="포인트", amrs=None):
        occupied_by = self._occupied_by(floor, display, exclude_amr=amr_name, amrs=amrs)
        if occupied_by:
            raise GuiError(
                f"{label}은(는) {occupied_by}이(가) 점유 중입니다. "
                "동일 포인트에는 AMR 1대만 진입할 수 있습니다."
            )

    def _ensure_room_point_free(self, amr_name, floor, room, amrs=None):
        if int(floor) == 1 and room == "보관실":
            info = self._point_info(1, f"home:{amr_name}")
        else:
            info = self._point_info(floor, f"poi:{int(floor)}:{room}")
        if info:
            self._ensure_display_free(amr_name, floor, info["display"], info.get("label", room), amrs=amrs)

    def _waiting_records(self, conflicts):
        active_yields = {item["yielding_amr"] for item in conflicts}
        records = {}
        for amr_name, waiting in self._traffic_waiting.items():
            item = dict(waiting)
            item["can_resume"] = amr_name not in active_yields
            records[amr_name] = item
        return records

    def _choose_traffic_priority(self, first, second, position_records):
        """요청한 층별 교행 규칙으로 우선 AMR을 결정합니다.

        기본은 MRI 방향(1층 병실→엘리베이터, 2층 엘리베이터→MRI) 우선입니다.
        다만 복귀 AMR이 목적지 직전의 지정 경유점까지 이미 진입한 경우에는
        그 AMR을 먼저 병실/엘리베이터 안으로 넣은 뒤 반대 AMR을 진행시킵니다.
        """
        floor = int(first.get("floor") or 0)
        corridor = TRAFFIC_CORRIDORS.get(floor, {})
        commit_ids = set(corridor.get("return_commit_point_ids", []))

        def current_point_id(planned):
            record = position_records.get(planned["amr"]["name"]) or {}
            point_key = str(record.get("point_key") or "")
            for point_id in commit_ids:
                if point_id and point_id in point_key:
                    return point_id
            return record.get("point_id")

        # 복귀 방향(-1) AMR이 목적지 직전 커밋 포인트까지 왔다면 먼저 통과시킵니다.
        for candidate, other in ((first, second), (second, first)):
            if candidate.get("direction") == -1 and current_point_id(candidate) in commit_ids:
                return candidate, other, corridor.get(
                    "return_commit_label", "복귀 AMR 목적지 직전 진입 우선"
                )

        default_direction = int(corridor.get("default_priority_direction", 1))
        if first.get("direction") != second.get("direction"):
            if first.get("direction") == default_direction:
                return first, second, corridor.get("default_priority_label", "MRI 방향 이송 우선")
            if second.get("direction") == default_direction:
                return second, first, corridor.get("default_priority_label", "MRI 방향 이송 우선")

        # 복도 정책으로 가르기 어려운 예외에서는 MRI 방향 여부, 이후 먼저 시작한 임무 순으로 결정합니다.
        if first["mri_bound"] != second["mri_bound"]:
            priority, yielding = (first, second) if first["mri_bound"] else (second, first)
            return priority, yielding, "MRI실 방향 이송 우선"
        priority, yielding = sorted([first, second], key=lambda item: item["job"]["id"])
        return priority, yielding, "동일 조건 · 먼저 시작한 임무 우선"

    def _traffic_conflicts(self, jobs, amrs):
        planned = []
        for job in jobs:
            target = self._job_target(job)
            if not target:
                continue
            amr = next((item for item in amrs if item["name"] == job["amr_name"]), None)
            if not amr:
                continue
            from_floor = target[0] if amr["floor"] != target[0] else amr["floor"]
            from_room = "엘리베이터 앞" if amr["floor"] != target[0] else amr["room"]
            planned.append(
                {
                    "job": job,
                    "amr": amr,
                    "floor": target[0],
                    "from_room": from_room,
                    "to_room": target[1],
                    "segments": self._route_segments(target[0], from_room, target[1]),
                    "direction": self._route_direction(target[0], from_room, target[1]),
                    "mri_bound": self._is_mri_bound(job),
                    "progress_ratio": float(self._route_progress.get(amr["name"], 0.0)),
                }
            )

        conflicts = []
        position_records = self._position_records(amrs)
        for index, first in enumerate(planned):
            for second in planned[index + 1 :]:
                if first["floor"] != second["floor"]:
                    continue
                overlap = first["segments"] & second["segments"]
                if not overlap:
                    continue

                # 같은 방향으로 같은 복도를 사용하는 경우에는 빨간 대기 포인트로 보내지 않습니다.
                # 앞 포인트 단독 점유 규칙이 자연스럽게 차간 간격을 만들므로 뒤 AMR은 현재 포인트에서 추종 대기합니다.
                if first["direction"] and first["direction"] == second["direction"]:
                    continue

                # 이미 서로 지나쳐 남은 물리 경로가 더 이상 겹치지 않으면 충돌을 해제합니다.
                first_remaining = {
                    item.get("canonical_id")
                    for item in self._remaining_path_infos(first, position_records)
                    if item.get("canonical_id")
                }
                second_remaining = {
                    item.get("canonical_id")
                    for item in self._remaining_path_infos(second, position_records)
                    if item.get("canonical_id")
                }
                if first_remaining and second_remaining and not (first_remaining & second_remaining):
                    continue

                priority, yielding, reason = self._choose_traffic_priority(first, second, position_records)
                wait = self._waiting_point_for_direction(
                    yielding["floor"], yielding["from_room"], yielding["to_room"],
                    yielding["amr"]["name"], position_records,
                )
                action, decision_reason = self._yield_resolution(
                    priority, yielding, wait, position_records
                )
                waiting_label = wait.get("display_name") if wait else None
                if action == "divert_to_wait" and waiting_label:
                    action_text = f"{waiting_label}(으)로 이탈 후 대기"
                elif action == "wait_at_waiting_point":
                    action_text = f"{waiting_label or '대기 포인트'}에서 계속 대기"
                else:
                    action_text = "현재 위치에서 정지 대기"

                conflicts.append(
                    {
                        "floor": first["floor"],
                        "priority_amr": priority["amr"]["name"],
                        "yielding_amr": yielding["amr"]["name"],
                        "priority_reason": reason,
                        "priority_progress": round(priority["progress_ratio"], 3),
                        "yielding_progress": round(yielding["progress_ratio"], 3),
                        "recommended_action": action,
                        "decision_reason": decision_reason,
                        "waiting_point": {
                            "id": wait.get("id"),
                            "display_name": wait.get("display_name"),
                        } if wait else None,
                        "yielding_waiting": yielding["amr"]["name"] in self._traffic_waiting,
                        "segments": [" ↔ ".join(segment) for segment in sorted(overlap)],
                        "message": (
                            f"{reason}: {priority['amr']['name']} 우선 · "
                            f"{yielding['amr']['name']}은(는) {action_text}"
                        ),
                    }
                )
        return conflicts

    def _exam_ready(self, job):
        if not job or job.get("phase") != "waiting_exam":
            return False
        updated = job.get("updated_at")
        if not updated:
            return False
        try:
            started = datetime.strptime(updated, "%Y-%m-%d %H:%M:%S")
        except ValueError:
            return False
        return datetime.now() >= started + timedelta(seconds=EXAM_READY_SECONDS)

    @synchronized
    def state(self):
        amrs = rows("SELECT * FROM amrs ORDER BY name")
        beds = rows("SELECT * FROM beds ORDER BY id")
        jobs = rows(
            "SELECT * FROM jobs WHERE phase NOT IN ('complete','cancelled') ORDER BY id"
        )
        enriched_jobs = []
        for job in jobs:
            target = self._job_target(job)
            step, label = PHASE_INFO.get(job["phase"], (0, job["phase"]))
            item = dict(job)
            item["target_floor"] = target[0] if target else None
            item["target_room"] = target[1] if target else None
            item["scenario_step"] = step
            item["phase_label"] = label
            item["bed_attached"] = job["phase"] in ATTACHED_PHASES
            item["auto_wait"] = job["phase"] in WAIT_PHASES
            item["exam_ready"] = self._exam_ready(job)
            item["return_ready"] = job["phase"] == "return_ready"
            enriched_jobs.append(item)

        floors = {}
        for floor, cfg in FLOORS.items():
            catalog = nav_point_catalog(floor)
            catalog_by_id = {}
            for catalog_item in catalog:
                point_id = catalog_item.get("point_id")
                if point_id and point_id not in catalog_by_id and catalog_item.get("nav_pose") is not None:
                    catalog_by_id[point_id] = catalog_item

            configured_points = []
            for point_id, point_pose_cfg in floor_point_poses(floor).items():
                catalog_item = catalog_by_id.get(point_id)
                display = list(catalog_item.get("display", ())) if catalog_item else None
                if not display:
                    display = map_config_module.nav_xy_to_display(
                        floor, point_pose_cfg["x"], point_pose_cfg["y"]
                    )
                if not display:
                    continue
                configured_points.append({
                    "id": point_id,
                    "display_name": (catalog_item.get("label") if catalog_item else point_id),
                    "display": list(display),
                    "nav_xy": [float(point_pose_cfg["x"]), float(point_pose_cfg["y"])],
                    "source": (catalog_item.get("source") if catalog_item else "reference"),
                    "active_route": bool(catalog_item),
                    "arrival_radius_m": float(point_pose_cfg.get("tolerance_m", DEFAULT_ARRIVAL_TOLERANCE_M)),
                })

            floors[str(floor)] = {
                "name": cfg["name"],
                "image": cfg["image"],
                "world_pose_snap_config": {
                    "mode": "continuous_world_pose_with_tolerance_snap",
                    "point_count": len(nav_point_catalog(floor)),
                },
                "configured_points": configured_points,
                "route": cfg.get("route", list(cfg["pois"])),
                "home_slots": {
                    amr_name: {
                        "id": slot.get("id", f"HOME-{amr_name}"),
                        "display": list(slot.get("display", ())),
                        "display_name": slot.get("display_name", amr_name),
                    }
                    for amr_name, slot in cfg.get("home_slots", {}).items()
                },
                "waiting_points": [
                    {
                        "id": point["id"],
                        "display": list(point["display"]),
                        "display_name": point.get("display_name", point["id"]),
                        "segment": point.get("segment", ""),
                        "description": point.get("description", ""),
                        "color": point.get("color", "red"),
                        "nav_xy": (list(point["nav_pose"][:2]) if point.get("nav_pose") is not None else None),
                        "arrival_radius_m": float(point.get("arrival_radius_m", 0.65)),
                    }
                    for point in cfg.get("waiting_points", [])
                ],
                "waypoint_routes": [
                    {
                        "id": wr["id"],
                        "from": wr["from"],
                        "to": wr["to"],
                        "color": wr.get("color", "blue"),
                        "waypoints": [
                            {
                                "id": wp["id"],
                                "display_name": wp.get("display_name", wp["id"]),
                                "display": list(wp["display"]),
                                "nav_xy": (list(wp["nav_pose"][:2]) if wp.get("nav_pose") is not None else None),
                                "arrival_radius_m": float(wp.get("arrival_radius_m", 0.65)),
                            }
                            for wp in wr.get("waypoints", [])
                        ],
                    }
                    for wr in cfg.get("waypoint_routes", [])
                ],
                "pois": {
                    name: {
                        "display": list(poi["display"]),
                        "display_name": poi.get("display_name", name),
                        "nav_xy": (
                            list(poi["nav_pose"][:2])
                            if poi.get("nav_pose") is not None
                            else None
                        ),
                        "amr_only": poi.get("amr_only", False),
                        "mission_destination": poi.get("mission_destination", False),
                        "elevator": poi.get("elevator", False),
                        "storage": poi.get("storage", False),
                        "waiting_area": poi.get("waiting_area", False),
                        "hide_marker": poi.get("hide_marker", False),
                        "point_id": poi.get("point_id"),
                        "arrival_radius_m": float(poi.get("arrival_radius_m", 0.65)),
                    }
                    for name, poi in cfg["pois"].items()
                },
            }

        conflicts = self._traffic_conflicts(jobs, amrs)
        position_records = self._position_records(amrs)
        return {
            "amrs": amrs,
            "beds": beds,
            "jobs": enriched_jobs,
            "events": rows("SELECT * FROM events ORDER BY id DESC LIMIT 60"),
            "floors": floors,
            "traffic": {
                "policy": "MRI 방향 이송 기본 우선 · 복귀 AMR이 목적지 직전 경유점까지 진입한 경우 복귀 우선 · 가장 가까운 빈 대기점 사용 · 모든 포인트 단독 점유",
                "priority_rules": PRIORITY_RULES,
                "route_progress": {
                    amr["name"]: round(float(self._route_progress.get(amr["name"], 0.0)), 3)
                    for amr in amrs
                },
                "conflicts": conflicts,
                "waiting_amrs": self._waiting_records(conflicts),
                "live_positions": {name: dict(position) for name, position in self._live_positions.items()},
                "amr_positions": position_records,
                "debug_floor_positions": {
                    str(floor): {name: dict(position) for name, position in records.items()}
                    for floor, records in self._floor_debug_positions.items()
                },
                "occupied_points": [dict(position) for position in position_records.values()],
            },
            "ros": self.navigator.status() if self.navigator else {"enabled": False},
            "exam_ready_seconds": EXAM_READY_SECONDS,
        }

    def _send_nav(self, amr_name, floor, room):
        # 실제 이동은 patient_transport_manager.py가 전담합니다.
        # GUI는 /world_pose를 수신하고 미션 로그를 상태로 반영할 뿐 Nav2 goal을 별도로 만들지 않습니다.
        return True

    def _start_navigation(self, amr_name, floor, room, *, preserve_debug_position=False):
        if not preserve_debug_position:
            self._route_progress[amr_name] = 0.0
            self._debug_positions.pop(amr_name, None)
        self._traffic_waiting.pop(amr_name, None)
        try:
            self._send_nav(amr_name, floor, room)
        except Exception as exc:
            self.navigation_failed(amr_name, str(exc))
            raise GuiError(f"이동 명령 전송 실패: {exc}") from exc

    @synchronized
    def start_mri_mission(self, amr_name, bed_id, dest_floor, dest_room):
        self._ensure_destination(dest_floor, dest_room)
        if (dest_floor, dest_room) != MRI_LOCATION:
            raise GuiError("현재 시나리오는 2층 MRI실 환자 이송만 지원합니다.")

        amr = row("SELECT * FROM amrs WHERE name=?", (amr_name,))
        bed = row("SELECT * FROM beds WHERE id=?", (bed_id,))
        if not amr or not bed:
            raise GuiError("AMR 또는 환자 정보가 없습니다.")
        if self.active_job(amr_name):
            raise GuiError(f"{amr_name}은 이미 임무 수행 중입니다.")
        if bed["assigned_amr"]:
            raise GuiError("이미 다른 AMR에 배정된 환자입니다.")
        if bed["status"] != "대기":
            raise GuiError("현재 이송할 수 없는 환자 상태입니다.")
        if row(
            "SELECT id FROM jobs WHERE bed_id=? AND phase NOT IN ('complete','cancelled') LIMIT 1",
            (bed_id,),
        ):
            raise GuiError("이미 진행 중인 환자 이송 임무가 있습니다.")

        timestamp = now_text()
        with connect() as con:
            con.execute(
                "INSERT INTO jobs(kind, amr_name, bed_id, dest_floor, dest_room, destination_pending, "
                "phase, origin_floor, origin_room, resume_phase, created_at, updated_at) "
                "VALUES('mri_transport',?,?,?,?,0,?,?,?,?,?,?)",
                (
                    amr_name,
                    bed_id,
                    dest_floor,
                    dest_room,
                    "moving_to_patient",
                    bed["floor"],
                    bed["room"],
                    None,
                    timestamp,
                    timestamp,
                ),
            )
            con.execute(
                "UPDATE beds SET assigned_amr=?, status='AMR 접근 중', updated_at=? WHERE id=?",
                (amr_name, timestamp, bed_id),
            )
            con.execute(
                "UPDATE amrs SET status=?, updated_at=? WHERE name=?",
                (
                    "환자 위치로 이동 중",
                    timestamp,
                    amr_name,
                ),
            )
        add_event(f"{amr_name} · {bed['label']} MRI 이송 시작")
        self._start_navigation(amr_name, bed["floor"], bed["room"])

    @synchronized
    def exam_complete(self, amr_name):
        job = self.active_job(amr_name)
        if not job or job["phase"] != "waiting_exam":
            raise GuiError("MRI 검사 중 단계에서만 검사 완료를 처리할 수 있습니다.")
        if not self._exam_ready(job):
            raise GuiError("아직 검사 진행 중입니다.")
        timestamp = now_text()
        with connect() as con:
            con.execute(
                "UPDATE jobs SET phase='return_ready', updated_at=? WHERE id=?",
                (timestamp, job["id"]),
            )
            con.execute(
                "UPDATE amrs SET status='검사 완료 · 복귀 대기', updated_at=? WHERE name=?",
                (timestamp, amr_name),
            )
        add_event(f"{amr_name} · MRI 검사 완료")

    @synchronized
    def start_return(self, amr_name):
        """복귀 버튼 하나로 검사 종료 확인과 복귀 시작을 함께 처리합니다.

        정상 시나리오에서는 ``waiting_exam`` 단계에서 검사 시간이 충족되면
        별도의 ``exam_complete`` 호출 없이 곧바로 MRI 재진입·환자 회수 단계로 진행합니다.
        기존 ``return_ready`` 상태도 하위 호환을 위해 계속 허용합니다.
        """
        job = self.active_job(amr_name)
        if not job or job["phase"] not in {"waiting_exam", "return_ready"}:
            raise GuiError("복귀를 시작할 수 있는 상태가 아닙니다.")
        if job["phase"] == "waiting_exam" and not self._exam_ready(job):
            raise GuiError("아직 검사 진행 중입니다.")

        timestamp = now_text()
        with connect() as con:
            con.execute(
                "UPDATE jobs SET phase='moving_to_repickup', updated_at=? WHERE id=?",
                (timestamp, job["id"]),
            )
            con.execute(
                "UPDATE amrs SET status='환자 재픽업을 위해 MRI실 재진입 중', updated_at=? WHERE name=?",
                (timestamp, amr_name),
            )
        if job["phase"] == "waiting_exam":
            add_event(f"{amr_name} · MRI 검사 종료 · 복귀 시작")
        else:
            add_event(f"{amr_name} · MRI 재진입 · 환자 회수 시작")
        self._start_navigation(amr_name, *MRI_LOCATION)

    @synchronized
    def advance_wait(self, amr_name, expected_phase=None):
        job = self.active_job(amr_name)
        if not job or job["phase"] not in WAIT_PHASES:
            raise GuiError("자동 대기 후 진행할 단계가 없습니다.")
        if expected_phase and expected_phase != job["phase"]:
            raise GuiError("이미 다음 단계로 진행되었습니다.")

        timestamp = now_text()
        phase = job["phase"]
        next_target = None
        if phase == "ward_attach_wait":
            bed = row("SELECT * FROM beds WHERE id=?", (job["bed_id"],))
            with connect() as con:
                con.execute(
                    "UPDATE jobs SET phase='moving_to_elevator_1f', updated_at=? WHERE id=?",
                    (timestamp, job["id"]),
                )
                con.execute(
                    "UPDATE amrs SET status='환자 자동 확인 · 침상 결합 완료 · 엘리베이터 이동 중', updated_at=? WHERE name=?",
                    (timestamp, amr_name),
                )
                con.execute(
                    "UPDATE beds SET status='도킹됨 · MRI 이송 중', updated_at=? WHERE id=?",
                    (timestamp, job["bed_id"]),
                )
            add_event(f"{amr_name} · {bed['label']} 결합 3초 대기 완료 · 엘리베이터 이동")
            next_target = ELEVATOR_1F

        elif phase == "elevator_transfer_to_2f":
            self._ensure_room_point_free(amr_name, 2, "엘리베이터 앞")
            with connect() as con:
                con.execute(
                    "UPDATE jobs SET phase='moving_to_mri', updated_at=? WHERE id=?",
                    (timestamp, job["id"]),
                )
                con.execute(
                    "UPDATE amrs SET floor=2, room='엘리베이터 앞', status='2층 맵 전환 완료 · MRI실 이동 중', "
                    "x=NULL, y=NULL, yaw=NULL, updated_at=? WHERE name=?",
                    (timestamp, amr_name),
                )
                con.execute(
                    "UPDATE beds SET floor=2, room='엘리베이터 앞', status='도킹됨 · MRI 이송 중', updated_at=? WHERE id=?",
                    (timestamp, job["bed_id"]),
                )
            self._live_positions.pop(amr_name, None)
            add_event(f"{amr_name} · 2층 도착 · MRI실 이동")
            next_target = MRI_LOCATION

        elif phase == "unloading_wait":
            with connect() as con:
                con.execute(
                    "UPDATE jobs SET phase='backing_out_after_drop', updated_at=? WHERE id=?",
                    (timestamp, job["id"]),
                )
                con.execute(
                    "UPDATE amrs SET status='환자 MRI 인계 완료 · 검사 대기 위치 이동 중', updated_at=? WHERE name=?",
                    (timestamp, amr_name),
                )
                con.execute(
                    "UPDATE beds SET status='환자 MRI 인계 완료 · 검사 준비', updated_at=? WHERE id=?",
                    (timestamp, job["bed_id"]),
                )
            add_event(f"{amr_name} · 환자 침상 → MRI 인계 · 검사 대기 위치 이동")
            next_target = MRI_FRONT_LOCATION

        elif phase == "boarding_wait":
            with connect() as con:
                con.execute(
                    "UPDATE jobs SET phase='backing_out_after_pickup', updated_at=? WHERE id=?",
                    (timestamp, job["id"]),
                )
                con.execute(
                    "UPDATE amrs SET status='환자 MRI → 침상 회수 완료 · 2층 엘리베이터 이동 준비', updated_at=? WHERE name=?",
                    (timestamp, amr_name),
                )
                con.execute(
                    "UPDATE beds SET status='환자 회수 완료 · 병실 복귀 중', updated_at=? WHERE id=?",
                    (timestamp, job["bed_id"]),
                )
            add_event(f"{amr_name} · 환자 MRI → 침상 회수 · 병실 복귀 준비")
            next_target = MRI_FRONT_LOCATION

        elif phase == "elevator_transfer_to_1f":
            self._ensure_room_point_free(amr_name, 1, "엘리베이터 앞")
            origin_floor = job.get("origin_floor") or 1
            origin_room = job.get("origin_room") or "병실"
            with connect() as con:
                con.execute(
                    "UPDATE jobs SET phase='returning_to_ward', updated_at=? WHERE id=?",
                    (timestamp, job["id"]),
                )
                con.execute(
                    "UPDATE amrs SET floor=1, room='엘리베이터 앞', status='1층 맵 전환 완료 · 병실 이동 중', "
                    "x=NULL, y=NULL, yaw=NULL, updated_at=? WHERE name=?",
                    (timestamp, amr_name),
                )
                con.execute(
                    "UPDATE beds SET floor=1, room='엘리베이터 앞', status='도킹됨 · 병실 복귀 중', updated_at=? WHERE id=?",
                    (timestamp, job["bed_id"]),
                )
            self._live_positions.pop(amr_name, None)
            add_event(f"{amr_name} · 1층 도착 · 병실 이동")
            next_target = (origin_floor, origin_room)

        elif phase == "ward_detach_wait":
            origin_floor = job.get("origin_floor") or 1
            origin_room = job.get("origin_room") or "병실"
            with connect() as con:
                con.execute(
                    "UPDATE jobs SET phase='returning_to_storage', updated_at=? WHERE id=?",
                    (timestamp, job["id"]),
                )
                con.execute(
                    "UPDATE amrs SET floor=?, room=?, status='환자 병실 복귀 완료 · 보관실 복귀 중', x=NULL, y=NULL, yaw=NULL, updated_at=? WHERE name=?",
                    (origin_floor, origin_room, timestamp, amr_name),
                )
                con.execute(
                    "UPDATE beds SET floor=?, room=?, status='대기', assigned_amr=NULL, updated_at=? WHERE id=?",
                    (origin_floor, origin_room, timestamp, job["bed_id"]),
                )
            add_event(f"{amr_name} · 병실 침상 언도킹/Lift 하강 완료 · 보관실 이동")
            next_target = STORAGE_LOCATION

        if next_target:
            self._start_navigation(amr_name, *next_target)

    def _complete_ward_docking_at_front(self, amr_name, job, timestamp=None):
        """병실 안 진입 후 1F-WARD-CORNER로 빠져나온 순간 도킹 완료로 전환합니다."""
        if not job or job.get("phase") != "ward_docking_ready":
            return False
        timestamp = timestamp or now_text()
        bed = row("SELECT * FROM beds WHERE id=?", (job["bed_id"],))
        if not bed:
            return False
        with connect() as con:
            con.execute(
                "UPDATE jobs SET phase='moving_to_elevator_1f', updated_at=? WHERE id=?",
                (timestamp, job["id"]),
            )
            con.execute(
                "UPDATE amrs SET floor=1, room='병실', status='도킹 완료 · MRI 이송중', updated_at=? WHERE name=?",
                (timestamp, amr_name),
            )
            con.execute(
                "UPDATE beds SET status='도킹됨 · MRI 이송 중', updated_at=? WHERE id=?",
                (timestamp, job["bed_id"]),
            )
        add_event(f"{amr_name} · {bed['label']} 도킹 완료 · MRI 이송 시작")
        self._route_progress[amr_name] = 0.0
        return True

    def _start_storage_return_at_ward_front(self, amr_name, job, timestamp=None):
        """병실에서 AMR/침상 언도킹 후 병실 앞 경유점으로 나오면 보관실 복귀를 시작합니다."""
        if not job or job.get("phase") != "ward_storage_ready":
            return False
        timestamp = timestamp or now_text()
        with connect() as con:
            con.execute(
                "UPDATE jobs SET phase='returning_to_storage', updated_at=? WHERE id=?",
                (timestamp, job["id"]),
            )
            con.execute(
                "UPDATE amrs SET floor=1, room='병실', status='보관실 이동 중', updated_at=? WHERE name=?",
                (timestamp, amr_name),
            )
            con.execute(
                "UPDATE beds SET status='병실 복귀 완료 · AMR 보관실 이동 중', updated_at=? WHERE id=?",
                (timestamp, job["bed_id"]),
            )
        add_event(f"{amr_name} · 병실 앞 경유점 확인 · 보관실 이동 시작")
        self._route_progress[amr_name] = 0.0
        self._start_navigation(amr_name, *STORAGE_LOCATION)
        return True

    @synchronized
    def navigation_arrived(self, amr_name):
        if amr_name in self._traffic_waiting:
            return False
        job = self.active_job(amr_name)
        if not job or job["phase"] not in MOVING_PHASES:
            return False

        target = self._job_target(job)
        if target:
            self._ensure_room_point_free(amr_name, target[0], target[1])
        self._route_progress[amr_name] = 1.0
        self._debug_positions.pop(amr_name, None)
        # ROS 연동에서는 도착 판정 직후에도 마지막 실제 world_pose 마커를 유지합니다.
        # 다음 callback이 올 때까지 POI 중심으로 순간 점프하는 현상을 방지합니다.
        if not self._ros_enabled():
            self._live_positions.pop(amr_name, None)
        timestamp = now_text()
        next_target = None

        if job["phase"] == "moving_to_patient":
            bed = row("SELECT * FROM beds WHERE id=?", (job["bed_id"],))
            # 병실 중심 좌표에서는 아직 침상을 결합하지 않습니다. 실제 AMR이 병실에
            # 진입했다는 것만 확인하고, 병실 앞 경유점으로 다시 나오는 순간을 도킹 완료로 봅니다.
            with connect() as con:
                con.execute(
                    "UPDATE jobs SET phase='ward_docking_ready', updated_at=? WHERE id=?",
                    (timestamp, job["id"]),
                )
                con.execute(
                    "UPDATE amrs SET floor=?, room=?, status='도킹 준비중', x=NULL, y=NULL, yaw=NULL, updated_at=? WHERE name=?",
                    (bed["floor"], bed["room"], timestamp, amr_name),
                )
                con.execute(
                    "UPDATE beds SET status='도킹 준비중', updated_at=? WHERE id=?",
                    (timestamp, job["bed_id"]),
                )
            add_event(f"{amr_name} · {bed['label']} 병실 도착 · 도킹 준비중")

        elif job["phase"] == "moving_to_elevator_1f":
            with connect() as con:
                con.execute(
                    "UPDATE jobs SET phase='elevator_transfer_to_2f', updated_at=? WHERE id=?",
                    (timestamp, job["id"]),
                )
                con.execute(
                    "UPDATE amrs SET floor=1, room='엘리베이터 앞', status='엘리베이터 탑승 · 2층 맵 전환 중', x=NULL, y=NULL, yaw=NULL, updated_at=? WHERE name=?",
                    (timestamp, amr_name),
                )
                con.execute(
                    "UPDATE beds SET floor=1, room='엘리베이터 앞', status='도킹 유지 · 엘리베이터 탑승', updated_at=? WHERE id=?",
                    (timestamp, job["bed_id"]),
                )
            add_event(f"{amr_name} · 1층 엘리베이터 탑승")

        elif job["phase"] == "moving_to_mri":
            # MRI 목적지 tolerance 진입 즉시 침상을 분리하고 검사 상태로 전환합니다.
            # 별도 3초 대기/버튼 없이 AMR은 MRI실 앞 경유점으로 바로 이동합니다.
            with connect() as con:
                con.execute(
                    "UPDATE jobs SET phase='backing_out_after_drop', updated_at=? WHERE id=?",
                    (timestamp, job["id"]),
                )
                con.execute(
                    "UPDATE amrs SET floor=2, room='MRI실', status='환자 침상 → MRI 인계 완료 · 검사 대기 위치 이동', x=NULL, y=NULL, yaw=NULL, updated_at=? WHERE name=?",
                    (timestamp, amr_name),
                )
                con.execute(
                    "UPDATE beds SET floor=2, room='MRI실', status='MRI 검사중 · 환자 MRI 인계 완료', updated_at=? WHERE id=?",
                    (timestamp, job["bed_id"]),
                )
            add_event(f"{amr_name} · MRI 목적지 도착 · 환자 침상 → MRI 인계")
            next_target = MRI_FRONT_LOCATION

        elif job["phase"] == "backing_out_after_drop":
            # MRI실 앞 경유점 tolerance 진입 시 검사 대기 상태를 표시하고
            # 환자 재결합을 위해 MRI실로 바로 재진입합니다.
            with connect() as con:
                con.execute(
                    "UPDATE jobs SET phase='moving_to_repickup', updated_at=? WHERE id=?",
                    (timestamp, job["id"]),
                )
                con.execute(
                    "UPDATE amrs SET floor=2, room='MRI실 앞', status='MRI 검사 대기중', x=NULL, y=NULL, yaw=NULL, updated_at=? WHERE name=?",
                    (timestamp, amr_name),
                )
                con.execute(
                    "UPDATE beds SET status='MRI 검사 대기중', updated_at=? WHERE id=?",
                    (timestamp, job["bed_id"]),
                )
            add_event(f"{amr_name} · MRI실 앞 도착 · MRI 검사 대기중")
            next_target = MRI_LOCATION

        elif job["phase"] == "moving_to_repickup":
            # MRI 목적지에 다시 진입하면 환자 침상과 즉시 재결합하고 복귀를 시작합니다.
            with connect() as con:
                con.execute(
                    "UPDATE jobs SET phase='backing_out_after_pickup', updated_at=? WHERE id=?",
                    (timestamp, job["id"]),
                )
                con.execute(
                    "UPDATE amrs SET floor=2, room='MRI실', status='환자 MRI → 침상 회수 완료 · 복귀중', x=NULL, y=NULL, yaw=NULL, updated_at=? WHERE name=?",
                    (timestamp, amr_name),
                )
                con.execute(
                    "UPDATE beds SET floor=2, room='MRI실', status='환자 회수 완료 · 복귀중', updated_at=? WHERE id=?",
                    (timestamp, job["bed_id"]),
                )
            add_event(f"{amr_name} · MRI실 재진입 · 환자 MRI → 침상 회수 · 복귀 시작")
            next_target = MRI_FRONT_LOCATION

        elif job["phase"] == "backing_out_after_pickup":
            with connect() as con:
                con.execute(
                    "UPDATE jobs SET phase='moving_to_elevator_2f', updated_at=? WHERE id=?",
                    (timestamp, job["id"]),
                )
                con.execute(
                    "UPDATE amrs SET floor=2, room='MRI실 앞', status='병실 복귀 · 2층 엘리베이터 이동 중', x=NULL, y=NULL, yaw=NULL, updated_at=? WHERE name=?",
                    (timestamp, amr_name),
                )
                con.execute(
                    "UPDATE beds SET floor=2, room='MRI실 앞', status='도킹됨 · 병실 복귀 중', updated_at=? WHERE id=?",
                    (timestamp, job["bed_id"]),
                )
            add_event(f"{amr_name} · 병실 복귀 시작")
            next_target = ELEVATOR_2F

        elif job["phase"] == "moving_to_elevator_2f":
            with connect() as con:
                con.execute(
                    "UPDATE jobs SET phase='elevator_transfer_to_1f', updated_at=? WHERE id=?",
                    (timestamp, job["id"]),
                )
                con.execute(
                    "UPDATE amrs SET floor=2, room='엘리베이터 앞', status='엘리베이터 탑승 · 1층 맵 전환 중', x=NULL, y=NULL, yaw=NULL, updated_at=? WHERE name=?",
                    (timestamp, amr_name),
                )
                con.execute(
                    "UPDATE beds SET floor=2, room='엘리베이터 앞', status='도킹 유지 · 엘리베이터 탑승', updated_at=? WHERE id=?",
                    (timestamp, job["bed_id"]),
                )
            add_event(f"{amr_name} · 2층 엘리베이터 탑승")

        elif job["phase"] == "returning_to_ward":
            origin_floor = job.get("origin_floor") or 1
            origin_room = job.get("origin_room") or "병실"
            with connect() as con:
                con.execute(
                    "UPDATE jobs SET phase='ward_storage_ready', updated_at=? WHERE id=?",
                    (timestamp, job["id"]),
                )
                con.execute(
                    "UPDATE amrs SET floor=?, room=?, status='침상 반환 완료 · AMR 보관실 이동 준비중', x=NULL, y=NULL, yaw=NULL, updated_at=? WHERE name=?",
                    (origin_floor, origin_room, timestamp, amr_name),
                )
                con.execute(
                    "UPDATE beds SET floor=?, room=?, status='병실 침상 반환 완료 · AMR 보관실 이동 준비중', updated_at=? WHERE id=?",
                    (origin_floor, origin_room, timestamp, job["bed_id"]),
                )
            add_event(f"{amr_name} · 병실 복귀 · 침상 반환/언도킹 완료 · 보관실 이동 준비")

        elif job["phase"] == "returning_to_storage":
            with connect() as con:
                con.execute(
                    "UPDATE jobs SET phase='complete', updated_at=? WHERE id=?",
                    (timestamp, job["id"]),
                )
                con.execute(
                    "UPDATE amrs SET floor=1, room='보관실', status='보관실 대기', x=NULL, y=NULL, yaw=NULL, updated_at=? WHERE name=?",
                    (timestamp, amr_name),
                )
                con.execute(
                    "UPDATE beds SET status='대기', assigned_amr=NULL, updated_at=? WHERE id=?",
                    (timestamp, job["bed_id"]),
                )
            add_event(f"{amr_name} · 임무 완료 · 보관실 도착")

        if next_target:
            self._start_navigation(amr_name, *next_target)
        return True

    @synchronized
    def navigation_failed(self, amr_name, reason="Nav2 실패"):
        job = self.active_job(amr_name)
        if not job or job["phase"] not in MOVING_PHASES:
            return False
        timestamp = now_text()
        with connect() as con:
            con.execute(
                "UPDATE jobs SET resume_phase=phase, phase='failed_navigation', updated_at=? WHERE id=?",
                (timestamp, job["id"]),
            )
            con.execute(
                "UPDATE amrs SET status='이동 실패 · 재시도 대기', updated_at=? WHERE name=?",
                (timestamp, amr_name),
            )
        add_event(f"{amr_name} · 이동 실패 · {reason}", "WARN")
        return True

    @synchronized
    def retry_navigation(self, amr_name):
        job = self.active_job(amr_name)
        if not job or job["phase"] != "failed_navigation" or not job.get("resume_phase"):
            raise GuiError("재시도할 이동 실패 작업이 없습니다.")
        resume = job["resume_phase"]
        if resume not in MOVING_PHASES:
            raise GuiError("복구할 이동 단계가 올바르지 않습니다.")
        timestamp = now_text()
        with connect() as con:
            con.execute(
                "UPDATE jobs SET phase=?, resume_phase=NULL, updated_at=? WHERE id=?",
                (resume, timestamp, job["id"]),
            )
            con.execute(
                "UPDATE amrs SET status='이동 재시도 중', updated_at=? WHERE name=?",
                (timestamp, amr_name),
            )
        refreshed = self.active_job(amr_name)
        target = self._job_target(refreshed)
        add_event(f"{amr_name} · 이동 재시도")
        if target:
            self._start_navigation(amr_name, *target, preserve_debug_position=True)

    @synchronized
    def cancel_job(self, amr_name):
        job = self.active_job(amr_name)
        if not job:
            raise GuiError("취소할 임무가 없습니다.")
        if self._ros_enabled() and job["phase"] in MOVING_PHASES:
            raise GuiError("ROS 이동 중인 임무는 GUI에서 즉시 취소할 수 없습니다.")
        timestamp = now_text()
        with connect() as con:
            con.execute(
                "UPDATE jobs SET phase='cancelled', updated_at=? WHERE id=?",
                (timestamp, job["id"]),
            )
            con.execute(
                "UPDATE amrs SET floor=1, room='보관실', status='보관실 대기', x=NULL, y=NULL, yaw=NULL, updated_at=? WHERE name=?",
                (timestamp, amr_name),
            )
            if job.get("bed_id"):
                origin_floor = job.get("origin_floor") or 1
                origin_room = job.get("origin_room") or '병실'
                con.execute(
                    "UPDATE beds SET floor=?, room=?, assigned_amr=NULL, status='대기', updated_at=? WHERE id=?",
                    (origin_floor, origin_room, timestamp, job["bed_id"]),
                )
        self._traffic_waiting.pop(amr_name, None)
        self._debug_positions.pop(amr_name, None)
        self._live_positions.pop(amr_name, None)
        add_event(f"{amr_name} · 임무 취소", "WARN")

    @synchronized
    def reset_demo(self):
        if self._ros_enabled():
            raise GuiError("ROS 연결 모드에서는 시나리오 초기화를 사용할 수 없습니다.")
        timestamp = now_text()
        self._route_progress.clear()
        self._traffic_waiting.clear()
        self._debug_positions.clear()
        self._live_positions.clear()
        with connect() as con:
            con.execute("DELETE FROM jobs")
            con.execute("DELETE FROM events")
            con.execute(
                "UPDATE amrs SET floor=1, room='보관실', status='보관실 대기', x=NULL, y=NULL, yaw=NULL, updated_at=?",
                (timestamp,),
            )
            con.execute(
                "UPDATE beds SET floor=1, room='병실', status='대기', assigned_amr=NULL, updated_at=?",
                (timestamp,),
            )
            con.execute(
                "INSERT INTO events(level, message, created_at) VALUES('INFO', ?, ?)",
                ("GUI 초기화", timestamp),
            )

    @synchronized
    def enter_traffic_wait(self, amr_name, floor, waiting_point_id):
        if self._ros_enabled():
            raise GuiError("ROS 연결 모드에서는 웹 디버그 교행 대기를 사용할 수 없습니다.")
        job = self.active_job(amr_name)
        if not job or job.get("phase") not in MOVING_PHASES:
            raise GuiError("현재 교행 대기로 전환할 이동 작업이 없습니다.")

        floor = int(floor)
        point = self._waiting_point_by_id(floor, waiting_point_id)
        if not point:
            raise GuiError("존재하지 않는 교행 대기 포인트입니다.")

        jobs = rows("SELECT * FROM jobs WHERE phase NOT IN ('complete','cancelled') ORDER BY id")
        amrs = rows("SELECT * FROM amrs ORDER BY name")
        conflict = next(
            (
                item
                for item in self._traffic_conflicts(jobs, amrs)
                if item["yielding_amr"] == amr_name and int(item["floor"]) == floor
            ),
            None,
        )
        if not conflict:
            raise GuiError("현재 이 AMR이 양보해야 하는 교행 충돌이 없습니다.")
        if conflict.get("recommended_action") != "divert_to_wait":
            raise GuiError("현재 상황은 대기 포인트 이탈이 아니라 현 위치 정지가 더 안전합니다.")
        expected = (conflict.get("waiting_point") or {}).get("id")
        if not expected:
            raise GuiError("현재 층에 비어 있는 교행 대기 포인트가 없습니다. 현재 위치에서 대기하세요.")
        if expected != waiting_point_id:
            raise GuiError(f"현재 양보 위치는 {expected}입니다.")
        self._ensure_display_free(
            amr_name, floor, point.get("display"), point.get("display_name", waiting_point_id)
        )

        resume_position = self._debug_positions.pop(amr_name, None)
        self._traffic_waiting[amr_name] = {
            "floor": floor,
            "waiting_point_id": waiting_point_id,
            "waiting_point_name": point.get("display_name", waiting_point_id),
            "priority_amr": conflict.get("priority_amr"),
            "priority_reason": conflict.get("priority_reason", ""),
            "resume_point_key": resume_position.get("point_key") if resume_position else None,
            "resume_progress_ratio": (
                float(resume_position.get("progress_ratio", self._route_progress.get(amr_name, 0.0)))
                if resume_position else float(self._route_progress.get(amr_name, 0.0))
            ),
            "entered_at": now_text(),
        }
        add_event(
            f"{amr_name} · 교행 양보 · {point.get('display_name', waiting_point_id)} 대기 진입 · "
            f"{conflict.get('priority_amr')} 우선"
        )
        return dict(self._traffic_waiting[amr_name])

    @synchronized
    def release_traffic_wait(self, amr_name):
        waiting = self._traffic_waiting.get(amr_name)
        if not waiting:
            raise GuiError("현재 교행 대기 중인 AMR이 아닙니다.")

        jobs = rows("SELECT * FROM jobs WHERE phase NOT IN ('complete','cancelled') ORDER BY id")
        amrs = rows("SELECT * FROM amrs ORDER BY name")
        still_yielding = next(
            (item for item in self._traffic_conflicts(jobs, amrs) if item["yielding_amr"] == amr_name),
            None,
        )
        if still_yielding:
            priority = still_yielding.get("priority_amr")
            raise GuiError(f"{priority}이(가) 아직 교행 구간을 통과하지 않아 대기를 해제할 수 없습니다.")

        amr = row("SELECT * FROM amrs WHERE name=?", (amr_name,))
        if amr:
            self._ensure_room_point_free(amr_name, amr["floor"], amr["room"])
        self._traffic_waiting.pop(amr_name, None)
        add_event(f"{amr_name} · 교행 대기 해제 · 경로 재진입")
        return True

    @synchronized
    def claim_debug_point(self, amr_name, floor, point_key, progress_ratio):
        if self._ros_enabled():
            raise GuiError("ROS 연결 모드에서는 웹 디버그 포인트 이동을 사용할 수 없습니다.")
        job = self.active_job(amr_name)
        if not job or job.get("phase") not in MOVING_PHASES:
            raise GuiError("현재 경유점 이동 중인 AMR이 아닙니다.")
        if amr_name in self._traffic_waiting:
            raise GuiError("교행 대기 중에는 일반 경유점으로 이동할 수 없습니다.")

        floor = int(floor)
        info = self._point_info(floor, point_key)
        if not info or info.get("source") not in {"poi", "waypoint"}:
            raise GuiError("존재하지 않는 이동 포인트입니다.")
        self._ensure_display_free(amr_name, floor, info["display"], info.get("label", point_key))

        try:
            ratio = float(progress_ratio)
        except (TypeError, ValueError) as exc:
            raise GuiError("이동 진행률은 숫자여야 합니다.") from exc
        ratio = max(0.0, min(1.0, ratio))
        record = dict(info)
        record["progress_ratio"] = ratio
        self._debug_positions[amr_name] = record
        self._route_progress[amr_name] = ratio
        return dict(record)

    @synchronized
    def resume_traffic_to_point(self, amr_name, floor, point_key, progress_ratio):
        waiting = self._traffic_waiting.get(amr_name)
        if not waiting:
            raise GuiError("현재 교행 대기 중인 AMR이 아닙니다.")

        jobs = rows("SELECT * FROM jobs WHERE phase NOT IN ('complete','cancelled') ORDER BY id")
        amrs = rows("SELECT * FROM amrs ORDER BY name")
        still_yielding = next(
            (item for item in self._traffic_conflicts(jobs, amrs) if item["yielding_amr"] == amr_name),
            None,
        )
        if still_yielding:
            priority = still_yielding.get("priority_amr")
            raise GuiError(f"{priority}이(가) 아직 교행 구간을 통과하지 않아 대기를 해제할 수 없습니다.")

        floor = int(floor)
        info = self._point_info(floor, point_key)
        if not info or info.get("source") not in {"poi", "waypoint"}:
            raise GuiError("재진입할 이동 포인트가 올바르지 않습니다.")
        self._ensure_display_free(amr_name, floor, info["display"], info.get("label", point_key))
        try:
            ratio = float(progress_ratio)
        except (TypeError, ValueError) as exc:
            raise GuiError("이동 진행률은 숫자여야 합니다.") from exc
        ratio = max(0.0, min(1.0, ratio))

        self._traffic_waiting.pop(amr_name, None)
        record = dict(info)
        record["progress_ratio"] = ratio
        self._debug_positions[amr_name] = record
        self._route_progress[amr_name] = ratio
        add_event(f"{amr_name} · 교행 대기 해제 · {info.get('label', point_key)} 재진입")
        return dict(record)

    @synchronized
    def set_route_progress(self, amr_name, progress_ratio):
        try:
            ratio = float(progress_ratio)
        except (TypeError, ValueError) as exc:
            raise GuiError("이동 진행률은 숫자여야 합니다.") from exc
        self._route_progress[amr_name] = max(0.0, min(1.0, ratio))
        return self._route_progress[amr_name]


    def _snapped_point_matches_target(self, amr_name, snapped, target):
        """world_pose 판정 포인트가 현재 GUI 단계의 목표 지점인지 확인합니다."""
        if not snapped or not target:
            return False
        floor, room = int(target[0]), target[1]
        if int(snapped.get("floor", -1)) != floor:
            return False

        # 환자 병실은 공용 좌표를 쓰지 않고 선택 환자별 OCR 좌표를 사용합니다.
        if (floor, room) == (1, "병실") and snapped.get("source") == "patient":
            job = self.active_job(amr_name)
            bed = row("SELECT * FROM beds WHERE id=?", (job.get("bed_id"),)) if job and job.get("bed_id") else None
            expected = str((bed or {}).get("patient_name") or (bed or {}).get("label") or "").strip()
            actual = str(snapped.get("patient_name") or "").strip()
            return bool(expected and actual and expected == actual)

        # 보관실 복귀는 AMR별 home 좌표를 각각 사용합니다.
        if (floor, room) == STORAGE_LOCATION and snapped.get("source") == "home":
            return snapped.get("amr_name") in (None, amr_name)

        if snapped.get("source") == "poi" and snapped.get("room") == room:
            return True

        # 2F-MRI-FRONT(-6.3,17.0)는 MRI 검사 대기 위치 판정용 기준점입니다.
        if (floor, room) == MRI_FRONT_LOCATION:
            return snapped.get("point_id") == "2F-MRI-FRONT"
        return False

    @synchronized
    def update_pose(self, amr_name, x, y, yaw):
        """/world_pose로 GUI의 현재 위치와 포인트 도착 상태를 갱신합니다.

        - tolerance 내부: 등록 목적지/경유점/대기점/홈 포인트 중심으로 스냅
        - tolerance 외부: raw world_pose를 첨부 지도 기준 경로에 연속 투영해 이동 위치 표시
        - 중첩 tolerance: 실제 world_pose와 가장 가까운 등록 포인트 하나를 선택
        - 도착/단계 판정: pose_config.py의 포인트별 tolerance 사용
        - 층 분리: DB의 현재 floor에 해당하는 좌표표만 실제 시나리오 판정에 사용
        """
        amr = row("SELECT * FROM amrs WHERE name=?", (amr_name,))
        if not amr:
            return None
        x, y, yaw = float(x), float(y), float(yaw)
        timestamp = now_text()
        with connect() as con:
            con.execute(
                "UPDATE amrs SET x=?, y=?, yaw=?, updated_at=? WHERE name=?",
                (x, y, yaw, timestamp, amr_name),
            )

        # 현재 AMR에 설정된 층의 좌표표만 사용합니다.
        # 1층 상태에서는 2층 좌표를, 2층 상태에서는 1층 좌표를 절대로 후보로 비교하지 않습니다.
        current_floor = int(amr.get("floor") or 1)
        snapped = nearest_nav_point(current_floor, x, y, amr_name=amr_name)
        previous = self._live_positions.get(amr_name)
        previous_snap_id = previous.get("snapped_canonical_id") if previous else None

        # 지도 마커는 raw world_pose를 첨부 지도상의 시나리오 경로에 연속 투영합니다.
        # tolerance 안에 들어오면 해당 설정 좌표 중심에 정확히 스냅하고 위치 의미를 확정합니다.
        continuous_display = map_config_module.nav_xy_to_display(current_floor, x, y)
        display = None
        if snapped is not None and snapped.get("display"):
            display = list(snapped.get("display") or ())
        elif continuous_display is not None:
            display = list(continuous_display)

        live = {
            "floor": current_floor,
            "display": list(display) if display is not None and len(display) >= 2 else None,
            "canonical_id": None,
            "source": "world_pose_live",
            "position_source": "world_pose",
            "display_mode": "fixed_point_snap" if snapped is not None else "continuous_world_pose",
            "raw_pose": {"x": x, "y": y, "yaw": yaw},
            "in_tolerance": snapped is not None,
            "holding_previous_point": False,
            "updated_at": timestamp,
        }
        if snapped is not None:
            snapped = dict(snapped)
            snapped_id = self._canonical_point_id(snapped["floor"], snapped["display"])
            # 실제 AMR이 tolerance 내부에 있을 때만 해당 물리 포인트를 점유한 것으로 봅니다.
            live["canonical_id"] = snapped_id
            live["snapped_canonical_id"] = snapped_id
            live["snapped_point_key"] = snapped.get("point_key")
            live["snapped_point_id"] = snapped.get("point_id")
            live["snapped_source"] = snapped.get("source")
            live["snapped_label"] = snapped.get("label")
            live["distance_m"] = snapped.get("distance_m")
            # 기존 호출부/테스트 호환을 위해 스냅된 포인트 필드를 최상위에도 제공합니다.
            for key in ("point_key", "point_id", "room", "label"):
                if key in snapped:
                    live[key] = snapped.get(key)
        self._live_positions[amr_name] = live

        # GUI의 상단 1층/2층 탭이 즉시 좌표 판정 기준을 바꿀 수 있도록,
        # 동일한 raw pose를 1층/2층 좌표표에 각각 독립 판정해 둡니다.
        # 아래 결과는 화면 표시용이며 실제 미션/DB 층 상태는 바꾸지 않습니다.
        for debug_floor in sorted(FLOORS):
            self._update_floor_debug_position(amr_name, debug_floor, x, y, yaw, timestamp)

        if snapped is None:
            return live

        if snapped.get("source") == "poi" and snapped.get("room"):
            with connect() as con:
                con.execute(
                    "UPDATE amrs SET floor=?, room=?, updated_at=? WHERE name=?",
                    (int(snapped["floor"]), snapped["room"], timestamp, amr_name),
                )

        # 현재 이동 경로에서 몇 번째 포인트인지로 진행률을 근사합니다.
        job = self.active_job(amr_name)
        target = self._job_target(job) if job else None
        if target and int(target[0]) == int(snapped["floor"]):
            refreshed_amr = row("SELECT * FROM amrs WHERE name=?", (amr_name,)) or amr
            from_room = refreshed_amr.get("room") or amr.get("room")
            path = self._ordered_path_infos(int(target[0]), from_room, target[1])
            if path:
                ids = [item.get("canonical_id") for item in path]
                snapped_id = live.get("snapped_canonical_id")
                if snapped_id in ids:
                    index = ids.index(snapped_id)
                    self._route_progress[amr_name] = index / max(1, len(ids) - 1)

        changed = previous_snap_id != live.get("snapped_canonical_id")
        if changed and live.get("snapped_canonical_id"):
            add_event(f"{amr_name} · {snapped.get('label', snapped.get('point_id', '포인트'))} 위치 확인")

        # 병실 중심 좌표에서는 도킹 준비만 하고, AMR이 침상과 함께 병실 앞 경유점으로
        # 빠져나오는 순간(1F-WARD-CORNER) 실제 도킹 완료 및 MRI 이송 시작으로 전환합니다.
        job = self.active_job(amr_name)
        if (
            job
            and job.get("phase") == "ward_docking_ready"
            and int(snapped.get("floor", -1)) == 1
            and snapped.get("point_id") == "1F-WARD-CORNER"
        ):
            self._complete_ward_docking_at_front(amr_name, job, timestamp)
            return live

        # 복귀 시 병실 중심에서 환자 결합을 해제한 뒤 병실 앞 경유점으로 나오면
        # 그 좌표를 기준으로 보관실 이동을 자동 시작합니다.
        if (
            job
            and job.get("phase") == "ward_storage_ready"
            and int(snapped.get("floor", -1)) == 1
            and snapped.get("point_id") == "1F-WARD-CORNER"
        ):
            self._start_storage_return_at_ward_front(amr_name, job, timestamp)
            return live

        # 실제 미션 단계 전환은 patient_transport_manager.py의 실행 로그/서비스 상태가 소유합니다.
        # /world_pose tolerance는 오직 "어느 설정 좌표에 존재하는지"와 지도 마커 표시를 위해 사용합니다.
        return live


# ---- actual mission runtime state bridge ---------------------------------
def _phase_update_sql(self, amr_name, phase, *, floor=None, room=None, amr_status=None, bed_status=None, event=None):
    job = self.active_job(amr_name)
    if not job:
        return False
    timestamp = now_text()
    with connect() as con:
        con.execute("UPDATE jobs SET phase=?, updated_at=? WHERE id=?", (phase, timestamp, job["id"]))
        fields = []
        params = []
        if floor is not None:
            fields.append("floor=?"); params.append(int(floor))
        if room is not None:
            fields.append("room=?"); params.append(room)
        if amr_status is not None:
            fields.append("status=?"); params.append(amr_status)
        fields.append("updated_at=?"); params.append(timestamp); params.append(amr_name)
        con.execute(f"UPDATE amrs SET {', '.join(fields)} WHERE name=?", params)
        if job.get("bed_id") and bed_status is not None:
            con.execute("UPDATE beds SET status=?, updated_at=? WHERE id=?", (bed_status, timestamp, job["bed_id"]))
        if job.get("bed_id") and floor is not None:
            con.execute("UPDATE beds SET floor=?, updated_at=? WHERE id=?", (int(floor), timestamp, job["bed_id"]))
    if event:
        add_event(f"{amr_name} · {event}")
    return True


def _apply_mission_log(self, amr_name, line):
    text = str(line)
    job = self.active_job(amr_name)
    if not job:
        return
    # 실제 mission stdout의 확정 이벤트만 단계 판정에 사용합니다.
    if "[Nav2 시작]" in text and "OCR 위치" in text:
        return _phase_update_sql(self, amr_name, "moving_to_patient", floor=1, amr_status="환자 병실 이동 중")
    if "[OCR 자동 접근]" in text:
        return _phase_update_sql(self, amr_name, "ward_docking_ready", floor=1, room="병실", amr_status="OCR/ArUco 환자 확인 · 도킹 진행 중", bed_status="환자 확인 · 도킹 진행 중")
    if "[결합 성공]" in text:
        return _phase_update_sql(self, amr_name, "ward_attach_wait", floor=1, room="병실", amr_status="침상 결합 완료 · Lift 상승/병실 이탈 중", bed_status="도킹됨 · MRI 이송 준비", event="침상 결합 확인")
    if "[Nav2 시작] 1층 엘리베이터 도달" in text:
        return _phase_update_sql(self, amr_name, "moving_to_elevator_1f", floor=1, amr_status="1층 엘리베이터 이동 중", bed_status="도킹됨 · 1층 엘리베이터 이동 중")
    if "[엘리베이터 상승 시작]" in text:
        return _phase_update_sql(self, amr_name, "elevator_transfer_to_2f", floor=1, room="엘리베이터 앞", amr_status="엘리베이터 탑승 · 2층 이동 중", bed_status="도킹 유지 · 엘리베이터 탑승", event="1층 엘리베이터 탑승")
    if "[2F MRI 이동]" in text or "[Nav2 시작] 2층 MRI 도착" in text:
        self._live_positions.pop(amr_name, None)
        return _phase_update_sql(self, amr_name, "moving_to_mri", floor=2, room="엘리베이터 앞", amr_status="2층 도착 · MRI실 이동 중", bed_status="도킹됨 · MRI 이송 중", event="2층 맵 전환")
    if "[MRI 도착]" in text:
        return _phase_update_sql(self, amr_name, "backing_out_after_drop", floor=2, room="MRI실", amr_status="MRI 환자 인계 · 11m 후진 중", bed_status="MRI 검사 준비")
    if "[MRI 검사 대기 위치]" in text:
        return _phase_update_sql(self, amr_name, "waiting_exam", floor=2, room="MRI실 앞", amr_status="MRI 검사 중 · 복귀 명령 대기", bed_status="MRI 검사중", event="MRI 11m 후진 완료 · 검사 대기")
    if "[검사완료 SERVICE]" in text or "[검사완료 K]" in text or "[K 이후 MRI 복귀]" in text:
        return _phase_update_sql(self, amr_name, "moving_to_repickup", floor=2, room="MRI실 앞", amr_status="복귀 명령 수신 · MRI 재진입 중", bed_status="MRI 검사 완료 · 환자 회수 중", event="검사완료 · 복귀 시작")
    if "[MRI 재도착]" in text:
        return _phase_update_sql(self, amr_name, "boarding_wait", floor=2, room="MRI실", amr_status="MRI → 침상 환자 회수 중", bed_status="환자 회수 중")
    if "[2F 엘리베이터 복귀]" in text:
        return _phase_update_sql(self, amr_name, "moving_to_elevator_2f", floor=2, amr_status="2층 엘리베이터 이동 중", bed_status="도킹됨 · 병실 복귀 중")
    if "[엘리베이터" in text and "하강 시작" in text:
        return _phase_update_sql(self, amr_name, "elevator_transfer_to_1f", floor=2, room="엘리베이터 앞", amr_status="엘리베이터 탑승 · 1층 이동 중", bed_status="도킹 유지 · 엘리베이터 탑승", event="2층 엘리베이터 탑승")
    if "[1F 엘리베이터 좌표 도착]" in text:
        self._live_positions.pop(amr_name, None)
        return _phase_update_sql(self, amr_name, "returning_to_ward", floor=1, room="엘리베이터 앞", amr_status="1층 도착 · 환자 병실 복귀 중", bed_status="도킹됨 · 병실 복귀 중", event="1층 맵 전환")
    if "[X 결합해체]" in text or "[결합해체 완료]" in text:
        return _phase_update_sql(self, amr_name, "returning_to_storage", floor=1, room="병실", amr_status="침상 반환 완료 · 보관실 복귀 중", bed_status="대기", event="침상 결합 해제")
    if "[도킹 복귀]" in text:
        return _phase_update_sql(self, amr_name, "returning_to_storage", floor=1, amr_status="보관실 복귀 중")
    if "[전체 성공]" in text:
        timestamp = now_text(); job = self.active_job(amr_name)
        if not job: return
        with connect() as con:
            con.execute("UPDATE jobs SET phase='complete', updated_at=? WHERE id=?", (timestamp, job["id"]))
            con.execute("UPDATE amrs SET floor=1, room='보관실', status='보관실 대기', updated_at=? WHERE name=?", (timestamp, amr_name))
            if job.get("bed_id"):
                con.execute("UPDATE beds SET floor=1, room='병실', status='대기', assigned_amr=NULL, updated_at=? WHERE id=?", (timestamp, job["bed_id"]))
        add_event(f"{amr_name} · 전체 미션 완료 · 보관실 복귀")


def _mission_process_finished(self, amr_name, returncode, tail):
    if returncode in (0, None):
        return
    job = self.active_job(amr_name)
    if not job:
        return
    reason = tail[-1] if tail else f"returncode={returncode}"
    timestamp = now_text()
    with connect() as con:
        con.execute("UPDATE jobs SET phase='failed_navigation', updated_at=? WHERE id=?", (timestamp, job["id"]))
        con.execute("UPDATE amrs SET status='실제 미션 실패', updated_at=? WHERE name=?", (timestamp, amr_name))
    add_event(f"{amr_name} · 실제 미션 실패(returncode={returncode}) · {reason}", "WARN")

HospitalService.apply_mission_log = synchronized(_apply_mission_log)
HospitalService.mission_process_finished = synchronized(_mission_process_finished)

def _abort_mission_start(self, amr_name, reason="실제 미션 시작 실패"):
    job = self.active_job(amr_name)
    if not job:
        return
    timestamp = now_text()
    with connect() as con:
        con.execute("UPDATE jobs SET phase='cancelled', updated_at=? WHERE id=?", (timestamp, job["id"]))
        con.execute("UPDATE amrs SET floor=1, room='보관실', status='보관실 대기', updated_at=? WHERE name=?", (timestamp, amr_name))
        if job.get("bed_id"):
            con.execute("UPDATE beds SET status='대기', assigned_amr=NULL, updated_at=? WHERE id=?", (timestamp, job["bed_id"]))
    add_event(f"{amr_name} · 실제 미션 시작 취소 · {reason}", "WARN")

HospitalService.abort_mission_start = synchronized(_abort_mission_start)
