"""DOOSIM 병원 GUI의 실제 ROS2 /world_pose 판정 좌표.

2026-08-09 사용자 지정 좌표만 사용합니다. 이전 좌표 세트는 모두 제거했습니다.
GUI는 /amr1/world_pose, /amr2/world_pose의 x/y를 1초 간격으로 샘플링해
현재 층의 좌표표와 비교합니다.

판정 원칙
- 각 목적지/경유점/대기점의 tolerance 안이면 그 포인트에 있다고 판정합니다.
- 둘 이상의 tolerance가 겹치면 실제 world_pose와 가장 가까운 포인트가 선택됩니다.
- 김서울/박인천 OCR 좌표는 약 0.283m 간격이므로 전용 0.45m tolerance를 사용하고
  겹치는 구간에서는 최단거리 규칙으로 구분합니다.
"""
from __future__ import annotations

import math
import os
import re

# 사용자가 요청한 1~2초 갱신 범위의 기본값: 1.0초
POSE_SAMPLE_INTERVAL_SEC = float(os.getenv("GUI_POSE_SAMPLE_INTERVAL_SEC", "1.0"))

HOME_TOLERANCE_M = float(os.getenv("GUI_HOME_TOLERANCE_M", "1.0"))
PATIENT_OCR_TOLERANCE_M = float(os.getenv("GUI_PATIENT_OCR_TOLERANCE_M", "0.45"))
PATIENT_OCR_SUWON_TOLERANCE_M = float(os.getenv("GUI_PATIENT_OCR_SUWON_TOLERANCE_M", "0.8"))
ELEVATOR_TOLERANCE_M = float(os.getenv("GUI_ELEVATOR_TOLERANCE_M", "1.0"))
WAYPOINT_TOLERANCE_M = float(os.getenv("GUI_WAYPOINT_TOLERANCE_M", "1.0"))
SECOND_FLOOR_WAYPOINT_TOLERANCE_M = float(os.getenv("GUI_2F_WAYPOINT_TOLERANCE_M", "1.3"))
WAITING_POINT_TOLERANCE_M = float(os.getenv("GUI_WAITING_POINT_TOLERANCE_M", "1.0"))
MRI_TOLERANCE_M = float(os.getenv("GUI_MRI_TOLERANCE_M", "1.0"))
MRI_FRONT_TOLERANCE_M = float(os.getenv("GUI_MRI_FRONT_TOLERANCE_M", "1.1"))
DEFAULT_ARRIVAL_TOLERANCE_M = WAYPOINT_TOLERANCE_M


def pose(x, y, yaw=0.0, *, prim=None, tolerance_m=DEFAULT_ARRIVAL_TOLERANCE_M):
    yaw = float(yaw)
    return {
        "prim": prim,
        "x": float(x),
        "y": float(y),
        "yaw": yaw,
        "yaw_deg": math.degrees(yaw),
        "qz": math.sin(yaw / 2.0),
        "qw": math.cos(yaw / 2.0),
        "tolerance_m": float(tolerance_m),
    }


# ----------------------------------------------------------------------
# 사용자 지정 좌표 세트 ONLY
# ----------------------------------------------------------------------
FLOOR_POINT_POSES = {
    1: {
        # 목적지: AMR별 보관/도킹 위치
        "1F-AMR1-HOME": pose(-45.0, 31.8, tolerance_m=HOME_TOLERANCE_M),
        "1F-AMR2-HOME": pose(-47.2, 26.5, tolerance_m=HOME_TOLERANCE_M),

        # 목적지: 환자별 병실/OCR 위치
        "1F-KIM-SEOUL-OCR": pose(-43.6, 11.4, tolerance_m=PATIENT_OCR_TOLERANCE_M),
        "1F-PARK-INCHEON-OCR": pose(-43.4, 11.2, tolerance_m=PATIENT_OCR_TOLERANCE_M),
        "1F-SEO-SUWON-OCR": pose(-43.8, 15.2, tolerance_m=PATIENT_OCR_SUWON_TOLERANCE_M),

        # 목적지: 1층 엘리베이터 앞
        "1F-ELEVATOR": pose(-26.2, 21.5, tolerance_m=ELEVATOR_TOLERANCE_M),

        # 경유점: 보관실 <-> 병실, x=-40.0 / y=23.5 -> 12.7
        # 약 2.2m 간격으로 분할
        "1F-SW-01": pose(-40.0, 23.5, tolerance_m=WAYPOINT_TOLERANCE_M),
        "1F-SW-02": pose(-40.0, 21.3, tolerance_m=WAYPOINT_TOLERANCE_M),
        "1F-SW-03": pose(-40.0, 19.1, tolerance_m=WAYPOINT_TOLERANCE_M),
        "1F-SW-04": pose(-40.0, 16.9, tolerance_m=WAYPOINT_TOLERANCE_M),
        "1F-SW-05": pose(-40.0, 14.7, tolerance_m=WAYPOINT_TOLERANCE_M),
        "1F-WARD-CORNER": pose(-40.0, 12.7, tolerance_m=WAYPOINT_TOLERANCE_M),

        # 경유점: 병실 <-> 엘리베이터, y=12.7 / x=-40.0 -> -26.0
        # 약 2.3m 간격으로 분할
        "1F-WE-X-01": pose(-37.7, 12.7, tolerance_m=WAYPOINT_TOLERANCE_M),
        "1F-WE-X-02": pose(-35.3, 12.7, tolerance_m=WAYPOINT_TOLERANCE_M),
        "1F-WE-X-03": pose(-33.0, 12.7, tolerance_m=WAYPOINT_TOLERANCE_M),
        "1F-WE-X-04": pose(-30.7, 12.7, tolerance_m=WAYPOINT_TOLERANCE_M),
        "1F-WE-X-05": pose(-28.3, 12.7, tolerance_m=WAYPOINT_TOLERANCE_M),
        "1F-ELEVATOR-CORNER": pose(-26.0, 12.7, tolerance_m=WAYPOINT_TOLERANCE_M),

        # 경유점: x=-26.0 / y=12.7 -> 19.0
        # 약 2.1m 간격으로 분할
        "1F-WE-Y-01": pose(-26.0, 14.8, tolerance_m=WAYPOINT_TOLERANCE_M),
        "1F-WE-Y-02": pose(-26.0, 16.9, tolerance_m=WAYPOINT_TOLERANCE_M),
        "1F-WE-Y-03": pose(-26.0, 19.0, tolerance_m=WAYPOINT_TOLERANCE_M),

        # 대기점
        "1F-WAIT-WARD": pose(-40.0, 8.0, tolerance_m=WAITING_POINT_TOLERANCE_M),
        "1F-WAIT-ELEVATOR": pose(-30.0, 18.0, tolerance_m=WAITING_POINT_TOLERANCE_M),
    },
    2: {
        # 목적지
        "2F-ELEVATOR": pose(-26.2, 21.5, tolerance_m=ELEVATOR_TOLERANCE_M),
        "2F-MRI": pose(6.3, 6.6, tolerance_m=MRI_TOLERANCE_M),

        # 경유점: 엘리베이터 앞(-26.2,21.5) 이후 MRI 방향 수평 복도.
        # 2F-EM-01(-26.0,22.0)은 엘리베이터 목적지와 거의 겹치므로 제거했습니다.
        "2F-EM-02": pose(-22.0, 22.0, tolerance_m=SECOND_FLOOR_WAYPOINT_TOLERANCE_M),
        "2F-EM-03": pose(-18.0, 22.0, tolerance_m=SECOND_FLOOR_WAYPOINT_TOLERANCE_M),
        "2F-EM-04": pose(-14.0, 22.0, tolerance_m=SECOND_FLOOR_WAYPOINT_TOLERANCE_M),
        "2F-EM-05": pose(-10.0, 22.0, tolerance_m=SECOND_FLOOR_WAYPOINT_TOLERANCE_M),
        "2F-EM-06": pose(-6.0, 22.0, tolerance_m=SECOND_FLOOR_WAYPOINT_TOLERANCE_M),
        "2F-EM-07": pose(-2.0, 22.0, tolerance_m=SECOND_FLOOR_WAYPOINT_TOLERANCE_M),
        "2F-EM-08": pose(2.0, 22.0, tolerance_m=SECOND_FLOOR_WAYPOINT_TOLERANCE_M),
        "2F-MRI-CORNER": pose(6.3, 22.0, tolerance_m=SECOND_FLOOR_WAYPOINT_TOLERANCE_M),

        # MRI 접근 경유점. 2F-EM-Y-02는 제거했습니다.
        "2F-EM-Y-01": pose(6.3, 20.0, tolerance_m=SECOND_FLOOR_WAYPOINT_TOLERANCE_M),

        # MRI 11m 후진 구간의 지도 표시용 기준점. 검사완료 버튼 활성화에는 사용하지 않습니다.
        # 실제 검사 대기 판정은 patient_transport_manager.py의 "11m 후진 완료" 로그 이벤트가 소유합니다.
        "2F-MRI-FRONT": pose(6.28808, 17.5918, tolerance_m=MRI_FRONT_TOLERANCE_M),
    },
}


def floor_point_poses(floor: int):
    return FLOOR_POINT_POSES.get(int(floor), {})


def floor_from_point_id(point_id: str):
    match = re.match(r"^(\d+)F-", str(point_id or ""), re.IGNORECASE)
    return int(match.group(1)) if match else None


def point_pose(point_id: str, floor: int | None = None):
    resolved_floor = int(floor) if floor is not None else floor_from_point_id(point_id)
    if resolved_floor is None:
        return None
    return floor_point_poses(resolved_floor).get(point_id)


def nav_pose_tuple(point_id: str, floor: int | None = None):
    item = point_pose(point_id, floor=floor)
    if not item:
        return None
    return (float(item["x"]), float(item["y"]), float(item.get("yaw", 0.0)))


def tolerance_for(point_id: str, fallback=DEFAULT_ARRIVAL_TOLERANCE_M, floor: int | None = None):
    item = point_pose(point_id, floor=floor)
    if not item:
        return float(fallback)
    return float(item.get("tolerance_m", fallback))
