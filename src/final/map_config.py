"""병원 GUI 지도 좌표/경유점/교행 정책.

첨부된 1층/2층 GUI 지도 이미지를 기준으로 사용자 지정 world_pose 좌표를
지도 정규화 좌표(display u/v)에 배치합니다. 이전 좌표 세트는 사용하지 않습니다.
"""
from __future__ import annotations

import math

from pose_config import DEFAULT_ARRIVAL_TOLERANCE_M, nav_pose_tuple, tolerance_for

UNIFIED_WAYPOINT_COLOR = "blue"
WAITING_POINT_COLOR = "red"


def _nav(point_id):
    return nav_pose_tuple(point_id)


def _tol(point_id):
    return tolerance_for(point_id)


def _lerp(a, b, t):
    return float(a) + (float(b) - float(a)) * float(t)


def _lerp_display(a, b, t):
    return (_lerp(a[0], b[0], t), _lerp(a[1], b[1], t))


def _ratio(value, start, end):
    if abs(float(end) - float(start)) < 1e-12:
        return 0.0
    return (float(value) - float(start)) / (float(end) - float(start))


# ----------------------------------------------------------------------
# 첨부 지도 이미지에 맞춘 화면 좌표
# ----------------------------------------------------------------------
# 1층 복도 기준점
D1_SW_TOP = (0.198184, 0.337414)        # (-40.0, 23.5)
# 보관실→병실 경유점은 실제 x=-40.0으로 동일하므로 화면에서도
# 1F-SW-01의 X(display u)를 그대로 사용해 수직 일직선으로 표시합니다.
D1_WARD_CORNER = (D1_SW_TOP[0], 0.566465)  # (-40.0, 12.7)
D1_ELEVATOR_CORNER = (0.388631, 0.566465)  # (-26.0, 12.7)
D1_ELEVATOR_Y19 = (0.388631, 0.475363)   # (-26.0, 19.0)
D1_ELEVATOR = (0.388631, 0.425000)       # (-26.2, 21.5)

# 2층 복도 기준점
D2_ELEVATOR = (0.406570, 0.371311)       # (-26.2, 21.5)
D2_CORRIDOR_START = (0.409200, 0.371311) # (-26.0, 22.0)
D2_MRI_CORNER = (0.835675, 0.371311)     # (6.3, 22.0)
# MRI 검사 대기 기준점의 지도 표시 위치. 실제 마커는 /world_pose를 계속 따라가며,
# 이 화면점은 검사 대기 구역을 MRI 접근 복도 위에 알아보기 쉽게 표시하기 위한 기준입니다.
D2_MRI_FRONT = (0.835675, 0.475000)      # 2F-MRI-FRONT (-6.3, 17.0)
D2_MRI = (0.835675, 0.695000)            # (6.3, 6.6)


def _display_1f_sw(y):
    t = _ratio(y, 23.5, 12.7)
    return _lerp_display(D1_SW_TOP, D1_WARD_CORNER, t)


def _display_1f_we_x(x):
    t = _ratio(x, -40.0, -26.0)
    return _lerp_display(D1_WARD_CORNER, D1_ELEVATOR_CORNER, t)


def _display_1f_we_y(y):
    t = _ratio(y, 12.7, 19.0)
    return _lerp_display(D1_ELEVATOR_CORNER, D1_ELEVATOR_Y19, t)


def _display_2f_x(x):
    t = _ratio(x, -26.0, 6.3)
    return _lerp_display(D2_CORRIDOR_START, D2_MRI_CORNER, t)


def _display_2f_y(y):
    t = _ratio(y, 22.0, 16.0)
    return _lerp_display(D2_MRI_CORNER, D2_MRI_FRONT, t)


# 교행 정책: 사용자 지정 대기점은 1층에만 존재합니다.
TRAFFIC_CORRIDORS = {
    1: {
        "id": "1F-WARD-ELEVATOR",
        "from": "병실",
        "to": "엘리베이터 앞",
        "default_priority_direction": 1,
        "return_commit_point_ids": ["1F-WARD-CORNER", "1F-WE-X-01"],
        "default_priority_label": "MRI 이송(병실 → 엘리베이터) 우선",
        "return_commit_label": "복귀 AMR이 병실 직전 경유점에 진입한 경우 복귀 우선",
    },
    2: {
        "id": "2F-ELEVATOR-MRI",
        "from": "엘리베이터 앞",
        "to": "MRI실 앞",
        "default_priority_direction": 1,
        "return_commit_point_ids": ["2F-EM-02", "2F-EM-03"],
        "default_priority_label": "MRI 이송(엘리베이터 → MRI) 우선",
        "return_commit_label": "복귀 AMR이 엘리베이터 직전 경유점에 진입한 경우 복귀 우선",
    },
}

PRIORITY_RULES = [
    {
        "id": "mri_direction_first",
        "title": "MRI 방향 AMR 기본 우선",
        "description": "1층은 병실→엘리베이터, 2층은 엘리베이터→MRI 방향 AMR을 기본 우선 처리합니다.",
    },
    {
        "id": "return_commit_override",
        "title": "복귀 AMR 목적지 직전 진입 시 우선권 전환",
        "description": "복귀 AMR이 목적지 직전 경유점에 이미 진입했다면 먼저 목적지 안으로 들어가게 합니다.",
    },
    {
        "id": "nearest_wait",
        "title": "가장 가까운 빈 대기점 사용",
        "description": "1층에서 양보가 필요한 경우 현재 위치에서 가장 가까운 비어 있는 빨간 대기점을 사용합니다.",
    },
    {
        "id": "exclusive_point",
        "title": "동일 포인트 동시 점유 금지",
        "description": "목적지·경유점·대기점은 tolerance 판정 기준으로 한 번에 AMR 1대만 점유합니다.",
    },
]


def _wp(point_id, display_name, display):
    return {
        "id": point_id,
        "display_name": display_name,
        "display": display,
        "nav_pose": _nav(point_id),
        "arrival_radius_m": _tol(point_id),
    }


FLOORS = {
    1: {
        "name": "1층",
        "image": "/static/maps/hospital_map_1f.png",
        "route": ["보관실", "병실", "엘리베이터 앞"],
        # 실제 목적지 좌표를 AMR별 home 슬롯에 직접 등록합니다.
        "home_slots": {
            "AMR-01": {
                "id": "1F-AMR1-HOME",
                # 보관실 텍스트의 오른쪽에 배치
                "display": (0.185, 0.200),
                "display_name": "AMR1 보관/도킹 위치",
                "nav_pose": _nav("1F-AMR1-HOME"),
                "arrival_radius_m": _tol("1F-AMR1-HOME"),
            },
            "AMR-02": {
                "id": "1F-AMR2-HOME",
                # 왼쪽 굵은 벽선을 침범하지 않도록 보관실 텍스트 아래쪽에 배치
                "display": (0.126, 0.255),
                "display_name": "AMR2 보관/도킹 위치",
                "nav_pose": _nav("1F-AMR2-HOME"),
                "arrival_radius_m": _tol("1F-AMR2-HOME"),
            },
        },
        # 환자별 병실/OCR 목적지. 논리적 room은 모두 '병실'이지만 물리 좌표는 환자별로 구분합니다.
        "patient_points": [
            {
                "id": "1F-KIM-SEOUL-OCR",
                "patient_name": "김서울",
                "display_name": "김서울 병실/OCR",
                "display": (0.121, 0.548),
                "nav_pose": _nav("1F-KIM-SEOUL-OCR"),
                "arrival_radius_m": _tol("1F-KIM-SEOUL-OCR"),
            },
            {
                "id": "1F-PARK-INCHEON-OCR",
                "patient_name": "박인천",
                "display_name": "박인천 병실/OCR",
                "display": (0.132, 0.558),
                "nav_pose": _nav("1F-PARK-INCHEON-OCR"),
                "arrival_radius_m": _tol("1F-PARK-INCHEON-OCR"),
            },
            {
                "id": "1F-SEO-SUWON-OCR",
                "patient_name": "서수원",
                "display_name": "서수원 병실/OCR",
                "display": (0.124, 0.492),
                "nav_pose": _nav("1F-SEO-SUWON-OCR"),
                "arrival_radius_m": _tol("1F-SEO-SUWON-OCR"),
            },
        ],
        "waypoint_routes": [
            {
                "id": "1f_storage_to_patient",
                "from": "보관실",
                "to": "병실",
                "color": UNIFIED_WAYPOINT_COLOR,
                "waypoints": [
                    _wp("1F-SW-01", "보관실↔병실 1", _display_1f_sw(23.5)),
                    _wp("1F-SW-02", "보관실↔병실 2", _display_1f_sw(21.3)),
                    _wp("1F-SW-03", "보관실↔병실 3", _display_1f_sw(19.1)),
                    _wp("1F-SW-04", "보관실↔병실 4", _display_1f_sw(16.9)),
                    _wp("1F-SW-05", "보관실↔병실 5", _display_1f_sw(14.7)),
                    _wp("1F-WARD-CORNER", "병실 앞 복도 경유점", D1_WARD_CORNER),
                ],
            },
            {
                "id": "1f_patient_to_elevator",
                "from": "병실",
                "to": "엘리베이터 앞",
                "color": UNIFIED_WAYPOINT_COLOR,
                "waypoints": [
                    _wp("1F-WARD-CORNER", "병실 앞 복도 경유점", D1_WARD_CORNER),
                    _wp("1F-WE-X-01", "병실↔엘리베이터 X1", _display_1f_we_x(-37.7)),
                    _wp("1F-WE-X-02", "병실↔엘리베이터 X2", _display_1f_we_x(-35.3)),
                    _wp("1F-WE-X-03", "병실↔엘리베이터 X3", _display_1f_we_x(-33.0)),
                    _wp("1F-WE-X-04", "병실↔엘리베이터 X4", _display_1f_we_x(-30.7)),
                    _wp("1F-WE-X-05", "병실↔엘리베이터 X5", _display_1f_we_x(-28.3)),
                    _wp("1F-ELEVATOR-CORNER", "엘리베이터 방향 코너", D1_ELEVATOR_CORNER),
                    _wp("1F-WE-Y-01", "엘리베이터 접근 Y1", _display_1f_we_y(14.8)),
                    _wp("1F-WE-Y-02", "엘리베이터 접근 Y2", _display_1f_we_y(16.9)),
                    _wp("1F-WE-Y-03", "엘리베이터 접근 Y3", D1_ELEVATOR_Y19),
                ],
            },
        ],
        "waiting_points": [
            {
                "id": "1F-WAIT-WARD",
                "display": (0.208474, 0.636998),
                "display_name": "병실 앞 대기점",
                "segment": "병실 ↔ 엘리베이터 앞",
                "description": "병실 앞 교행 대기",
                "color": WAITING_POINT_COLOR,
                "nav_pose": _nav("1F-WAIT-WARD"),
                "arrival_radius_m": _tol("1F-WAIT-WARD"),
            },
            {
                "id": "1F-WAIT-ELEVATOR",
                "display": (0.342545, 0.425000),
                "display_name": "엘리베이터 앞 대기점",
                "segment": "병실 ↔ 엘리베이터 앞",
                "description": "엘리베이터 진입 전 교행 대기",
                "color": WAITING_POINT_COLOR,
                "nav_pose": _nav("1F-WAIT-ELEVATOR"),
                "arrival_radius_m": _tol("1F-WAIT-ELEVATOR"),
            },
        ],
        # 보관실/병실은 시나리오용 논리 구역이며 별도 공용 좌표는 두지 않습니다.
        # 사용자 지정 물리 목적지는 home_slots와 patient_points가 담당합니다.
        "pois": {
            "보관실": {"point_id": None, "display": (), "nav_pose": None, "arrival_radius_m": 0.0, "display_name": "보관실", "amr_only": True, "storage": True, "hide_marker": True},
            "병실": {"point_id": None, "display": (), "nav_pose": None, "arrival_radius_m": 0.0, "display_name": "환자 병실/OCR", "patient_room": True, "hide_marker": True},
            "엘리베이터 앞": {"point_id": "1F-ELEVATOR", "display": D1_ELEVATOR, "nav_pose": _nav("1F-ELEVATOR"), "arrival_radius_m": _tol("1F-ELEVATOR"), "display_name": "1층 엘리베이터 앞", "elevator": True},
        },
    },
    2: {
        "name": "2층",
        "image": "/static/maps/hospital_map_2f.png",
        "route": ["엘리베이터 앞", "MRI실 앞", "MRI실"],
        "waypoint_routes": [
            {
                "id": "2f_elevator_to_mri_front",
                "from": "엘리베이터 앞",
                "to": "MRI실 앞",
                "color": UNIFIED_WAYPOINT_COLOR,
                "waypoints": [
                    _wp("2F-EM-02", "엘리베이터↔MRI 2", _display_2f_x(-22.0)),
                    _wp("2F-EM-03", "엘리베이터↔MRI 3", _display_2f_x(-18.0)),
                    _wp("2F-EM-04", "엘리베이터↔MRI 4", _display_2f_x(-14.0)),
                    _wp("2F-EM-05", "엘리베이터↔MRI 5", _display_2f_x(-10.0)),
                    _wp("2F-EM-06", "엘리베이터↔MRI 6", _display_2f_x(-6.0)),
                    _wp("2F-EM-07", "엘리베이터↔MRI 7", _display_2f_x(-2.0)),
                    _wp("2F-EM-08", "엘리베이터↔MRI 8", _display_2f_x(2.0)),
                    _wp("2F-MRI-CORNER", "MRI 방향 코너", D2_MRI_CORNER),
                    _wp("2F-EM-Y-01", "MRI 접근 Y1", _display_2f_y(20.0)),
                    _wp("2F-MRI-FRONT", "MRI 검사 대기 기준", D2_MRI_FRONT),
                ],
            },
            {
                "id": "2f_mri_front_to_mri",
                "from": "MRI실 앞",
                "to": "MRI실",
                "color": UNIFIED_WAYPOINT_COLOR,
                "waypoints": [],
            },
        ],
        "waiting_points": [],
        "pois": {
            "엘리베이터 앞": {"point_id": "2F-ELEVATOR", "display": D2_ELEVATOR, "nav_pose": _nav("2F-ELEVATOR"), "arrival_radius_m": _tol("2F-ELEVATOR"), "display_name": "2층 엘리베이터 앞", "elevator": True},
            # MRI실 앞은 시나리오용 논리 구역입니다. 실제 AMR 위치는 /world_pose로 계속 갱신되고,
            # 2F-MRI-FRONT(-6.3,17.0)는 검사 대기 위치 판정용 기준점으로 사용합니다.
            "MRI실 앞": {"point_id": None, "display": (), "nav_pose": None, "arrival_radius_m": 0.0, "display_name": "MRI실 앞", "waiting_area": True, "amr_only": True, "hide_marker": True},
            "MRI실": {"point_id": "2F-MRI", "display": D2_MRI, "nav_pose": _nav("2F-MRI"), "arrival_radius_m": _tol("2F-MRI"), "display_name": "MRI실", "mission_destination": True},
        },
    },
}


def get_nav_pose(floor: int, room: str):
    return FLOORS.get(int(floor), {}).get("pois", {}).get(room, {}).get("nav_pose")


def is_elevator_front(floor: int, room: str) -> bool:
    return bool(FLOORS.get(int(floor), {}).get("pois", {}).get(room, {}).get("elevator", False))


def nav_point_catalog(floor: int):
    """world_pose를 목적지/경유점/대기점으로 판정하기 위한 카탈로그."""
    floor = int(floor)
    cfg = FLOORS.get(floor, {})
    result = []
    for room, poi in cfg.get("pois", {}).items():
        result.append({
            "floor": floor,
            "point_key": f"poi:{floor}:{room}",
            "point_id": poi.get("point_id"),
            "room": room,
            "source": "poi",
            "display": list(poi.get("display") or ()),
            "nav_pose": poi.get("nav_pose"),
            "arrival_radius_m": float(poi.get("arrival_radius_m", DEFAULT_ARRIVAL_TOLERANCE_M)),
            "label": poi.get("display_name", room),
        })
    for patient in cfg.get("patient_points", []):
        result.append({
            "floor": floor,
            "point_key": f"patient:{patient.get('patient_name')}",
            "point_id": patient.get("id"),
            "room": "병실",
            "patient_name": patient.get("patient_name"),
            "source": "patient",
            "display": list(patient.get("display") or ()),
            "nav_pose": patient.get("nav_pose"),
            "arrival_radius_m": float(patient.get("arrival_radius_m", DEFAULT_ARRIVAL_TOLERANCE_M)),
            "label": patient.get("display_name", patient.get("patient_name")),
        })
    for route in cfg.get("waypoint_routes", []):
        for wp in route.get("waypoints", []):
            result.append({
                "floor": floor,
                "point_key": f"{route.get('id')}:{wp.get('id')}",
                "point_id": wp.get("id"),
                "source": "waypoint",
                "display": list(wp.get("display") or ()),
                "nav_pose": wp.get("nav_pose"),
                "arrival_radius_m": float(wp.get("arrival_radius_m", DEFAULT_ARRIVAL_TOLERANCE_M)),
                "label": wp.get("display_name", wp.get("id")),
            })
    for point in cfg.get("waiting_points", []):
        result.append({
            "floor": floor,
            "point_key": f"wait:{floor}:{point.get('id')}",
            "point_id": point.get("id"),
            "source": "waiting",
            "display": list(point.get("display") or ()),
            "nav_pose": point.get("nav_pose"),
            "arrival_radius_m": float(point.get("arrival_radius_m", DEFAULT_ARRIVAL_TOLERANCE_M)),
            "label": point.get("display_name", point.get("id")),
        })
    for amr_name, slot in cfg.get("home_slots", {}).items():
        result.append({
            "floor": floor,
            "point_key": f"home:{amr_name}",
            "point_id": slot.get("id"),
            "source": "home",
            "display": list(slot.get("display") or ()),
            "nav_pose": slot.get("nav_pose"),
            "arrival_radius_m": float(slot.get("arrival_radius_m", DEFAULT_ARRIVAL_TOLERANCE_M)),
            "label": slot.get("display_name", amr_name),
            "amr_name": amr_name,
        })
    return result


def nearest_nav_point(floor: int, x: float, y: float, *, amr_name=None):
    """설정 tolerance 안에 들어온 가장 가까운 포인트를 반환합니다.

    김서울/박인천처럼 tolerance가 겹치더라도 distance_m 최솟값으로 하나만 선택합니다.
    AMR home은 자신의 home 좌표에만 스냅할 수 있습니다.
    """
    candidates = []
    for item in nav_point_catalog(floor):
        nav_pose = item.get("nav_pose")
        if nav_pose is None:
            continue
        if item.get("source") == "home" and item.get("amr_name") not in (None, amr_name):
            continue
        dx = float(x) - float(nav_pose[0])
        dy = float(y) - float(nav_pose[1])
        distance = math.hypot(dx, dy)
        if distance <= float(item.get("arrival_radius_m", DEFAULT_ARRIVAL_TOLERANCE_M)):
            copy = dict(item)
            copy["distance_m"] = distance
            candidates.append(copy)
    if not candidates:
        return None
    # 같은 좌표 alias에서는 대기점 > 환자/홈/POI > 경유점 순으로 의미를 보존하되,
    # 가장 중요한 기준은 실제 좌표와의 거리입니다.
    source_priority = {"waiting": 0, "patient": 1, "home": 1, "poi": 2, "waypoint": 3}
    return min(candidates, key=lambda item: (item["distance_m"], source_priority.get(item.get("source"), 9)))


# ----------------------------------------------------------------------
# world_pose 연속 지도 표시
# ----------------------------------------------------------------------
def _catalog_by_point_id(floor: int):
    result = {}
    for item in nav_point_catalog(floor):
        point_id = item.get("point_id")
        if point_id and item.get("nav_pose") is not None and item.get("display"):
            result.setdefault(point_id, item)
    return result


def _display_projection_paths(floor: int):
    if int(floor) == 1:
        shared_storage = ["1F-SW-01", "1F-SW-02", "1F-SW-03", "1F-SW-04", "1F-SW-05", "1F-WARD-CORNER"]
        to_elevator = ["1F-WARD-CORNER", "1F-WE-X-01", "1F-WE-X-02", "1F-WE-X-03", "1F-WE-X-04", "1F-WE-X-05", "1F-ELEVATOR-CORNER", "1F-WE-Y-01", "1F-WE-Y-02", "1F-WE-Y-03", "1F-ELEVATOR"]
        return [
            ["1F-AMR1-HOME", "1F-SW-01"],
            ["1F-AMR2-HOME", "1F-SW-01"],
            shared_storage,
            ["1F-WARD-CORNER", "1F-KIM-SEOUL-OCR"],
            ["1F-WARD-CORNER", "1F-PARK-INCHEON-OCR"],
            ["1F-WARD-CORNER", "1F-SEO-SUWON-OCR"],
            to_elevator,
            ["1F-WARD-CORNER", "1F-WAIT-WARD"],
            ["1F-WE-X-04", "1F-WAIT-ELEVATOR"],
        ]
    if int(floor) == 2:
        return [
            [
                "2F-ELEVATOR", "2F-EM-02", "2F-EM-03", "2F-EM-04", "2F-EM-05",
                "2F-EM-06", "2F-EM-07", "2F-EM-08", "2F-MRI-CORNER", "2F-EM-Y-01", "2F-MRI",
            ],
            # MRI 도착 뒤 강제 11m 후진 구간은 별도 선분으로 투영합니다.
            ["2F-MRI", "2F-MRI-FRONT"],
        ]
    return []


def _project_segment(x, y, a, b):
    ax, ay = float(a["nav_pose"][0]), float(a["nav_pose"][1])
    bx, by = float(b["nav_pose"][0]), float(b["nav_pose"][1])
    vx, vy = bx - ax, by - ay
    length_sq = vx * vx + vy * vy
    if length_sq <= 1e-12:
        t = 0.0
    else:
        t = ((float(x) - ax) * vx + (float(y) - ay) * vy) / length_sq
        t = max(0.0, min(1.0, t))
    px, py = ax + t * vx, ay + t * vy
    distance = math.hypot(float(x) - px, float(y) - py)
    da, db = a["display"], b["display"]
    display = [_lerp(da[0], db[0], t), _lerp(da[1], db[1], t)]
    return distance, display


def nav_xy_to_display(floor: int, x: float, y: float):
    """raw world_pose를 첨부 지도상의 연속 위치로 변환합니다.

    시나리오가 실제로 이동하는 복도/병실 진입/보관실 진입 선분 중 가장 가까운 선분에
    투영하여, 등록 좌표 사이에서도 AMR 마커가 1초마다 자연스럽게 이동하도록 합니다.
    """
    floor = int(floor)
    catalog = _catalog_by_point_id(floor)
    best = None
    for path in _display_projection_paths(floor):
        items = [catalog.get(point_id) for point_id in path]
        items = [item for item in items if item is not None]
        for a, b in zip(items, items[1:]):
            distance, display = _project_segment(x, y, a, b)
            if best is None or distance < best[0]:
                best = (distance, display)
    if best is None:
        return None
    # 지도 경로에서 너무 멀리 벗어난 좌표는 잘못된 위치로 투영하지 않습니다.
    max_distance = 4.0 if floor == 1 else 4.5
    return best[1] if best[0] <= max_distance else None


# 기존 진단/테스트에서 사용하던 affine helper는 일반 수학 함수로 유지합니다.
def _solve_3x3(matrix, vector):
    a = [list(map(float, row)) + [float(rhs)] for row, rhs in zip(matrix, vector)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(a[r][col]))
        if abs(a[pivot][col]) < 1e-12:
            return None
        if pivot != col:
            a[col], a[pivot] = a[pivot], a[col]
        scale = a[col][col]
        a[col] = [value / scale for value in a[col]]
        for row in range(3):
            if row == col:
                continue
            factor = a[row][col]
            a[row] = [a[row][idx] - factor * a[col][idx] for idx in range(4)]
    return [a[row][3] for row in range(3)]


def _fit_affine(anchors):
    if len(anchors) < 3:
        return None
    ata = [[0.0] * 3 for _ in range(3)]
    atu = [0.0] * 3
    atv = [0.0] * 3
    for item in anchors:
        row = [item["x"], item["y"], 1.0]
        for i in range(3):
            atu[i] += row[i] * item["u"]
            atv[i] += row[i] * item["v"]
            for j in range(3):
                ata[i][j] += row[i] * row[j]
    u_coeff = _solve_3x3(ata, atu)
    v_coeff = _solve_3x3(ata, atv)
    if u_coeff is None or v_coeff is None:
        return None
    squared_error = 0.0
    for item in anchors:
        u = u_coeff[0] * item["x"] + u_coeff[1] * item["y"] + u_coeff[2]
        v = v_coeff[0] * item["x"] + v_coeff[1] * item["y"] + v_coeff[2]
        squared_error += (u - item["u"]) ** 2 + (v - item["v"]) ** 2
    return {
        "u_coeff": u_coeff,
        "v_coeff": v_coeff,
        "anchor_count": len(anchors),
        "rmse_display": math.sqrt(squared_error / max(1, len(anchors))),
    }


def map_transform_info(floor: int):
    paths = _display_projection_paths(int(floor))
    return {
        "ready": bool(paths),
        "mode": "piecewise_route_projection",
        "path_count": len(paths),
        "anchor_count": len(_catalog_by_point_id(int(floor))),
    }
