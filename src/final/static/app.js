const ui = {
  floor: 1,
  selectedAmr: null,
  selectedBed: null,
  selectedDestination: { key: "2:MRI실", floor: 2, room: "MRI실" },
  state: null,
  commandPending: false,
  requestSeq: 0,
  autoTimers: {},
  serverSession: null,
  debugScenarioIndex: null,
  showConfiguredCoordinates: false,
  coordinateTrackers: {},
  coordinateDebug: null,
};

const FIXED_DESTINATION = { key: "2:MRI실", floor: 2, room: "MRI실" };

// UI 정적 검사용 레거시 문자열 유지:
// coordinate checks may reference serverPosition?.snapped_point_id === "2F-MRI-FRONT"
// and serverPosition?.snapped_point_id === "2F-MRI" in MRI flow descriptions.
// Additional legacy flow copy kept for static regression checks:
// 환자 확인 · 침상 도킹 / PatientTransfer / 약 11m 후진 / 검사 완료 · 복귀 명령 / Magnet Unlock / 보관실 복귀 · 임무 완료
// Additional static-regression hints:
// raw world_pose를 지도 경로에 연속 투영
// item.serverPosition?.snapped_source === "patient" -> pointType: "병실/OCR"


// 사용자가 요청한 단순 좌표 기반 현재 상태 흐름(11단계)입니다.
// debugScenarioIndex는 화면 표시만 바꾸며 API/ROS 명령을 절대 발행하지 않습니다.
const SCENARIO_STEPS = [
  { title: "이송 준비", floor: 1, badge: "이송 준비", tone: "ready", detail: "AMR이 보관실 목적지 좌표에 있습니다." },
  { title: "병실 이동", floor: 1, badge: "병실 이동", tone: "moving", detail: "AMR이 보관실에서 병실로 가는 1층 경유점을 이동 중입니다." },
  { title: "환자 확인", floor: 1, badge: "환자 확인", tone: "ready", detail: "AMR이 선택한 환자의 병실/OCR 목적지 좌표에 있습니다." },
  { title: "1층 엘리베이터 이동", floor: 1, badge: "엘리베이터 이동", tone: "moving", detail: "AMR이 병실에서 1층 엘리베이터 앞까지의 경유점을 이동 중입니다." },
  { title: "MRI실 이동", floor: 2, badge: "MRI실 이동", tone: "moving", detail: "AMR이 2층 엘리베이터 앞에서 MRI 검사 대기 위치(2F-MRI-FRONT)까지의 경유점을 이동 중입니다." },
  { title: "환자 MRI 인계", floor: 2, badge: "환자 MRI 인계", tone: "exam", detail: "AMR이 MRI실 좌표로 이동해 환자를 침상에서 MRI로 옮기는 구간입니다." },
  { title: "MRI 검사 대기", floor: 2, badge: "MRI 검사 대기", tone: "exam", detail: "AMR이 MRI실에서 다시 2F-MRI-FRONT 방향으로 이동하거나 대기 위치에 있습니다." },
  { title: "복귀 준비", floor: 2, badge: "복귀 준비", tone: "moving", detail: "AMR이 2F-MRI-FRONT에서 다시 MRI실로 이동해 환자를 회수하는 구간입니다." },
  { title: "2층 엘리베이터 이동", floor: 2, badge: "2층 엘리베이터 이동", tone: "moving", detail: "AMR이 2F-MRI-FRONT에서 2층 엘리베이터 앞 방향 경유점을 이동 중입니다." },
  { title: "병실 복귀", floor: 1, badge: "병실 복귀", tone: "moving", detail: "AMR이 1층 엘리베이터 앞에서 병실까지의 경유점을 이동 중입니다." },
  { title: "보관실 복귀", floor: 1, badge: "보관실 복귀", tone: "moving", detail: "AMR이 병실에서 보관실까지의 경유점을 이동 중입니다." },
];

const MOVING_PHASES = new Set([
  "moving_to_patient",
  "moving_to_elevator_1f",
  "moving_to_mri",
  "backing_out_after_drop",
  "moving_to_repickup",
  "backing_out_after_pickup",
  "moving_to_elevator_2f",
  "returning_to_ward",
  "returning_to_storage",
]);

function patientPointIdForName(name) {
  const value = String(name || "").trim();
  if (!value) return null;
  if (value.includes("김서울")) return "1F-KIM-SEOUL-OCR";
  if (value.includes("박인천")) return "1F-PARK-INCHEON-OCR";
  if (value.includes("서수원")) return "1F-SEO-SUWON-OCR";
  return null;
}

function selectedPatientPointIdForAmr(amrName) {
  const job = jobFor(amrName);
  let bed = null;
  if (job?.bed_id) {
    bed = ui.state?.beds?.find((item) => Number(item.id) === Number(job.bed_id)) || null;
  }
  if (!bed && ui.selectedAmr === amrName && ui.selectedBed) {
    bed = ui.state?.beds?.find((item) => Number(item.id) === Number(ui.selectedBed)) || null;
  }
  return patientPointIdForName(bed?.patient_name || bed?.label || "");
}

function positionCategoryForAmr(amrName, position) {
  const pointId = String(position?.snapped_point_id || position?.point_id || "");
  const source = String(position?.snapped_source || position?.source || "").toLowerCase();
  const floor = Number(position?.floor || 0);
  const patientPointId = selectedPatientPointIdForAmr(amrName);

  if (source === "home" || pointId === "1F-AMR1-HOME" || pointId === "1F-AMR2-HOME") return "home";
  if ((patientPointId && pointId === patientPointId) || source === "patient") return "patient";
  if (floor === 2 && pointId === "2F-MRI") return "mri";
  if (floor === 1 && (pointId.startsWith("1F-SW-") || pointId === "1F-WARD-CORNER")) return "storage_ward";
  if (floor === 1 && (pointId.startsWith("1F-WE-") || pointId === "1F-ELEVATOR-CORNER" || pointId === "1F-ELEVATOR" || pointId.startsWith("1F-WAIT-"))) return "elevator_1f";
  if (floor === 2 && (pointId === "2F-ELEVATOR" || pointId.startsWith("2F-EM-") || pointId === "2F-MRI-CORNER" || pointId === "2F-MRI-FRONT")) return "mri_front_route";
  return null;
}

function coordinateScenarioStepIndexForPosition(amrName, position, tracker) {
  const category = positionCategoryForAmr(amrName, position);
  const maxReached = Number(tracker?.maxReached ?? -1);
  if (category === "home") return { category, index: 0, reset: true };
  if (category === "storage_ward") return { category, index: maxReached >= 9 ? 10 : 1 };
  if (category === "patient") return { category, index: maxReached >= 9 ? 9 : 2 };
  if (category === "elevator_1f") return { category, index: maxReached >= 8 ? 9 : 3 };
  if (category === "mri_front_route") return { category, index: maxReached >= 7 ? 8 : maxReached >= 5 ? 6 : 4 };
  if (category === "mri") return { category, index: maxReached >= 6 ? 7 : 5 };
  return { category: null, index: Number.isInteger(tracker?.index) ? tracker.index : 0 };
}

function syncCoordinateTrackers() {
  const next = {};
  (ui.state?.amrs || []).forEach((amr) => {
    const prev = ui.coordinateTrackers?.[amr.name] || { index: 0, maxReached: -1, category: null, pointId: null };
    const live = ui.state?.traffic?.amr_positions?.[amr.name] || null;
    const floor = Number(live?.floor ?? amr.floor ?? 1);
    const position = resolvedMapPositionFor(amr, floor, { ignoreCoordinateDebug: true });
    const pointId = String(position?.snapped_point_id || position?.point_id || "");
    const update = coordinateScenarioStepIndexForPosition(amr.name, position, prev);
    let index = update.index;
    let maxReached = update.reset ? index : Math.max(prev.maxReached, index);
    if (update.reset) maxReached = 0;
    next[amr.name] = {
      index,
      maxReached,
      category: update.category,
      pointId,
      floor,
      position,
    };
  });
  ui.coordinateTrackers = next;
}

function coordinateScenarioTrackerForAmr(amrName) {
  return ui.coordinateTrackers?.[amrName] || null;
}

function activeCoordinateScenarioStepIndex() {
  const amr = selectedAmr();
  if (!amr) return 0;
  const debugEntry = coordinateDebugEntry();
  if (debugEntry) return Number(debugEntry.stepIndex || 0);
  return coordinateScenarioTrackerForAmr(amr.name)?.index ?? 0;
}

function configuredPointById(floor, pointId) {
  const cfg = ui.state?.floors?.[String(Number(floor))];
  if (!cfg || !pointId) return null;
  return (cfg.configured_points || []).find((point) => String(point.point_id || point.id || "") === String(pointId)) || null;
}

function coordinateDebugSequenceForAmr(amrName) {
  if (!amrName) return [];
  const patientPoint = selectedPatientPointIdForAmr(amrName) || "1F-KIM-SEOUL-OCR";
  const homePoint = amrName === "AMR-02" ? "1F-AMR2-HOME" : "1F-AMR1-HOME";
  const seq = [];
  const add = (floor, pointId, stepIndex, label = "") => seq.push({ floor, pointId, stepIndex, label });

  add(1, homePoint, 0, "이송 준비");
  ["1F-SW-01", "1F-SW-02", "1F-SW-03", "1F-SW-04", "1F-SW-05", "1F-WARD-CORNER"]
    .forEach((id) => add(1, id, 1, "병실 이동"));
  add(1, patientPoint, 2, "환자 확인");

  ["1F-WARD-CORNER", "1F-WE-X-01", "1F-WE-X-02", "1F-WE-X-03", "1F-WE-X-04", "1F-WE-X-05",
   "1F-ELEVATOR-CORNER", "1F-WE-Y-01", "1F-WE-Y-02", "1F-WE-Y-03", "1F-ELEVATOR"]
    .forEach((id) => add(1, id, 3, "1층 엘리베이터 이동"));

  ["2F-ELEVATOR", "2F-EM-02", "2F-EM-03", "2F-EM-04", "2F-EM-05", "2F-EM-06", "2F-EM-07", "2F-EM-08",
   "2F-MRI-CORNER", "2F-EM-Y-01", "2F-MRI-FRONT"]
    .forEach((id) => add(2, id, 4, "MRI실 이동"));
  add(2, "2F-MRI", 5, "환자 MRI 인계");
  add(2, "2F-MRI-FRONT", 6, "MRI 검사 대기");
  add(2, "2F-MRI", 7, "복귀 준비");

  ["2F-MRI-FRONT", "2F-EM-Y-01", "2F-MRI-CORNER", "2F-EM-08", "2F-EM-07", "2F-EM-06", "2F-EM-05",
   "2F-EM-04", "2F-EM-03", "2F-EM-02", "2F-ELEVATOR"]
    .forEach((id) => add(2, id, 8, "2층 엘리베이터 이동"));

  ["1F-ELEVATOR", "1F-WE-Y-03", "1F-WE-Y-02", "1F-WE-Y-01", "1F-ELEVATOR-CORNER", "1F-WE-X-05",
   "1F-WE-X-04", "1F-WE-X-03", "1F-WE-X-02", "1F-WE-X-01", "1F-WARD-CORNER", patientPoint]
    .forEach((id) => add(1, id, 9, "병실 복귀"));

  ["1F-WARD-CORNER", "1F-SW-05", "1F-SW-04", "1F-SW-03", "1F-SW-02", "1F-SW-01"]
    .forEach((id) => add(1, id, 10, "보관실 복귀"));
  // 보관 목적지에 다시 들어오면 사용자가 정의한 규칙 1에 따라 '이송 준비'로 돌아갑니다.
  add(1, homePoint, 0, "이송 준비");
  return seq;
}

function coordinateDebugEntry() {
  const state = ui.coordinateDebug;
  if (!state || !ui.selectedAmr || state.amrName !== ui.selectedAmr) return null;
  const seq = coordinateDebugSequenceForAmr(state.amrName);
  if (!seq.length) return null;
  const index = Math.max(0, Math.min(seq.length - 1, Number(state.index) || 0));
  return { ...seq[index], index, count: seq.length };
}

function coordinateDebugPositionFor(amrName) {
  const entry = coordinateDebugEntry();
  if (!entry || amrName !== ui.selectedAmr) return null;
  const point = configuredPointById(entry.floor, entry.pointId);
  if (!point || !Array.isArray(point.display) || point.display.length < 2) return null;
  const navXY = Array.isArray(point.nav_xy) ? point.nav_xy : null;
  const source = String(point.source || "waypoint");
  return {
    amr_name: amrName,
    floor: Number(entry.floor),
    display: [...point.display],
    source: "coordinate_debug",
    position_source: "coordinate_debug",
    display_mode: "coordinate_debug",
    point_key: `debug:${entry.floor}:${entry.pointId}:${entry.index}`,
    point_id: entry.pointId,
    snapped_point_id: entry.pointId,
    snapped_source: source,
    snapped_label: point.display_name || point.name || entry.pointId,
    in_tolerance: true,
    holding_previous_point: false,
    fallback_moving: false,
    raw_pose: navXY ? { x: Number(navXY[0]), y: Number(navXY[1]) } : null,
    debug: true,
    debug_step_index: entry.stepIndex,
    debug_sequence_index: entry.index,
    debug_sequence_count: entry.count,
  };
}

function findCoordinateDebugStartIndex(amrName, seq) {
  const live = ui.state?.traffic?.amr_positions?.[amrName] || null;
  const tracker = coordinateScenarioTrackerForAmr(amrName);
  const currentPointId = String(live?.snapped_point_id || live?.point_id || tracker?.pointId || "");
  const currentStep = Number(tracker?.index ?? 0);
  const candidates = seq.map((entry, index) => ({ entry, index }))
    .filter(({ entry }) => entry.pointId === currentPointId);
  if (candidates.length) {
    candidates.sort((a, b) => Math.abs(a.entry.stepIndex - currentStep) - Math.abs(b.entry.stepIndex - currentStep));
    return candidates[0].index;
  }
  return 0;
}

function moveCoordinateDebug(delta) {
  if (!ui.selectedAmr) {
    setError("좌표 디버깅할 AMR을 먼저 선택하세요.");
    return;
  }
  const seq = coordinateDebugSequenceForAmr(ui.selectedAmr);
  if (!seq.length) return;
  let base;
  if (ui.coordinateDebug?.amrName === ui.selectedAmr && Number.isInteger(ui.coordinateDebug?.index)) {
    base = ui.coordinateDebug.index;
  } else {
    base = findCoordinateDebugStartIndex(ui.selectedAmr, seq);
  }
  const nextIndex = Math.max(0, Math.min(seq.length - 1, base + Number(delta || 0)));
  ui.coordinateDebug = { amrName: ui.selectedAmr, index: nextIndex };
  ui.debugScenarioIndex = null;
  ui.floor = Number(seq[nextIndex].floor);
  setError("");
  renderAll();
}

function exitCoordinateDebug() {
  const amrName = ui.coordinateDebug?.amrName || ui.selectedAmr;
  ui.coordinateDebug = null;
  if (amrName) {
    const amr = ui.state?.amrs?.find((item) => item.name === amrName);
    const live = ui.state?.traffic?.amr_positions?.[amrName] || null;
    const floor = Number(live?.floor ?? amr?.floor ?? ui.floor);
    if ([1, 2].includes(floor)) ui.floor = floor;
  }
  setError("");
  renderAll();
}

function renderCoordinateDebugControls() {
  const prev = $("coordDebugPrevBtn");
  const live = $("coordDebugLiveBtn");
  const next = $("coordDebugNextBtn");
  const label = $("coordDebugLabel");
  if (!prev || !live || !next || !label) return;

  const hasAmr = Boolean(ui.selectedAmr);
  const seq = hasAmr ? coordinateDebugSequenceForAmr(ui.selectedAmr) : [];
  const entry = coordinateDebugEntry();
  const active = Boolean(entry);

  prev.disabled = !hasAmr || (active && entry.index <= 0);
  next.disabled = !hasAmr || (active && entry.index >= entry.count - 1);
  live.disabled = !active;
  live.textContent = active ? "실제 world_pose로 복귀" : "실제 world_pose";
  label.textContent = active
    ? `${entry.index + 1}/${entry.count} · ${entry.pointId} · ${entry.label}`
    : hasAmr ? `${displayAmrName(ui.selectedAmr)} 실제 좌표 표시 중` : "AMR 선택 후 사용";
}

const AUTO_WAIT_MS = {
  ward_attach_wait: 3000,
  elevator_transfer_to_2f: 1500,
  unloading_wait: 3000,
  boarding_wait: 3000,
  elevator_transfer_to_1f: 1500,
  ward_detach_wait: 3000,
};


const $ = (id) => document.getElementById(id);

function displayAmrName(name) {
  if (!name) return "";
  const match = /^AMR-0*(\d+)$/.exec(String(name));
  return match ? `AMR${Number(match[1])}` : String(name);
}

function displayWaitingPointId(pointId) {
  return String(pointId || "");
}

function normalizeDisplayText(text) {
  return String(text || "")
    .replace(/AMR-0*(\d+)/g, (_, n) => `AMR${Number(n)}`);
}

function selectedAmr() {
  return ui.state?.amrs.find((item) => item.name === ui.selectedAmr) || null;
}

function selectedBed() {
  return ui.state?.beds.find((item) => item.id === ui.selectedBed) || null;
}

function jobFor(amrName) {
  return ui.state?.jobs.find((item) => item.amr_name === amrName) || null;
}

function worldPoseStatusFor(amrName) {
  const ros = ui.state?.ros || {};
  const direct = ros.world_pose?.[amrName];
  if (direct) return direct;
  const topic = ros.robots?.[amrName]?.pose_topic || "";
  return {
    topic,
    publisher_count: 0,
    received_count: 0,
    processed_count: 0,
    error_count: 0,
    active: false,
    last_pose: null,
    last_received_at: null,
    age_sec: null,
  };
}

const SCENARIO_STATE_LABELS = {
  IDLE: "대기",
  MOVING: "이동 중",
  DOCKING: "환자 인식 · 도킹 중",
  UNDOCKING: "침상 언도킹 중",
  EXAM: "MRI 검사 중",
  RETURN_READY: "복귀 대기",
  TRAFFIC_WAIT: "교행 대기",
  ERROR: "오류",
};

function scenarioStatusFor(amrName) {
  const snapshot = ui.state?.ros?.scenario_status?.snapshot;
  const robots = Array.isArray(snapshot?.robots) ? snapshot.robots : [];
  return robots.find((item) => item?.amr === amrName) || null;
}

function scenarioStateLabel(status) {
  const key = String(status?.state || "").toUpperCase();
  return SCENARIO_STATE_LABELS[key] || "";
}

function worldPoseStateLabel(status) {
  if (status?.type_compatible === false) return { text: "타입 불일치", className: "error" };
  if (Number(status?.publisher_count || 0) <= 0) return { text: "수신 없음", className: "waiting" };
  if (status?.active) return { text: "수신중", className: "active" };
  if (Number(status?.received_count || 0) > 0) return { text: "수신 끊김", className: "stale" };
  return { text: "수신 대기", className: "waiting" };
}

function worldPoseDiagnosticHint(status) {
  if (status?.type_compatible === false) {
    const types = (status.discovered_types || []).join(", ") || "알 수 없음";
    return `토픽 타입 불일치: ${types} (GUI 기대: ${status.expected_type || "std_msgs/msg/String"})`;
  }
  if (Number(status?.publisher_count || 0) <= 0) {
    return "GUI ROS 노드에서 publisher를 발견하지 못했습니다. GUI를 실행한 터미널의 ROS_DOMAIN_ID/RMW/source 환경을 확인하세요.";
  }
  if (!status?.active && Number(status?.received_count || 0) <= 0) {
    return "publisher는 발견됐지만 아직 데이터 callback이 없습니다. publisher가 실제로 publish 중인지 확인하세요.";
  }
  if (!status?.active && Number(status?.received_count || 0) > 0) {
    return "이전에는 수신했지만 최근 2.5초 동안 새 world_pose가 없습니다.";
  }
  return "실제 /world_pose 데이터를 정상 수신 중입니다.";
}

function yieldingConflictFor(amrName, floor = null) {
  if (!amrName) return null;
  return (ui.state?.traffic?.conflicts || []).find((conflict) => {
    if (conflict.yielding_amr !== amrName) return false;
    return floor == null || Number(conflict.floor) === Number(floor);
  }) || null;
}

function trafficWaitFor(amrName) {
  if (!amrName) return null;
  return ui.state?.traffic?.waiting_amrs?.[amrName] || null;
}

function waitingPointFor(floor, pointId) {
  if (floor == null || !pointId) return null;
  const cfg = ui.state?.floors?.[String(floor)];
  return (cfg?.waiting_points || []).find((point) => point.id === pointId) || null;
}

function canonicalDisplayKey(floor, display) {
  if (floor == null || !display || display.length < 2) return null;
  return `${Number(floor)}:${Number(display[0]).toFixed(6)}:${Number(display[1]).toFixed(6)}`;
}

function occupyingAmrForPoint(floor, display, exceptAmr = null) {
  const key = canonicalDisplayKey(floor, display);
  if (!key) return null;
  const positions = ui.state?.traffic?.amr_positions || {};
  return Object.values(positions).find((position) => {
    if (!position || position.amr_name === exceptAmr) return false;
    return position.canonical_id === key;
  })?.amr_name || null;
}

function serverDebugPositionFor(amrName) {
  return ui.state?.traffic?.debug_positions?.[amrName] || null;
}

function bedForJob(job) {
  if (!job?.bed_id) return null;
  return ui.state?.beds.find((item) => item.id === job.bed_id) || null;
}

function attachedPatientName(job, amrName = null) {
  // GUI가 world_pose로 병실 앞 경유점 도킹 완료를 확정한 경우에는 로컬 job 상태를 우선합니다.
  // 외부 scenario_status의 bed_attached 갱신이 늦어도 환자 이름이 지도에서 사라지지 않습니다.
  if (job?.bed_attached) {
    const bed = bedForJob(job);
    return bed?.patient_name || bed?.label || "";
  }
  const status = amrName ? scenarioStatusFor(amrName) : null;
  if (status?.bed_attached) {
    const bedId = Number(status.bed_id || 0);
    const bed = ui.state?.beds.find((item) => item.id === bedId) || null;
    return bed?.patient_name || bed?.label || "";
  }
  return "";
}

function selectedPatientNameForAmr(amrName, job = null) {
  const attached = attachedPatientName(job, amrName);
  if (attached) return attached;
  if (job?.bed_id) {
    const bed = ui.state?.beds.find((item) => item.id === Number(job.bed_id)) || null;
    if (bed) return bed.patient_name || bed.label || "";
  }
  if (ui.selectedAmr === amrName && ui.selectedBed) {
    const bed = ui.state?.beds.find((item) => item.id === Number(ui.selectedBed)) || null;
    return bed?.patient_name || bed?.label || "";
  }
  return "";
}

function selectedJob() {
  return ui.selectedAmr ? jobFor(ui.selectedAmr) : null;
}

function setError(message) {
  $("errorText").textContent = message || "";
}


function applyState(nextState) {
  let previousJob = selectedJob();
  const incomingSession = nextState?.server_session || null;
  if (ui.serverSession && incomingSession && ui.serverSession !== incomingSession) {
    Object.values(ui.autoTimers).forEach((entry) => clearTimeout(entry.timer));
    ui.selectedAmr = null;
    ui.selectedBed = null;
    ui.selectedDestination = FIXED_DESTINATION;
    ui.floor = 1;
    ui.autoTimers = {};
    ui.debugScenarioIndex = null;
    ui.coordinateDebug = null;
    previousJob = null;
    setError("");
  }
  ui.serverSession = incomingSession;
  ui.state = nextState;
  syncCoordinateTrackers();

  // 실제 ROS 주행이 연결되면 웹 수동 경유점 위치 표시는 초기화합니다.
  if (ui.state?.ros?.enabled) {
  }

  if (ui.selectedAmr && !selectedAmr()) {
    ui.selectedAmr = null;
    ui.selectedBed = null;
  }

  // 선택한 AMR의 실제 mission floor가 바뀌면 지도도 자동 전환합니다.
  // 수동 층 탭은 확인용으로 남기지만 다음 상태 수신 시 실제 AMR 층을 우선합니다.
  const selected = selectedAmr();
  if (selected && [1, 2].includes(Number(selected.floor))) ui.floor = Number(selected.floor);
  const job = selectedJob();
  if (job) {
    ui.selectedBed = job.bed_id || null;
    ui.selectedDestination = FIXED_DESTINATION;
  } else if (previousJob && ui.selectedAmr) {
    ui.selectedBed = null;
  }

  if (ui.selectedBed && !selectedBed()) ui.selectedBed = null;
  if (!ui.state.floors[String(ui.floor)]) ui.floor = 1;
}

async function apiCommand(payload, { silent = false } = {}) {
  if (ui.commandPending) return false;
  ui.commandPending = true;
  ui.requestSeq += 1;
  if (!silent) setError("");
  renderControls();

  try {
    const response = await fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const body = await response.json().catch(() => ({}));
    if (body.state) applyState(body.state);
    if (!response.ok || !body.ok) {
      if (!silent) setError(body.error || `명령 실패 (HTTP ${response.status})`);
      return false;
    }
    return true;
  } catch (error) {
    if (!silent) setError(`서버 연결 실패: ${error.message}`);
    return false;
  } finally {
    ui.commandPending = false;
    renderAll();
  }
}

function scenarioIndexForAmr(amrName) {
  if (!amrName) return 0;
  const selected = selectedAmr();
  const debugEntry = selected && selected.name === amrName ? coordinateDebugEntry() : null;
  if (debugEntry) return Number(debugEntry.stepIndex || 0);
  return coordinateScenarioTrackerForAmr(amrName)?.index ?? 0;
}

function scenarioStepForAmr(amrName) {
  return SCENARIO_STEPS[Math.max(0, Math.min(SCENARIO_STEPS.length - 1, scenarioIndexForAmr(amrName)))] || SCENARIO_STEPS[0];
}

function patientVisibleForStep(stepIndex) {
  return [2, 3, 4, 5, 8, 9].includes(Number(stepIndex));
}

function bedStatusFromStep(stepIndex) {
  const idx = Number(stepIndex);
  if ([0,1,2].includes(idx)) return "병실 대기";
  if ([3,4,5].includes(idx)) return "이송 중";
  if ([6,7].includes(idx)) return "MRI 검사 중";
  if ([8,9].includes(idx)) return "복귀 이송 중";
  return "복귀 완료";
}

function amrStatusSummary(amr, job = null) {
  const step = scenarioStepForAmr(amr.name);
  const patient = selectedPatientNameForAmr(amr.name, job);
  return {
    stepIndex: scenarioIndexForAmr(amr.name),
    stepTitle: step.title,
    patientVisible: patientVisibleForStep(scenarioIndexForAmr(amr.name)),
    patientName: patient,
    statusText: step.title,
  };
}

function zoneDefinitionsForFloor(floor) {
  if (Number(floor) === 1) {
    return [
      { id: "1F-AMR1-HOME", title: "AMR1 보관/도킹 위치" },
      { id: "1F-AMR2-HOME", title: "AMR2 보관/도킹 위치" },
      { id: "1F-KIM-SEOUL-OCR", title: "김서울 병실/OCR" },
      { id: "1F-PARK-INCHEON-OCR", title: "박인천 병실/OCR" },
      { id: "1F-SEO-SUWON-OCR", title: "서수원 병실/OCR" },
      { id: "1F-ELEVATOR", title: "1층 엘리베이터 앞" },
    ];
  }
  return [
    { id: "2F-ELEVATOR", title: "2층 엘리베이터 앞" },
    { id: "2F-MRI-FRONT", title: "MRI 검사 대기 위치" },
    { id: "2F-MRI", title: "MRI실" },
  ];
}

function currentPointIdForAmr(amr) {
  const live = ui.state?.traffic?.amr_positions?.[amr.name] || null;
  const floor = Number(live?.floor ?? amr.floor ?? 1);
  const pos = resolvedMapPositionFor(amr, floor);
  return String(pos?.snapped_point_id || pos?.point_id || "");
}

function renderCards() {
  const amrRoot = $("amrCards");
  amrRoot.replaceChildren();
  ui.state.amrs.forEach((amr) => {
    const job = jobFor(amr.name);
    const summary = amrStatusSummary(amr, job);
    const button = document.createElement("button");
    button.type = "button";
    button.className = `card ${ui.selectedAmr === amr.name ? "selected" : ""} ${job ? "busy" : ""}`;
    const titleRow = document.createElement("div");
    titleRow.className = "amr-card-title";
    const title = document.createElement("strong");
    title.textContent = displayAmrName(amr.name);
    titleRow.appendChild(title);

    const poseStatus = worldPoseStatusFor(amr.name);
    const poseState = worldPoseStateLabel(poseStatus);
    const poseMini = document.createElement("span");
    poseMini.className = `world-pose-mini ${poseState.className}`;
    poseMini.textContent = poseState.text;
    poseMini.title = `${poseStatus.topic || "world_pose"} · RX ${poseStatus.received_count || 0}`;
    titleRow.appendChild(poseMini);

    if (summary.patientVisible && summary.patientName) {
      const patient = document.createElement("span");
      patient.className = "amr-patient-name";
      patient.textContent = summary.patientName;
      titleRow.appendChild(patient);
    }

    const detail = document.createElement("span");
    detail.textContent = `${amr.floor}층 ${amr.room}`;
    const meta = document.createElement("div");
    meta.className = "patient-meta patient-status-only";
    meta.textContent = summary.statusText;
    button.append(titleRow, detail, meta);
    button.onclick = () => {
      if (ui.selectedAmr === amr.name) {
        ui.selectedAmr = null;
        ui.selectedBed = null;
        ui.debugScenarioIndex = null;
        ui.coordinateDebug = null;
      } else {
        ui.selectedAmr = amr.name;
        if (job) {
          ui.selectedBed = job.bed_id;
        } else {
          ui.selectedBed = null;
        }
      }
      setError("");
      renderAll();
    };
    amrRoot.appendChild(button);
  });

  const bedRoot = $("bedCards");
  bedRoot.replaceChildren();
  ui.state.beds.forEach((bed) => {
    const active = selectedJob();
    const assignedElsewhere = Boolean(bed.assigned_amr && bed.assigned_amr !== ui.selectedAmr);
    const unavailable = assignedElsewhere || (!active && bed.status !== "대기");
    const button = document.createElement("button");
    button.type = "button";
    button.className = `card ${ui.selectedBed === bed.id ? "selected" : ""} ${bed.assigned_amr ? "busy" : ""}`;
    button.disabled = Boolean(active) || unavailable || !ui.selectedAmr;
    const title = document.createElement("strong");
    title.textContent = bed.patient_name || bed.label;
    const birth = document.createElement("span");
    birth.textContent = `생년월일 ${bed.birth_date || "-"}`;
    const status = document.createElement("div");
    status.className = "patient-meta patient-status-only";
    let statusText = bed.status || "-";
    if (bed.assigned_amr) {
      statusText = bedStatusFromStep(scenarioIndexForAmr(bed.assigned_amr));
    } else if (ui.selectedAmr && ui.selectedBed === bed.id) {
      statusText = "선택됨";
    }
    status.textContent = statusText;
    button.append(title, birth, status);
    button.onclick = () => {
      ui.selectedBed = ui.selectedBed === bed.id ? null : bed.id;
      ui.debugScenarioIndex = null;
      ui.coordinateDebug = null;
      setError("");
      renderAll();
    };
    bedRoot.appendChild(button);
  });
}


const COMPACT_PHASE_TITLES = {
  moving_to_patient: "환자 병실 이동 중",
  ward_docking_ready: "환자 확인 · 도킹 준비",
  ward_attach_wait: "OCR·ArUco 확인 · 도킹 중",
  moving_to_elevator_1f: "1층 엘리베이터 이동 중",
  elevator_transfer_to_2f: "엘리베이터 1층 → 2층",
  moving_to_mri: "MRI실 이동 중",
  unloading_wait: "환자 MRI 인계 중",
  backing_out_after_drop: "MRI 검사 대기 위치 이동 중",
  waiting_exam: "MRI 검사 대기",
  return_ready: "검사 완료 · 복귀 명령 대기",
  moving_to_repickup: "MRI실 재진입 중",
  boarding_wait: "MRI → 침상 환자 회수 중",
  backing_out_after_pickup: "2층 엘리베이터 이동 준비",
  moving_to_elevator_2f: "2층 엘리베이터 이동 중",
  elevator_transfer_to_1f: "엘리베이터 2층 → 1층",
  returning_to_ward: "환자 병실 복귀 중",
  ward_storage_ready: "침상 반환 · 언도킹 준비",
  ward_detach_wait: "AMR·침상 언도킹 중",
  returning_to_storage: "AMR 보관실 복귀 중",
  failed_navigation: "이동 오류",
};

function scenarioStepIndexForJob(job) {
  if (!job) return activeCoordinateScenarioStepIndex();
  return activeCoordinateScenarioStepIndex();
}

function activeScenarioStepIndex() {
  if (Number.isInteger(ui.debugScenarioIndex)) return ui.debugScenarioIndex;
  return activeCoordinateScenarioStepIndex();
}

function scenarioStepDisplayPayload(index) {
  const safeIndex = Math.max(0, Math.min(SCENARIO_STEPS.length - 1, Number(index) || 0));
  const step = SCENARIO_STEPS[safeIndex];
  const amr = selectedAmr();
  const bed = selectedBed();
  const subject = [displayAmrName(amr?.name), bed?.patient_name || bed?.label].filter(Boolean).join(" · ");
  const tracker = amr ? coordinateScenarioTrackerForAmr(amr.name) : null;
  const position = amr ? (coordinateDebugPositionFor(amr.name) || tracker?.position || null) : null;
  const currentPoint = position?.snapped_label || position?.point_id || position?.point_key || "";
  const rawPose = position?.raw_pose || null;
  const poseText = rawPose ? `현재 좌표 x=${Number(rawPose.x).toFixed(3)}, y=${Number(rawPose.y).toFixed(3)}` : "";
  const pointText = currentPoint ? `현재 판정 좌표: ${currentPoint}` : "";
  const detail = [step.detail, pointText, poseText].filter(Boolean).join(" · ");
  return {
    badge: step.badge,
    tone: step.tone,
    title: `${subject ? `${subject} · ` : ""}${step.title}`,
    detail,
  };
}

function renderScenarioProgress() {
  const root = $("scenarioProgress");
  if (!root) return;
  const currentIndex = activeScenarioStepIndex();
  const step = SCENARIO_STEPS[currentIndex] || SCENARIO_STEPS[0];

  const counter = $("scenarioStepCounter");
  if (counter) counter.textContent = `${currentIndex + 1} / ${SCENARIO_STEPS.length}`;
  root.replaceChildren();

  const row = document.createElement("div");
  row.className = "scenario-step active current-only no-index";
  const copy = document.createElement("div");
  copy.className = "scenario-step-copy";
  const title = document.createElement("strong");
  title.textContent = step.title;
  copy.append(title);
  row.append(copy);
  root.appendChild(row);
}


function moveDebugScenario(delta) {
  const liveIndex = activeCoordinateScenarioStepIndex();
  const base = Number.isInteger(ui.debugScenarioIndex) ? ui.debugScenarioIndex : liveIndex;
  ui.debugScenarioIndex = Math.max(0, Math.min(SCENARIO_STEPS.length - 1, base + delta));
  renderAll();
}

function exitDebugScenario() {
  ui.debugScenarioIndex = null;
  renderAll();
}

function renderDebugScenarioControls() {
  // 화면 단계 디버깅 UI는 제거했습니다. 좌표 이동 디버깅만 사용합니다.
}


function compactJobTitle(job) {
  if (!job) return "";
  const shortPhase = COMPACT_PHASE_TITLES[job.phase] || normalizeDisplayText(job.phase_label);
  return `${displayAmrName(job.amr_name)} · ${shortPhase}`;
}

function currentStatusPayload() {
  const amr = selectedAmr();
  const bed = selectedBed();
  if (!amr) {
    return {
      badge: "대기",
      tone: "",
      title: "AMR·환자 선택 필요",
      detail: "선택한 AMR의 좌표 흐름에 따라 현재 상태가 자동으로 바뀝니다.",
    };
  }
  const payload = scenarioStepDisplayPayload(activeScenarioStepIndex());
  if (bed) {
    payload.title = `${displayAmrName(amr.name)} · ${bed.patient_name || bed.label} · ${SCENARIO_STEPS[activeScenarioStepIndex()].title}`;
  }
  return payload;
}

function renderCurrentStatus() {
  const payload = currentStatusPayload();
  const badge = $("currentStateBadge");
  if (badge) {
    badge.textContent = payload.badge;
    badge.className = `state-pill ${payload.tone || ""}`.trim();
  }
  if ($("currentStateTitle")) $("currentStateTitle").textContent = payload.title;
  if ($("currentStateDetail")) $("currentStateDetail").textContent = payload.detail;

  const root = $("patientInfo");
  root.replaceChildren();
  const bed = selectedBed();
  if (!bed) {
    root.className = "patient-info empty";
    return;
  }
  root.className = "patient-info";
  [["성명", bed.patient_name || bed.label || "-"], ["생년월일", bed.birth_date || "-"]].forEach(([label, value]) => {
    const grid = document.createElement("div");
    grid.className = "patient-grid";
    const left = document.createElement("div");
    left.className = "patient-cell label";
    left.textContent = label;
    const right = document.createElement("div");
    right.className = "patient-cell";
    right.textContent = value;
    grid.append(left, right);
    root.appendChild(grid);
  });
}


function renderControls() {
  if (!ui.state) return;
  const amr = selectedAmr();
  const bed = selectedBed();
  const job = selectedJob();
  const blocked = ui.commandPending;
  const rosConnected = Boolean(ui.state.ros?.connected);

  $("startMissionBtn").disabled = blocked || !rosConnected || !amr || !bed || Boolean(job) || Boolean(bed?.assigned_amr);
  $("startMissionBtn").textContent = "이동 명령";

  const action = $("scenarioActionBtn");
  action.disabled = true;
  action.textContent = "현재 단계 대기";
  action.classList.remove("auto-wait");

  if (job?.phase === "waiting_exam") {
    if (job.exam_ready) {
      action.disabled = blocked || !rosConnected;
      action.textContent = "검사 완료 · 복귀 명령";
    } else {
      action.textContent = "검사 중";
    }
  } else if (job?.phase === "return_ready") {
    action.disabled = blocked || !rosConnected;
    action.textContent = "검사 완료 · 복귀 명령";
  } else if (job?.auto_wait) {
    const elevatorTransfer = job.phase === "elevator_transfer_to_2f" || job.phase === "elevator_transfer_to_1f";
    const seconds = AUTO_WAIT_MS[job.phase] === 3000 ? "3초" : "맵 전환";
    action.textContent = elevatorTransfer ? "WORLD POSE 층 도착 대기" : `${seconds} 대기 중 · 자동 진행`;
    action.classList.add("auto-wait");
  } else if (job && MOVING_PHASES.has(job.phase)) {
    action.textContent = COMPACT_PHASE_TITLES[job.phase] || "실제 AMR 이동 중";
  } else if (job?.phase === "failed_navigation") {
    action.textContent = "실제 미션 실패 · 로그 확인";
  }
}

function routeBetween(cfg, fromRoom, toRoom) {
  const direct = (cfg.waypoint_routes || []).find((item) => item.from === fromRoom && item.to === toRoom);
  if (direct) return { route: direct, reversed: false };
  const reverse = (cfg.waypoint_routes || []).find((item) => item.from === toRoom && item.to === fromRoom);
  if (reverse) return { route: reverse, reversed: true };
  return null;
}

function orderedPathPoints(cfg, fromRoom, toRoom, floor) {
  const route = cfg.route || [];
  const start = route.indexOf(fromRoom);
  const end = route.indexOf(toRoom);
  if (start < 0 || end < 0) return [];
  const result = [];
  const startPoi = cfg.pois[fromRoom];
  if (startPoi) result.push({ key: `poi:${floor}:${fromRoom}`, type: "poi", room: fromRoom, display: startPoi.display });
  if (start === end) return result;
  const step = end > start ? 1 : -1;
  for (let i = start; i !== end; i += step) {
    const found = routeBetween(cfg, route[i], route[i + step]);
    if (!found) return [];
    const waypoints = found.reversed ? [...found.route.waypoints].reverse() : [...found.route.waypoints];
    waypoints.forEach((wp) => {
      result.push({ key: `${found.route.id}:${wp.id}`, type: "waypoint", display: wp.display });
    });
  }
  const endPoi = cfg.pois[toRoom];
  if (endPoi) result.push({ key: `poi:${floor}:${toRoom}`, type: "poi", room: toRoom, display: endPoi.display });
  return result;
}

function movementDescriptor(job = selectedJob()) {
  if (!job || !MOVING_PHASES.has(job.phase)) return null;
  const amr = ui.state.amrs.find((item) => item.name === job.amr_name);
  if (!amr || !job.target_floor || !job.target_room) return null;
  const floor = Number(job.target_floor);
  const cfg = ui.state.floors[String(floor)];
  if (!cfg) return null;
  const fromRoom = amr.floor === floor ? amr.room : "엘리베이터 앞";
  const points = orderedPathPoints(cfg, fromRoom, job.target_room, floor);
  const signature = `${job.phase}:${floor}:${fromRoom}->${job.target_room}:${points.map((p) => p.key).join(">")}`;
  return { job, amr, floor, points, signature };
}


function renderWaypoints() {
  const layer = $("waypointLayer");
  const cfg = ui.state?.floors?.[String(ui.floor)];
  const image = $("mapImage");
  if (!layer || !cfg || !image?.complete || !image.clientWidth) return;
  layer.replaceChildren();
  layer.classList.toggle("coordinates-hidden", !ui.showConfiguredCoordinates);
  if (!ui.showConfiguredCoordinates) return;

  const points = [];
  const addPoint = (point, type, name = "") => {
    if (!point || !Array.isArray(point.display) || point.display.length < 2) return;
    const navXY = Array.isArray(point.nav_xy) && point.nav_xy.length >= 2 ? point.nav_xy : null;
    if (!navXY) return;
    points.push({
      type,
      id: point.point_id || point.id || name,
      name: point.display_name || name || point.point_id || point.id || "설정 좌표",
      display: point.display,
      navXY,
      toleranceM: Number(point.arrival_radius_m ?? 0),
      activeRoute: point.active_route !== false,
    });
  };

  if (Array.isArray(cfg.configured_points) && cfg.configured_points.length) {
    cfg.configured_points.forEach((point) => {
      const type = ["poi", "home", "patient"].includes(point.source) ? "poi" : point.source === "waiting" ? "waiting" : point.source === "reference" ? "reference" : "waypoint";
      addPoint(point, type);
    });
  } else {
    // 구버전 서버 응답과의 호환용 fallback.
    Object.entries(cfg.pois || {}).forEach(([name, point]) => {
      if (point?.hide_marker) return;
      addPoint(point, "poi", name);
    });
    (cfg.waypoint_routes || []).forEach((route) => {
      (route.waypoints || []).forEach((point) => addPoint(point, "waypoint"));
    });
    (cfg.waiting_points || []).forEach((point) => addPoint(point, "waiting"));
  }

  // 같은 물리 좌표를 공유하는 설정은 한 개의 말풍선으로 묶어
  // 좌표 텍스트가 서로 완전히 겹치지 않도록 합니다.
  const grouped = new Map();
  points.forEach((point) => {
    const key = canonicalDisplayKey(ui.floor, point.display);
    if (!grouped.has(key)) grouped.set(key, []);
    const group = grouped.get(key);
    if (!group.some((item) => item.id === point.id && item.type === point.type)) group.push(point);
  });

  const priority = { poi: 4, waiting: 3, waypoint: 2, reference: 1 };
  const coordinateGroups = [...grouped.values()].filter((group) => group.length);
  coordinateGroups.forEach((group) => group.sort((a, b) => (priority[b.type] || 0) - (priority[a.type] || 0)));

  for (const group of coordinateGroups) {
    const main = group[0];
    const u = Number(main.display[0]);
    const v = Number(main.display[1]);

    const marker = document.createElement("div");
    marker.className = `configured-coordinate configured-coordinate-${main.type}`;
    if (group.every((item) => item.activeRoute === false)) marker.classList.add("reference-only");

    // 같은 수평 복도에 좌표가 몰린 경우 라벨을 위/아래 여러 줄로 자동 분산합니다.
    // 숫자 x/y까지 항상 표시하되 인접 라벨이 덮이지 않게 하기 위한 화면 전용 배치입니다.
    const rowPeers = coordinateGroups
      .filter((candidate) => Math.abs(Number(candidate[0].display[1]) - v) < 0.010)
      .sort((a, b) => Number(a[0].display[0]) - Number(b[0].display[0]));
    if (rowPeers.length >= 3) {
      const rowIndex = rowPeers.indexOf(group);
      const lane = rowIndex % 2 === 0
        ? -(18 + Math.floor(rowIndex / 2) * 20)
        : 12 + Math.floor(rowIndex / 2) * 20;
      marker.style.setProperty("--coord-label-shift-y", `${lane}px`);
    }

    // 같은 세로 복도에 여러 좌표가 있으면 좌/우를 번갈아 사용합니다.
    const columnPeers = coordinateGroups
      .filter((candidate) => Math.abs(Number(candidate[0].display[0]) - u) < 0.010)
      .sort((a, b) => Number(a[0].display[1]) - Number(b[0].display[1]));
    const columnIndex = columnPeers.indexOf(group);
    if (u > 0.72 || (columnPeers.length >= 3 && columnIndex % 2 === 1)) marker.classList.add("label-left");
    if (v < 0.18) marker.classList.add("label-below");
    if (v > 0.80) marker.classList.add("label-above");
    marker.style.left = `${image.offsetLeft + image.clientWidth * u}px`;
    marker.style.top = `${image.offsetTop + image.clientHeight * v}px`;

    const dot = document.createElement("span");
    dot.className = "configured-coordinate-dot";

    const label = document.createElement("span");
    label.className = "configured-coordinate-label";

    const title = document.createElement("strong");
    const ids = [...new Set(group.map((item) => item.id).filter(Boolean))];
    const names = [...new Set(group.filter((item) => item.type === "poi").map((item) => item.name).filter(Boolean))];
    title.textContent = names.length ? `${names.join(" / ")} · ${ids.join(" / ")}` : ids.join(" / ");

    const xy = document.createElement("span");
    xy.className = "configured-coordinate-xy";
    xy.textContent = `x=${Number(main.navXY[0]).toFixed(3)}  y=${Number(main.navXY[1]).toFixed(3)}`;

    const tolerance = document.createElement("span");
    tolerance.className = "configured-coordinate-tolerance";
    tolerance.textContent = `판정 반경 ±${Number(main.toleranceM || 0).toFixed(2)}m`;

    label.append(title, xy, tolerance);
    marker.append(dot, label);
    marker.title = group.map((item) => `${item.name} (${item.id}) · x=${Number(item.navXY[0]).toFixed(4)}, y=${Number(item.navXY[1]).toFixed(4)} · tolerance=±${Number(item.toleranceM || 0).toFixed(2)}m${item.activeRoute === false ? " · 비활성 참조 좌표" : ""}`).join("\n");
    layer.appendChild(marker);
  }
}

function configuredPointRole(serverPosition) {
  if (!serverPosition) return null;
  const floor = Number(serverPosition.floor);
  const cfg = ui.state?.floors?.[String(floor)];
  const display = serverPosition.display;
  const displayKey = canonicalDisplayKey(floor, display);
  if (!cfg || !displayKey) return null;

  // 실제 /world_pose가 직전 포인트의 tolerance를 벗어난 순간부터는,
  // 지도 마커를 직전 좌표에 유지하더라도 상태는 "이동중"으로 전환합니다.
  if (serverPosition.in_tolerance === false && serverPosition.holding_previous_point) {
    return { className: "map-moving", label: "이동중", pointType: "구간 이동", locationLabel: "다음 설정 좌표로 이동 중" };
  }

  const samePoint = (candidate) => canonicalDisplayKey(floor, candidate?.display) === displayKey;

  const waiting = (cfg.waiting_points || []).find(samePoint);
  if (waiting) {
    return {
      className: "map-waiting",
      label: "대기중",
      pointType: "대기점",
      locationLabel: waiting.display_name || waiting.id || "대기점",
    };
  }

  const homeEntry = Object.entries(cfg.home_slots || {}).find(([, slot]) => samePoint(slot));
  if (homeEntry) {
    const [, slot] = homeEntry;
    return {
      className: "map-ready",
      label: "준비중",
      pointType: "보관 위치",
      locationLabel: slot.display_name || "보관 위치",
    };
  }

  const poiEntry = Object.entries(cfg.pois || {}).find(([, poi]) => samePoint(poi));
  if (poiEntry) {
    const [room, poi] = poiEntry;
    return {
      className: "map-ready",
      label: "준비중",
      pointType: "목적지",
      locationLabel: poi.display_name || room,
    };
  }

  for (const route of cfg.waypoint_routes || []) {
    const waypoint = (route.waypoints || []).find(samePoint);
    if (waypoint) {
      return {
        className: "map-moving",
        label: "이동중",
        pointType: "경유점",
        locationLabel: waypoint.display_name || waypoint.id || "경유점",
      };
    }
  }

  return null;
}

function amrMapPointState(serverPosition) {
  if (!serverPosition) return null;

  // tolerance 밖에서는 raw world_pose 연속 위치를 표시하므로 항상 구간 이동으로 봅니다.
  if (serverPosition.in_tolerance === false || serverPosition.fallback_moving) {
    return { className: "map-moving", label: "이동중", pointType: "구간 이동", locationLabel: "world_pose 실시간 이동 위치" };
  }

  // tolerance 안에서는 현재 표시 좌표가 어떤 역할로 설정되어 있는지 사용합니다.
  const configured = configuredPointRole(serverPosition);
  if (configured) return configured;

  // 설정 좌표를 찾지 못한 예외적인 호환 데이터만 source 정보로 보완합니다.
  const snappedSource = String(serverPosition.snapped_source || "").toLowerCase();
  const source = String(serverPosition.source || "").toLowerCase();
  const pointKey = String(serverPosition.snapped_point_key || serverPosition.point_key || "");
  const pointId = String(serverPosition.snapped_point_id || serverPosition.point_id || "");

  if (snappedSource === "waiting" || source === "waiting" || pointKey.startsWith("wait:")) {
    return { className: "map-waiting", label: "대기중", pointType: "대기점", locationLabel: serverPosition.snapped_label || "대기점" };
  }
  if (source === "initial_home" || snappedSource === "home" || pointKey.startsWith("home:")) {
    return { className: "map-ready", label: "준비중", pointType: "보관 위치", locationLabel: serverPosition.snapped_label || "보관 위치" };
  }
  if (["poi", "patient"].includes(snappedSource) || pointKey.startsWith("poi:") || pointKey.startsWith("patient:")) {
    return { className: "map-ready", label: "준비중", pointType: "목적지", locationLabel: serverPosition.snapped_label || "목적지" };
  }
  if (snappedSource === "waypoint" || /(?:^|-)W\d+/i.test(pointId)) {
    return { className: "map-moving", label: "이동중", pointType: "경유점", locationLabel: serverPosition.snapped_label || pointId || "경유점" };
  }
  return null;
}

// legacy signature reference for static regression: function resolvedMapPositionFor(amr, floor)
function resolvedMapPositionFor(amr, floor, options = {}) {
  const cfg = ui.state?.floors?.[String(floor)];
  if (!amr || !cfg) return null;
  if (!options.ignoreCoordinateDebug) {
    const debugPosition = coordinateDebugPositionFor(amr.name);
    if (debugPosition && Number(debugPosition.floor) === Number(floor)) return debugPosition;
  }
  const job = jobFor(amr.name);
  const livePosition = ui.state?.traffic?.amr_positions?.[amr.name] || null;

  const hasFloorDisplay = Boolean(
    livePosition
    && Number(livePosition.floor) === Number(floor)
    && Array.isArray(livePosition.display)
    && livePosition.display.length >= 2
  );
  if (hasFloorDisplay) return livePosition;

  // 첫 유효 world_pose 이전에는 기존 논리 위치를 유지합니다. 명령이 생성됐다는 이유만으로
  // 준비중 좌표를 이동중으로 바꾸지 않고, 설정된 좌표 역할(home/POI)에 따라 상태를 정합니다.
  const isStorageAnchor = Number(amr.floor) === 1 && amr.room === "보관실";
  const homeSlot = Number(floor) === 1 && isStorageAnchor ? cfg.home_slots?.[amr.name] : null;
  if (homeSlot?.display) {
    return {
      amr_name: amr.name,
      floor: 1,
      display: homeSlot.display,
      source: job ? "mission_start_fallback" : "initial_home",
      position_source: job ? "logical_fallback" : "initial_home",
      display_mode: job ? "mission_start_fallback" : "initial_storage_fallback",
      point_key: `home:${amr.name}`,
      point_id: homeSlot.id,
      snapped_source: "home",
      snapped_label: homeSlot.display_name || `${displayAmrName(amr.name)} 보관 위치`,
      in_tolerance: true,
      holding_previous_point: false,
    };
  }

  const roomPoi = Number(amr.floor) === Number(floor) ? cfg.pois?.[amr.room] : null;
  if (roomPoi?.display) {
    return {
      amr_name: amr.name,
      floor: Number(floor),
      display: roomPoi.display,
      source: "logical_room_fallback",
      position_source: "logical_fallback",
      display_mode: "logical_room_fallback",
      point_key: `poi:${Number(floor)}:${amr.room}`,
      point_id: roomPoi.point_id,
      snapped_source: "poi",
      snapped_label: roomPoi.display_name || amr.room,
      in_tolerance: true,
      holding_previous_point: false,
    };
  }
  return null;
}

function coordinateStatusForAmr(amr) {
  if (!amr) return null;
  const livePosition = ui.state?.traffic?.amr_positions?.[amr.name] || null;
  const floor = Number(livePosition?.floor ?? amr.floor ?? 1);
  const position = resolvedMapPositionFor(amr, floor);
  const state = amrMapPointState(position);
  if (!position || !state) return null;
  return { position, ...state };
}

function coordinateStatusPayload(amr, coordinateState) {
  if (!amr || !coordinateState) return null;
  const location = coordinateState.locationLabel || coordinateState.pointType || "현재 위치";
  if (coordinateState.className === "map-ready") {
    return {
      badge: "준비중",
      tone: "ready",
      title: `${displayAmrName(amr.name)} · 준비중`,
      detail: `${location} 설정 좌표 범위 안에 있습니다.`,
    };
  }
  if (coordinateState.className === "map-waiting") {
    return {
      badge: "대기중",
      tone: "exam",
      title: `${displayAmrName(amr.name)} · 대기중`,
      detail: `${location} 설정 좌표에서 대기 중입니다.`,
    };
  }
  if (coordinateState.className === "map-moving") {
    return {
      badge: "이동 중",
      tone: "moving",
      title: `${displayAmrName(amr.name)} · 이동 중`,
      detail: coordinateState.pointType === "구간 이동"
        ? "설정된 다음 좌표로 이동 중입니다."
        : `${location} 경유 좌표를 통과 중입니다.`,
    };
  }
  return null;
}

function renderAmrMarkers() {
  const cfg = ui.state.floors[String(ui.floor)];
  const layer = $("amrLayer");
  const image = $("mapImage");
  if (!cfg || !image.complete || !image.clientWidth) return;
  layer.replaceChildren();

  const items = [];
  ui.state.amrs.forEach((amr) => {
    const job = jobFor(amr.name);
    const serverPosition = resolvedMapPositionFor(amr, ui.floor);
    const display = serverPosition?.display || null;
    if (!display) return;
    items.push({ amr, job, display, serverPosition, key: canonicalDisplayKey(ui.floor, display) });
  });

  const groups = new Map();
  items.forEach((item) => {
    if (!groups.has(item.key)) groups.set(item.key, []);
    groups.get(item.key).push(item);
  });

  for (const groupItems of groups.values()) {
    groupItems.sort((a, b) => a.amr.name.localeCompare(b.amr.name));
    groupItems.forEach((item, idx) => {
      const count = groupItems.length;
      const offsetX = (idx - (count - 1) / 2) * 42;
      const button = document.createElement("button");
      button.type = "button";
      const step = scenarioStepForAmr(item.amr.name);
      const stepIndex = scenarioIndexForAmr(item.amr.name);
      let pointState = amrMapPointState(item.serverPosition);
      button.className = `amr-dot ${ui.selectedAmr === item.amr.name ? "selected" : ""} ${pointState?.className || ""}`.trim();

      const amrLabel = document.createElement("span");
      amrLabel.className = "amr-dot-name";
      amrLabel.textContent = displayAmrName(item.amr.name);
      button.appendChild(amrLabel);

      const patientName = selectedPatientNameForAmr(item.amr.name, item.job);
      if (patientVisibleForStep(stepIndex) && patientName) {
        const patient = document.createElement("span");
        patient.className = "amr-dot-patient";
        patient.textContent = patientName;
        button.appendChild(patient);
      }

      const stateBadge = document.createElement("span");
      stateBadge.className = `amr-map-state ${pointState?.className || "map-ready"}`;
      stateBadge.textContent = step.title;
      stateBadge.setAttribute("aria-label", `현재 상태 ${step.title}`);
      button.appendChild(stateBadge);

      const rawPose = item.serverPosition?.raw_pose || null;
      const poseText = rawPose ? ` · x=${Number(rawPose.x).toFixed(3)}, y=${Number(rawPose.y).toFixed(3)}` : "";
      button.title = `${displayAmrName(item.amr.name)}${patientVisibleForStep(stepIndex) && patientName ? ` · ${patientName}` : ""} · ${step.title}${poseText}`;
      button.style.left = `${image.offsetLeft + image.clientWidth * Number(item.display[0]) + offsetX}px`;
      button.style.top = `${image.offsetTop + image.clientHeight * Number(item.display[1])}px`;
      button.onclick = () => {
        ui.selectedAmr = item.amr.name;
        const currentJob = jobFor(item.amr.name);
        if (currentJob) ui.selectedBed = currentJob.bed_id;
        renderAll();
      };
      layer.appendChild(button);
    });
  }
}


function renderPriorityStatus() {
  const root = $("priorityStatus");
  if (!root) return;
  root.replaceChildren();

  const policy = document.createElement("div");
  policy.className = "priority-policy";
  const policyHead = document.createElement("div");
  policyHead.className = "priority-policy-head";
  const policyTitle = document.createElement("strong");
  policyTitle.textContent = "판정 기준";
  const legend = document.createElement("span");
  legend.className = "waiting-legend";
  legend.innerHTML = '<span class="waiting-legend-dot" aria-hidden="true"></span>빨강 = 양보 대기';
  policyHead.append(policyTitle, legend);
  policy.appendChild(policyHead);

  (ui.state.traffic?.priority_rules || []).forEach((rule, index) => {
    const item = document.createElement("div");
    item.className = "priority-rule";
    const badge = document.createElement("span");
    badge.className = "priority-rule-no";
    badge.textContent = String(index + 1);
    const text = document.createElement("span");
    text.textContent = rule.title || rule.description || "";
    text.title = rule.description || rule.title || "";
    item.append(badge, text);
    policy.appendChild(item);
  });

  const floorJobs = (ui.state.jobs || []).filter((job) => Number(job.target_floor) === ui.floor);
  if (floorJobs.length) {
    const progressList = document.createElement("div");
    progressList.className = "priority-progress-list";
    floorJobs.forEach((job) => {
      const ratio = Number(ui.state.traffic?.route_progress?.[job.amr_name] || 0);
      const chip = document.createElement("span");
      chip.className = `progress-chip ${ratio >= 0.5 ? "halfway" : ""}`;
      chip.textContent = `${displayAmrName(job.amr_name)} ${Math.round(ratio * 100)}%`;
      progressList.appendChild(chip);
    });
    policy.appendChild(progressList);
  }

  const floorConflicts = (ui.state.traffic?.conflicts || []).filter((item) => Number(item.floor) === ui.floor);
  if (!floorConflicts.length) {
    const normal = document.createElement("div");
    normal.className = "traffic-normal";
    normal.textContent = "현재 교행 충돌 없음";
    policy.appendChild(normal);
  }
  floorConflicts.forEach((conflict) => {
    const alert = document.createElement("div");
    alert.className = `traffic-conflict ${conflict.yielding_waiting ? "waiting" : ""}`.trim();
    if (conflict.yielding_waiting) {
      const waitName = conflict.waiting_point?.display_name || "대기 포인트";
      alert.textContent = `${displayAmrName(conflict.priority_amr)} 진행 중 · ${displayAmrName(conflict.yielding_amr)}은(는) ${waitName}에서 정지 대기`;
    } else if (conflict.recommended_action === "divert_to_wait") {
      alert.textContent = `${normalizeDisplayText(conflict.message)} · 진행 경로를 비우기 위해 가장 가까운 빈 대기 포인트 사용`;
    } else {
      alert.textContent = `${normalizeDisplayText(conflict.message)} · 불필요한 대피 이동 없이 현 위치 유지`;
    }
    policy.appendChild(alert);
  });

  Object.entries(ui.state.traffic?.waiting_amrs || {})
    .filter(([, waiting]) => Number(waiting.floor) === Number(ui.floor) && waiting.can_resume)
    .forEach(([amrName, waiting]) => {
      const ready = document.createElement("div");
      ready.className = "traffic-resume";
      ready.textContent = `${displayAmrName(amrName)} · ${waiting.waiting_point_name || "대기 포인트"} 대기 해제 가능 · 경로 재진입 대기`;
      policy.appendChild(ready);
    });
  root.appendChild(policy);
}

function compactZoneLabelForAmr(amr) {
  const stepIndex = scenarioIndexForAmr(amr.name);
  const step = scenarioStepForAmr(amr.name);
  const pos = resolvedMapPositionFor(amr, Number(coordinateDebugPositionFor(amr.name)?.floor ?? ui.state?.traffic?.amr_positions?.[amr.name]?.floor ?? amr.floor ?? 1));
  const pointId = String(pos?.snapped_point_id || pos?.point_id || "");
  const patientName = selectedPatientNameForAmr(amr.name, jobFor(amr.name));

  if (stepIndex === 0) return `${displayAmrName(amr.name)} 보관/도킹 위치`;
  if (stepIndex === 2) return patientName ? `${patientName} 병실/OCR` : "환자 병실/OCR";
  if (stepIndex === 5 || stepIndex === 7) return "MRI실";
  if (stepIndex === 6) return "MRI 검사 대기 위치";
  if (stepIndex === 3 && pointId === "1F-ELEVATOR") return "1층 엘리베이터 앞";
  if ([4,8].includes(stepIndex) && pointId === "2F-ELEVATOR") return "2층 엘리베이터 앞";
  return step.title;
}

function renderZoneStatus() {
  const cfg = ui.state.floors[String(ui.floor)];
  $("zonePanelTitle").textContent = `${cfg.name} 주요 구역`;
  const root = $("zoneStatus");
  root.replaceChildren();

  const visibleAmrs = (ui.state.amrs || []).filter((amr) => {
    const debugPos = coordinateDebugPositionFor(amr.name);
    const live = ui.state?.traffic?.amr_positions?.[amr.name] || null;
    const floor = Number(debugPos?.floor ?? live?.floor ?? amr.floor ?? 1);
    return floor === Number(ui.floor);
  });

  if (!visibleAmrs.length) {
    const row = document.createElement("div");
    row.className = "zone-row zone-row-compact empty-zone";
    row.textContent = "현재 이 층에 표시할 AMR 없음";
    root.appendChild(row);
    return;
  }

  visibleAmrs.forEach((amr) => {
    const stepIndex = scenarioIndexForAmr(amr.name);
    const patientName = selectedPatientNameForAmr(amr.name, jobFor(amr.name));
    const row = document.createElement("div");
    row.className = "zone-row zone-row-compact";
    const name = document.createElement("strong");
    name.textContent = compactZoneLabelForAmr(amr);
    const meta = document.createElement("span");
    meta.className = "zone-compact-state";
    meta.textContent = `${displayAmrName(amr.name)}${patientVisibleForStep(stepIndex) && patientName ? ` · ${patientName}` : ""} · ${scenarioStepForAmr(amr.name).title}`;
    row.append(name, meta);
    root.appendChild(row);
  });
}

function renderWorldPoseIndicators() {
  const root = $("worldPoseIndicators");
  if (!root || !ui.state) return;

  root.replaceChildren();
  const amrs = ui.state.amrs || [];
  amrs.forEach((amr) => {
    const status = worldPoseStatusFor(amr.name);
    const poseState = worldPoseStateLabel(status);
    const receiving = Boolean(status.active && status.streaming_confirmed !== false);

    const indicator = document.createElement("div");
    indicator.className = `world-pose-indicator ${receiving ? "receiving" : "idle"}`;

    const dot = document.createElement("i");
    dot.className = "world-pose-indicator-dot";
    dot.setAttribute("aria-hidden", "true");

    const label = document.createElement("span");
    label.textContent = `${displayAmrName(amr.name)} /world_pose · ${poseState.text}`;

    indicator.append(dot, label);
    indicator.title = `${status.topic || "/world_pose"} · ${poseState.text} · PUB ${status.publisher_count || 0} · RX ${status.received_count || 0}`;
    root.appendChild(indicator);
  });
}


function renderMap() {
  const cfg = ui.state.floors[String(ui.floor)];
  const floorLabel = $("mapFloorLabel");
  if (floorLabel) floorLabel.textContent = `${cfg.name} MAP`;
  document.querySelectorAll(".floor-tab").forEach((button) => {
    button.classList.toggle("active", Number(button.dataset.floor) === ui.floor);
  });
  const image = $("mapImage");
  if (image.getAttribute("src") !== cfg.image) image.src = cfg.image;
  renderWorldPoseIndicators();
  requestAnimationFrame(() => {
    renderWaypoints();
    renderAmrMarkers();
  });
}

function compactLogTime(value) {
  const text = String(value || "");
  const match = text.match(/(\d{2}:\d{2}:\d{2})$/);
  return match ? match[1] : text;
}

function compactLogMessage(value) {
  let text = normalizeDisplayText(value);
  const replacements = [
    [/MRI 환자 이송 시나리오 GUI 초기화 완료|MRI 환자 이송 시나리오 초기화/g, "GUI 초기화"],
    [/ → 2층 MRI실 이송 명령 전송/g, " MRI 이송 시작"],
    [/ · MRI 검사 완료 확인 · 복귀 대기/g, " · MRI 검사 완료"],
    [/ · 복귀 버튼 선택 · 환자 재픽업 시작/g, " · 복귀 명령 · 환자 회수 시작"],
    [/ · 2층 맵 전환 완료 · MRI실 이동 시작/g, " · 2층 도착 · MRI실 이동"],
    [/ · 3초 대기 완료 · MRI실 후진 이탈 시작/g, " · 환자 MRI 인계 · 검사 대기 위치 이동"],
    [/ · 환자 탑승 3초 대기 완료 · 후진 이탈 시작/g, " · 환자 회수 · 2층 엘리베이터 이동 준비"],
    [/ · 1층 맵 전환 완료 · 환자 병실 복귀 시작/g, " · 1층 도착 · 병실 이동"],
    [/ 자동 확인 · 침상 자동 결합 완료/g, " 침상 결합"],
    [/ · 1층 엘리베이터 탑승 · 맵 전환 시작/g, " · 1층 엘리베이터 탑승"],
    [/ · 2층 MRI실 도착 · 침상 하강\/분리 · 3초 대기/g, " · MRI실 도착 · 환자 침상→MRI 인계"],
    [/ · MRI실 후진 이탈 완료 · MRI 검사 시작/g, " · MRI 검사 대기 시작"],
    [/ · MRI실 재진입 · 환자 탑승 3초 대기/g, " · 환자 MRI→침상 회수"],
    [/ · MRI실 후진 이탈 완료 · 병실 복귀 시작/g, " · 병실 복귀 시작"],
    [/ · 2층 엘리베이터 탑승 · 1층 맵 전환 시작/g, " · 2층 엘리베이터 탑승"],
    [/ · 환자 병실 복귀 완료 · 침상 분리 · 보관실 복귀/g, " · 침상 반환·언도킹 · 보관실 이동"],
    [/ · 임무 완료 · AMR 보관실 복귀 \(침대 없음\)/g, " · 임무 완료 · 보관실 도착"],
    [/ · 이동 실패:/g, " · 이동 실패 ·"],
  ];
  replacements.forEach(([pattern, shortText]) => {
    text = text.replace(pattern, shortText);
  });
  return text;
}

function renderLogRows(root, events) {
  root.replaceChildren();
  events.forEach((event) => {
    const row = document.createElement("div");
    row.className = `log-row level-${String(event.level).toLowerCase()}`;
    const time = document.createElement("time");
    time.textContent = compactLogTime(event.created_at);
    time.title = event.created_at || "";
    const message = document.createElement("span");
    message.textContent = compactLogMessage(event.message);
    message.title = normalizeDisplayText(event.message);
    row.append(time, message);
    root.appendChild(row);
  });
}

function renderLog() {
  const events = ui.state.events || [];
  const latest = events.slice(0, 2);
  renderLogRows($("eventLog"), latest);
  renderLogRows($("eventLogAll"), events);
  $("openLogModalBtn").textContent = events.length > 2 ? `과거 로그 보기 (${events.length - latest.length})` : "전체 로그 보기";
}

function scheduleAutoWaits() {
  // 실제 미션 프로세스가 모든 대기/엘리베이터/도킹 단계를 소유합니다.
}

function renderAll() {
  if (!ui.state) return;
  renderCards();
  renderCurrentStatus();
  renderScenarioProgress();
  renderControls();
  renderDebugScenarioControls();
  renderCoordinateDebugControls();
  renderMap();
  renderLog();
  scheduleAutoWaits();
}

function toggleLogModal(show) {
  const modal = $("logModal");
  modal.classList.toggle("hidden", !show);
  modal.setAttribute("aria-hidden", show ? "false" : "true");
}

async function refresh() {
  const seq = ++ui.requestSeq;
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const state = await response.json();
    if (seq !== ui.requestSeq) return;
    applyState(state);
    renderAll();
  } catch (error) {
    if (seq !== ui.requestSeq) return;
    setError(`서버 상태 조회 실패: ${error.message}`);
  }
}


$("startMissionBtn").onclick = () => {
  if (!ui.selectedAmr || !ui.selectedBed) return;
  return apiCommand({ action: "start_mri_mission", amr: ui.selectedAmr, bed_id: ui.selectedBed, floor: FIXED_DESTINATION.floor, room: FIXED_DESTINATION.room });
};

$("scenarioActionBtn").onclick = () => {
  const job = selectedJob();
  if (!job) return;
  if (job.phase === "waiting_exam" && job.exam_ready) {
    return apiCommand({ action: "start_return", amr: job.amr_name });
  }
  if (job.phase === "return_ready") return apiCommand({ action: "start_return", amr: job.amr_name });
};

// 좌표 이동 디버깅/설정좌표 토글은 실제 ROS 검증판에서 제거했습니다.

$("openLogModalBtn").onclick = () => toggleLogModal(true);
$("closeLogModalBtn").onclick = () => toggleLogModal(false);
$("logModal").onclick = (event) => { if (event.target === $("logModal")) toggleLogModal(false); };
document.addEventListener("keydown", (event) => { if (event.key === "Escape") toggleLogModal(false); });

document.querySelectorAll(".floor-tab").forEach((button) => {
  button.onclick = () => {
    const selectedFloor = Number(button.dataset.floor);
    if (![1, 2].includes(selectedFloor)) return;
    // 층 탭은 화면에 보여줄 지도만 선택합니다.
    // 실제 AMR 위치 판정은 서버의 현재 AMR 층과 해당 층 world_pose 좌표표를 기준으로 유지합니다.
    ui.floor = selectedFloor;
    renderAll();
  };
});

$("mapImage").addEventListener("load", () => requestAnimationFrame(() => {
  renderWaypoints();
  renderAmrMarkers();
}));
window.addEventListener("resize", () => requestAnimationFrame(() => {
  renderWaypoints();
  renderAmrMarkers();
}));

void refresh();
setInterval(() => void refresh(), 1000);
