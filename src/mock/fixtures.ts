import type { Vda5050Configuration, LocalRobotMap } from "../services/api";
import type {
  RobotDetailMapSnapshot,
  RobotDetailLidar3D,
  RobotDetailScan,
  RobotLidarStreamDiagnostics,
  RobotRuntimeCapabilities,
  RobotState,
  RobotSystemDiagnostics,
  TwinState,
} from "../schema/twin_state";
import warehouseLayout from "../layout/warehouse_layout.json";

export const STATIC_ROBOT_ID = "R01";
export const STATIC_MAP_ID = "STATIC-PREVIEW-01";
export const STATIC_MAP_REVISION = "fixture-1";
export const SHOWCASE_MAP_ID = "north-hall-01";
export const SHOWCASE_MAP_REVISION = "rev-12";

const width = Math.max(1, Math.round(warehouseLayout.size.width));
const height = Math.max(1, Math.round(warehouseLayout.size.depth));

function createOccupancyPreview(): number[] {
  return Array.from({ length: width * height }, (_, index) => {
    const x = index % width;
    const y = Math.floor(index / width);
    const boundary = x === 0 || y === 0 || x === width - 1 || y === height - 1;
    const rackRows = (x >= 18 && x <= 20 || x >= 36 && x <= 38 || x >= 54 && x <= 56)
      && y >= 12 && y <= 56 && ![21, 33, 45].some((crossAisle) => Math.abs(y - crossAisle) <= 1);
    const workcells = x >= 73 && x <= 81 && y >= 12 && y <= 23;
    return boundary || rackRows || workcells ? 100 : 0;
  });
}

const occupancyPreview = createOccupancyPreview();
const occupiedCells = occupancyPreview.filter((value) => value > 0).length;

export const STATIC_MAP_SNAPSHOT: RobotDetailMapSnapshot = {
  robot_id: STATIC_ROBOT_ID,
  frame_id: "map",
  map_source: "LOCAL_MAP",
  mapping_session_id: null,
  map_version: 1,
  active_map_id: STATIC_MAP_ID,
  active_map_revision: STATIC_MAP_REVISION,
  map_content_revision: "preview-content-1",
  canonical_map_revision: null,
  timestamp: null,
  width,
  height,
  resolution: 1,
  known_cells: width * height,
  unknown_cells: 0,
  occupied_cells: occupiedCells,
  free_cells: width * height - occupiedCells,
  explored_area_m2: width * height,
  origin: { x: 0, y: 0, yaw: 0 },
  data: occupancyPreview,
};

/** Consistent, deterministic visual sample used only by the standalone showcase. */
export const SHOWCASE_MAP_SNAPSHOT: RobotDetailMapSnapshot = {
  ...STATIC_MAP_SNAPSHOT,
  active_map_id: SHOWCASE_MAP_ID,
  active_map_revision: SHOWCASE_MAP_REVISION,
  map_content_revision: "north-hall-content-12",
  timestamp: null,
};

export const STATIC_LOCAL_MAP: LocalRobotMap = {
  id: STATIC_MAP_ID,
  name: "SAMPLE MAP · NOT SAVED",
  robot_id: STATIC_ROBOT_ID,
  created_at: "2026-01-01T00:00:00Z",
  resolution: 1,
  origin: [0, 0, 0],
  revision: STATIC_MAP_REVISION,
  frame_id: "map",
  width,
  height,
  map_id: STATIC_MAP_ID,
  map_kind: "SAVED_LOCAL_MAP",
  canonical_map_promoted: false,
  known_cells: width * height,
  unknown_cells: 0,
  free_cells: STATIC_MAP_SNAPSHOT.free_cells ?? 0,
  occupied_cells: STATIC_MAP_SNAPSHOT.occupied_cells ?? 0,
  explored_area_m2: width * height,
  slam_session_state: { status: "UNAVAILABLE", engine: "fixture", artifact_id: null },
  navigation_artifacts: { yaml: false, image: false },
};

export const SHOWCASE_LOCAL_MAP: LocalRobotMap = {
  ...STATIC_LOCAL_MAP,
  id: SHOWCASE_MAP_ID,
  map_id: SHOWCASE_MAP_ID,
  name: "North Hall",
  revision: SHOWCASE_MAP_REVISION,
  created_at: "2026-10-21T09:40:00Z",
  slam_session_state: { status: "UNAVAILABLE", engine: "none", artifact_id: null },
  navigation_artifacts: { yaml: false, image: false },
};

const sampleScanPoints: Array<[number, number]> = [
  [31.4, 17.3], [30.8, 18.1], [30.0, 18.5], [29.2, 18.2], [28.8, 17.4],
  [28.9, 16.5], [29.6, 15.9], [30.5, 15.9], [31.2, 16.4], [32.0, 17.2],
  [33.0, 18.0], [34.1, 18.4], [35.0, 18.0], [34.8, 16.9], [33.6, 16.4],
];

export const STATIC_SCAN: RobotDetailScan = {
  robot_id: STATIC_ROBOT_ID,
  topic: "STATIC_FIXTURE_ONLY",
  source_frame_id: "base_scan",
  mapping_session_id: null,
  frame_id: "map",
  sensor_pose: { x: 30, y: 17, yaw: 0.4 },
  trajectory: [[28, 14], [29, 15], [30, 17]],
  timestamp: null,
  angle_min: -2.35,
  angle_max: 2.35,
  angle_increment: 0.3,
  range_min: 0.1,
  range_max: 8,
  point_count: sampleScanPoints.length,
  minimum_range: 0.8,
  maximum_range: 4.2,
  scan_hz_sim: null,
  scan_hz_wall: null,
  points: sampleScanPoints,
};

export const STATIC_ROBOT: RobotState = {
  id: STATIC_ROBOT_ID,
  model: "AMR-L",
  floor: 1,
  lift_id: null,
  lift_stage: null,
  position: [30, 0, 17],
  heading: Math.PI / 8,
  velocity: 0,
  max_speed: 1.2,
  battery: 84,
  status: "IDLE",
  fsm: "IDLE",
  health: 100,
  current_task_id: null,
  destination: null,
  path: [],
  path_index: 0,
  load: { current: 0, capacity: 2 },
  zone: "ZONE-A",
  eta_s: null,
  fsm_since_tick: 0,
  stats: { distance_m: 18.4, tasks_completed: 6, energy_wh: 42, busy_ticks: 0, wait_ticks: 0 },
  perception: null,
  vx: 0,
  vy: 0,
  wz: 0,
  navigation_state: "UNKNOWN",
  control_mode: "AUTONOMOUS",
  last_telemetry_at: null,
  pose_frame_id: "map",
  pose_map_id: STATIC_MAP_ID,
  pose_map_revision: STATIC_MAP_REVISION,
  pose_map_source: "STATIC_UI_FIXTURE",
  pose_source: "STATIC_UI_FIXTURE",
  active_map_pose: {
    x: 30,
    y: 17,
    yaw: Math.PI / 8,
    frame_id: "map",
    map_id: STATIC_MAP_ID,
    map_revision: STATIC_MAP_REVISION,
    map_source: "STATIC_UI_FIXTURE",
    pose_source: "STATIC_UI_FIXTURE",
    timestamp: "2026-10-29T04:00:00.000Z",
    valid: true,
  },
};

export const SHOWCASE_ROBOT: RobotState = {
  ...STATIC_ROBOT,
  status: "OFFLINE",
  fsm: "IDLE",
  pose_map_id: SHOWCASE_MAP_ID,
  pose_map_revision: SHOWCASE_MAP_REVISION,
  pose_map_source: "LOCAL_MAP",
  pose_source: "LOCAL_SAMPLE",
  active_map_pose: {
    ...STATIC_ROBOT.active_map_pose!,
    map_id: SHOWCASE_MAP_ID,
    map_revision: SHOWCASE_MAP_REVISION,
    map_source: "LOCAL_MAP",
    pose_source: "LOCAL_SAMPLE",
    timestamp: "2026-10-29T04:00:00.000Z",
    valid: true,
  },
};

const showcaseCloudPoints = STATIC_SCAN.points.map(([x, y]) => [x, y, 0] as [number, number, number]);
const showcaseCloudBounds = {
  min: [0, 1, 2].map((axis) => Math.min(...showcaseCloudPoints.map((point) => point[axis]))) as [number, number, number],
  max: [0, 1, 2].map((axis) => Math.max(...showcaseCloudPoints.map((point) => point[axis]))) as [number, number, number],
};

/** A deterministic planar sample shown only as a local 3D visualization. */
export const SHOWCASE_LIDAR_3D: RobotDetailLidar3D = {
  robot_id: SHOWCASE_ROBOT.id,
  timestamp: "2026-10-29T04:00:00.000Z",
  frame_id: "map",
  source_frame_id: "map",
  point_count: showcaseCloudPoints.length,
  points: showcaseCloudPoints,
  bounds: showcaseCloudBounds,
  accumulated: false,
  trajectory: STATIC_SCAN.trajectory ?? [],
  slam_pose: null,
  render_fps: 0,
  source_fps: 0,
  web_output_fps: 0,
  dropped_frames: 0,
  epoch: "showcase-planar-scan",
  revision: 1,
  path: [],
  goal: null,
};

export const STATIC_DIAGNOSTICS: RobotSystemDiagnostics = {
  robot_id: STATIC_ROBOT_ID,
  ros_bridge: false,
  ros: false,
  gazebo: false,
  controller_manager: false,
  slam: false,
  nav2: false,
  nav2_ready: false,
  tf: false,
  lidar: false,
  nodes: [],
  topics: [],
  controllers: [],
  simulation_time: null,
  last_update_at: null,
  localization: "UNKNOWN",
  gazebo_rtf: null,
  cpu_percent: null,
  ram_percent: null,
  websocket_latency_ms: null,
  command_ownership: {
    active_command_source: null,
    active_control_mode: null,
    last_command_age: null,
    manual_source_active: false,
    nav_source_active: false,
    tag_source_active: false,
  },
  mapping: {
    slam_state: "UNAVAILABLE",
    mapping_session_id: null,
    scan_live: false,
    scan_hz: null,
    scan_frame: "STATIC_FIXTURE_ONLY",
    scan_age_s: null,
    odom_live: false,
    odom_hz: null,
    odom_frame: null,
    base_frame: null,
    tf_valid: false,
    tf_lidar_to_map_valid: false,
    map_live: false,
    map_hz: null,
    map_width_cells: width,
    map_height_cells: height,
    resolution_m_per_cell: 1,
    origin_x: 0,
    origin_y: 0,
    known_cells: width * height,
    unknown_cells: 0,
    free_cells: STATIC_MAP_SNAPSHOT.free_cells,
    occupied_cells: STATIC_MAP_SNAPSHOT.occupied_cells,
    explored_area_m2: width * height,
    map_age_s: null,
  },
  metrics: { fixture: "STATIC_UI", dropped_frames: 0 },
  errors: [],
};

export const STATIC_LIDAR_DIAGNOSTICS: RobotLidarStreamDiagnostics = {
  source_timestamp: null,
  send_timestamp: null,
  frame_id: "STATIC_FIXTURE_ONLY",
  point_count: sampleScanPoints.length,
  source_fps: null,
  web_output_fps: null,
  dropped_frames: 0,
  view: "STATIC_UI_FIXTURE",
};

export const STATIC_CAPABILITIES: RobotRuntimeCapabilities = {
  mapping_available: false,
  mapping_active: false,
  nav2_available: false,
  nav2_ready: false,
  manual_available: false,
  goal_available: false,
  localization_ready: false,
  map_ready: true,
  tag_navigation_available: false,
  goal_blocker_code: "STATIC_UI",
  goal_blocker_reason: "UNAVAILABLE",
};

export const STATIC_VDA5050_CONFIGURATION: Vda5050Configuration = {
  robot_id: STATIC_ROBOT_ID,
  enabled: false,
  mqtt_host: "broker.invalid",
  mqtt_port: 1883,
  mqtt_username: "",
  password_configured: false,
  tls_enabled: true,
  topic_prefix: "vda5050/ptagv",
  interface_name: "PTAGV",
  manufacturer: "PTAGV",
  serial_number: "SAMPLE-SERIAL",
  protocol_version: "2.1.0",
  mqtt_protocol_version: "5.0",
  allow_task: false,
  allow_instant_actions: false,
  instant_actions_supported: false,
  instant_actions_status: "UNAVAILABLE",
  auto_reconnect: false,
  reconnect_interval: 5,
  connection_timeout: 10,
  keepalive: 30,
  client_id: "ptagv-sample-ui",
  connection_status: "UNAVAILABLE",
  last_error: null,
  ignored_orders: 0,
  updated_at: null,
};

export function createStaticTwin(base: TwinState): TwinState {
  return {
    ...base,
    robots: { [STATIC_ROBOT_ID]: STATIC_ROBOT },
    kpi: {
      ...base.kpi,
      fleet: { total: 1, active: 0, charging: 0, idle: 1, warning: 0, error: 0, offline: 0 },
    },
  };
}
