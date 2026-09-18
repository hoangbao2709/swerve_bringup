/** warehouse_layout.json 的型別，對應 docs/layout/warehouse_layout_格式說明.md */
export type P2 = [number, number];
export type WarehousePoint = { x: number; y: number };
export type P3 = [number, number, number];
export type Rect = [number, number, number, number];

export interface LayoutZone { id: string; name: string; color: string; polygon: P2[]; floor?: number }
export interface LayoutDock { id: string; kind: "INBOUND" | "OUTBOUND"; zone: string; rect: Rect; door: P2 }
export interface LayoutRack { id: string; zone: string; position: P3; size: P3; rotation: number; levels: number; model: string; blocks_grid: boolean; floor?: number; /** Fixed operational capacity: 8 orders. */ capacity?: number; /** Current number of orders physically on this shelf. */ current_load?: number }
export interface LayoutConveyor { id: string; name: string; zone: string; path: P2[]; width: number; speed_mps: number; direction: string; blocks_grid: boolean; /** 這條輸送帶供應的工作站；故障時該站的卸貨時間變長 */ feeds?: string }
export interface LayoutStation { id: string; kind: string; zone: string; rect: Rect; access_point: P2 }
export interface LayoutCharging { id: string; zone: string; position: P3; heading: number; power_kw: number; access_point: P2 }
export interface LayoutParking { id: string; zone: string; rect: Rect; slots: number }
export interface LayoutRestricted { id: string; name: string; rect: Rect; robots_allowed: boolean }
export interface LayoutWalkway { id: string; polygon: P2[]; robots_allowed: boolean; speed_limit_mps: number }
export interface LayoutCamera { id: string; zone: string; floor?: number; position: P3; look_at: P3; fov_deg: number; range_m: number }
export interface LayoutSensor { id: string; kind: string; zone: string; position: P3 }
export interface LayoutLocation { id: string; kind: string; zone: string; floor?: number; rack_id: string | null; level_range: [number, number] | null; access_point: P2 }
export interface LayoutSpawnRobot { id: string; position: P3; heading: number; battery: number; floor?: number }
export interface LayoutFloor { id: number; name: string; elevation: number; footprint?: P2[]; boundary?: WarehousePoint[] | P2[]; holes?: Array<WarehousePoint[] | P2[]> }
export interface LayoutAisle { id: string; floor_id?: number | string; centerline: WarehousePoint[]; width: number; direction: "bidirectional" | "forward" | "reverse"; speed_limit?: number; tag_rule?: { enabled: boolean; spacing: number; start_offset?: number; end_offset?: number } }
export interface LayoutNavigationTag { uuid: string; tag_id: number; floor_id?: number | string; x: number; y: number; z?: number; yaw: number; placement: "auto" | "manual"; locked: boolean; generated_from?: string }
export interface LayoutNavigationEdge { uuid?: string; from_tag_uuid?: string; to_tag_uuid?: string; from_tag_id?: number; to_tag_id?: number; enabled?: boolean; bidirectional?: boolean; cost?: number }
export interface LayoutLift { id: string; cell: [number, number]; floors: number[]; ride_ticks: number }

export interface WarehouseLayout {
  schema_version: string | number;
  coordinate_system?: { unit: "meter" | string; frame: string; yaw_unit: "radian" | string };
  id: string;
  name: string;
  units: string;
  size: { width: number; depth: number; height: number };
  grid: { cell_size: number; cols: number; rows: number };
  floors: LayoutFloor[];
  aisles?: LayoutAisle[];
  navigation_tags?: LayoutNavigationTag[];
  navigation_edges?: LayoutNavigationEdge[];
  /** 夾層支撐柱位（立在 F1 地面）：F1 導航網格以柱底板 0.9×0.9 m 封成障礙 */
  columns?: Array<[number, number]>;
  lifts: LayoutLift[];
  zones: LayoutZone[];
  docks: LayoutDock[];
  racks: LayoutRack[];
  conveyors: LayoutConveyor[];
  stations: LayoutStation[];
  charging_stations: LayoutCharging[];
  parking: LayoutParking[];
  restricted_areas: LayoutRestricted[];
  walkways: LayoutWalkway[];
  cameras: LayoutCamera[];
  sensors: LayoutSensor[];
  locations: LayoutLocation[];
  /** 建築結構柱等實體障礙：F1 導航網格整塊封鎖；WarehouseShell 由此渲染柱子 */
  obstacles: Array<{ id: string; kind: string; rect: [number, number, number, number] }>;
  spawn: { robots: LayoutSpawnRobot[] };
}
