import { useEffect, useMemo, useState, type PointerEvent as ReactPointerEvent, type WheelEvent as ReactWheelEvent } from "react";
import { STATUS_COLOR, layout, useStore, type TagGraph } from "../../state/store";
import { buildNavGrid } from "../../layout/navgrid";
import { getEngine } from "../../simulation/runner";
import { rackOccupancy } from "../../layout/shelfOccupancy";
import { floorBoundary, polygonPoints, worldToSvgTransform } from "../../layout/coordinates";
import { buildAisleFootprint, rackFootprint2D, resolveNavigationEdgeEndpoints, zoneLabelLayout } from "../../layout/geometry";
import { canonicalFloorId, resolveRuntimeFloorIndex, sameFloor, type WarehouseLayout } from "../../layout/types";
import { robotPoseMatchesMap } from "../../layout/robotPoseFrame";
import type { TwinState } from "../../schema/twin_state";

type MapViewProps = { mode: "MAP" | "TRAFFIC" | "HEATMAP"; size?: { width: number; height: number } };
type CanonicalMapLayout = WarehouseLayout & Required<Pick<WarehouseLayout, "aisles" | "navigation_tags" | "navigation_edges">>;

const EMPTY_ROBOTS: TwinState["robots"] = {};
const EMPTY_ZONES: TwinState["zones"] = {};
const EMPTY_LAYOUT_ITEMS: never[] = [];
const EMPTY_TAGS: TagGraph["tags"] = [];
const EMPTY_EDGES: TagGraph["edges"] = [];

/** Turn null/missing backend collections into their canonical empty forms. */
function canonicalMapLayout(value: unknown): CanonicalMapLayout | null {
  if (!value || typeof value !== "object") return null;
  const candidate = value as Partial<WarehouseLayout>;
  if (!candidate.size || !candidate.grid
    || !Number.isFinite(candidate.size.width) || !Number.isFinite(candidate.size.depth)
    || !Number.isFinite(candidate.grid.cell_size) || !Number.isFinite(candidate.grid.cols) || !Number.isFinite(candidate.grid.rows)) return null;
  const collection = <T,>(items: T[] | null | undefined): T[] => Array.isArray(items) ? items : EMPTY_LAYOUT_ITEMS as T[];
  return {
    ...candidate,
    floors: collection(candidate.floors),
    aisles: collection(candidate.aisles),
    navigation_tags: collection(candidate.navigation_tags),
    navigation_edges: collection(candidate.navigation_edges),
    lifts: collection(candidate.lifts),
    zones: collection(candidate.zones),
    docks: collection(candidate.docks),
    racks: collection(candidate.racks),
    conveyors: collection(candidate.conveyors),
    stations: collection(candidate.stations),
    charging_stations: collection(candidate.charging_stations),
    parking: collection(candidate.parking),
    restricted_areas: collection(candidate.restricted_areas),
    walkways: collection(candidate.walkways),
    cameras: collection(candidate.cameras),
    sensors: collection(candidate.sensors),
    locations: collection(candidate.locations),
    obstacles: collection(candidate.obstacles),
    spawn: candidate.spawn && Array.isArray(candidate.spawn.robots) ? candidate.spawn : { robots: [] },
  } as CanonicalMapLayout;
}

/** 俯視 2D 地圖：導航網格障礙、Zone、輸送帶、機器人。TRAFFIC / HEATMAP 模式疊上熱區。 */
export function MapView2D(props: MapViewProps) {
  const layoutRevision = useStore((state) => state.layoutRevision);
  const mapLayout = useMemo(() => canonicalMapLayout(layout), [layoutRevision]);
  if (!mapLayout) return <div className="control-map-error">Warehouse map is refreshing.</div>;
  return <MapView2DCanvas {...props} layout={mapLayout} layoutRevision={layoutRevision} />;
}

function MapView2DCanvas({ mode, size, layout: mapLayout, layoutRevision }: MapViewProps & { layout: CanonicalMapLayout; layoutRevision: number }) {
  const [theme, setTheme] = useState<"dark" | "light">(() => {
    try {
      return typeof window !== "undefined" && window.localStorage.getItem("waretwin.map-theme") === "light" ? "light" : "dark";
    } catch {
      return "dark";
    }
  });
  useEffect(() => {
    try { window.localStorage.setItem("waretwin.map-theme", theme); } catch { /* storage is optional */ }
  }, [theme]);
  const light = theme === "light";
  const [zoom, setZoom] = useState(1);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [panDrag, setPanDrag] = useState<{ x: number; y: number; panX: number; panY: number } | null>(null);
  const mapColors = light
    ? { floor: "#f8fafc", hole: "#cbd5e1", grid: "#cbd5e1", blocked: "#94a3b8", rack: "#e2e8f0", rackText: "#0f172a", robotStroke: "#0f172a", label: "#0f172a", border: "#64748b" }
    : { floor: "#0a1020", hole: "#020617", grid: "#16213a", blocked: "#334155", rack: "#0b1220", rackText: "#e2e8f0", robotStroke: "#05080f", label: "#f8fafc", border: "#334155" };
  const twin = useStore((state) => state.twin);
  const runtimeMode = useStore((state) => state.runtimeMode);
  const runtimeState = useStore((state) => state.runtimeState);
  const activeFloorSel = useStore((s) => s.activeFloor);
  const mapFloor = typeof activeFloorSel === "number" ? activeFloorSel : 1;   // 2D 圖一次畫一層；All/Exploded 時畫一樓
  const canonicalMapFloor = canonicalFloorId(mapLayout, mapFloor);
  const allRobots = twin?.robots && typeof twin.robots === "object" && !Array.isArray(twin.robots) ? twin.robots : EMPTY_ROBOTS;
  const externalRuntime = runtimeMode === "GAZEBO_ROS" || runtimeMode === "REAL_ROBOT";
  const warehouseMapIdentity = {
    frame_id: mapLayout.coordinate_system?.frame ?? "",
    active_map_id: "CANONICAL",
    active_map_revision: layoutRevision,
    map_source: "CANONICAL",
  };
  const robots = useMemo(() => Object.fromEntries(Object.entries(allRobots).filter(([, robot]) => {
    if (robot?.floor !== mapFloor) return false;
    if (!externalRuntime) return true;
    return runtimeState !== "MAPPING" && robotPoseMatchesMap(robot, warehouseMapIdentity);
  })), [allRobots, externalRuntime, layoutRevision, mapFloor, mapLayout, runtimeState]);
  const hiddenRobotCount = externalRuntime ? Object.values(allRobots).filter((robot) => robot?.floor === mapFloor
    && (runtimeState === "MAPPING" || !robotPoseMatchesMap(robot, warehouseMapIdentity))).length : 0;
  const zones = twin?.zones && typeof twin.zones === "object" && !Array.isArray(twin.zones) ? twin.zones : EMPTY_ZONES;
  const selected = useStore((s) => s.selectedRobot);
  const openRobotQuickDetail = useStore((s) => s.openRobotQuickDetail);
  const openWindow = useStore((s) => s.openWindow);
  const selectedShelf = useStore((s) => s.selectedShelf);
  const selectShelf = useStore((s) => s.selectShelf);
  const tagGraph = useStore((s) => s.tagGraph);
  const tagMission = useStore((s) => s.tagNavigation);
  const setTargetTagId = useStore((s) => s.setTargetTagId);
  const { width: W, depth: D } = mapLayout.size;
  const activeFloor = mapLayout.floors.find((f) => sameFloor(f.id, canonicalMapFloor));
  const boundary = activeFloor ? floorBoundary(activeFloor, W, D) : [{ x: 0, y: 0 }, { x: W, y: 0 }, { x: W, y: D }, { x: 0, y: D }];
  const mapBounds = {
    minX: Math.min(...boundary.map((point) => point.x)), maxX: Math.max(...boundary.map((point) => point.x)),
    minY: Math.min(...boundary.map((point) => point.y)), maxY: Math.max(...boundary.map((point) => point.y)),
  };
  const mapOrigin = { x: mapBounds.minX, y: mapBounds.minY };
  const sameLayoutFloor = (id: number | string | undefined) => {
    const value = id ?? 1;
    return sameFloor(value, canonicalMapFloor) || resolveRuntimeFloorIndex(mapLayout, value) === resolveRuntimeFloorIndex(mapLayout, canonicalMapFloor);
  };
  const activeAisles = mapLayout.aisles.filter((aisle) => sameLayoutFloor(aisle.floor_id));
  const canonicalTags = mapLayout.navigation_tags.filter((tag) => sameLayoutFloor(tag.floor_id));
  const graphTags = tagGraph?.frame_id === "map" && tagGraph.units === "m" && Array.isArray(tagGraph.tags) ? tagGraph.tags : EMPTY_TAGS;
  const renderTags = canonicalTags.length > 0 ? canonicalTags : graphTags;
  const canonicalEdges = mapLayout.navigation_edges.filter((edge) => sameLayoutFloor(edge.floor_id));
  const graphEdges = tagGraph?.frame_id === "map" && tagGraph.units === "m" && Array.isArray(tagGraph.edges) ? tagGraph.edges : EMPTY_EDGES;
  const renderEdges = canonicalEdges.length > 0 ? canonicalEdges : graphEdges;
  const grid = useMemo(() => buildNavGrid(mapLayout, canonicalMapFloor, { origin: mapOrigin }), [canonicalMapFloor, layoutRevision, mapLayout, mapBounds.minX, mapBounds.minY]);

  // 障礙格合併成矩形 (逐列 run-length) 以減少 SVG 元素
  const blocks = useMemo(() => {
    const out: Array<[number, number, number]> = [];
    for (let r = 0; r < grid.rows; r++) {
      let c = 0;
      while (c < grid.cols) {
        if (grid.cells[r * grid.cols + c] === 1) { let e = c; while (e < grid.cols && grid.cells[r * grid.cols + e] === 1) e++; out.push([c, r, e - c]); c = e; } else c++;
      }
    }
    return out;
  }, [grid, mapFloor]);

  // TRAFFIC：即時密度 — 每台機器人以高斯核心擴散，速度越慢（塞住）越熱，加上最近 ~20 s 的短期軌跡
  // HEATMAP：長期累積 — 引擎的 traffic 陣列（幾乎不衰減），看的是「哪些走道一直在被使用」
  const tick = typeof twin?.sim?.tick === "number" ? twin.sim.tick : 0;
  const source = useStore((s) => s.source);
  const remoteHeat = useStore((s) => s.heat);
  const heat = useMemo(() => {
    if (mode === "MAP") return null;
    const cs = 2, cols = Math.ceil(W / cs), rows = Math.ceil(D / cs), v = new Float32Array(cols * rows);
    const layer = source === "online" ? remoteHeat?.[`${mode === "HEATMAP" ? "CONGESTION" : "TRAFFIC"}:${mapFloor}`] : undefined;
    if (layer) {
      // 後端已降採樣到 2 m 格並正規化
      for (let i = 0; i < Math.min(v.length, layer.values.length); i++) v[i] = layer.values[i] * 100;
    } else if (source !== "online") {
      const eng = getEngine(); const g = eng.grid;
      const src = (mode === "HEATMAP" ? eng.traffic : eng.trafficShort)[mapFloor];
      if (src) for (let r = 0; r < g.rows; r++) for (let c = 0; c < g.cols; c++) { const t = src[r * g.cols + c]; if (t > 0) v[Math.floor(r / cs) * cols + Math.floor(c / cs)] += t; }
    }
    if (mode === "TRAFFIC") {
      // 即時密度核心：半徑 ~5 m；停著不動且非閒置/充電的機器人權重加倍（瓶頸）
      for (const r of Object.values(robots)) {
        if (r.fsm === "IDLE" || r.fsm === "CHARGING" || r.fsm === "OFFLINE") continue;
        const w = r.velocity < 0.1 ? 60 : 30, cx = r.position[0] / cs, cz = r.position[2] / cs;
        for (let dr = -3; dr <= 3; dr++) for (let dc = -3; dc <= 3; dc++) { const rr = Math.floor(cz) + dr, cc = Math.floor(cx) + dc; if (rr < 0 || cc < 0 || rr >= rows || cc >= cols) continue; v[rr * cols + cc] += w * Math.exp(-(dr * dr + dc * dc) / 3); }
      }
    }
    // 平滑一次 (3x3)
    const out = new Float32Array(v.length);
    for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) { let s = 0, n = 0; for (let dr = -1; dr <= 1; dr++) for (let dc = -1; dc <= 1; dc++) { const rr = r + dr, cc = c + dc; if (rr < 0 || cc < 0 || rr >= rows || cc >= cols) continue; s += v[rr * cols + cc]; n++; } out[r * cols + c] = s / n; }
    let max = 0; for (let i = 0; i < out.length; i++) if (out[i] > max) max = out[i];
    return { cs, cols, rows, v: out, max: Math.max(max, 1) };
  // mapFloor / allRobots 必須在 deps 裡：暫停時切樓層才會重算，不會殘留上一層的資料（round-6 P2）
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [mode, tick, W, D, source, remoteHeat, mapFloor, allRobots]);

  const heatColor = (t: number) => {
    const stops = [[0, 37, 99, 235], [0.35, 34, 197, 94], [0.6, 234, 179, 8], [0.8, 249, 115, 22], [1, 239, 68, 68]];
    let i = 1; while (i < stops.length - 1 && stops[i][0] < t) i++;
    const [t0, r0, g0, b0] = stops[i - 1], [t1, r1, g1, b1] = stops[i]; const k = Math.min(1, Math.max(0, (t - t0) / (t1 - t0)));
    return `rgb(${Math.round(r0 + (r1 - r0) * k)},${Math.round(g0 + (g1 - g0) * k)},${Math.round(b0 + (b1 - b0) * k)})`;
  };

  const viewPadding = Math.max(mapBounds.maxX - mapBounds.minX, mapBounds.maxY - mapBounds.minY) * 0.03;
  const baseView = { x: mapBounds.minX - viewPadding, y: mapBounds.minY - viewPadding,
    width: mapBounds.maxX - mapBounds.minX + viewPadding * 2,
    height: mapBounds.maxY - mapBounds.minY + viewPadding * 2 };
  const zoomBy = (factor: number) => setZoom((value) => Math.max(0.5, Math.min(4, value * factor)));
  const resetView = () => { setZoom(1); setPan({ x: 0, y: 0 }); };
  const beginPan = (event: ReactPointerEvent<SVGSVGElement>) => {
    if (event.button !== 1 && !(event.button === 0 && event.shiftKey)) return;
    event.preventDefault();
    event.stopPropagation();
    setPanDrag({ x: event.clientX, y: event.clientY, panX: pan.x, panY: pan.y });
    event.currentTarget.setPointerCapture(event.pointerId);
  };
  const movePan = (event: ReactPointerEvent<SVGSVGElement>) => {
    if (!panDrag) return;
    const rect = event.currentTarget.getBoundingClientRect();
    const visibleWidth = baseView.width / zoom;
    const visibleHeight = baseView.height / zoom;
    setPan({
      x: panDrag.panX - ((event.clientX - panDrag.x) / Math.max(1, rect.width)) * visibleWidth,
      y: panDrag.panY - ((event.clientY - panDrag.y) / Math.max(1, rect.height)) * visibleHeight,
    });
  };
  const endPan = (event: ReactPointerEvent<SVGSVGElement>) => {
    if (panDrag && event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId);
    setPanDrag(null);
  };
  const zoomWheel = (event: ReactWheelEvent<SVGSVGElement>) => {
    event.preventDefault();
    const rect = event.currentTarget.getBoundingClientRect();
    const fractionX = (event.clientX - rect.left) / Math.max(1, rect.width);
    const fractionY = (event.clientY - rect.top) / Math.max(1, rect.height);
    const oldZoom = zoom;
    const nextZoom = Math.max(0.5, Math.min(4, oldZoom * (event.deltaY < 0 ? 1.12 : 0.89)));
    if (nextZoom === oldZoom) return;
    const oldWidth = baseView.width / oldZoom, oldHeight = baseView.height / oldZoom;
    const worldX = baseView.x + pan.x + fractionX * oldWidth;
    const worldY = baseView.y + pan.y + fractionY * oldHeight;
    const nextWidth = baseView.width / nextZoom, nextHeight = baseView.height / nextZoom;
    setZoom(nextZoom);
    setPan({ x: worldX - baseView.x - fractionX * nextWidth, y: worldY - baseView.y - fractionY * nextHeight });
  };

  return (
    <div className={`map2d-shell ${light ? "map2d-light" : "map2d-dark"}`} style={size ? { width: "100%", height: "100%" } : undefined}>
      <button type="button" className="map2d-theme-toggle" onClick={() => setTheme((value) => value === "dark" ? "light" : "dark")} aria-label={`Switch to ${light ? "dark" : "light"} map`}>
        {light ? "☾ Dark" : "☀ Light"}
      </button>
      {hiddenRobotCount > 0 && <div className="map2d-frame-notice" role="status">ROBOT OVERLAY HIDDEN · POSE DOES NOT MATCH WAREHOUSE MAP</div>}
      <div className="map-view-controls" aria-label="Map view controls">
        <button type="button" onClick={() => zoomBy(1.2)} aria-label="Zoom in">+</button>
        <span>{Math.round(zoom * 100)}%</span>
        <button type="button" onClick={() => zoomBy(0.833333)} aria-label="Zoom out">−</button>
        <button type="button" onClick={resetView} aria-label="Reset map view">⌂</button>
      </div>
      <svg className={`map2d-canvas${panDrag ? " is-panning" : ""}`} viewBox={`${baseView.x + pan.x} ${baseView.y + pan.y} ${baseView.width / zoom} ${baseView.height / zoom}`} preserveAspectRatio="xMidYMid meet" onClick={() => selectShelf(null)} onPointerDown={beginPan} onPointerMove={movePan} onPointerUp={endPan} onPointerCancel={endPan} onWheel={zoomWheel}>
      <g className="map2d-world" transform={worldToSvgTransform(mapBounds)}>
      <polygon points={polygonPoints(boundary)} fill={mapColors.floor} stroke={mapColors.border} strokeWidth="0.4" />
      {activeFloor?.holes?.map((hole, index) => <polygon key={`floor-hole-${index}`} points={polygonPoints(hole)} fill={mapColors.hole} stroke={mapColors.border} strokeWidth="0.25" />)}
      {/* 格線 */}
      {Array.from({ length: W / 10 + 1 }, (_, i) => <line key={"v" + i} x1={mapBounds.minX + i * 10} x2={mapBounds.minX + i * 10} y1={mapBounds.minY} y2={mapBounds.maxY} stroke={mapColors.grid} strokeWidth="0.15" />)}
      {Array.from({ length: D / 10 + 1 }, (_, i) => <line key={"h" + i} y1={mapBounds.minY + i * 10} y2={mapBounds.minY + i * 10} x1={mapBounds.minX} x2={mapBounds.maxX} stroke={mapColors.grid} strokeWidth="0.15" />)}
      {heat && (
        <g opacity="0.75">
          {Array.from(heat.v).map((s, i) => { const t = s / heat.max; if (t < 0.08) return null; return <rect key={i} x={mapBounds.minX + (i % heat.cols) * heat.cs} y={mapBounds.minY + Math.floor(i / heat.cols) * heat.cs} width={heat.cs} height={heat.cs} fill={heatColor(t)} opacity={Math.min(0.85, t + 0.15)} />; })}
        </g>
      )}
      {mapFloor !== 1 && <text x={1.5} y={-3.5} fill="#0f766e" fontSize="2.6" fontWeight="700">FLOOR {mapFloor} · MEZZANINE</text>}
      {mapLayout.zones.filter((z) => sameLayoutFloor(z.floor)).map((z) => {
        const st = zones[z.id]?.status; const col = st === "BLOCKED" ? "#ef4444" : st === "CONGESTED" ? "#f97316" : z.color;
        const points = z.polygon.map(([x, y]) => ({ x, y }));
        const label = zoneLabelLayout(points, Math.min(W, D));
        return <g key={z.id}><polygon points={z.polygon.map((p) => p.join(",")).join(" ")} fill={col} fillOpacity="0.05" stroke={col} strokeWidth="0.35" /><text x={label.x} y={label.y} fill={col} fontSize={label.fontSize} fontWeight="700" textAnchor="middle" dominantBaseline="middle">{z.name.toUpperCase()}</text></g>;
      })}
      {activeFloor && <polygon points={polygonPoints(boundary)} fill="#14b8a6" fillOpacity={light ? "0.08" : "0.03"} stroke="#14b8a6" strokeWidth="0.3" strokeDasharray="1.5 0.8" />}
      {activeAisles.map((aisle) => {
        const footprint = buildAisleFootprint(aisle.centerline, aisle.width);
        const footprintPoints = footprint.map((point) => `${point.x},${point.y}`).join(" ");
        const centerlinePoints = aisle.centerline.map((point) => `${point.x},${point.y}`).join(" ");
        return <g key={`live-aisle-${aisle.id}`}>
          {footprint.length > 0 && <polygon points={footprintPoints} fill="#22d3ee" fillOpacity="0.12" stroke="#22d3ee" strokeWidth="0.2" />}
          <polyline points={centerlinePoints} fill="none" stroke="#67e8f9" strokeWidth="0.18" strokeDasharray="0.7 0.45" />
        </g>;
      })}
      {mapLayout.lifts.map((l) => (
        <g key={l.id}><rect x={l.cell[0] - 0.7} y={l.cell[1] - 0.7} width="2.4" height="2.4" fill="none" stroke="#a78bfa" strokeWidth="0.35" /><text x={l.cell[0] + 2} y={l.cell[1] + 0.6} fill="#a78bfa" fontSize="1.8">{l.id}</text></g>
      ))}
      {blocks.map(([c, r, len], i) => <rect key={i} x={mapBounds.minX + c * mapLayout.grid.cell_size} y={mapBounds.minY + r * mapLayout.grid.cell_size} width={len * mapLayout.grid.cell_size} height={mapLayout.grid.cell_size} fill={mapColors.blocked} />)}
      {/* Exact rack footprints from the shared database map (nav cells above are only a conservative collision mask). */}
      {mapLayout.racks.filter((r) => sameLayoutFloor(r.floor)).map((r) => {
        const footprint = rackFootprint2D(r.position, r.size);
        const x=footprint.x, y=footprint.y, w=footprint.width, d=footprint.depth, cx=x+w/2, cy=y+d/2;
        const isSelected = selectedShelf === r.id;
        const occupancy = rackOccupancy(r);
        const ratio = occupancy.load / occupancy.capacity;
        // Shelf-level occupancy is rendered inside the physical shelf footprint.
        return (
          <g
            key={`rack-${r.id}`}
            transform={`rotate(${r.rotation ?? 0} ${cx} ${cy})`}
            style={{cursor:"pointer"}}
            onClick={(event)=>{event.stopPropagation();selectShelf(r.id);openWindow({ id: `shelf:${r.id}`, kind: "shelf", entityId: r.id, title: `Shelf ${r.id}` });}}
          >
            <rect x={x} y={y} width={w} height={d} rx="0.08" fill={isSelected ? (light ? "#bae6fd" : "#111f35") : mapColors.rack} stroke={isSelected ? "#0891b2" : "#d97706"} strokeWidth={isSelected ? 0.32 : 0.12} />
            {ratio > 0 && <rect x={x} y={y} width={w * ratio} height={d} rx="0.08" fill={isSelected ? "#2563eb" : "#1d4ed8"} fillOpacity={isSelected ? 0.72 : 0.58} />}
            <text x={cx} y={cy} fill={mapColors.rackText} fontSize="0.5" fontWeight="800" textAnchor="middle" dominantBaseline="middle" pointerEvents="none">{occupancy.percentLabel}%</text>
          </g>
        );
      })}
      {mapFloor === 1 && mapLayout.conveyors.map((c) => <polyline key={c.id} points={c.path.map((p) => p.join(",")).join(" ")} fill="none" stroke="#22d3ee" strokeWidth="1" strokeOpacity="0.9" />)}
      {mapFloor === 1 && mapLayout.docks.map((d) => <rect key={d.id} x={d.rect[0]} y={d.rect[1]} width={d.rect[2] - d.rect[0]} height={d.rect[3] - d.rect[1]} fill="none" stroke={d.kind === "INBOUND" ? "#22c55e" : "#22d3ee"} strokeWidth="0.3" strokeDasharray="1 0.6" />)}
      {mapFloor === 1 && mapLayout.charging_stations.map((c) => <circle key={c.id} cx={c.position[0]} cy={c.position[2]} r="0.6" fill="#3b82f6" />)}
      {renderTags.length > 0 && <g className="tag-navigation-overlay">
        {renderEdges.map((edge, index) => {
          const endpoints = resolveNavigationEdgeEndpoints(renderTags, edge);
          return endpoints ? <line key={`tag-edge-${"uuid" in edge ? edge.uuid : index}`} x1={endpoints.from.x} y1={endpoints.from.y} x2={endpoints.to.x} y2={endpoints.to.y} stroke="#f59e0b" strokeWidth="0.22" strokeDasharray="0.8 0.5" opacity="0.9" /> : null;
        })}
        {renderTags.map((tag) => { const state = tag.tag_id === tagMission?.current_tag_id ? "current" : tag.tag_id === tagMission?.next_tag_id ? "next" : tag.tag_id === tagMission?.target_tag_id ? "target" : tagMission?.route?.includes(tag.tag_id) ? "route" : "normal"; const identity = "uuid" in tag ? tag.uuid : String(tag.id); return <g key={`tag-${identity ?? tag.tag_id}`} transform={`translate(${tag.x},${tag.y})`} onClick={event => { event.stopPropagation(); setTargetTagId(tag.tag_id); }} style={{ cursor: "pointer" }}><circle r={state === "target" ? 0.55 : 0.4} fill={state === "current" ? "#22c55e" : state === "next" ? "#0284c7" : state === "target" ? "#e11d48" : state === "route" ? "#d97706" : "#64748b"} stroke={mapColors.label} strokeWidth="0.13" /><text x="0.65" y="-0.55" fill={mapColors.label} fontSize="0.85" fontWeight="700">{tag.tag_id}</text></g>; })}
      </g>}
      {mapLayout.cameras.filter((c) => sameLayoutFloor(c.floor)).map((c) => <rect key={c.id} x={c.position[0] - 0.5} y={c.position[2] - 0.5} width="1" height="1" fill="#facc15" />)}
      {Object.values(robots).map((r) => r.path.length > r.path_index && (
        <polyline key={"p" + r.id} points={[[r.position[0], r.position[2]], ...r.path.slice(r.path_index).map((c) => [c[0] + 0.5, c[1] + 0.5])].map((p) => p.join(",")).join(" ")} fill="none" stroke={r.id === selected ? "#fff" : "#22d3ee"} strokeWidth={r.id === selected ? 0.5 : 0.25} strokeOpacity={r.id === selected ? 1 : 0.5} strokeDasharray="1 0.6" />
      ))}
      {Object.values(robots).map((r) => {
        const sel = r.id === selected;
        return (
          <g key={r.id} transform={`translate(${r.position[0]},${r.position[2]})`} onClick={(event) => { event.stopPropagation(); openRobotQuickDetail(r.id); }} style={{ cursor: "pointer" }}>
            {sel && <circle r="2.2" fill="none" stroke="#60a5fa" strokeWidth="0.3" />}
            <circle r="1" fill={STATUS_COLOR[r.status]} stroke={mapColors.robotStroke} strokeWidth="0.25" />
            <line x1="0" y1="0" x2={Math.cos(r.heading) * 1.6} y2={Math.sin(r.heading) * 1.6} stroke="#fff" strokeWidth="0.25" />
            <text x="1.4" y="-1" fill={sel ? "#fff" : mapColors.label} fontSize="1.8" fontFamily="JetBrains Mono, monospace">{r.id}</text>
          </g>
        );
      })}
      {heat && (
        <g transform={`translate(${W - 22}, ${D - 3})`}>
          {Array.from({ length: 20 }, (_, i) => <rect key={i} x={i} y="0" width="1" height="1.2" fill={heatColor(i / 19)} />)}
          <text x="0" y="-0.6" fill={mapColors.label} fontSize="1.6">LOW</text><text x="20" y="-0.6" fill={mapColors.label} fontSize="1.6" textAnchor="end">HIGH</text>
          <text x="20" y="-3" fill={mapColors.label} fontSize="1.7" textAnchor="end" fontWeight="600">{mode === "TRAFFIC" ? "LIVE ROBOT DENSITY" : "ACCUMULATED TRAFFIC"}</text>
        </g>
      )}
      </g>
      </svg>
    </div>
  );
}
