import { useMemo, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import { Shape, Path, DoubleSide } from "three";
import type { LayoutFloor, WarehouseLayout } from "../../layout/types";
import type { OverviewRobot } from "./overviewRuntime";

type ViewMode = "3D" | "MAP";
type FloorGeometryPoint = [number, number] | { x: number; y: number };

function FloorSurface({ layout, floor }: { layout: WarehouseLayout; floor: LayoutFloor }) {
  const elevation = Number.isFinite(floor.elevation) ? floor.elevation : 0;
  const floorShape = useMemo(() => {
    const points = (floor.boundary ?? floor.footprint
      ?? [[0, 0], [layout.size.width, 0], [layout.size.width, layout.size.depth], [0, layout.size.depth]]) as FloorGeometryPoint[];
    const xy = (point: FloorGeometryPoint): [number, number] => Array.isArray(point)
      ? [point[0], point[1]] : [point.x, point.y];
    const shape = new Shape();
    points.forEach((point, index) => {
      const [x, y] = xy(point);
      if (index === 0) shape.moveTo(x, -y);
      else shape.lineTo(x, -y);
    });
    shape.closePath();
    for (const hole of floor.holes ?? []) {
      const path = new Path();
      hole.forEach((point, index) => {
        const [x, y] = xy(point);
        if (index === 0) path.moveTo(x, -y);
        else path.lineTo(x, -y);
      });
      path.closePath();
      shape.holes.push(path);
    }
    return shape;
  }, [floor, layout.size.depth, layout.size.width]);
  return <group position={[0, elevation, 0]}>
    <mesh rotation={[-Math.PI / 2, 0, 0]} receiveShadow>
      <shapeGeometry args={[floorShape]} />
      <meshStandardMaterial color="#111e28" roughness={0.92} side={DoubleSide} />
    </mesh>
    <gridHelper args={[Math.max(layout.size.width, layout.size.depth), Math.max(1, Math.round(Math.max(layout.size.width, layout.size.depth))), "#24404c", "#142a36"]}
      position={[layout.size.width / 2, 0.015, layout.size.depth / 2]} />
    <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.035, 0]}>
      <shapeGeometry args={[floorShape]} />
      <meshBasicMaterial color="#47c4cc" wireframe transparent opacity={0.22} />
    </mesh>
  </group>;
}

function Rack({ rack, elevation }: { rack: WarehouseLayout["racks"][number]; elevation: number }) {
  const [x, , z] = rack.position;
  const [width, height, depth] = rack.size;
  return <group position={[x, elevation + height / 2, z]} rotation={[0, -rack.rotation, 0]}>
    <mesh castShadow receiveShadow>
      <boxGeometry args={[width, height, depth]} />
      <meshStandardMaterial color="#365363" roughness={0.68} metalness={0.15} />
    </mesh>
    <mesh position={[0, 0, depth / 2 + 0.006]}>
      <boxGeometry args={[width * 0.88, Math.max(0.025, height * 0.025), 0.012]} />
      <meshBasicMaterial color="#55d1cc" />
    </mesh>
  </group>;
}

function RobotModel({ robot, floorElevation, onOpen }: { robot: OverviewRobot; floorElevation: number; onOpen: (id: string) => void }) {
  if (!robot.pose) return null;
  const [x, , z] = robot.pose.position;
  const heading = robot.pose.heading;
  return <group position={[x, floorElevation + 0.24, z]} rotation={[0, -heading, 0]}
    onClick={(event) => { event.stopPropagation(); onOpen(robot.id); }}
    onPointerOver={(event) => { event.stopPropagation(); document.body.style.cursor = "pointer"; }}
    onPointerOut={() => { document.body.style.cursor = ""; }}>
    <mesh castShadow>
      <boxGeometry args={[0.72, 0.34, 0.58]} />
      <meshStandardMaterial color="#26c6c8" emissive="#063b40" metalness={0.2} roughness={0.38} />
    </mesh>
    <mesh position={[0.4, 0.03, 0]}>
      <coneGeometry args={[0.17, 0.34, 3]} />
      <meshStandardMaterial color="#8af2df" emissive="#145e55" />
    </mesh>
  </group>;
}

function Warehouse3D({ layout, floorId, robots, onOpen }: {
  layout: WarehouseLayout; floorId: string; robots: OverviewRobot[]; onOpen: (id: string) => void;
}) {
  const floor = layout.floors.find((row) => String(row.id) === floorId) ?? layout.floors[0];
  const elevation = floor?.elevation ?? 0;
  const cameraHeight = Math.max(layout.size.width, layout.size.depth) * 0.82;
  if (!floor) return <div className="overview-no-map">ACTIVE FLOOR · UNAVAILABLE</div>;
  return <Canvas camera={{ position: [layout.size.width * 0.5 + cameraHeight * 0.72, elevation + cameraHeight, layout.size.depth * 0.5 + cameraHeight], fov: 42 }}>
    <color attach="background" args={["#07111a"]} />
    <ambientLight intensity={1.05} />
    <directionalLight position={[layout.size.width * 0.4, elevation + cameraHeight, layout.size.depth * 0.45]} intensity={1.4} castShadow />
    <FloorSurface layout={layout} floor={floor} />
    {(layout.racks ?? []).filter((rack) => rack.floor == null || String(rack.floor) === floorId)
      .map((rack) => <Rack key={rack.id} rack={rack} elevation={elevation} />)}
    {robots.filter((robot) => robot.pose && String(robot.pose.floor) === floorId)
      .map((robot) => <RobotModel key={robot.id} robot={robot} floorElevation={elevation} onOpen={onOpen} />)}
    <OrbitControls makeDefault enableDamping minDistance={2} maxDistance={cameraHeight * 3} target={[layout.size.width / 2, elevation, layout.size.depth / 2]} />
  </Canvas>;
}

function MapView({ layout, floorId, robots, onOpen }: {
  layout: WarehouseLayout; floorId: string; robots: OverviewRobot[]; onOpen: (id: string) => void;
}) {
  const floor = layout.floors.find((row) => String(row.id) === floorId) ?? layout.floors[0];
  if (!floor) return <div className="overview-no-map">ACTIVE FLOOR · UNAVAILABLE</div>;
  const width = Math.max(0.1, layout.size.width), depth = Math.max(0.1, layout.size.depth);
  const floorPoints = floor.boundary ?? floor.footprint ?? [[0, 0], [width, 0], [width, depth], [0, depth]];
  const floorPolygon = floorPoints.map((point) => {
    const xy = Array.isArray(point) ? { x: point[0], y: point[1] } : point;
    return `${xy.x},${depth - xy.y}`;
  }).join(" ");
  return <svg className="overview-map-svg" viewBox={`0 0 ${width} ${depth}`} role="img" aria-label={`${floor.name} warehouse map`}>
    <defs><pattern id="overview-grid" width="1" height="1" patternUnits="userSpaceOnUse"><path d="M 1 0 L 0 0 0 1" fill="none" stroke="#18303c" strokeWidth="0.025" /></pattern></defs>
    <polygon points={floorPolygon} fill="#0c1a24" stroke="#36717b" strokeWidth="0.08" />
    <rect x="0" y="0" width={width} height={depth} fill="url(#overview-grid)" />
    {(layout.racks ?? []).filter((rack) => rack.floor == null || String(rack.floor) === floorId)
      .map((rack) => {
        const [x, , y] = rack.position;
        const [w, , d] = rack.size;
        const angle = -rack.rotation * 180 / Math.PI;
        return <g key={rack.id} transform={`rotate(${angle} ${x} ${depth - y})`}>
          <rect x={x - w / 2} y={depth - y - d / 2} width={w} height={d} rx="0.04" fill="#294455" stroke="#4b8790" strokeWidth="0.04" />
          <text x={x} y={depth - y} textAnchor="middle" dominantBaseline="central" className="overview-rack-label">{rack.id}</text>
        </g>;
      })}
    {(layout.docks ?? []).filter((dock) => dock.floor == null || String(dock.floor) === floorId).map((dock) => {
      const [x, y, w, h] = dock.rect;
      return <rect key={dock.id} x={x} y={depth - y - h} width={w} height={h} fill={dock.kind === "INBOUND" ? "#184737" : "#4a3327"} stroke="#6e9287" strokeWidth="0.04" />;
    })}
    {robots.filter((robot) => robot.pose && String(robot.pose.floor) === floorId).map((robot) => {
      const pose = robot.pose!.canonical_pose!;
      const markerY = depth - pose.y;
      const angle = -pose.yaw * 180 / Math.PI;
      return <g key={robot.id} className="overview-map-robot" role="button" tabIndex={0} aria-label={`Open robot ${robot.id}`}
        transform={`translate(${pose.x} ${markerY})`} onClick={() => onOpen(robot.id)}
        onKeyDown={(event) => { if (event.key === "Enter" || event.key === " ") { event.preventDefault(); onOpen(robot.id); } }}>
        <circle r="0.48" fill="#37d5d1" fillOpacity="0.2" stroke="#57e9df" strokeWidth="0.055" />
        <g transform={`rotate(${angle})`}><path d="M 0.35 0 L -0.2 -0.22 L -0.2 0.22 Z" fill="#e6ffff" /></g>
        <text x="0.58" y="-0.28" className="overview-robot-label">{robot.id}</text>
      </g>;
    })}
  </svg>;
}

export function OverviewWarehouseView({ layout, robots, onOpenRobot }: {
  layout: WarehouseLayout | null; robots: OverviewRobot[]; onOpenRobot: (id: string) => void;
}) {
  const [mode, setMode] = useState<ViewMode>("3D");
  const floors = layout?.floors ?? [];
  const [floorId, setFloorId] = useState(() => String(floors[0]?.id ?? ""));
  const selectedFloorId = floors.some((floor) => String(floor.id) === floorId) ? floorId : String(floors[0]?.id ?? "");
  if (!layout || !floors.length) return <div className="overview-no-map" role="status">WAREHOUSE MAP · UNAVAILABLE</div>;
  return <div className="overview-visualization">
    <div className="overview-view-toolbar">
      <div className="overview-view-tabs" role="tablist" aria-label="Warehouse view">
        {(["3D", "MAP"] as const).map((value) => <button key={value} type="button" role="tab" aria-selected={mode === value}
          className={mode === value ? "is-active" : ""} onClick={() => setMode(value)}>{value === "3D" ? "3D VIEW" : "MAP VIEW"}</button>)}
      </div>
      <label className="overview-floor-select"><span>FLOOR</span><select value={selectedFloorId} onChange={(event) => setFloorId(event.target.value)}>
        {floors.map((floor) => <option key={String(floor.id)} value={String(floor.id)}>{floor.name}</option>)}
      </select></label>
      <span className="overview-map-name">{layout.name || layout.id}</span>
    </div>
    <div className="overview-visualization-stage" data-testid={mode === "3D" ? "warehouse-view-3d" : "warehouse-view-map"}>
      {mode === "3D" ? <Warehouse3D layout={layout} floorId={selectedFloorId} robots={robots} onOpen={onOpenRobot} />
        : <MapView layout={layout} floorId={selectedFloorId} robots={robots} onOpen={onOpenRobot} />}
    </div>
  </div>;
}
