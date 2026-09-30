import { useEffect, useMemo, useRef, useState, type MouseEvent } from "react";
import { Canvas, useThree } from "@react-three/fiber";
import { Grid, Line, OrbitControls } from "@react-three/drei";
import * as THREE from "three";
import type { RobotDetailLidar2D, RobotDetailLidar3D, RobotDetailPathPreview, RobotState } from "../../schema/twin_state";
import { createWorldTransform, screenToWorld, worldToScreen, type WorldBounds } from "../../layout/coordinates";

type LocalPoint = { x: number; y: number };

export function RobotLidar2DView({ frame, robot, onPick, overlay }: {
  frame: RobotDetailLidar2D | null;
  robot?: RobotState;
  onPick?: (point: LocalPoint) => void;
  overlay?: { path: Array<[number, number]>; goal: { x: number; y: number; yaw: number } | null } | null;
}) {
  const hostRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });
  const [range, setRange] = useState(12);
  const transform = useMemo(() => createWorldTransform(
    size,
    { minX: -range, maxX: range, minY: -range, maxY: range },
    1,
    { x: 0, y: 0 },
    24,
  ), [range, size]);

  useEffect(() => {
    const host = hostRef.current;
    if (!host) return;
    const resize = () => setSize({ width: host.clientWidth, height: host.clientHeight });
    resize();
    const observer = typeof ResizeObserver === "undefined" ? null : new ResizeObserver(resize);
    observer?.observe(host);
    window.addEventListener("resize", resize);
    return () => { observer?.disconnect(); window.removeEventListener("resize", resize); };
  }, []);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || size.width <= 0 || size.height <= 0) return;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    canvas.width = Math.round(size.width * dpr);
    canvas.height = Math.round(size.height * dpr);
    canvas.style.width = `${size.width}px`;
    canvas.style.height = `${size.height}px`;
    const context = canvas.getContext("2d");
    if (!context) return;
    context.setTransform(dpr, 0, 0, dpr, 0, 0);
    context.fillStyle = "#06101a";
    context.fillRect(0, 0, size.width, size.height);
    context.strokeStyle = "rgba(84, 128, 160, .2)";
    context.lineWidth = 1;
    const gridStep = range <= 8 ? 1 : 2;
    for (let value = -range; value <= range; value += gridStep) {
      const a = worldToScreen({ x: value, y: -range }, transform);
      const b = worldToScreen({ x: value, y: range }, transform);
      context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y); context.stroke();
      const c = worldToScreen({ x: -range, y: value }, transform);
      const d = worldToScreen({ x: range, y: value }, transform);
      context.beginPath(); context.moveTo(c.x, c.y); context.lineTo(d.x, d.y); context.stroke();
    }
    const origin = worldToScreen({ x: 0, y: 0 }, transform);
    context.strokeStyle = "#39d6c7"; context.lineWidth = 2;
    context.beginPath(); context.moveTo(origin.x - 13, origin.y - 9); context.lineTo(origin.x + 13, origin.y - 9); context.lineTo(origin.x + 13, origin.y + 9); context.lineTo(origin.x - 13, origin.y + 9); context.closePath(); context.stroke();
    context.beginPath(); context.moveTo(origin.x, origin.y); context.lineTo(origin.x + 20, origin.y); context.stroke();

    if (frame) {
      context.fillStyle = "#41edda";
      for (const [x, y] of frame.points) {
        if (Math.abs(x) > range || Math.abs(y) > range) continue;
        const point = worldToScreen({ x, y }, transform);
        context.fillRect(point.x - 1.5, point.y - 1.5, 3, 3);
      }
      const path = overlay?.path ?? frame.path;
      if (path.length > 1) {
        context.beginPath();
        path.forEach(([x, y], index) => {
          const point = worldToScreen({ x, y }, transform);
          if (index === 0) context.moveTo(point.x, point.y); else context.lineTo(point.x, point.y);
        });
        context.strokeStyle = "#aa91ff"; context.lineWidth = 2.5; context.stroke();
      }
      const goal = overlay?.goal ?? frame.goal;
      if (goal) {
        const point = worldToScreen(goal, transform);
        context.strokeStyle = "#f4cf52"; context.lineWidth = 2;
        context.beginPath(); context.arc(point.x, point.y, 7, 0, Math.PI * 2); context.stroke();
      }
    }
    context.fillStyle = "#8aa4bf"; context.font = "10px JetBrains Mono, monospace";
    context.fillText(`BASE FRAME · ${frame?.frame_id ?? "WAITING FOR SCAN"}`, 10, 18);
  }, [frame, range, size, transform]);

  const pick = (event: MouseEvent<HTMLCanvasElement>) => {
    if (!onPick) return;
    const rect = event.currentTarget.getBoundingClientRect();
    onPick(screenToWorld({ x: event.clientX - rect.left, y: event.clientY - rect.top }, transform));
  };
  const age = frame?.timestamp ? Math.max(0, (Date.now() - Date.parse(frame.timestamp)) / 1000) : null;
  return <div className="robot-lidar-view" ref={hostRef}>
    <canvas ref={canvasRef} onClick={pick} className={onPick ? "is-target-pick" : ""} aria-label="Live 2D LiDAR map in the robot base frame" />
    <div className="robot-lidar-view-controls"><button type="button" onClick={() => setRange((value) => Math.max(3, value / 1.25))}>ZOOM +</button><button type="button" onClick={() => setRange((value) => Math.min(40, value * 1.25))}>ZOOM −</button></div>
    <div className="robot-lidar-view-readout"><span>{frame?.point_count ?? 0} PTS</span><span>{frame?.render_fps?.toFixed(1) ?? "0.0"} FPS</span><span>{age === null ? "NO FRAME" : `${age.toFixed(2)} s`}</span><span>POSE {robot ? `${robot.position[0].toFixed(2)}, ${robot.position[2].toFixed(2)}` : "N/A"}</span></div>
  </div>;
}

export function RobotLidar3DView({ frame, overlay }: {
  frame: RobotDetailLidar3D | null;
  overlay?: { path: Array<[number, number]>; goal: { x: number; y: number; yaw: number } | null } | null;
}) {
  const [pointGeometry, setPointGeometry] = useState<THREE.BufferGeometry | null>(null);
  useEffect(() => {
    if (!frame?.points.length) { setPointGeometry(null); return; }
    const packed = new Float32Array(frame.points.length * 3);
    frame.points.forEach((point, index) => packed.set(point, index * 3));
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(packed, 3));
    geometry.computeBoundingSphere();
    setPointGeometry(geometry);
    return () => geometry.dispose();
  }, [frame?.epoch, frame?.revision, frame?.points]);

  return <div className="robot-lidar-view robot-lidar-3d-view">
    <Canvas camera={{ position: [7, -7, 6], up: [0, 0, 1], fov: 55, near: 0.05, far: 100 }} dpr={[1, 1.5]}>
      <color attach="background" args={["#06101a"]} />
      <ambientLight intensity={0.8} />
      <axesHelper args={[1.2]} />
      <Grid args={[30, 30]} position={[0, 0, -0.03]} rotation={[Math.PI / 2, 0, 0]} cellSize={1} sectionSize={5} cellColor="#173146" sectionColor="#31526b" fadeDistance={24} infiniteGrid />
      {pointGeometry && <points geometry={pointGeometry}><pointsMaterial color="#45e3d4" size={0.045} sizeAttenuation /></points>}
      {(overlay?.path ?? frame?.path ?? []).length > 1 && <Line points={(overlay?.path ?? frame?.path ?? []).map(([x, y]) => [x, y, 0.05])} color="#aa91ff" lineWidth={2} />}
      {(overlay?.goal ?? frame?.goal) && <mesh position={[(overlay?.goal ?? frame?.goal)!.x, (overlay?.goal ?? frame?.goal)!.y, 0.08]}><sphereGeometry args={[0.12, 12, 8]} /><meshBasicMaterial color="#f4cf52" /></mesh>}
      <mesh position={[0, 0, 0.12]}><boxGeometry args={[0.62, 0.42, 0.24]} /><meshStandardMaterial color="#39d6c7" wireframe /></mesh>
      <OrbitControls makeDefault enableDamping dampingFactor={0.08} />
      <CameraFit frame={frame} />
    </Canvas>
    <div className="robot-lidar-view-readout"><span>{frame?.point_count ?? 0} PTS</span><span>{frame?.render_fps?.toFixed(1) ?? "0.0"} FPS</span><span>{frame?.source_frame_id ?? "WAITING FOR CLOUD"}</span><span>REV {frame?.revision ?? "—"}</span></div>
  </div>;
}

function CameraFit({ frame }: { frame: RobotDetailLidar3D | null }) {
  const { camera } = useThree();
  useEffect(() => {
    if (!frame?.bounds) return;
    const maxExtent = Math.max(...frame.bounds.max.map((value, index) => Math.abs(value - frame.bounds!.min[index])), 4);
    camera.up.set(0, 0, 1);
    camera.position.set(maxExtent * 0.6, -maxExtent * 0.7, maxExtent * 0.55);
    camera.lookAt(0, 0, 0);
  }, [camera, frame?.epoch]);
  return null;
}

export function localLidarPointToMap(point: LocalPoint, robot: RobotState): LocalPoint {
  const yaw = robot.heading;
  const cosine = Math.cos(yaw), sine = Math.sin(yaw);
  return {
    x: robot.position[0] + cosine * point.x - sine * point.y,
    y: robot.position[2] + sine * point.x + cosine * point.y,
  };
}

export function previewLidarPath(preview: RobotDetailPathPreview | null, frame: RobotDetailLidar2D | RobotDetailLidar3D | null) {
  if (!preview || preview.status !== "VALID" || !frame) return null;
  return { path: preview.local_path ?? [], goal: preview.local_goal ?? null };
}

export function withinWorldBounds(point: LocalPoint, bounds: WorldBounds): boolean {
  return point.x >= bounds.minX && point.x <= bounds.maxX && point.y >= bounds.minY && point.y <= bounds.maxY;
}
