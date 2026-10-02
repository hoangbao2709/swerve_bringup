import { memo, useEffect, useMemo, useRef, useState, type MouseEvent } from "react";
import { Canvas, useThree, useFrame } from "@react-three/fiber";
import { detailPerformance } from "./detailPerformance";
import { Line, OrbitControls } from "@react-three/drei";
import * as THREE from "three";
import type { RobotDetailLidar2D, RobotDetailLidar3D, RobotDetailPathPreview, RobotState } from "../../schema/twin_state";
import { createWorldTransform, screenToWorld, worldToScreen, type WorldBounds } from "../../layout/coordinates";
import { useStableDisplayedFramePose, type MapPoseIdentity } from "../../layout/robotPoseFrame";
import type { RobotDetailMapSnapshot } from "../../schema/twin_state";

type LocalPoint = { x: number; y: number };

export function RobotLidar2DView({ frame, robot, onPick, overlay, active = true }: {
  active?: boolean;
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
    if (!active || !canvas || size.width <= 0 || size.height <= 0) return;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    if (canvas.width !== Math.round(size.width * dpr)) canvas.width = Math.round(size.width * dpr);
    if (canvas.height !== Math.round(size.height * dpr)) canvas.height = Math.round(size.height * dpr);
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
    detailPerformance("view_render", { view: "LIDAR_2D", useful: Boolean(frame?.point_count), robot_id: frame?.robot_id });
  }, [active, frame, overlay, range, size, transform]);

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

export const RobotLidar3DView = memo(function RobotLidar3DView({ frame, robot, slamMap, active = true }: {
  active?: boolean;
  frame: RobotDetailLidar3D | null;
  robot?: RobotState;
  slamMap: RobotDetailMapSnapshot | null;
}) {
  const [renderActive, setRenderActive] = useState(false);
  useEffect(() => {
    if (!active) { setRenderActive(false); return; }
    let second = 0;
    const first = requestAnimationFrame(() => {
      second = requestAnimationFrame(() => setRenderActive(true));
    });
    return () => { cancelAnimationFrame(first); cancelAnimationFrame(second); };
  }, [active]);
  const mapIdentity = useMemo<MapPoseIdentity>(() => slamMap ?? {
    frame_id: frame?.slam_pose?.frame_id ?? "map",
    active_map_id: frame?.slam_pose?.map_id ?? "",
    active_map_revision: frame?.slam_pose?.map_revision ?? null,
    map_source: frame?.slam_pose?.map_source ?? "SLAM_TOOLBOX",
    mapping_session_id: frame?.slam_pose?.mapping_session_id ?? null,
  }, [frame?.slam_pose?.frame_id, frame?.slam_pose?.map_id, frame?.slam_pose?.map_revision,
    frame?.slam_pose?.map_source, frame?.slam_pose?.mapping_session_id, slamMap]);
  const poseRobot = useMemo(() => robot && frame?.slam_pose
    ? { ...robot, slam_pose: frame.slam_pose } : robot, [frame?.slam_pose, robot]);
  const displayedPose = useStableDisplayedFramePose(poseRobot, mapIdentity);
  const pointGeometry = useMemo(() => {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(20000 * 3), 3));
    geometry.setDrawRange(0, 0);
    return geometry;
  }, []);
  useEffect(() => {
    const attribute = pointGeometry.getAttribute("position") as THREE.BufferAttribute;
    const count = Math.min(frame?.points.length ?? 0, 20000);
    for (let index = 0; index < count; index++) attribute.setXYZ(index, ...frame!.points[index]);
    attribute.needsUpdate = true;
    pointGeometry.setDrawRange(0, count);
    pointGeometry.computeBoundingSphere();
  }, [pointGeometry, frame?.epoch, frame?.revision, frame?.points]);
  useEffect(() => () => pointGeometry.dispose(), [pointGeometry]);

  const trajectory = frame?.trajectory ?? [];
  return <div className="robot-lidar-view robot-lidar-3d-view" data-testid="slam-map-3d"
    data-accumulated-points={frame?.point_count ?? 0} data-map-frame={frame?.frame_id}
    data-map-source="SLAM_3D_ACCUMULATED_CLOUD" data-accumulation-mode={frame?.accumulation_mode}
    data-mapping-session-id={frame?.slam_pose?.mapping_session_id}
    data-trajectory-points={trajectory.length}>
    <Canvas frameloop={active && renderActive ? "demand" : "never"} onCreated={() => detailPerformance("canvas_3d_created")} camera={{ position: [7, -7, 6], up: [0, 0, 1], fov: 55, near: 0.05, far: 100 }} dpr={1}>
      <color attach="background" args={["#06101a"]} />
      <ambientLight intensity={0.8} />
      <axesHelper args={[1.2]} />
      {pointGeometry && <points geometry={pointGeometry}><pointsMaterial color="#45e3d4" size={0.045} sizeAttenuation /></points>}
      <gridHelper args={[30, 30, "#31526b", "#173146"]} position={[0, 0, -0.03]} rotation={[Math.PI / 2, 0, 0]} />
      {trajectory.length > 1 && <Line points={trajectory.map(([x, y]) => [x, y, 0.04])} color="#f5a64a" lineWidth={2} />}
      {displayedPose && <mesh position={[displayedPose.x, displayedPose.y, 0.12]} rotation={[0, 0, displayedPose.yaw]}>
        <boxGeometry args={[0.62, 0.42, 0.24]} /><meshBasicMaterial color="#39d6c7" wireframe />
      </mesh>}
      <OrbitControls makeDefault enableDamping={false} enabled={active} />
      <CameraFit frame={frame} />
      <RenderProbe frame={frame} active={active && renderActive} />
    </Canvas>
    <div className="robot-lidar-view-readout"><span>{frame?.point_count ?? 0} ACCUMULATED PTS</span><span>{frame?.render_fps?.toFixed(1) ?? "0.0"} FPS</span><span>{frame?.frame_id ?? "WAITING FOR SLAM MAP"}</span><span>VISUALIZATION ACCUMULATION · NOT 3D SLAM</span></div>
  </div>;
});

function RenderProbe({ frame, active }: { frame: RobotDetailLidar3D | null; active: boolean }) {
  const last = useRef("");
  const { invalidate } = useThree();
  useEffect(() => { if (active) { last.current = ""; invalidate(); } }, [active, frame, invalidate]);
  useFrame(() => {
    if (!active) return;
    const key = `${frame?.epoch}/${frame?.revision}`;
    if (last.current === key) return;
    last.current = key;
    detailPerformance("view_render", { view: "LIDAR_3D", useful: Boolean(frame?.point_count), robot_id: frame?.robot_id });
  });
  return null;
}

function CameraFit({ frame }: { frame: RobotDetailLidar3D | null }) {
  const { camera } = useThree();
  const fitted = useRef("");
  useEffect(() => {
    const mapIdentity = `${frame?.epoch ?? ""}/${frame?.slam_pose?.mapping_session_id ?? ""}`;
    if (!frame?.bounds || !mapIdentity || fitted.current === mapIdentity) return;
    const maxExtent = Math.max(...frame.bounds.max.map((value, index) => Math.abs(value - frame.bounds!.min[index])), 4);
    const center = frame.bounds.min.map((value, index) => (value + frame.bounds!.max[index]) / 2);
    camera.up.set(0, 0, 1);
    camera.position.set(center[0] + maxExtent * 0.6, center[1] - maxExtent * 0.7, center[2] + maxExtent * 0.55);
    camera.lookAt(center[0], center[1], center[2]);
    fitted.current = mapIdentity;
  }, [camera, frame?.bounds, frame?.epoch, frame?.slam_pose?.mapping_session_id]);
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
