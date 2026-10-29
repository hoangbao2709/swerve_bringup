import { useLayoutEffect, useMemo, useRef } from "react";
import { Billboard, Text } from "@react-three/drei";
import * as THREE from "three";
import { layout, useStore } from "../../state/store";
import { rackOccupancy } from "../../layout/shelfOccupancy";
import { sameFloor } from "../../layout/types";

const dummy = new THREE.Object3D();
const color = new THREE.Color();
const BOX_PALETTE = ["#b8834a", "#c99a63", "#a87440", "#d2a874", "#9c6b3c"];

/**
 * Rack geometry is derived from the same x/z/size/rotation values used by the editor.
 * Each rack has 8 order slots. `current_load` controls the exact number of boxes that
 * are rendered, so 0/1/2/3 orders in the DB/layout become exactly 0/1/2/3 boxes in 3D.
 */
export function RackInstances({
  castShadow = true,
  floor = 1,
  yOffset = 0,
  labels = true,
}: {
  castShadow?: boolean;
  floor?: number;
  yOffset?: number;
  labels?: boolean;
}) {
  const revision = useStore((s) => s.layoutRevision);
  const tool = useStore((s) => s.tool);
  const showLabels = useStore((s) => s.showLabels);
  const rackLoadLabels = useStore((s) => s.labelLayers.rackLoads);
  const selectedShelf = useStore((s) => s.selectedShelf);
  const selectShelf = useStore((s) => s.selectShelf);
  const openWindow = useStore((s) => s.openWindow);
  const postRef = useRef<THREE.InstancedMesh>(null!);
  const beamRef = useRef<THREE.InstancedMesh>(null!);
  const boxRef = useRef<THREE.InstancedMesh>(null!);

  const data = useMemo(() => {
    const posts: THREE.Matrix4[] = [];
    const beams: THREE.Matrix4[] = [];
    const boxes: { m: THREE.Matrix4; c: string }[] = [];
    const occupancyLabels: Array<{
      id: string;
      position: [number, number, number];
      text: string;
    }> = [];

    for (const [rackIndex, r] of layout.racks.entries()) {
      if (!sameFloor(r.floor ?? 1, floor)) continue;
      const [x, , z] = r.position;
      const [w, h, d] = r.size;
      const levelH = h / Math.max(1, r.levels);
      const angle = ((r.rotation ?? 0) * Math.PI) / 180;
      const cx = x + w / 2;
      const cz = z + d / 2;
      const c = Math.cos(angle);
      const s = Math.sin(angle);

      const world = (px: number, pz: number): [number, number] => {
        const dx = px - cx;
        const dz = pz - cz;
        return [cx + dx * c + dz * s, cz - dx * s + dz * c];
      };
      const matrix = (
        px: number,
        py: number,
        pz: number,
        sx: number,
        sy: number,
        sz: number,
        extraYaw = 0,
      ) => {
        const [wx, wz] = world(px, pz);
        dummy.position.set(wx, py, wz);
        dummy.scale.set(sx, sy, sz);
        dummy.rotation.set(0, angle + extraYaw, 0);
        dummy.updateMatrix();
        return dummy.matrix.clone();
      };

      for (const [dx, dz] of [[0, 0], [w, 0], [0, d], [w, d]]) {
        posts.push(matrix(x + dx, h / 2, z + dz, 0.1, h, 0.1));
      }

      for (let l = 0; l < r.levels; l++) {
        const y = l * levelH + 0.05;
        for (const dz of [0.02, d - 0.02]) {
          beams.push(matrix(x + w / 2, y, z + dz, w, 0.1, 0.08));
        }
        beams.push(matrix(x + w / 2, y - 0.02, z + d / 2, w, 0.04, d));
      }

      const occupancy = rackOccupancy(r);
      // The standard model is 4 levels x 2 slots = 8. Keep the same 8-slot
      // contract even if an editor changes the number of rack levels.
      const slotsPerLevel = Math.max(1, Math.ceil(occupancy.capacity / Math.max(1, r.levels)));
      for (let slotIndex = 0; slotIndex < occupancy.load; slotIndex++) {
        const level = Math.min(Math.max(1, r.levels) - 1, Math.floor(slotIndex / slotsPerLevel));
        const column = slotIndex % slotsPerLevel;
        const shelfY = level * levelH + 0.05;
        const bw = Math.max(0.25, Math.min((w / slotsPerLevel) * 0.72, 1.05));
        const bh = Math.max(0.35, Math.min(levelH - 0.22, levelH * 0.56));
        const bd = Math.max(0.2, d - 0.24);
        const px = x + w * ((column + 0.5) / slotsPerLevel);
        const m = matrix(px, shelfY + bh / 2 + 0.04, z + d / 2, bw, bh, bd);
        boxes.push({ m, c: BOX_PALETTE[(rackIndex + slotIndex) % BOX_PALETTE.length] });
      }

      occupancyLabels.push({
        id: r.id,
        position: [cx, h + 0.52, cz],
        text: `${occupancy.percentLabel}%`,
      });

    }
    return { posts, beams, boxes, occupancyLabels };
  }, [floor, revision]);

  useLayoutEffect(() => {
    data.posts.forEach((m, i) => postRef.current.setMatrixAt(i, m));
    data.beams.forEach((m, i) => beamRef.current.setMatrixAt(i, m));
    data.boxes.forEach((b, i) => {
      boxRef.current.setMatrixAt(i, b.m);
      boxRef.current.setColorAt(i, color.set(b.c));
    });
    postRef.current.instanceMatrix.needsUpdate = true;
    beamRef.current.instanceMatrix.needsUpdate = true;
    boxRef.current.instanceMatrix.needsUpdate = true;
    if (boxRef.current.instanceColor) boxRef.current.instanceColor.needsUpdate = true;
    [postRef, beamRef, boxRef].forEach((r) => r.current.computeBoundingSphere());
  }, [data]);

  const selectableRacks = useMemo(
    () => layout.racks.filter((r) => (r.floor ?? 1) === floor),
    [floor, revision],
  );

  return (
    <group position-y={yOffset}>
      <instancedMesh ref={postRef} args={[undefined, undefined, data.posts.length]} castShadow={castShadow} receiveShadow frustumCulled={false}>
        <boxGeometry />
        <meshStandardMaterial color="#2f3a4a" roughness={0.6} metalness={0.6} />
      </instancedMesh>
      <instancedMesh ref={beamRef} args={[undefined, undefined, data.beams.length]} frustumCulled={false}>
        <boxGeometry />
        <meshStandardMaterial color="#e07a1f" roughness={0.5} metalness={0.5} />
      </instancedMesh>
      <instancedMesh ref={boxRef} args={[undefined, undefined, data.boxes.length]} castShadow={castShadow} receiveShadow frustumCulled={false}>
        <boxGeometry />
        <meshStandardMaterial roughness={0.9} metalness={0} />
      </instancedMesh>

      {labels && showLabels && rackLoadLabels.visible && data.occupancyLabels.map((label) => (
        <Billboard key={`rack-load-${label.id}`} position={label.position} follow lockX={false} lockY={false} lockZ={false}>
          <Text
            renderOrder={rackLoadLabels.zIndex * 10}
            fontSize={0.48}
            color={selectedShelf === label.id ? "#67e8f9" : "#e2e8f0"}
            anchorX="center"
            anchorY="middle"
            outlineWidth={0.035}
            outlineColor="#05080f"
          >
            {label.text}
          </Text>
        </Billboard>
      ))}

      {/*
        Per-rack invisible hit targets make the Select tool deterministic. The
        visual rack uses instancing for performance, while this lightweight box
        gives every physical shelf/rack its own click target and exact ID.
      */}
      {selectableRacks.map((r) => {
        const [x, , z] = r.position;
        const [w, h, d] = r.size;
        const angle = ((r.rotation ?? 0) * Math.PI) / 180;
        const selected = selectedShelf === r.id;
        return (
          <group key={`rack-select-${r.id}`} position={[x + w / 2, h / 2, z + d / 2]} rotation={[0, angle, 0]}>
            <mesh
              onClick={(event) => {
                if (tool !== "select") return;
                event.stopPropagation();
                selectShelf(r.id);
                openWindow({ id: `shelf:${r.id}`, kind: "shelf", entityId: r.id, title: `Shelf ${r.id}` });
              }}
              onPointerOver={(event) => {
                if (tool !== "select") return;
                event.stopPropagation();
                document.body.style.cursor = "pointer";
              }}
              onPointerOut={() => {
                if (tool === "select") document.body.style.cursor = "";
              }}
            >
              <boxGeometry args={[Math.max(w, 0.12), Math.max(h, 0.12), Math.max(d, 0.12)]} />
              <meshBasicMaterial transparent opacity={0} depthWrite={false} />
            </mesh>
            {selected && (
              <mesh scale={[1.035, 1.035, 1.035]} raycast={() => null}>
                <boxGeometry args={[Math.max(w, 0.12), Math.max(h, 0.12), Math.max(d, 0.12)]} />
                <meshBasicMaterial color="#22d3ee" wireframe depthTest={false} />
              </mesh>
            )}
          </group>
        );
      })}
    </group>
  );
}
