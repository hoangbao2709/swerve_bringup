import { useMemo } from "react";
import * as THREE from "three";
import { layout } from "../../state/store";

/** 地板與 canonical layout 定義的結構柱、碼頭 */
export function WarehouseShell({ lite = false, light = false }: { lite?: boolean; light?: boolean }) {
  const { width: W, depth: D, height: H } = layout.size;
  const floorTex = useMemo(() => {
    const c = document.createElement("canvas"); c.width = c.height = 512;
    const g = c.getContext("2d")!;
    g.fillStyle = light ? "#dce6f0" : "#1b2230"; g.fillRect(0, 0, 512, 512);
    // 細緻噪點
    for (let i = 0; i < 9000; i++) { g.fillStyle = `rgba(255,255,255,${Math.random() * 0.045})`; g.fillRect(Math.random() * 512, Math.random() * 512, 2, 2); }
    // 格線 (每 1m，一格 = 512/8 px => texture repeat 每 8 m)
    g.strokeStyle = light ? "rgba(71,85,105,0.22)" : "rgba(120,140,170,0.18)"; g.lineWidth = 1.5;
    for (let i = 0; i <= 8; i++) { const p = (i * 512) / 8; g.beginPath(); g.moveTo(p, 0); g.lineTo(p, 512); g.moveTo(0, p); g.lineTo(512, p); g.stroke(); }
    const t = new THREE.CanvasTexture(c); t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(W / 8, D / 8); t.anisotropy = 8; t.colorSpace = THREE.SRGBColorSpace;
    return t;
  }, [W, D, light]);

  // round-9d：柱位不再程序生成（舊版會把柱子長在輸送帶與中央走道上、且導航網格不知道）。
  // 單一事實來源 = layout.obstacles（kind PILLAR），F1 網格用同一份資料封鎖 —— 視覺與路徑永遠一致。
  const pillars = (layout.obstacles ?? []).filter((o) => o.kind === "PILLAR");

  return (
    <group>
      {/* 地板 */}
      <mesh rotation-x={-Math.PI / 2} position={[W / 2, 0, D / 2]} receiveShadow>
        <planeGeometry args={[W, D]} />
        <meshStandardMaterial map={floorTex} roughness={0.85} metalness={0.15} />
      </mesh>
      {/* Four perimeter walls are intentionally hidden to keep the 3D warehouse view open. */}
      {/* 柱子 */}
      {!lite && pillars.map((pillar) => {
        const [x0, z0, x1, z1] = pillar.rect;
        return (
          <mesh key={pillar.id} position={[(x0 + x1) / 2, H / 2, (z0 + z1) / 2]} castShadow>
            <boxGeometry args={[x1 - x0, H, z1 - z0]} />
            <meshStandardMaterial color="#27313f" roughness={0.8} metalness={0.3} />
          </mesh>
        );
      })}
      {/* 碼頭門 */}
      {layout.docks.map((d) => (
        <group key={d.id} position={[d.door[0], 0, 0.25]}>
          <mesh position={[0, 2.4, 0]}>
            <boxGeometry args={[5.5, 4.8, 0.3]} />
            <meshStandardMaterial color="#0e1a2b" roughness={0.6} metalness={0.4} emissive={d.kind === "INBOUND" ? "#14532d" : "#164e63"} emissiveIntensity={0.6} />
          </mesh>
          <mesh position={[0, 5, 0]}>
            <boxGeometry args={[6.2, 0.3, 0.5]} />
            <meshBasicMaterial color={d.kind === "INBOUND" ? "#22c55e" : "#22d3ee"} />
          </mesh>
        </group>
      ))}
    </group>
  );
}
