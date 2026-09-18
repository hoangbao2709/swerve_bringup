import React, { useEffect, useMemo, useRef, useState } from "react";
import { Canvas, useThree, type ThreeEvent } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import type { OrbitControls as OrbitControlsImpl } from "three-stdlib";
import * as THREE from "three";
import { apiFetch } from "../../services/api";
import { onLayoutUpdated, wsSend } from "../../services/ws";
import { DEMO_MODE } from "../../config";
import { useStore } from "../../state/store";
import type { WarehouseLayout } from "../../layout/types";
import { buildAisleFootprint, pointInPolygon, polygonContainedInFloor, snapPoint, validateAisleCenterline, validateFloorPolygon, validateHolePolygon, validatePolygon, type Point } from "../../layout/geometry";
import { applyNavigationTagPhysicalOverride, generateAisleTags, nextAisleId, validateNavigationTagIdChange, validateNavigationTagsForLayout } from "../../layout/navigation";
import { generateNavigationEdges, validateNavigationEdges } from "../../layout/navigation_graph";
import { validatePhysicalObjectOverlaps } from "../../layout/validation";
import layoutJson from "../../layout/warehouse_layout.json";

type Kind =
  | "floor"
  | "aisle"
  | "navigation-tag"
  | "navigation-edge"
  | "rack"
  | "wall"
  | "conveyor"
  | "dock"
  | "lift"
  | "charger"
  | "station"
  | "zone"
  | "parking"
  | "restricted"
  | "walkway"
  | "camera"
  | "sensor"
  | "location"
  | "spawn"
  | "column";

type Draft = WarehouseLayout;
type Selection = { kind: Kind; id: string };
type EditorBlock = {
  id: string;
  name: string;
  members: Selection[];
};
type Vec3 = [number, number, number];
type Mode = "select" | "move" | "measure" | "draw-floor" | "edit-vertex" | "add-hole" | "draw-aisle" | "edit-aisle" | "add-manual-tag";
type ContextAction =
  | "focus"
  | "rotateLeft"
  | "rotateRight"
  | "duplicate"
  | "delete"
  | "bringFront"
  | "sendBack"
  | "group"
  | "enterBlock"
  | "ungroup"
  | "renameBlock";

const base = layoutJson as unknown as Draft;
const clone = <T,>(value: T): T =>
  JSON.parse(JSON.stringify(value)) as T;

/**
 * Keep the editor render-safe when the API returns null, a wrapped layout,
 * or a partial layout. The warehouse editor must never replace a valid local
 * draft with null/undefined data.
 */
function normalizeDraft(value: unknown): Draft {
  const fallback = clone(base);
  const candidate =
    value && typeof value === "object"
      ? ((value as { layout?: unknown }).layout ?? value)
      : null;

  if (!candidate || typeof candidate !== "object") return fallback;

  const source = candidate as Partial<Draft> & { spawn?: Partial<Draft["spawn"]> };
  const normalized = {
    ...fallback,
    ...source,
    size: { ...fallback.size, ...(source.size ?? {}) },
    grid: { ...fallback.grid, ...(source.grid ?? {}) },
    spawn: {
      ...fallback.spawn,
      ...(source.spawn ?? {}),
      robots: Array.isArray(source.spawn?.robots)
        ? source.spawn!.robots
        : fallback.spawn.robots,
    },
  } as Draft;

  normalized.schema_version = 2;
  normalized.coordinate_system = source.coordinate_system ?? { unit: "meter", frame: "warehouse_map", yaw_unit: "radian" };
  normalized.aisles = Array.isArray(source.aisles) ? source.aisles : [];
  normalized.navigation_tags = Array.isArray(source.navigation_tags)
    ? source.navigation_tags.map((tag) => ({ ...tag, id: tag.id ?? tag.uuid }))
    : [];
  normalized.navigation_edges = Array.isArray(source.navigation_edges) ? source.navigation_edges : [];
  normalized.floors = normalized.floors.map((floor) => ({
    ...floor,
    boundary: floor.boundary ?? floor.footprint ?? [[0, 0], [normalized.size.width, 0], [normalized.size.width, normalized.size.depth], [0, normalized.size.depth]],
    holes: floor.holes ?? [],
  }));

  const arrayKeys: (keyof Draft)[] = [
    "floors", "columns", "lifts", "zones", "docks", "racks",
    "conveyors", "stations", "charging_stations", "parking",
    "restricted_areas", "walkways", "cameras", "sensors",
    "locations", "obstacles",
  ];

  for (const key of arrayKeys) {
    const current = normalized[key];
    if (!Array.isArray(current)) {
      normalized[key] = clone(fallback[key]) as never;
    }
  }

  return normalized;
}

const COLORS: Record<Kind, string> = {
  floor: "#38bdf8",
  aisle: "#22d3ee",
  "navigation-tag": "#a78bfa",
  "navigation-edge": "#c084fc",
  rack: "#64748b",
  wall: "#94a3b8",
  conveyor: "#f59e0b",
  dock: "#22c55e",
  lift: "#38bdf8",
  charger: "#a78bfa",
  station: "#06b6d4",
  zone: "#22c55e",
  parking: "#8b5cf6",
  restricted: "#ef4444",
  walkway: "#0ea5e9",
  camera: "#f97316",
  sensor: "#ec4899",
  location: "#eab308",
  spawn: "#14b8a6",
  column: "#64748b",
};

const LABELS: Record<Kind, string> = {
  floor: "Floor",
  aisle: "Aisle",
  "navigation-tag": "Navigation Tag",
  "navigation-edge": "Navigation Graph",
  rack: "Rack",
  wall: "Wall",
  conveyor: "Conveyor",
  dock: "Dock",
  lift: "Lift",
  charger: "Charger",
  station: "Station",
  zone: "Zone",
  parking: "Parking",
  restricted: "Restricted Area",
  walkway: "Walkway",
  camera: "Camera",
  sensor: "Sensor",
  location: "Location",
  spawn: "Robot Spawn",
  column: "Column",
};

const ICONS: Record<Kind, string> = {
  floor: "⌂",
  aisle: "╱",
  "navigation-tag": "●",
  "navigation-edge": "⇢",
  rack: "▥",
  wall: "▤",
  conveyor: "→",
  dock: "⇥",
  lift: "⇅",
  charger: "⚡",
  station: "⌖",
  zone: "▧",
  parking: "P",
  restricted: "!",
  walkway: "═",
  camera: "◉",
  sensor: "◇",
  location: "⌂",
  spawn: "●",
  column: "■",
};

const GROUPS: { title: string; kinds: Kind[] }[] = [
  {
    title: "Storage",
    kinds: ["rack", "station", "parking", "location"],
  },
  {
    title: "Infrastructure",
    kinds: ["wall", "column", "dock", "lift"],
  },
  {
    title: "Material Flow",
    kinds: ["conveyor", "charger"],
  },
  {
    title: "Navigation",
    kinds: ["floor", "aisle", "navigation-tag", "navigation-edge", "zone", "walkway", "restricted", "spawn"],
  },
  {
    title: "Sensors",
    kinds: ["camera", "sensor"],
  },
];

const KEY_MAP: Partial<Record<Kind, keyof Draft>> = {
  floor: "floors",
  aisle: "aisles",
  "navigation-tag": "navigation_tags",
  "navigation-edge": "navigation_edges",
  rack: "racks",
  zone: "zones",
  station: "stations",
  charger: "charging_stations",
  dock: "docks",
  conveyor: "conveyors",
  lift: "lifts",
  parking: "parking",
  restricted: "restricted_areas",
  walkway: "walkways",
  camera: "cameras",
  sensor: "sensors",
  location: "locations",
  column: "columns",
};

const DEFAULT_LAYER_ORDER: Kind[] = [
  "floor",
  "aisle",
  "navigation-tag",
  "navigation-edge",
  "column",
  "wall",
  "lift",
  "dock",
  "restricted",
  "zone",
  "walkway",
  "parking",
  "rack",
  "station",
  "conveyor",
  "charger",
  "location",
  "camera",
  "sensor",
  "spawn",
];

const LAYER_LABELS: Record<Kind, string> = {
  floor: "Floors",
  aisle: "Aisles",
  "navigation-tag": "Navigation Tags",
  "navigation-edge": "Navigation Graph",
  column: "Structure",
  wall: "Walls",
  lift: "Lifts",
  dock: "Docks",
  restricted: "Restricted",
  zone: "Zones",
  walkway: "Walkways",
  parking: "Parking",
  rack: "Racks",
  station: "Stations",
  conveyor: "Conveyors",
  charger: "Chargers",
  location: "Locations",
  camera: "Cameras",
  sensor: "Sensors",
  spawn: "Robot Spawns",
};

const BLOCKS_STORAGE_KEY = "waretwin.editor.blocks.v1";
const LAYERS_STORAGE_KEY = "waretwin.editor.layers.v1";

const selectionKey = (selection: Selection) => `${selection.kind}:${selection.id}`;


function nextId(arr: any[], prefix: string) {
  const used = new Set(arr.map((x) => String(x.id)));
  let i = 1;
  let id = "";
  do {
    id = `${prefix}-${String(i).padStart(3, "0")}`;
    i += 1;
  } while (used.has(id));
  return id;
}

function getCollection(draft: Draft, kind: Kind): any[] {
  if (kind === "spawn") return draft.spawn.robots as any[];
  if (kind === "floor") return draft.floors as any[];
  if (kind === "navigation-edge") return (draft.navigation_edges ?? []) as any[];
  const key = KEY_MAP[kind];
  return key ? ((draft as any)[key] as any[]) ?? [] : [];
}

function objectId(kind: Kind, obj: any): string {
  return kind === "navigation-tag" || kind === "navigation-edge" ? String(obj.uuid ?? obj.id) : String(obj.id);
}

function getFloorForObject(obj: any): number | string {
  return obj?.floor_id ?? obj?.floor ?? 1;
}

function findCollectionObject(draft: Draft, kind: Kind, id: string): any | undefined {
  return getCollection(draft, kind).find((item) => objectId(kind, item) === String(id));
}

function addObject(d: Draft, kind: Kind): Selection {
  if (kind === "column") {
    const id = `COL-${String((d.columns?.length ?? 0) + 1).padStart(3, "0")}`;
    d.columns ??= [];
    d.columns.push([20, 20]);
    return { kind, id: `${id}-${d.columns.length - 1}` };
  }

  const arr = getCollection(d, kind);
  const id = nextId(arr, kind === "spawn" ? "SPAWN" : kind.toUpperCase());

  switch (kind) {
    case "rack":
      d.racks.push({
        id,
        zone: d.zones[0]?.id ?? "A",
        position: [20, 0, 20],
        size: [3, 6, 1.2],
        rotation: 0,
        levels: 4,
        model: "rack_double",
        blocks_grid: true,
        floor: 1,
      });
      break;
    case "zone":
      d.zones.push({
        id,
        name: id,
        color: "#22c55e",
        polygon: [
          [20, 20],
          [30, 20],
          [30, 30],
          [20, 30],
        ],
        floor: 1,
      });
      break;
    case "station":
      d.stations.push({
        id,
        kind: "PACKING",
        zone: d.zones[0]?.id ?? "A",
        rect: [20, 20, 26, 24],
        access_point: [23, 19],
      });
      break;
    case "charger":
      d.charging_stations.push({
        id,
        zone: d.zones[0]?.id ?? "A",
        position: [20, 0.8, 20],
        heading: 0,
        power_kw: 12,
        access_point: [20, 19],
      });
      break;
    case "dock":
      d.docks.push({
        id,
        kind: "INBOUND",
        zone: d.zones[0]?.id ?? "A",
        rect: [20, 0, 28, 6],
        door: [24, 0],
      });
      break;
    case "conveyor":
      d.conveyors.push({
        id,
        name: id,
        zone: d.zones[0]?.id ?? "A",
        path: [
          [20, 20],
          [32, 20],
        ],
        width: 1.5,
        speed_mps: 0.8,
        direction: "BIDIRECTIONAL",
        blocks_grid: true,
      });
      break;
    case "lift":
      d.lifts.push({
        id,
        cell: [30, 30],
        floors: d.floors.map((f) => f.id),
        ride_ticks: 60,
      });
      break;
    case "parking":
      d.parking.push({
        id,
        zone: d.zones[0]?.id ?? "A",
        rect: [20, 20, 26, 24],
        slots: 4,
      });
      break;
    case "restricted":
      d.restricted_areas.push({
        id,
        name: id,
        rect: [20, 20, 28, 24],
        robots_allowed: false,
      });
      break;
    case "walkway":
      d.walkways.push({
        id,
        polygon: [
          [20, 20],
          [35, 20],
          [35, 23],
          [20, 23],
        ],
        robots_allowed: false,
        speed_limit_mps: 1.0,
      });
      break;
    case "camera":
      d.cameras.push({
        id,
        zone: d.zones[0]?.id ?? "A",
        floor: 1,
        position: [20, 4, 20],
        look_at: [25, 0, 25],
        fov_deg: 80,
        range_m: 12,
      });
      break;
    case "sensor":
      d.sensors.push({
        id,
        kind: "ENVIRONMENT",
        zone: d.zones[0]?.id ?? "A",
        position: [20, 2, 20],
      });
      break;
    case "location":
      d.locations.push({
        id,
        kind: "PICKUP",
        zone: d.zones[0]?.id ?? "A",
        floor: 1,
        rack_id: null,
        level_range: null,
        access_point: [22, 20],
      });
      break;
    case "spawn":
      d.spawn.robots.push({
        id,
        position: [20, 0.2, 20],
        heading: 0,
        battery: 100,
        floor: 1,
      });
      break;
    default:
      break;
  }

  return { kind, id };
}

function removeObject(d: Draft, selection: Selection) {
  if (selection.kind === "floor") return;
  if (selection.kind === "column") {
    const index = Number(selection.id.split("-").pop() ?? "-1");
    if (d.columns && index >= 0) d.columns.splice(index, 1);
    return;
  }

  const key = KEY_MAP[selection.kind];
  if (!key) return;
  const arr = (d as any)[key] as any[];
  if (Array.isArray(arr)) {
    (d as any)[key] = arr.filter((x: any) => objectId(selection.kind, x) !== selection.id);
  }
}

function get2DBox(kind: Kind, obj: any): {
  x: number;
  z: number;
  w: number;
  h: number;
  rotation: number;
} {
  const safePoint = (value: unknown, fallback: [number, number, number] = [0, 0, 0]): [number, number, number] => {
    if (Array.isArray(value)) {
      return [
        Number.isFinite(Number(value[0])) ? Number(value[0]) : fallback[0],
        Number.isFinite(Number(value[1])) ? Number(value[1]) : fallback[1],
        Number.isFinite(Number(value[2])) ? Number(value[2]) : fallback[2],
      ];
    }
    return fallback;
  };

  const safeRect = (value: unknown, fallback: [number, number, number, number] = [0, 0, 1, 1]): [number, number, number, number] => {
    if (Array.isArray(value) && value.length >= 4) {
      const r = value.slice(0, 4).map(Number);
      if (r.every(Number.isFinite)) return [r[0], r[1], r[2], r[3]];
    }
    return fallback;
  };

  if (kind === "navigation-tag") {
    return { x: Number(obj?.x) || 0, z: Number(obj?.y) || 0, w: 0.6, h: 0.6, rotation: Number(obj?.yaw) || 0 };
  }
  if (kind === "navigation-edge") {
    const from = obj?.from_point ?? { x: 0, y: 0 };
    const to = obj?.to_point ?? from;
    const x = Math.min(from.x, to.x), z = Math.min(from.y, to.y);
    return { x, z, w: Math.max(0.6, Math.abs(to.x - from.x)), h: Math.max(0.6, Math.abs(to.y - from.y)), rotation: 0 };
  }
  if (kind === "aisle") {
    const footprint = buildAisleFootprint(obj?.centerline ?? [], Number(obj?.width));
    if (footprint.length) {
      const xs = footprint.map((p) => p.x), ys = footprint.map((p) => p.y);
      return { x: Math.min(...xs), z: Math.min(...ys), w: Math.max(...xs) - Math.min(...xs), h: Math.max(...ys) - Math.min(...ys), rotation: 0 };
    }
    const line = obj?.centerline ?? [];
    const xs = line.map((p: Point) => p.x), ys = line.map((p: Point) => p.y);
    return { x: Math.min(...xs, 0), z: Math.min(...ys, 0), w: Math.max(...xs, 1) - Math.min(...xs, 0), h: Math.max(...ys, 1) - Math.min(...ys, 0), rotation: 0 };
  }
  if (kind === "floor") {
    const points = obj?.boundary ?? obj?.footprint ?? [];
    const xs = points.map((p: Point | [number, number]) => Array.isArray(p) ? p[0] : p.x), ys = points.map((p: Point | [number, number]) => Array.isArray(p) ? p[1] : p.y);
    return { x: Math.min(...xs, 0), z: Math.min(...ys, 0), w: Math.max(...xs, 1) - Math.min(...xs, 0), h: Math.max(...ys, 1) - Math.min(...ys, 0), rotation: 0 };
  }

  if (kind === "column") {
    const p = Array.isArray(obj) ? obj : [0, 0];
    return { x: Number(p[0]) || 0, z: Number(p[1]) || 0, w: 0.9, h: 0.9, rotation: 0 };
  }

  if (kind === "location") {
    const p = Array.isArray(obj?.access_point) ? obj.access_point : [0, 0];
    const x = Number.isFinite(Number(p[0])) ? Number(p[0]) : 0;
    const z = Number.isFinite(Number(p[1])) ? Number(p[1]) : 0;
    return { x: x - 0.45, z: z - 0.45, w: 0.9, h: 0.9, rotation: 0 };
  }

  if (kind === "conveyor") {
    const path = Array.isArray(obj?.path)
      ? obj.path.filter((p: unknown) => Array.isArray(p) && p.length >= 2 && Number.isFinite(Number(p[0])) && Number.isFinite(Number(p[1])))
      : [];
    if (path.length >= 2) {
      const xs = path.map((p: any) => Number(p[0]));
      const zs = path.map((p: any) => Number(p[1]));
      const minX = Math.min(...xs), maxX = Math.max(...xs);
      const minZ = Math.min(...zs), maxZ = Math.max(...zs);
      return {
        x: minX,
        z: minZ,
        w: Math.max(1, maxX - minX),
        h: Math.max(1, maxZ - minZ),
        rotation: 0,
      };
    }
    return { x: 0, z: 0, w: 1, h: 1, rotation: 0 };
  }

  if (kind === "zone" || kind === "walkway") {
    const pts = Array.isArray(obj?.polygon)
      ? obj.polygon.filter((p: unknown) => Array.isArray(p) && p.length >= 2 && Number.isFinite(Number(p[0])) && Number.isFinite(Number(p[1])))
      : [];
    if (pts.length >= 1) {
      const xs = pts.map((p: any) => Number(p[0]));
      const zs = pts.map((p: any) => Number(p[1]));
      return {
        x: Math.min(...xs),
        z: Math.min(...zs),
        w: Math.max(1, Math.max(...xs) - Math.min(...xs)),
        h: Math.max(1, Math.max(...zs) - Math.min(...zs)),
        rotation: 0,
      };
    }
    return { x: 0, z: 0, w: 1, h: 1, rotation: 0 };
  }

  if (kind === "lift") {
    const cell = Array.isArray(obj?.cell) ? obj.cell : [0, 0];
    return {
      x: Number.isFinite(Number(cell[0])) ? Number(cell[0]) : 0,
      z: Number.isFinite(Number(cell[1])) ? Number(cell[1]) : 0,
      w: 3,
      h: 4,
      rotation: 0,
    };
  }

  if (kind === "camera") {
    const p = safePoint(obj?.position);
    return { x: p[0] - 0.5, z: p[2] - 0.5, w: 1, h: 1, rotation: 0 };
  }

  if (["rack", "charger", "spawn", "sensor"].includes(kind)) {
    const p = safePoint(obj?.position);
    const defaultSize = kind === "rack" ? [2, 4, 1] : kind === "spawn" ? [1.4, 1.4, 1.4] : kind === "sensor" ? [0.8, 0.8, 0.8] : [1, 1, 1];
    const rawSize = kind === "rack" ? obj?.size : undefined;
    const sx = Number.isFinite(Number(rawSize?.[0])) ? Number(rawSize[0]) : defaultSize[0];
    const sz = Number.isFinite(Number(rawSize?.[2])) ? Number(rawSize[2]) : defaultSize[2];
    return {
      x: p[0] - sx / 2,
      z: p[2] - sz / 2,
      w: Math.max(0.6, sx),
      h: Math.max(0.6, sz),
      rotation: Number.isFinite(Number(obj?.rotation)) ? Number(obj.rotation) : Number(obj?.heading) || 0,
    };
  }

  const r = safeRect(obj?.rect);
  return {
    x: r[0],
    z: r[1],
    w: Math.max(0.5, r[2] - r[0]),
    h: Math.max(0.5, r[3] - r[1]),
    rotation: Number.isFinite(Number(obj?.rotation)) ? Number(obj.rotation) : 0,
  };
}

function moveObject(d: Draft, selection: Selection, centerX: number, centerZ: number, snapStep: number | null = null) {
  const sx = (v: number) => snapPoint({ x: v, y: 0 }, snapStep).x;
  const obj = selection.kind === "column" ? undefined : findCollectionObject(d, selection.kind, selection.id);

  if (selection.kind === "column") {
    const index = Number(selection.id.split("-").pop() ?? "-1");
    if (d.columns?.[index]) {
      d.columns[index] = [sx(centerX), sx(centerZ)];
    }
    return;
  }
  if (!obj) return;
  if (selection.kind === "navigation-edge") return;

  if (selection.kind === "navigation-tag") { obj.x = sx(centerX); obj.y = sx(centerZ); if (obj.placement === "auto") { obj.placement = "manual"; obj.locked = true; } return; }
  if (selection.kind === "aisle") { const box = get2DBox("aisle", obj); const dx = sx(centerX) - (box.x + box.w / 2), dz = sx(centerZ) - (box.z + box.h / 2); obj.centerline = obj.centerline.map((p: Point) => ({ x: sx(p.x + dx), y: sx(p.y + dz) })); return; }

  if (["rack", "charger", "spawn", "sensor", "camera"].includes(selection.kind)) {
    obj.position = [sx(centerX), obj.position[1], sx(centerZ)];
    if (selection.kind === "camera") {
      obj.look_at = [sx(centerX), obj.look_at?.[1] ?? 0, sx(centerZ + 5)];
    }
  } else if (selection.kind === "lift") {
    obj.cell = [Math.round(centerX), Math.round(centerZ)];
  } else if (selection.kind === "location") {
    const p = Array.isArray(obj.access_point) ? obj.access_point : [0, 0];
    obj.access_point = [sx(centerX), sx(centerZ)];
    void p;
  } else if (selection.kind === "zone" || selection.kind === "walkway") {
    const box = get2DBox(selection.kind, obj);
    const cx = box.x + box.w / 2;
    const cz = box.z + box.h / 2;
    const dx = sx(centerX - cx);
    const dz = sx(centerZ - cz);
    obj.polygon = obj.polygon.map((p: number[]) => [
      sx(p[0] + dx),
      sx(p[1] + dz),
    ]);
  } else {
    const r = obj.rect as number[];
    const w = r[2] - r[0];
    const h = r[3] - r[1];
    const x0 = sx(centerX - w / 2);
    const z0 = sx(centerZ - h / 2);
    obj.rect = [x0, z0, sx(x0 + w), sx(z0 + h)];

    if (Array.isArray(obj.access_point)) {
      const oldCenter = [(r[0] + r[2]) / 2, (r[1] + r[3]) / 2];
      obj.access_point = [
        sx(obj.access_point[0] + (centerX - oldCenter[0])),
        sx(obj.access_point[1] + (centerZ - oldCenter[1])),
      ];
    }
    if (Array.isArray(obj.door)) {
      const oldCenter = [(r[0] + r[2]) / 2, (r[1] + r[3]) / 2];
      obj.door = [
        sx(obj.door[0] + (centerX - oldCenter[0])),
        sx(obj.door[1] + (centerZ - oldCenter[1])),
      ];
    }
  }
}

function rotateObject(d: Draft, selection: Selection, delta: number) {
  const obj = selection.kind === "column" ? undefined : findCollectionObject(d, selection.kind, selection.id);
  if (!obj) return;

  if (selection.kind === "rack") obj.rotation = ((obj.rotation ?? 0) + delta + 360) % 360;
  else if (selection.kind === "charger") obj.heading = ((obj.heading ?? 0) + delta + 360) % 360;
  else if (selection.kind === "spawn") obj.heading = ((obj.heading ?? 0) + delta + 360) % 360;
  else if (selection.kind === "camera") {
    const look = obj.look_at ?? obj.position;
    const angle = Math.atan2(look[2] - obj.position[2], look[0] - obj.position[0]);
    const next = angle + (delta * Math.PI) / 180;
    const distance = Math.hypot(look[0] - obj.position[0], look[2] - obj.position[2]) || 5;
    obj.look_at = [
      obj.position[0] + Math.cos(next) * distance,
      look[1] ?? 0,
      obj.position[2] + Math.sin(next) * distance,
    ];
  }
}

function setObjectProperty(obj: any, path: string, raw: string) {
  const parts = path.split(".");
  let cursor = obj;
  for (let i = 0; i < parts.length - 1; i += 1) {
    cursor = cursor[parts[i]];
    if (!cursor) return;
  }
  const key = parts[parts.length - 1];
  const parsed = Number(raw);
  cursor[key] = raw === "true" ? true : raw === "false" ? false : raw.trim() !== "" && Number.isFinite(parsed) ? parsed : raw;
}

function objectGeometry(kind: Kind, obj: any): {
  pos: Vec3;
  size: Vec3;
  rotation: number;
} {
  if (kind === "rack") {
    return { pos: obj.position, size: obj.size, rotation: obj.rotation ?? 0 };
  }
  if (kind === "charger") {
    return { pos: obj.position, size: [1, 1.8, 1], rotation: obj.heading ?? 0 };
  }
  if (kind === "spawn") {
    return { pos: obj.position, size: [1.6, 0.8, 1.6], rotation: obj.heading ?? 0 };
  }
  if (kind === "sensor" || kind === "camera") {
    return { pos: obj.position, size: [0.8, 1.0, 0.8], rotation: 0 };
  }
  if (kind === "lift") {
    return {
      pos: [obj.cell[0] + 0.5, 4, obj.cell[1] + 0.5],
      size: [2.8, 8, 3.6],
      rotation: 0,
    };
  }
  if (kind === "zone" || kind === "walkway") {
    const box = get2DBox(kind, obj);
    return {
      pos: [box.x + box.w / 2, 0.05, box.z + box.h / 2],
      size: [box.w, 0.1, box.h],
      rotation: 0,
    };
  }
  const r = get2DBox(kind, obj);
  const height =
    kind === "dock" ? 4.8 :
    kind === "wall" ? 4 :
    kind === "parking" ? 0.4 :
    kind === "restricted" ? 0.15 :
    kind === "location" ? 0.3 :
    kind === "column" ? 4 :
    1.6;
  return {
    pos: [r.x + r.w / 2, height / 2, r.z + r.h / 2],
    size: [r.w, height, r.h],
    rotation: 0,
  };
}

function selectionBounds(draft: Draft, selections: Selection[]) {
  const boxes = selections
    .map((selection) => {
      const obj =
        selection.kind === "column"
          ? null
          : findCollectionObject(draft, selection.kind, selection.id);
      if (!obj && selection.kind !== "column") return null;
      if (selection.kind === "column") {
        const index = Number(selection.id.split("-").pop() ?? "-1");
        const point = draft.columns?.[index];
        return point
          ? { x: point[0] - 0.4, z: point[1] - 0.4, w: 0.8, h: 0.8 }
          : null;
      }
      const b = get2DBox(selection.kind, obj);
      return { x: b.x, z: b.z, w: b.w, h: b.h };
    })
    .filter(Boolean) as { x: number; z: number; w: number; h: number }[];

  if (!boxes.length) return null;
  const minX = Math.min(...boxes.map((b) => b.x));
  const minZ = Math.min(...boxes.map((b) => b.z));
  const maxX = Math.max(...boxes.map((b) => b.x + b.w));
  const maxZ = Math.max(...boxes.map((b) => b.z + b.h));
  return {
    x: minX,
    z: minZ,
    w: maxX - minX,
    h: maxZ - minZ,
    centerX: (minX + maxX) / 2,
    centerZ: (minZ + maxZ) / 2,
  };
}

type Editor3DObjectProps = {
  kind: Kind;
  obj: any;
  lockedLayers?: Record<Kind, boolean>;
  selected: boolean;
  onSelect: (additive: boolean) => void;
  onDragState: (value: boolean) => void;
  onCommit: (point: Vec3) => void;
  onHover?: (active: boolean) => void;
  snapStep: number | null;
};

function Editor3DObject({
  kind,
  obj,
  selected,
  onSelect,
  lockedLayers,
  onDragState,
  onCommit,
  onHover,
  snapStep,
}: Editor3DObjectProps) {
  const { gl } = useThree();
  const { pos, size, rotation } = objectGeometry(kind, obj);
  const [displayPos, setDisplayPos] = useState<Vec3>(pos);
  const [hovered, setHovered] = useState(false);
  const dragging = useRef(false);
  const pointerId = useRef<number | null>(null);
  const startClient = useRef<[number, number]>([0, 0]);
  const offset = useRef<[number, number]>([0, 0]);
  const target = useRef(new THREE.Vector3());
  const plane = useRef(new THREE.Plane(new THREE.Vector3(0, 1, 0), 0));

  useEffect(() => {
    if (!dragging.current) setDisplayPos(pos);
  }, [pos[0], pos[1], pos[2]]);

  const getPoint = (event: ThreeEvent<PointerEvent>) => {
    if (!event.ray.intersectPlane(plane.current, target.current)) return null;
    return [
      target.current.x + offset.current[0],
      0,
      target.current.z + offset.current[1],
    ] as Vec3;
  };

  const onDown = (event: ThreeEvent<PointerEvent>) => {
    if (event.button !== 0) return;
    if (lockedLayers?.[kind]) return;
    event.stopPropagation();
    onSelect(event.shiftKey || event.ctrlKey || event.metaKey);
    pointerId.current = event.pointerId;
    startClient.current = [event.clientX, event.clientY];
    offset.current = [pos[0] - event.point.x, pos[2] - event.point.z];
    gl.domElement.setPointerCapture(event.pointerId);
  };

  const onMove = (event: ThreeEvent<PointerEvent>) => {
    if (pointerId.current !== event.pointerId) return;
    const dx = event.clientX - startClient.current[0];
    const dy = event.clientY - startClient.current[1];
    if (!dragging.current && Math.hypot(dx, dy) < 4) return;
    event.stopPropagation();
    if (!dragging.current) {
      dragging.current = true;
      onDragState(true);
      gl.domElement.style.cursor = "grabbing";
    }
    const point = getPoint(event);
    if (point) {
      const snapped = snapPoint({ x: point[0], y: point[2] }, snapStep);
      const sx = snapped.x;
      const sz = snapped.y;
      setDisplayPos([sx, pos[1], sz]);
    }
  };

  const onUp = (event: ThreeEvent<PointerEvent>) => {
    if (pointerId.current !== event.pointerId) return;
    event.stopPropagation();
    const point = getPoint(event);
    if (dragging.current && point) onCommit(point);
    dragging.current = false;
    pointerId.current = null;
    onDragState(false);
    gl.domElement.style.cursor = "grab";
    if (gl.domElement.hasPointerCapture(event.pointerId)) {
      gl.domElement.releasePointerCapture(event.pointerId);
    }
  };

  const materialColor = selected ? "#ffffff" : hovered ? "#cbd5e1" : COLORS[kind];
  const opacity =
    kind === "zone" ? 0.18 :
    kind === "walkway" ? 0.12 :
    kind === "restricted" ? 0.25 :
    0.9;

  return (
    <mesh
      position={displayPos}
      rotation-y={(rotation * Math.PI) / 180}
      castShadow
      receiveShadow
      onPointerDown={onDown}
      onPointerMove={onMove}
      onPointerUp={onUp}
      onPointerCancel={onUp}
      onPointerOver={() => { setHovered(true); onHover?.(true); gl.domElement.style.cursor = selected ? "move" : "pointer"; }}
      onPointerOut={() => { setHovered(false); onHover?.(false); if (!dragging.current) gl.domElement.style.cursor = "grab"; }}
    >
      <boxGeometry args={size} />
      <meshStandardMaterial
        color={materialColor}
        transparent
        opacity={opacity}
        emissive={selected ? COLORS[kind] : hovered ? COLORS[kind] : "#000000"}
        emissiveIntensity={selected ? 0.4 : hovered ? 0.16 : 0}
      />
    </mesh>
  );
}

function EditorScene3D({
  draft,
  selected,
  onSelect,
  onCommit,
  onDragState,
  floor,
  visibleLayers,
  lockedLayers,
  layerOrder,
  snapStep,
}: {
  draft: Draft;
  selected: Selection[];
  onSelect: (selection: Selection, additive: boolean) => void;
  onCommit: (point: Vec3) => void;
  onDragState: (active: boolean) => void;
  floor: number | string | "all";
  visibleLayers: Record<Kind, boolean>;
  lockedLayers: Record<Kind, boolean>;
  layerOrder: Kind[];
  snapStep: number | null;
}) {
  const controlsRef = useRef<OrbitControlsImpl | null>(null);

  const items = useMemo(() => {
    const entries: [Kind, any[]][] = [];
    layerOrder.forEach((kind) => {
      if (kind === "column" || kind === "spawn" || kind === "floor" || kind === "navigation-edge") return;
      const arr = getCollection(draft, kind);
      if (arr?.length) entries.push([kind, arr]);
    });
    if (layerOrder.includes("spawn") && draft.spawn?.robots?.length) {
      entries.push(["spawn", draft.spawn.robots]);
    }
    return entries;
  }, [draft, layerOrder]);

  return (
    <Canvas
      camera={{
        position: [draft.size.width / 2, 55, draft.size.depth + 35],
        fov: 42,
      }}
      shadows
      style={{ cursor: "grab" }}
      onPointerMissed={() => onSelect({ kind: "zone", id: "__clear__" }, false)}
    >
      <ambientLight intensity={0.55} />
      <directionalLight position={[40, 60, 20]} intensity={1.4} castShadow />

      {draft.floors.filter((item) => floor === "all" || String(item.id) === String(floor)).map((item) => {
        const boundary = (item.boundary ?? item.footprint ?? [[0, 0], [draft.size.width, 0], [draft.size.width, draft.size.depth], [0, draft.size.depth]]) as Array<[number, number] | { x: number; y: number }>;
        const xy = (p: [number, number] | { x: number; y: number }) => Array.isArray(p) ? p : [p.x, p.y];
        const shape = new THREE.Shape(boundary.map((p) => { const [x, y] = xy(p); return new THREE.Vector2(x, -y); }));
        for (const hole of item.holes ?? []) {
          const holeShape = new THREE.Path((hole as Array<[number, number] | { x: number; y: number }>).map((p) => { const [x, y] = xy(p); return new THREE.Vector2(x, -y); }));
          shape.holes.push(holeShape);
        }
        return <mesh key={`floor-${item.id}`} geometry={new THREE.ShapeGeometry(shape)} rotation-x={-Math.PI / 2} position={[0, item.elevation - 0.05, 0]} onPointerDown={(e) => e.stopPropagation()}><meshStandardMaterial color="#0b1220" /></mesh>;
      })}

      <Grid
        args={[draft.size.width, draft.size.depth]}
        position={[draft.size.width / 2, 0, draft.size.depth / 2]}
        cellSize={draft.grid.cell_size}
        sectionSize={5}
        fadeDistance={150}
        infiniteGrid={false}
      />

      {items.flatMap(([kind, arr]) =>
        visibleLayers[kind]
          ? arr
              .filter((o: any) => {
                if (floor === "all") return true;
                if (kind === "floor") return String(o.id) === String(floor);
                if ("floor" in o || "floor_id" in o) return String(getFloorForObject(o)) === String(floor);
                return true;
              })
              .map((obj: any) => (
                <Editor3DObject
                  key={`${kind}-${objectId(kind, obj)}`}
                  kind={kind}
                  obj={obj}
                  selected={selected.some(
                    (s) => s.kind === kind && s.id === objectId(kind, obj),
                  )}
                  onSelect={(additive) =>
                    onSelect({ kind, id: objectId(kind, obj) }, additive)
                  }
                  onDragState={onDragState}
                  onCommit={onCommit}
                  lockedLayers={lockedLayers}
                  snapStep={snapStep}
                />
              ))
          : [],
      )}

      <OrbitControls
        ref={controlsRef}
        makeDefault
        enableDamping
        dampingFactor={0.08}
        rotateSpeed={0.5}
        zoomSpeed={0.9}
        panSpeed={0.7}
        maxPolarAngle={Math.PI / 2.12}
        minDistance={6}
        maxDistance={180}
        mouseButtons={{ LEFT: -1 as any, MIDDLE: 2, RIGHT: 0 }}
        enabled
      />
    </Canvas>
  );
}

function polygonPoints(points: number[][], xScale: number, zScale: number) {
  return points
    .map(([x, z]) => `${x * xScale},${z * zScale}`)
    .join(" ");
}

function MapObject({
  kind,
  obj,
  selected,
  scale,
  onPointerDown,
  onContextMenu,
}: {
  kind: Kind;
  obj: any;
  selected: boolean;
  scale: { x: number; z: number };
  onPointerDown: (event: React.PointerEvent<SVGElement>) => void;
  onContextMenu?: (event: React.MouseEvent<SVGElement>) => void;
}) {
  const [hovered, setHovered] = useState(false);
  const box = get2DBox(kind, obj);
  const x = box.x * scale.x;
  const y = box.z * scale.z;
  const w = box.w * scale.x;
  const h = box.h * scale.z;
  const color = COLORS[kind];
  const stroke = selected ? "#ffffff" : hovered ? "#bae6fd" : color;
  const strokeWidth = selected ? 2 : hovered ? 1.75 : 1;
  const tooltip = `${obj?.id ?? "Object"} · ${LABELS[kind]} · Floor ${obj?.floor ?? 1}`;

  if (kind === "aisle") {
    const footprint = buildAisleFootprint(obj.centerline ?? [], Number(obj.width));
    const footprintPoints = footprint.map((p) => `${p.x * scale.x},${p.y * scale.z}`).join(" ");
    const centerlinePoints = (obj.centerline ?? []).map((p: Point) => `${p.x * scale.x},${p.y * scale.z}`).join(" ");
    return (
      <g onPointerDown={onPointerDown} onContextMenu={onContextMenu} onPointerEnter={() => setHovered(true)} onPointerLeave={() => setHovered(false)} className="cursor-pointer">
        <title>{tooltip}</title>
        <polygon points={footprintPoints} fill={color} fillOpacity={selected ? 0.36 : 0.22} stroke={stroke} strokeWidth={selected ? 2 : 1} vectorEffect="non-scaling-stroke" />
        <polyline points={centerlinePoints} fill="none" stroke="#cffafe" strokeWidth="1" strokeDasharray="4 3" vectorEffect="non-scaling-stroke" pointerEvents="none" />
      </g>
    );
  }

  if (kind === "navigation-tag") {
    return <g onPointerDown={onPointerDown} onContextMenu={onContextMenu} className="cursor-pointer"><title>{tooltip}</title><circle cx={obj.x * scale.x} cy={obj.y * scale.z} r={selected ? 7 : 5} fill={obj.locked ? "#f59e0b" : obj.placement === "manual" ? "#f472b6" : color} stroke={stroke} strokeWidth={selected ? 2 : 1} vectorEffect="non-scaling-stroke" /><text x={obj.x * scale.x + 8} y={obj.y * scale.z - 6} fill="#e2e8f0" fontSize="10" vectorEffect="non-scaling-stroke" pointerEvents="none">{obj.tag_id}</text></g>;
  }

  if (kind === "navigation-edge") {
    const from = obj.from_point ?? { x: 0, y: 0 };
    const to = obj.to_point ?? from;
    const marker = obj.direction === "reverse" ? "url(#nav-edge-arrow-reverse)" : "url(#nav-edge-arrow)";
    return (
      <g onPointerDown={onPointerDown} onContextMenu={onContextMenu} className="cursor-pointer">
        <title>{`${obj.from_tag_id ?? obj.from_tag_uuid} → ${obj.to_tag_id ?? obj.to_tag_uuid} · ${obj.distance?.toFixed?.(2) ?? obj.distance} m`}</title>
        <line x1={from.x * scale.x} y1={from.y * scale.z} x2={to.x * scale.x} y2={to.y * scale.z} stroke={stroke} strokeWidth={selected ? 3 : 2} strokeDasharray={obj.enabled === false ? "5 4" : undefined} markerStart={obj.direction === "bidirectional" ? "url(#nav-edge-arrow-both)" : undefined} markerEnd={marker} vectorEffect="non-scaling-stroke" />
      </g>
    );
  }

  if (kind === "zone" || kind === "walkway") {
    return (
      <polygon
        points={polygonPoints(obj.polygon, scale.x, scale.z)}
        onPointerEnter={() => setHovered(true)}
        onPointerLeave={() => setHovered(false)}
        fill={color}
        fillOpacity={kind === "zone" ? 0.14 : 0.08}
        stroke={stroke}
        strokeWidth={strokeWidth}
        strokeDasharray={kind === "walkway" ? "5 4" : undefined}
        vectorEffect="non-scaling-stroke"
        onPointerDown={onPointerDown}
        onContextMenu={onContextMenu}
      >
        <title>{tooltip}</title>
      </polygon>
    );
  }

  if (kind === "conveyor") {
    const path = obj.path as number[][];
    const points = path.map(([px, pz]) => `${px * scale.x},${pz * scale.z}`).join(" ");
    return (
      <polyline
        points={points}
        onPointerEnter={() => setHovered(true)}
        onPointerLeave={() => setHovered(false)}
        fill="none"
        stroke={stroke}
        strokeWidth={hovered || selected ? Math.max(6, obj.width * scale.x + 2) : Math.max(5, obj.width * scale.x)}
        vectorEffect="non-scaling-stroke"
        strokeLinecap="round"
        onPointerDown={onPointerDown}
        onContextMenu={onContextMenu}
      >
        <title>{tooltip}</title>
      </polyline>
    );
  }

  if (kind === "camera") {
    return (
      <g
        onPointerDown={onPointerDown}
        onContextMenu={onContextMenu}
        onPointerEnter={() => setHovered(true)}
        onPointerLeave={() => setHovered(false)}
        className="cursor-pointer"
      >
        <title>{tooltip}</title>
        <circle
          cx={(obj.position[0]) * scale.x}
          cy={(obj.position[2]) * scale.z}
          r={Math.max(5, 0.6 * Math.min(scale.x, scale.z))}
          fill={color}
          fillOpacity={0.85}
          stroke={stroke}
          strokeWidth={strokeWidth}
        />
        <line
          x1={obj.position[0] * scale.x}
          y1={obj.position[2] * scale.z}
          x2={(obj.look_at?.[0] ?? obj.position[0]) * scale.x}
          y2={(obj.look_at?.[2] ?? obj.position[2] + 4) * scale.z}
          stroke={color}
          strokeWidth={2}
          vectorEffect="non-scaling-stroke"
        />
      </g>
    );
  }

  if (["sensor", "spawn", "location"].includes(kind)) {
    return (
      <g
        onPointerDown={onPointerDown}
        onContextMenu={onContextMenu}
        onPointerEnter={() => setHovered(true)}
        onPointerLeave={() => setHovered(false)}
        className="cursor-pointer"
        transform={`translate(${(box.x + box.w / 2) * scale.x} ${(box.z + box.h / 2) * scale.z}) rotate(${box.rotation})`}
      >
        <title>{tooltip}</title>
        <rect
          x={-w / 2}
          y={-h / 2}
          width={w}
          height={h}
          rx={kind === "spawn" ? 999 : 4}
          fill={color}
          fillOpacity={0.85}
          stroke={stroke}
          strokeWidth={selected ? 2 : 1}
          vectorEffect="non-scaling-stroke"
        />
        <text
          x={0}
          y={4}
          textAnchor="middle"
          fill="#020617"
          fontSize={Math.max(8, Math.min(12, w * 0.4))}
          fontWeight={700}
          pointerEvents="none"
        >
          {ICONS[kind]}
        </text>
      </g>
    );
  }

  return (
    <rect
      x={x}
      y={y}
      width={w}
      height={h}
      rx={kind === "rack" ? 2 : 4}
      fill={color}
      fillOpacity={
        kind === "restricted" ? 0.20 :
        kind === "parking" ? 0.08 :
        0.75
      }
      stroke={stroke}
      strokeWidth={strokeWidth}
      strokeDasharray={kind === "restricted" ? "5 4" : undefined}
      vectorEffect="non-scaling-stroke"
      transform={`rotate(${box.rotation} ${x + w / 2} ${y + h / 2})`}
      onPointerDown={onPointerDown}
      onContextMenu={onContextMenu}
      onPointerEnter={() => setHovered(true)}
      onPointerLeave={() => setHovered(false)}
      className="cursor-pointer"
    >
      <title>{tooltip}</title>
    </rect>
  );
}



function setObjectRotation(draft: Draft, selection: Selection, angle: number) {
  const arr = selection.kind === "column" ? null : getCollection(draft, selection.kind);
  const obj = arr?.find((item) => objectId(selection.kind, item) === selection.id);
  if (!obj) return;

  const normalized = ((Math.round(angle / 15) * 15) % 360 + 360) % 360;

  if (selection.kind === "rack") obj.rotation = normalized;
  else if (selection.kind === "charger" || selection.kind === "spawn") obj.heading = normalized;
}


function resizeObject(
  draft: Draft,
  selection: Selection,
  handle: "nw" | "ne" | "sw" | "se",
  x: number,
  z: number,
) {
  if (selection.kind === "rack") {
    const arr = getCollection(draft, "rack");
    const obj = arr.find((item) => item.id === selection.id);
    if (!obj) return;

    const box = get2DBox("rack", obj);
    const right = box.x + box.w;
    const bottom = box.z + box.h;

    let left = box.x;
    let top = box.z;
    let nextRight = right;
    let nextBottom = bottom;

    if (handle.includes("w")) left = Math.min(x, right - 0.5);
    if (handle.includes("e")) nextRight = Math.max(x, left + 0.5);
    if (handle.includes("n")) top = Math.min(z, bottom - 0.5);
    if (handle.includes("s")) nextBottom = Math.max(z, top + 0.5);

    const width = nextRight - left;
    const depth = nextBottom - top;

    obj.position[0] = Math.round(((left + nextRight) / 2) * 2) / 2;
    obj.position[2] = Math.round(((top + nextBottom) / 2) * 2) / 2;
    obj.size[0] = Math.round(width * 10) / 10;
    obj.size[2] = Math.round(depth * 10) / 10;
    return;
  }

  if (["wall", "station", "dock", "parking", "restricted"].includes(selection.kind)) {
    const obj = getCollection(draft, selection.kind).find(
      (item) => objectId(selection.kind, item) === selection.id,
    );
    if (!obj || !Array.isArray(obj.rect)) return;

    const r = obj.rect as number[];
    const right = r[2];
    const bottom = r[3];

    let left = r[0];
    let top = r[1];
    let nextRight = right;
    let nextBottom = bottom;

    if (handle.includes("w")) left = Math.min(x, right - 0.5);
    if (handle.includes("e")) nextRight = Math.max(x, left + 0.5);
    if (handle.includes("n")) top = Math.min(z, bottom - 0.5);
    if (handle.includes("s")) nextBottom = Math.max(z, top + 0.5);

    obj.rect = [
      Math.round(left * 2) / 2,
      Math.round(top * 2) / 2,
      Math.round(nextRight * 2) / 2,
      Math.round(nextBottom * 2) / 2,
    ];
  }
}

function Editor2DMap({
  draft,
  selected,
  floor,
  viewRotation,
  visibleLayers,
  lockedLayers,
  layerOrder,
  blocks,
  activeBlockId,
  background,
  backgroundOpacity,
  backgroundScale,
  backgroundOffsetX,
  backgroundOffsetY,
  backgroundRotation,
  mode,
  measurePoints,
  drawingPoints,
  onSelect,
  onMove,
  onCommitMove,
  onResize,
  onRotate,
  onMeasurePoint,
  onDrawPoint,
  onFinishDrawing,
  onAddManualTag,
  onEditVertex,
  onEditAisleVertex,
  onInsertVertex,
  onDeleteVertex,
  snapStep,
  onBoxSelect,
  onContextMenu,
  smartGuidesEnabled,
}: {
  draft: Draft;
  selected: Selection[];
  floor: number | string | "all";
  viewRotation: number;
  visibleLayers: Record<Kind, boolean>;
  lockedLayers: Record<Kind, boolean>;
  layerOrder: Kind[];
  blocks: EditorBlock[];
  activeBlockId: string | null;
  background: string | null;
  backgroundOpacity: number;
  backgroundScale: number;
  backgroundOffsetX: number;
  backgroundOffsetY: number;
  backgroundRotation: number;
  mode: Mode;
  measurePoints: { x: number; z: number }[];
  drawingPoints: Point[];
  onSelect: (selection: Selection, additive: boolean) => void;
  onMove: (selection: Selection, x: number, z: number) => void;
  onCommitMove: () => void;
  onResize: (
    selection: Selection,
    handle: "nw" | "ne" | "sw" | "se",
    x: number,
    z: number,
  ) => void;
  onRotate: (selection: Selection, angle: number) => void;
  onMeasurePoint: (point: { x: number; z: number }) => void;
  onDrawPoint: (point: Point) => void;
  onFinishDrawing: () => void;
  onAddManualTag: (point: Point) => void;
  onEditVertex: (floorId: number | string, holeIndex: number | null, vertexIndex: number, point: Point) => void;
  onEditAisleVertex: (aisleId: string, vertexIndex: number, point: Point) => void;
  onInsertVertex: (kind: "floor" | "aisle", id: string, holeIndex: number | null, vertexIndex: number, point: Point) => void;
  onDeleteVertex: (kind: "floor" | "aisle", id: string, holeIndex: number | null, vertexIndex: number) => void;
  snapStep: number | null;
  onBoxSelect: (bounds: { x: number; z: number; w: number; h: number }, additive: boolean) => void;
  onContextMenu?: (selection: Selection | null, x: number, y: number) => void;
  smartGuidesEnabled: boolean;
}) {
  const hostRef = useRef<SVGSVGElement>(null);
  const [drag, setDrag] = useState<{
    selection: Selection;
    start: { x: number; z: number };
    offsetX: number;
    offsetZ: number;
  } | null>(null);
  const [selectionBox, setSelectionBox] = useState<{
    start: { x: number; z: number };
    current: { x: number; z: number };
    additive: boolean;
  } | null>(null);
  const [resizeDrag, setResizeDrag] = useState<{
    selection: Selection;
    handle: "nw" | "ne" | "sw" | "se";
  } | null>(null);
  const [rotateDrag, setRotateDrag] = useState<Selection | null>(null);
  const [zoom, setZoom] = useState(1);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [panDrag, setPanDrag] = useState<{ x: number; y: number; panX: number; panY: number } | null>(null);
  const [smartGuides, setSmartGuides] = useState<{ x?: number; z?: number }>({});
  const [vertexDrag, setVertexDrag] = useState<{ kind: "floor" | "aisle"; id: string; holeIndex: number | null; index: number } | null>(null);
  const [activeVertex, setActiveVertex] = useState<typeof vertexDrag>(null);
  const panMargin = 180;
  const cycleRef = useRef<{ key: string; candidates: Selection[]; index: number } | null>(null);

  useEffect(() => {
    const onKey = (event: KeyboardEvent) => {
      const target = event.target as HTMLElement | null;
      if ((event.key === "Delete" || event.key === "Backspace") && activeVertex && !target?.matches("input,textarea,select")) {
        event.preventDefault(); onDeleteVertex(activeVertex.kind, activeVertex.id, activeVertex.holeIndex, activeVertex.index); setActiveVertex(null);
      }
    };
    window.addEventListener("keydown", onKey); return () => window.removeEventListener("keydown", onKey);
  }, [activeVertex, onDeleteVertex]);

  const clampPan = (x: number, y: number, currentZoom = zoom) => {
    const visibleW = viewW / currentZoom;
    const visibleH = viewH / currentZoom;
    return {
      x: Math.max(-panMargin, Math.min(viewW - visibleW + panMargin, x)),
      y: Math.max(-panMargin, Math.min(viewH - visibleH + panMargin, y)),
    };
  };

  const scale = {
    x: 1000 / draft.size.width,
    z: 650 / draft.size.depth,
  };
  const viewW = draft.size.width * scale.x;
  const viewH = draft.size.depth * scale.z;

  const getPoint = (event: React.PointerEvent<SVGSVGElement | SVGElement> | React.MouseEvent<SVGSVGElement>) => {
    const rect = hostRef.current?.getBoundingClientRect();
    if (!rect) return null;
    const visibleW = viewW / zoom;
    const visibleH = viewH / zoom;
    const xPx = event.clientX - rect.left;
    const zPx = event.clientY - rect.top;
    return {
      x: Math.max(0, Math.min(draft.size.width, ((pan.x + (xPx / rect.width) * visibleW) / scale.x))),
      z: Math.max(0, Math.min(draft.size.depth, ((pan.y + (zPx / rect.height) * visibleH) / scale.z))),
    };
  };

  const getSvgPx = (event: React.WheelEvent<SVGSVGElement>) => {
    const rect = hostRef.current?.getBoundingClientRect();
    if (!rect) return null;
    return { x: event.clientX - rect.left, y: event.clientY - rect.top, rect };
  };

  const items = useMemo(() => {
    const entries: [Kind, any[]][] = [];
    layerOrder.forEach((kind) => {
      if (kind === "column" || kind === "spawn" || kind === "floor") return;
      const arr = kind === "navigation-edge"
        ? getCollection(draft, kind).map((edge: any) => ({
            ...edge,
            from_point: draft.navigation_tags?.find((tag) => tag.uuid === edge.from_tag_uuid) ?? { x: 0, y: 0 },
            to_point: draft.navigation_tags?.find((tag) => tag.uuid === edge.to_tag_uuid) ?? { x: 0, y: 0 },
          }))
        : getCollection(draft, kind);
      if (arr?.length) entries.push([kind, arr]);
    });
    if (layerOrder.includes("spawn") && draft.spawn?.robots?.length) {
      entries.push(["spawn", draft.spawn.robots]);
    }
    return entries;
  }, [draft, layerOrder]);

  const beginItem = (
    event: React.PointerEvent<SVGElement>,
    kind: Kind,
    obj: any,
  ) => {
    if (event.button !== 0) return;
    if (lockedLayers[kind]) {
      event.preventDefault();
      event.stopPropagation();
      return;
    }
    event.stopPropagation();
    const point = getPoint(event);
    if (!point) return;

    const additive = event.shiftKey || event.ctrlKey || event.metaKey;
    if (mode === "measure") {
      onMeasurePoint(point);
      return;
    }
    if (mode === "add-manual-tag") {
      onAddManualTag(snapPoint({ x: point.x, y: point.z }, snapStep));
      return;
    }
    if (mode === "draw-floor" || mode === "add-hole" || mode === "draw-aisle") {
      onDrawPoint(snapPoint({ x: point.x, y: point.z }, snapStep));
      return;
    }

    onSelect({ kind, id: objectId(kind, obj) }, additive);
    const box = get2DBox(kind, obj);
    setDrag({
      selection: { kind, id: objectId(kind, obj) },
      start: point,
      offsetX: box.x + box.w / 2 - point.x,
      offsetZ: box.z + box.h / 2 - point.z,
    });
    hostRef.current?.setPointerCapture(event.pointerId);
  };

  const downCanvas = (event: React.PointerEvent<SVGSVGElement>) => {
    const point = getPoint(event);
    if (!point) return;

    if (event.button === 0 && event.altKey) {
      event.preventDefault();
      event.stopPropagation();

      const pointCandidates: Selection[] = [];
      const seen = new Set<string>();

      [...layerOrder].reverse().forEach((kind) => {
        if (!visibleLayers[kind] || lockedLayers[kind]) return;
        getCollection(draft, kind).forEach((obj: any) => {
          const box = get2DBox(kind, obj);
          if (
            point.x >= box.x &&
            point.x <= box.x + box.w &&
            point.z >= box.z &&
            point.z <= box.z + box.h
          ) {
            const candidate = { kind, id: objectId(kind, obj) };
            const key = selectionKey(candidate);
            if (!seen.has(key)) {
              seen.add(key);
              pointCandidates.push(candidate);
            }
          }
        });
      });

      if (visibleLayers.column && !lockedLayers.column) {
        draft.columns?.forEach(([x, z], index) => {
          if (
            point.x >= x - 0.45 &&
            point.x <= x + 0.45 &&
            point.z >= z - 0.45 &&
            point.z <= z + 0.45
          ) {
            pointCandidates.push({ kind: "column", id: `COL-${index}` });
          }
        });
      }

      if (pointCandidates.length) {
        const key = `${Math.round(point.x * 20)}:${Math.round(point.z * 20)}`;
        const previous = cycleRef.current;

        let index = 0;
        if (
          previous &&
          previous.key === key &&
          previous.candidates.length === pointCandidates.length &&
          previous.candidates.every(
            (candidate, i) => selectionKey(candidate) === selectionKey(pointCandidates[i]),
          )
        ) {
          index = (previous.index + 1) % pointCandidates.length;
        }

        cycleRef.current = {
          key,
          candidates: pointCandidates,
          index,
        };
        onSelect(pointCandidates[index], false);
        return;
      }
    }

    if (event.button === 1) {
      event.preventDefault();
      setPanDrag({ x: event.clientX, y: event.clientY, panX: pan.x, panY: pan.y });
      hostRef.current?.setPointerCapture(event.pointerId);
      return;
    }

    if (event.button !== 0) return;

    if (mode === "measure") {
      onMeasurePoint(point);
      return;
    }
    if (mode === "add-manual-tag") {
      onAddManualTag(snapPoint({ x: point.x, y: point.z }, snapStep));
      return;
    }
    if (mode === "draw-floor" || mode === "add-hole" || mode === "draw-aisle") {
      onDrawPoint(snapPoint({ x: point.x, y: point.z }, snapStep));
      return;
    }

    if (event.target === event.currentTarget) {
      if (!event.shiftKey && !event.ctrlKey && !event.metaKey) onSelect({ kind: "zone", id: "__clear__" }, false);
      setSelectionBox({
        start: point,
        current: point,
        additive: event.shiftKey || event.ctrlKey || event.metaKey,
      });
      hostRef.current?.setPointerCapture(event.pointerId);
    }
  };

  const getSmartSnap = (selection: Selection, proposedX: number, proposedZ: number) => {
    if (!smartGuidesEnabled) {
      return { x: proposedX, z: proposedZ, guides: {} as { x?: number; z?: number } };
    }

    const tolerance = 0.4;
    const movingObj = selection.kind === "column"
      ? null
      : findCollectionObject(draft, selection.kind, selection.id);
    const movingBox = selection.kind === "column"
      ? { x: proposedX - 0.45, z: proposedZ - 0.45, w: 0.9, h: 0.9 }
      : movingObj
        ? get2DBox(selection.kind, movingObj)
        : { x: proposedX - 0.5, z: proposedZ - 0.5, w: 1, h: 1 };

    const halfW = movingBox.w / 2;
    const halfH = movingBox.h / 2;
    const xTargets: number[] = [];
    const zTargets: number[] = [];
    const selectedKey = selectionKey(selection);

    layerOrder.forEach((kind) => {
      if (!visibleLayers[kind] || lockedLayers[kind]) return;
      getCollection(draft, kind).forEach((obj: any) => {
        const candidate = { kind, id: objectId(kind, obj) } as Selection;
        if (selectionKey(candidate) === selectedKey) return;
        const b = get2DBox(kind, obj);
        const centerX = b.x + b.w / 2;
        const centerZ = b.z + b.h / 2;
        // Smart guides align like a diagram editor: center-to-center,
        // left-to-left/right-to-right and top-to-top/bottom-to-bottom.
        xTargets.push(
          centerX,
          b.x + halfW,
          b.x + b.w - halfW,
        );
        zTargets.push(
          centerZ,
          b.z + halfH,
          b.z + b.h - halfH,
        );
      });
    });

    const closest = (value: number, targets: number[]) => {
      let best = value;
      let bestDistance = tolerance;
      for (const target of targets) {
        const distance = Math.abs(target - value);
        if (distance <= bestDistance) {
          best = target;
          bestDistance = distance;
        }
      }
      return best;
    };

    const snappedX = closest(proposedX, xTargets);
    const snappedZ = closest(proposedZ, zTargets);

    return {
      x: snappedX,
      z: snappedZ,
      guides: {
        ...(Math.abs(snappedX - proposedX) > 1e-6 ? { x: snappedX } : {}),
        ...(Math.abs(snappedZ - proposedZ) > 1e-6 ? { z: snappedZ } : {}),
      },
    };
  };

  const moveCanvas = (event: React.PointerEvent<SVGSVGElement>) => {
    if (panDrag) {
      const rect = hostRef.current?.getBoundingClientRect();
      if (rect) {
        const visibleW = viewW / zoom;
        const visibleH = viewH / zoom;
        const dx = (event.clientX - panDrag.x) / rect.width * visibleW;
        const dy = (event.clientY - panDrag.y) / rect.height * visibleH;
        setPan(
          clampPan(
            panDrag.panX - dx,
            panDrag.panY - dy,
          ),
        );
      }
      return;
    }

    const point = getPoint(event);
    if (!point) return;

    if (vertexDrag) {
      const snapped = snapPoint({ x: point.x, y: point.z }, snapStep);
      if (vertexDrag.kind === "floor") onEditVertex(vertexDrag.id, vertexDrag.holeIndex, vertexDrag.index, snapped);
      else onEditAisleVertex(vertexDrag.id, vertexDrag.index, snapped);
      return;
    }

    if (resizeDrag) {
      onResize(
        resizeDrag.selection,
        resizeDrag.handle,
        snapPoint({ x: point.x, y: point.z }, snapStep).x,
        snapPoint({ x: point.x, y: point.z }, snapStep).y,
      );
      return;
    }

    if (rotateDrag) {
      const obj = getCollection(draft, rotateDrag.kind).find(
        (item) => item.id === rotateDrag.id,
      );
      if (obj) {
        const box = get2DBox(rotateDrag.kind, obj);
        const cx = box.x + box.w / 2;
        const cz = box.z + box.h / 2;
        const angle = Math.atan2(point.z - cz, point.x - cx) * 180 / Math.PI;
        onRotate(rotateDrag, angle);
      }
      return;
    }

    if (drag) {
      const proposed = snapPoint({ x: point.x + drag.offsetX, y: point.z + drag.offsetZ }, snapStep);
      const proposedX = proposed.x;
      const proposedZ = proposed.y;
      const snapped = getSmartSnap(drag.selection, proposedX, proposedZ);
      setSmartGuides(snapped.guides);
      onMove(drag.selection, snapped.x, snapped.z);
      return;
    }

    if (selectionBox) {
      setSelectionBox((v) => v ? { ...v, current: point } : null);
    }
  };

  const upCanvas = (event: React.PointerEvent<SVGSVGElement>) => {
    const point = getPoint(event);
    if ((drag || resizeDrag || rotateDrag) && point) {
      onCommitMove();
    }

    if (selectionBox && point) {
      const x = Math.min(selectionBox.start.x, point.x);
      const z = Math.min(selectionBox.start.z, point.z);
      const w = Math.abs(point.x - selectionBox.start.x);
      const h = Math.abs(point.z - selectionBox.start.z);
      if (w > 0.2 || h > 0.2) {
        onBoxSelect({ x, z, w, h }, selectionBox.additive);
      }
    }
    setDrag(null);
    setResizeDrag(null);
    setRotateDrag(null);
    setSelectionBox(null);
    setPanDrag(null);
    setSmartGuides({});
    setVertexDrag(null);
  };

  const handleWheel = (event: React.WheelEvent<SVGSVGElement>) => {
    event.preventDefault();
    const target = getSvgPx(event);
    if (!target) return;
    const oldZoom = zoom;
    const nextZoom = Math.max(0.65, Math.min(4, oldZoom * (event.deltaY < 0 ? 1.12 : 0.89)));
    if (Math.abs(nextZoom - oldZoom) < 0.001) return;

    const oldW = viewW / oldZoom;
    const oldH = viewH / oldZoom;
    const worldX = pan.x + (target.x / target.rect.width) * oldW;
    const worldY = pan.y + (target.y / target.rect.height) * oldH;
    const newW = viewW / nextZoom;
    const newH = viewH / nextZoom;
    const nextPan = clampPan(
      worldX - (target.x / target.rect.width) * newW,
      worldY - (target.y / target.rect.height) * newH,
      nextZoom,
    );
    setZoom(nextZoom);
    setPan(nextPan);
  };

  const insertVertexAt = (event: React.MouseEvent<SVGSVGElement>) => {
    if (mode !== "edit-vertex" && mode !== "edit-aisle") return;
    const point = getPoint(event); if (!point) return;
    const snapped = snapPoint({ x: point.x, y: point.z }, snapStep);
    const activeAisle = selected.find((item) => item.kind === "aisle");
    if (mode === "edit-aisle" && activeAisle) {
      const aisle = findCollectionObject(draft, "aisle", activeAisle.id); if (!aisle) return;
      let best = 0, bestDistance = Infinity;
      for (let i = 1; i < aisle.centerline.length; i += 1) { const a = aisle.centerline[i - 1], b = aisle.centerline[i], dx = b.x - a.x, dy = b.y - a.y, t = Math.max(0, Math.min(1, ((snapped.x - a.x) * dx + (snapped.y - a.y) * dy) / (dx * dx + dy * dy || 1))), d = Math.hypot(snapped.x - (a.x + t * dx), snapped.y - (a.y + t * dy)); if (d < bestDistance) { bestDistance = d; best = i; } }
      if (bestDistance < Math.max(0.5, aisle.width)) onInsertVertex("aisle", activeAisle.id, null, best, snapped);
      return;
    }
    const activeFloor = selected.find((item) => item.kind === "floor")?.id ?? (floor === "all" ? draft.floors[0]?.id : floor);
    const floorItem = draft.floors.find((item) => String(item.id) === String(activeFloor)); if (!floorItem) return;
    const boundary = (floorItem.boundary ?? floorItem.footprint ?? []).map((entry) => Array.isArray(entry) ? { x: entry[0], y: entry[1] } : entry);
    let best = 0, bestDistance = Infinity;
    for (let i = 0; i < boundary.length; i += 1) { const a = boundary[i], b = boundary[(i + 1) % boundary.length], dx = b.x - a.x, dy = b.y - a.y, t = Math.max(0, Math.min(1, ((snapped.x - a.x) * dx + (snapped.y - a.y) * dy) / (dx * dx + dy * dy || 1))), d = Math.hypot(snapped.x - (a.x + t * dx), snapped.y - (a.y + t * dy)); if (d < bestDistance) { bestDistance = d; best = i + 1; } }
    if (bestDistance < 1) onInsertVertex("floor", String(floorItem.id), null, best, snapped);
  };

  return (
    <div className="absolute inset-0 overflow-auto bg-[#07101b]">
      <svg
        ref={hostRef}
        viewBox={`${pan.x} ${pan.y} ${viewW / zoom} ${viewH / zoom}`}
        className="h-full w-full select-none"
        style={{
          width: "100%",
          height: "100%",
          transform: `rotate(${viewRotation}deg)`,
          transformOrigin: "center",
          cursor: panDrag ? "grabbing" : mode === "measure" ? "crosshair" : mode === "move" ? "move" : "default",
          touchAction: "none",
        }}
        onPointerDown={downCanvas}
        onPointerMove={moveCanvas}
        onPointerUp={upCanvas}
        onPointerCancel={upCanvas}
        onDoubleClick={(event) => { event.preventDefault(); if (mode === "draw-floor" || mode === "add-hole" || mode === "draw-aisle") onFinishDrawing(); else insertVertexAt(event); }}
        onWheel={handleWheel}
        onAuxClick={(event) => event.preventDefault()}
        onContextMenu={(event) => {
          event.preventDefault();
          const target = event.target === event.currentTarget ? null : (selected[0] ?? null);
          onContextMenu?.(target, event.clientX, event.clientY);
        }}
      >
        <rect x={0} y={0} width={viewW} height={viewH} fill="#08111d" />

        <defs>
          <marker id="nav-edge-arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#c084fc" /></marker>
          <marker id="nav-edge-arrow-reverse" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#f0abfc" /></marker>
          <marker id="nav-edge-arrow-both" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#c084fc" /></marker>
          <pattern
            id="editor-grid"
            width={draft.grid.cell_size * scale.x}
            height={draft.grid.cell_size * scale.z}
            patternUnits="userSpaceOnUse"
          >
            <path
              d={`M ${draft.grid.cell_size * scale.x} 0 L 0 0 0 ${draft.grid.cell_size * scale.z}`}
              fill="none"
              stroke="#1e293b"
              strokeWidth="1"
              opacity="0.65"
            />
          </pattern>
        </defs>

        <rect x={0} y={0} width={viewW} height={viewH} fill="url(#editor-grid)" />

        {(() => {
          const active = floor === "all" ? draft.floors[0] : draft.floors.find((item) => String(item.id) === String(floor));
          const boundary = active?.boundary ?? active?.footprint ?? [[0, 0], [draft.size.width, 0], [draft.size.width, draft.size.depth], [0, draft.size.depth]];
          const path = (points: Array<Point | [number, number]>) => points.map((p) => { const x = Array.isArray(p) ? p[0] : p.x; const y = Array.isArray(p) ? p[1] : p.y; return `${x * scale.x},${y * scale.z}`; }).join(" ");
          return <g>
            <polygon points={path(boundary as Array<Point | [number, number]>)} fill="#0b1727" fillOpacity="0.55" stroke="#38bdf8" strokeWidth="1.5" vectorEffect="non-scaling-stroke" />
            {(active?.holes ?? []).map((hole, index) => <polygon key={`editor-hole-${index}`} points={path(hole as Array<Point | [number, number]>)} fill="#020617" stroke="#f97316" strokeWidth="1.2" vectorEffect="non-scaling-stroke" />)}
            {drawingPoints.length > 0 && <polyline points={path(drawingPoints)} fill="none" stroke="#facc15" strokeWidth="2" vectorEffect="non-scaling-stroke" />}
            {drawingPoints.map((p, index) => <circle key={`draw-point-${index}`} cx={p.x * scale.x} cy={p.y * scale.z} r="4" fill="#facc15" vectorEffect="non-scaling-stroke" />)}
          </g>;
        })()}

        {(() => {
          const active = floor === "all" ? draft.floors[0] : draft.floors.find((item) => String(item.id) === String(floor));
          if (!active) return null;
          const toPoint = (entry: Point | [number, number]): Point => Array.isArray(entry) ? { x: entry[0], y: entry[1] } : entry;
          const handle = (kind: "floor" | "aisle", id: string, holeIndex: number | null, index: number) => (event: React.PointerEvent<SVGCircleElement>) => { event.stopPropagation(); const value = { kind, id, holeIndex, index } as const; setVertexDrag(value); setActiveVertex(value); onSelect({ kind, id }, false); hostRef.current?.setPointerCapture(event.pointerId); };
          const floorHandles = mode === "edit-vertex" || selected.some((item) => item.kind === "floor" && item.id === String(active.id)) ? <g key={`floor-handles-${active.id}`} pointerEvents="all">{(active.boundary ?? active.footprint ?? []).map((entry, index) => { const p = toPoint(entry); return <circle key={`floor-v-${index}`} cx={p.x * scale.x} cy={p.y * scale.z} r="6" fill="#0b1220" stroke="#38bdf8" strokeWidth="1.5" vectorEffect="non-scaling-stroke" onPointerDown={handle("floor", String(active.id), null, index)} />; })}{(active.holes ?? []).flatMap((hole, holeIndex) => hole.map((entry, index) => { const p = toPoint(entry); return <circle key={`hole-v-${holeIndex}-${index}`} cx={p.x * scale.x} cy={p.y * scale.z} r="5" fill="#0b1220" stroke="#f97316" strokeWidth="1.5" vectorEffect="non-scaling-stroke" onPointerDown={handle("floor", String(active.id), holeIndex, index)} />; }))}</g> : null;
          const aisleHandles = (draft.aisles ?? []).filter((aisle) => (floor === "all" || String(aisle.floor_id ?? 1) === String(floor)) && (mode === "edit-aisle" || selected.some((item) => item.kind === "aisle" && item.id === String(aisle.id)))).map((aisle) => <g key={`aisle-handles-${aisle.id}`} pointerEvents="all">{aisle.centerline.map((point, index) => <circle key={`aisle-v-${aisle.id}-${index}`} cx={point.x * scale.x} cy={point.y * scale.z} r="5" fill="#0b1220" stroke="#67e8f9" strokeWidth="1.5" vectorEffect="non-scaling-stroke" onPointerDown={handle("aisle", String(aisle.id), null, index)} />)}</g>);
          return <>{floorHandles}{aisleHandles}</>;
        })()}

        {background && (
          <image
            href={background}
            x={(viewW - viewW * backgroundScale) / 2 + backgroundOffsetX * scale.x}
            y={(viewH - viewH * backgroundScale) / 2 + backgroundOffsetY * scale.z}
            width={viewW * backgroundScale}
            height={viewH * backgroundScale}
            opacity={backgroundOpacity}
            preserveAspectRatio="none"
            pointerEvents="none"
            transform={`rotate(${backgroundRotation} ${viewW / 2} ${viewH / 2})`}
          />
        )}

        <g opacity={0.65}>
          <line
            x1={20}
            y1={20}
            x2={120}
            y2={20}
            stroke="#334155"
            strokeWidth="1"
            vectorEffect="non-scaling-stroke"
          />
        </g>

        {items.flatMap(([kind, arr]) =>
          visibleLayers[kind]
            ? arr
                .filter((obj) => {
                  if (floor === "all") return true;
                if (kind === "floor") return String(obj.id) === String(floor);
                if ("floor" in obj || "floor_id" in obj) return String(getFloorForObject(obj)) === String(floor);
                  return true;
                })
                .map((obj) => (
                  <MapObject
                    key={`${kind}-${objectId(kind, obj)}`}
                    kind={kind}
                    obj={obj}
                    selected={selected.some(
                      (s) => s.kind === kind && s.id === objectId(kind, obj),
                    )}
                    scale={scale}
                    onPointerDown={(event) =>
                      beginItem(event, kind, obj)
                    }
                    onContextMenu={(event) => {
                      event.preventDefault();
                      event.stopPropagation();
                      onContextMenu?.({ kind, id: objectId(kind, obj) }, event.clientX, event.clientY);
                    }}
                  />
                ))
            : [],
        )}

        {blocks.map((block) => {
          const members = block.members
            .map((member) => {
              if (member.kind === "column") {
                const index = Number(member.id.split("-").pop() ?? "-1");
                const point = draft.columns?.[index];
                return point
                  ? { x: point[0] - 0.45, z: point[1] - 0.45, w: 0.9, h: 0.9 }
                  : null;
              }
              const obj = getCollection(draft, member.kind).find(
                (item) => item.id === member.id,
              );
              return obj ? get2DBox(member.kind, obj) : null;
            })
            .filter(Boolean) as { x: number; z: number; w: number; h: number }[];

          if (!members.length) return null;

          const minX = Math.min(...members.map((b) => b.x));
          const minZ = Math.min(...members.map((b) => b.z));
          const maxX = Math.max(...members.map((b) => b.x + b.w));
          const maxZ = Math.max(...members.map((b) => b.z + b.h));
          const active = activeBlockId === block.id;

          return (
            <g key={`block-${block.id}`} pointerEvents="none">
              <rect
                x={minX * scale.x - 7}
                y={minZ * scale.z - 7}
                width={(maxX - minX) * scale.x + 14}
                height={(maxZ - minZ) * scale.z + 14}
                rx="7"
                fill="none"
                stroke={active ? "#c4b5fd" : "#818cf8"}
                strokeWidth={active ? 2 : 1}
                strokeDasharray={active ? "3 3" : "7 5"}
                vectorEffect="non-scaling-stroke"
                opacity={0.7}
              />
              <rect
                x={minX * scale.x - 5}
                y={minZ * scale.z - 18}
                width={Math.max(52, block.name.length * 6 + 12)}
                height={14}
                rx="4"
                fill="#0b1220"
                stroke={active ? "#c4b5fd" : "#818cf8"}
                vectorEffect="non-scaling-stroke"
                opacity={0.92}
              />
              <text
                x={minX * scale.x + 2}
                y={minZ * scale.z - 8}
                fill="#c4b5fd"
                fontSize="8"
                fontWeight={700}
              >
                {block.name}
              </text>
            </g>
          );
        })}

        {visibleLayers.column && !lockedLayers.column && draft.columns?.map(([cx, cz], index) => {
          const selectedColumn = selected.some(
            (item) => item.kind === "column" && item.id === `COL-${index}`,
          );
          return (
            <g
              key={`column-COL-${index}`}
              onPointerDown={(event) =>
                beginItem(
                  event,
                  "column",
                  [cx, cz],
                )
              }
              className="cursor-pointer"
            >
              <rect
                x={(cx - 0.45) * scale.x}
                y={(cz - 0.45) * scale.z}
                width={0.9 * scale.x}
                height={0.9 * scale.z}
                rx={2}
                fill={COLORS.column}
                fillOpacity={0.9}
                stroke={selectedColumn ? "#ffffff" : COLORS.column}
                strokeWidth={selectedColumn ? 2 : 1}
                vectorEffect="non-scaling-stroke"
              />
            </g>
          );
        })}

        {selected.length === 1 &&
          !["zone", "walkway", "conveyor", "camera", "sensor", "location", "spawn", "lift"].includes(
            selected[0].kind,
          ) && (() => {
            const selection = selected[0];
            const obj = getCollection(draft, selection.kind).find(
              (item) => objectId(selection.kind, item) === selection.id,
            );
            if (!obj) return null;
            const box = get2DBox(selection.kind, obj);
            const handles = [
              ["nw", box.x, box.z],
              ["ne", box.x + box.w, box.z],
              ["sw", box.x, box.z + box.h],
              ["se", box.x + box.w, box.z + box.h],
            ] as const;

            return (
              <g pointerEvents="all">
                <rect
                  x={box.x * scale.x}
                  y={box.z * scale.z}
                  width={box.w * scale.x}
                  height={box.h * scale.z}
                  fill="none"
                  stroke="#38bdf8"
                  strokeDasharray="4 4"
                  strokeWidth="1"
                  vectorEffect="non-scaling-stroke"
                  pointerEvents="none"
                />
                {handles.map(([handle, hx, hz]) => (
                  <rect
                    key={handle}
                    x={hx * scale.x - 5}
                    y={hz * scale.z - 5}
                    width="10"
                    height="10"
                    rx="2"
                    fill="#0b1220"
                    stroke="#38bdf8"
                    strokeWidth="1.5"
                    vectorEffect="non-scaling-stroke"
                    className="cursor-nwse-resize"
                    onPointerDown={(event) => {
                      event.stopPropagation();
                      if (mode !== "select" && mode !== "move") return;
                      if (lockedLayers[selection.kind]) return;
                      setResizeDrag({ selection, handle });
                      hostRef.current?.setPointerCapture(event.pointerId);
                    }}
                  />
                ))}

                {["rack", "charger", "spawn"].includes(selection.kind) && (
                  <>
                    <line
                      x1={(box.x + box.w / 2) * scale.x}
                      y1={(box.z) * scale.z}
                      x2={(box.x + box.w / 2) * scale.x}
                      y2={(box.z * scale.z) - 18}
                      stroke="#a78bfa"
                      strokeWidth="1"
                      vectorEffect="non-scaling-stroke"
                      pointerEvents="none"
                    />
                    <circle
                      cx={(box.x + box.w / 2) * scale.x}
                      cy={(box.z * scale.z) - 24}
                      r="6"
                      fill="#0b1220"
                      stroke="#a78bfa"
                      strokeWidth="1.5"
                      vectorEffect="non-scaling-stroke"
                      className="cursor-crosshair"
                      onPointerDown={(event) => {
                        event.stopPropagation();
                        if (mode !== "select" && mode !== "move") return;
                        if (lockedLayers[selection.kind]) return;
                        setRotateDrag(selection);
                        hostRef.current?.setPointerCapture(event.pointerId);
                      }}
                    />
                  </>
                )}
                <line
                  x1={(box.x + box.w / 2) * scale.x}
                  y1={box.z * scale.z}
                  x2={(box.x + box.w / 2) * scale.x}
                  y2={(box.z * scale.z) - 18}
                  stroke="#38bdf8"
                  strokeWidth="1"
                  vectorEffect="non-scaling-stroke"
                  pointerEvents="none"
                />
                <circle
                  cx={(box.x + box.w / 2) * scale.x}
                  cy={(box.z * scale.z) - 24}
                  r="6"
                  fill="#0b1220"
                  stroke="#38bdf8"
                  strokeWidth="1.5"
                  vectorEffect="non-scaling-stroke"
                  pointerEvents="none"
                />
              </g>
            );
          })()}

        {selectionBox && (
          <rect
            x={Math.min(selectionBox.start.x, selectionBox.current.x) * scale.x}
            y={Math.min(selectionBox.start.z, selectionBox.current.z) * scale.z}
            width={Math.abs(selectionBox.current.x - selectionBox.start.x) * scale.x}
            height={Math.abs(selectionBox.current.z - selectionBox.start.z) * scale.z}
            fill="#38bdf8"
            fillOpacity={0.06}
            stroke="#38bdf8"
            strokeDasharray="6 4"
            strokeWidth="1.5"
            vectorEffect="non-scaling-stroke"
            pointerEvents="none"
          />
        )}

        {measurePoints.length === 2 && (
          <>
            <line
              x1={measurePoints[0].x * scale.x}
              y1={measurePoints[0].z * scale.z}
              x2={measurePoints[1].x * scale.x}
              y2={measurePoints[1].z * scale.z}
              stroke="#f59e0b"
              strokeWidth="2"
              vectorEffect="non-scaling-stroke"
            />
            <circle
              cx={measurePoints[0].x * scale.x}
              cy={measurePoints[0].z * scale.z}
              r="4"
              fill="#f59e0b"
              vectorEffect="non-scaling-stroke"
            />
            <circle
              cx={measurePoints[1].x * scale.x}
              cy={measurePoints[1].z * scale.z}
              r="4"
              fill="#f59e0b"
              vectorEffect="non-scaling-stroke"
            />
            <rect
              x={
                ((measurePoints[0].x + measurePoints[1].x) / 2) * scale.x - 36
              }
              y={
                ((measurePoints[0].z + measurePoints[1].z) / 2) * scale.z - 12
              }
              width="72"
              height="20"
              rx="5"
              fill="#111827"
              stroke="#f59e0b"
              strokeWidth="1"
              vectorEffect="non-scaling-stroke"
            />
            <text
              x={
                ((measurePoints[0].x + measurePoints[1].x) / 2) * scale.x
              }
              y={
                ((measurePoints[0].z + measurePoints[1].z) / 2) * scale.z + 3
              }
              textAnchor="middle"
              fill="#f8fafc"
              fontSize="10"
              fontWeight={700}
              pointerEvents="none"
            >
              {Math.hypot(
                measurePoints[1].x - measurePoints[0].x,
                measurePoints[1].z - measurePoints[0].z,
              ).toFixed(2)} m
            </text>
          </>
        )}

        {/* north / coordinate origin */}
        <g transform="translate(16 16)">
          <circle cx="12" cy="12" r="12" fill="#0f172a" stroke="#334155" />
          <path d="M 12 4 L 8 13 L 12 11 L 16 13 Z" fill="#e2e8f0" />
          <text x="12" y="30" textAnchor="middle" fill="#94a3b8" fontSize="9">N</text>
        </g>
        {smartGuides.x !== undefined && (
          <line
            x1={smartGuides.x * scale.x}
            y1={0}
            x2={smartGuides.x * scale.x}
            y2={viewH}
            stroke="#38bdf8"
            strokeWidth={1}
            strokeDasharray="6 4"
            vectorEffect="non-scaling-stroke"
            opacity={0.8}
            pointerEvents="none"
          />
        )}
        {smartGuides.z !== undefined && (
          <line
            x1={0}
            y1={smartGuides.z * scale.z}
            x2={viewW}
            y2={smartGuides.z * scale.z}
            stroke="#38bdf8"
            strokeWidth={1}
            strokeDasharray="6 4"
            vectorEffect="non-scaling-stroke"
            opacity={0.8}
            pointerEvents="none"
          />
        )}

      </svg>
    </div>
  );
}

function ToolButton({
  active,
  label,
  icon,
  onClick,
}: {
  active?: boolean;
  label: string;
  icon: React.ReactNode;
  onClick: () => void;
}) {
  return (
    <button
      type="button"
      onClick={onClick}
      className={[
        "group flex w-full items-center gap-2 rounded-lg border px-2 py-2 text-left transition-all",
        active
          ? "border-sky-400/25 bg-sky-400/10 text-white"
          : "border-transparent bg-transparent text-slate-400 hover:border-white/[0.06] hover:bg-white/[0.04] hover:text-slate-100",
      ].join(" ")}
    >
      <span className="grid h-7 w-7 shrink-0 place-items-center rounded-md bg-white/[0.04] text-xs">
        {icon}
      </span>
      <span className="text-[11px] font-medium">{label}</span>
    </button>
  );
}

function LayerRow({
  kind,
  visible,
  locked,
  count,
  isTop,
  isBottom,
  onToggle,
  onToggleLock,
  onMove,
  onIsolate,
}: {
  kind: Kind;
  visible: boolean;
  locked: boolean;
  count: number;
  isTop: boolean;
  isBottom: boolean;
  onToggle: () => void;
  onToggleLock: () => void;
  onMove: (delta: -1 | 1) => void;
  onIsolate: () => void;
}) {
  return (
    <div
      className={[
        "group flex items-center gap-1 rounded-lg border px-1.5 py-1.5 transition-colors",
        locked
          ? "border-amber-400/10 bg-amber-400/[0.025]"
          : "border-transparent hover:border-white/[0.05] hover:bg-white/[0.035]",
      ].join(" ")}
      title={`${LABELS[kind]} · ${count} object${count === 1 ? "" : "s"}`}
    >
      <button
        type="button"
        onClick={onToggle}
        className="grid h-6 w-6 shrink-0 place-items-center rounded-md text-[10px] text-slate-500 hover:bg-white/[0.06] hover:text-slate-200"
        title={visible ? "Hide layer" : "Show layer"}
      >
        {visible ? "◉" : "○"}
      </button>

      <button
        type="button"
        onClick={onIsolate}
        className="h-6 w-2.5 shrink-0 rounded-full transition-transform hover:scale-125"
        style={{ background: COLORS[kind] }}
        title="Isolate layer"
        aria-label={`Isolate ${LABELS[kind]}`}
      />

      <div className="min-w-0 flex-1">
        <div className={[
          "truncate text-[10px] font-medium",
          visible ? "text-slate-300" : "text-slate-600",
        ].join(" ")}>
          {LAYER_LABELS[kind]}
        </div>
        <div className="text-[8px] text-slate-700">
          {count} {count === 1 ? "object" : "objects"}
        </div>
      </div>

      <button
        type="button"
        onClick={onToggleLock}
        className={[
          "grid h-6 w-6 shrink-0 place-items-center rounded-md text-[10px] transition-colors",
          locked
            ? "bg-amber-400/10 text-amber-300"
            : "text-slate-700 hover:bg-white/[0.06] hover:text-slate-400",
        ].join(" ")}
        title={locked ? "Unlock layer" : "Lock layer"}
      >
        {locked ? "🔒" : "○"}
      </button>

      <div className="hidden items-center gap-0.5 opacity-0 transition-opacity group-hover:flex group-hover:opacity-100">
        <button
          type="button"
          disabled={isTop}
          onClick={() => onMove(1)}
          className="grid h-5 w-5 place-items-center rounded text-[9px] text-slate-600 hover:bg-white/[0.06] hover:text-slate-300 disabled:opacity-20"
          title="Bring layer forward"
        >
          ↑
        </button>
        <button
          type="button"
          disabled={isBottom}
          onClick={() => onMove(-1)}
          className="grid h-5 w-5 place-items-center rounded text-[9px] text-slate-600 hover:bg-white/[0.06] hover:text-slate-300 disabled:opacity-20"
          title="Send layer backward"
        >
          ↓
        </button>
      </div>
    </div>
  );
}

function PropertyInput({
  label,
  value,
  type = "text",
  step,
  onChange,
}: {
  label: string;
  value: any;
  type?: "text" | "number";
  step?: string;
  onChange: (value: string) => void;
}) {
  return (
    <label className="block">
      <span className="mb-1 block text-[9px] font-medium uppercase tracking-[0.08em] text-slate-500">
        {label}
      </span>
      <input
        type={type}
        step={step}
        value={value ?? ""}
        onChange={(e) => onChange(e.target.value)}
        className="w-full rounded-md border border-white/[0.08] bg-[#070d16] px-2.5 py-2 text-[11px] text-slate-200 outline-none transition focus:border-sky-400/40 focus:bg-[#09111d]"
      />
    </label>
  );
}

export function WarehouseEditorPage() {
  const layoutRevision = useStore((s) => s.layoutRevision);
  const activeWarehouseId = useStore((s) => s.activeWarehouseId);
  const [draft, setDraft] = useState<Draft>(() => normalizeDraft(base));
  const [selected, setSelected] = useState<Selection[]>([]);
  const [view, setView] = useState<"2d" | "3d" | "code">("2d");
  const [mode, setMode] = useState<Mode>("select");
  const [code, setCode] = useState(() => JSON.stringify(base, null, 2));
  const [status, setStatus] = useState("LOCAL DRAFT");
  const [publishedVersion, setPublishedVersion] = useState<number | null>(null);
  const [artifactStatus, setArtifactStatus] = useState("");
  const [errors, setErrors] = useState<string[]>([]);
  const [saving, setSaving] = useState(false);
  const [floor, setFloor] = useState<number | string | "all">(1);
  const [viewRotation, setViewRotation] = useState(0);
  const [history, setHistory] = useState<Draft[]>([]);
  const [future, setFuture] = useState<Draft[]>([]);
  const [dragging, setDragging] = useState(false);
  const [visibleLayers, setVisibleLayers] = useState<Record<Kind, boolean>>(
    () =>
      Object.fromEntries(
        Object.keys(COLORS).map((key) => [key, true]),
      ) as Record<Kind, boolean>,
  );
  const [lockedLayers, setLockedLayers] = useState<Record<Kind, boolean>>(
    () =>
      Object.fromEntries(
        Object.keys(COLORS).map((key) => [key, false]),
      ) as Record<Kind, boolean>,
  );
  const [layerOrder, setLayerOrder] = useState<Kind[]>(() => [...DEFAULT_LAYER_ORDER]);
  const [blocks, setBlocks] = useState<EditorBlock[]>([]);
  const [activeBlockId, setActiveBlockId] = useState<string | null>(null);
  const [background, setBackground] = useState<string | null>(null);
  const [backgroundOpacity, setBackgroundOpacity] = useState(0.35);
  const [backgroundScale, setBackgroundScale] = useState(1);
  const [backgroundOffsetX, setBackgroundOffsetX] = useState(0);
  const [backgroundOffsetY, setBackgroundOffsetY] = useState(0);
  const [backgroundRotation, setBackgroundRotation] = useState(0);
  const [calibratingBackground, setCalibratingBackground] = useState(false);
  const [settingReferenceOrigin, setSettingReferenceOrigin] = useState(false);
  const [measurePoints, setMeasurePoints] = useState<
    { x: number; z: number }[]
  >([]);
  const [drawingPoints, setDrawingPoints] = useState<Point[]>([]);
  const [arrayOpen, setArrayOpen] = useState(false);
  const [contextMenu, setContextMenu] = useState<{ x: number; y: number; selection: Selection | null } | null>(null);
  const [arrayRows, setArrayRows] = useState(2);
  const [arrayCols, setArrayCols] = useState(5);
  const [smartGuidesEnabled, setSmartGuidesEnabled] = useState(true);
  const [snapStep, setSnapStep] = useState<number | null>(0.5);
  const [arrayGapX, setArrayGapX] = useState(1.0);
  const [arrayGapZ, setArrayGapZ] = useState(1.0);
  const [search, setSearch] = useState("");
  const fileInputRef = useRef<HTMLInputElement>(null);

  useEffect(() => {
    const finish = (event: KeyboardEvent) => { if (event.key === "Enter" && ["draw-floor", "add-hole", "draw-aisle"].includes(mode)) { event.preventDefault(); finishDrawing(); } };
    window.addEventListener("keydown", finish);
    return () => window.removeEventListener("keydown", finish);
  }, [mode, drawingPoints]);

  const selectedObjects = useMemo(
    () =>
      selected
        .map((selection) => {
          if (selection.kind === "column") return { selection, obj: null };
          const obj = findCollectionObject(draft, selection.kind, selection.id);
          return obj ? { selection, obj } : null;
        })
        .filter(Boolean) as { selection: Selection; obj: any }[],
    [draft, selected],
  );

  const primary = selectedObjects[0];

  useEffect(() => {
    setCode(JSON.stringify(draft, null, 2));
  }, [draft]);

  async function reloadSharedLayout(reason = "DB MAP LOADED") {
    const response = await apiFetch("/api/layout/draft");
    if (!response.ok) throw new Error(await response.text());
    const remote = await response.json();
    const revision = Number(response.headers.get("x-layout-revision"));
    if (Number.isFinite(revision)) useStore.setState({ layoutRevision: revision });
    const version = Number(response.headers.get("x-layout-version"));
    if (Number.isFinite(version)) setPublishedVersion(version);
    const normalized = normalizeDraft(remote);
    setDraft(normalized);
    setSelected([]);
    setHistory([]);
    setFuture([]);
    setErrors([]);
    setStatus(reason);
  }

  useEffect(() => {
    if (DEMO_MODE) {
      setStatus("LOCAL DEMO · NO BACKEND");
      return;
    }
    void reloadSharedLayout("DB MAP LOADED").catch((error) => setErrors([String(error)]));
    return onLayoutUpdated((meta) => {
      if (meta.is_active === false) return;
      void reloadSharedLayout(`SYNCED r${meta.revision} · ${meta.source}`).catch((error) => setErrors([String(error)]));
    });
  }, []);

  useEffect(() => {
    try {
      const savedBlocks = window.localStorage.getItem(BLOCKS_STORAGE_KEY);
      const savedLayers = window.localStorage.getItem(LAYERS_STORAGE_KEY);
      if (savedBlocks) {
        const parsed = JSON.parse(savedBlocks) as EditorBlock[];
        if (Array.isArray(parsed)) setBlocks(parsed);
      }
      if (savedLayers) {
        const parsed = JSON.parse(savedLayers) as {
          order?: Kind[];
          visible?: Record<Kind, boolean>;
          locked?: Record<Kind, boolean>;
        };
        if (Array.isArray(parsed.order)) {
          const known = new Set(DEFAULT_LAYER_ORDER);
          const sanitized = parsed.order.filter((kind) => known.has(kind));
          setLayerOrder([
            ...sanitized,
            ...DEFAULT_LAYER_ORDER.filter((kind) => !sanitized.includes(kind)),
          ]);
        }
        if (parsed.visible && typeof parsed.visible === "object") {
          setVisibleLayers((current) => ({ ...current, ...parsed.visible }));
        }
        if (parsed.locked && typeof parsed.locked === "object") {
          setLockedLayers((current) => ({ ...current, ...parsed.locked }));
        }
      }
    } catch {
      // Ignore malformed editor metadata; geometry remains authoritative.
    }
  }, []);

  useEffect(() => {
    try {
      window.localStorage.setItem(BLOCKS_STORAGE_KEY, JSON.stringify(blocks));
      window.localStorage.setItem(
        LAYERS_STORAGE_KEY,
        JSON.stringify({
          order: layerOrder,
          visible: visibleLayers,
          locked: lockedLayers,
        }),
      );
    } catch {
      // localStorage can be unavailable in private/restricted browsing contexts.
    }
  }, [blocks, layerOrder, visibleLayers, lockedLayers]);

  useEffect(() => {
    const onKey = (event: KeyboardEvent) => {
      const target = event.target as HTMLElement | null;
      const editing =
        !!target && ["INPUT", "TEXTAREA", "SELECT"].includes(target.tagName);
      if (editing) return;

      if (event.key === "Escape") {
        setContextMenu(null);
      }

      if (event.key === "Delete" || event.key === "Backspace") {
        if (selected.length) {
          event.preventDefault();
          remove();
        }
      } else if (event.key === "Escape") {
        setSelected([]);
        setMode("select");
        setMeasurePoints([]);
        setStatus("SELECTION CLEARED");
      } else if (
        (event.ctrlKey || event.metaKey) &&
        event.key.toLowerCase() === "z"
      ) {
        event.preventDefault();
        event.shiftKey ? redo() : undo();
      } else if (
        (event.ctrlKey || event.metaKey) &&
        event.key.toLowerCase() === "y"
      ) {
        event.preventDefault();
        redo();
      } else if (
        (event.ctrlKey || event.metaKey) &&
        event.key.toLowerCase() === "d"
      ) {
        event.preventDefault();
        duplicate();
      } else if (
        (event.ctrlKey || event.metaKey) &&
        event.key.toLowerCase() === "g"
      ) {
        event.preventDefault();
        createBlock();
      } else if (
        (event.ctrlKey || event.metaKey) &&
        event.shiftKey &&
        event.key.toLowerCase() === "g"
      ) {
        event.preventDefault();
        ungroupBlock();
      } else if (
        event.key === "Enter"
      ) {
        if (activeBlockId) {
          event.preventDefault();
          exitBlock();
        } else {
          event.preventDefault();
          enterBlock();
        }
      } else if (
        (event.ctrlKey || event.metaKey) &&
        event.key.toLowerCase() === "c"
      ) {
        // Copy is intentionally handled via an in-memory clipboard below.
        copySelected();
      } else if (
        (event.ctrlKey || event.metaKey) &&
        event.key.toLowerCase() === "v"
      ) {
        event.preventDefault();
        pasteSelected();
      }
    };

    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  });

  function selectionExistsInDraft(source: Draft, selection: Selection) {
    if (selection.kind === "column") {
      const index = Number(selection.id.split("-").pop() ?? "-1");
      return Number.isInteger(index) && !!source.columns?.[index];
    }
    return getCollection(source, selection.kind).some(
      (item) => String(item.id) === selection.id,
    );
  }

  function sanitizeBlocks(source: Draft, input: EditorBlock[]) {
    return input
      .map((block) => ({
        ...block,
        members: block.members.filter((member) =>
          selectionExistsInDraft(source, member),
        ),
      }))
      .filter((block) => block.members.length >= 2);
  }

  function commit(next: Draft, label = "UNSAVED") {
    const normalized = normalizeDraft(next);
    setHistory((current) => [
      ...current.slice(-39),
      clone(draft),
    ]);
    setFuture([]);
    setBlocks((current) => sanitizeBlocks(normalized, current));
    setDraft(normalized);
    setStatus(label);
  }

  function undo() {
    if (!history.length) return;
    const next = history[history.length - 1];
    setFuture((current) => [clone(draft), ...current.slice(0, 39)]);
    setHistory((current) => current.slice(0, -1));
    setDraft(normalizeDraft(next));
    setStatus("UNDO");
  }

  function redo() {
    if (!future.length) return;
    const next = future[0];
    setHistory((current) => [
      ...current,
      clone(draft),
    ].slice(-40));
    setFuture((current) => current.slice(1));
    setDraft(normalizeDraft(next));
    setStatus("REDO");
  }

  function add(kind: Kind) {
    const next = clone(draft);
    const created = addObject(next, kind);
    commit(next, `${LABELS[kind].toUpperCase()} ADDED · UNSAVED`);
    setSelected([created]);
  }

  function removeSelections(selections: Selection[]) {
    if (!selections.length) return;
    const next = clone(draft);
    const editable = selections.filter((selection) => !lockedLayers[selection.kind]);
    editable.forEach((selection) => removeObject(next, selection));
    if (editable.length) commit(next, "DELETED · UNSAVED");
    setSelected([]);
  }

  function remove() {
    removeSelections(selected);
  }

  function findBlockForSelection(selection: Selection) {
    return blocks.find((block) =>
      block.members.some((member) => selectionKey(member) === selectionKey(selection)),
    );
  }

  function selectObject(selection: Selection, additive: boolean) {
    if (selection.id === "__clear__") {
      if (!activeBlockId) setSelected([]);
      return;
    }

    if (lockedLayers[selection.kind]) return;

    const block = findBlockForSelection(selection);
    const effectiveSelection =
      block && block.id !== activeBlockId
        ? block.members.filter((member) => !lockedLayers[member.kind])
        : [selection];

    setSelected((current) => {
      if (!additive) return effectiveSelection;

      const currentKeys = new Set(current.map(selectionKey));
      const allPresent = effectiveSelection.every((item) => currentKeys.has(selectionKey(item)));

      if (allPresent) {
        const removeKeys = new Set(effectiveSelection.map(selectionKey));
        return current.filter((item) => !removeKeys.has(selectionKey(item)));
      }

      return [
        ...current,
        ...effectiveSelection.filter((item) => !currentKeys.has(selectionKey(item))),
      ];
    });
  }

  function onBoxSelect(bounds: { x: number; z: number; w: number; h: number }, additive: boolean) {
    const found: Selection[] = [];

    (Object.keys(KEY_MAP) as Kind[]).forEach((kind) => {
      const arr = getCollection(draft, kind);
      if (!visibleLayers[kind] || lockedLayers[kind]) return;

      arr.forEach((obj) => {
        if (["floor", "aisle", "navigation-tag"].includes(kind)) return;
        const box = get2DBox(kind, obj);
        const intersects = !(
          box.x + box.w < bounds.x ||
          box.x > bounds.x + bounds.w ||
          box.z + box.h < bounds.z ||
          box.z > bounds.z + bounds.h
        );
        if (intersects) found.push({ kind, id: objectId(kind, obj) });
      });
    });

    if (visibleLayers.column && !lockedLayers.column) {
      draft.columns?.forEach(([x, z], index) => {
        if (
          x >= bounds.x &&
          x <= bounds.x + bounds.w &&
          z >= bounds.z &&
          z <= bounds.z + bounds.h
        ) {
          found.push({ kind: "column", id: `COL-${index}` });
        }
      });
    }

    setSelected((current) => {
      if (!additive) return found;
      const merged = [...current];
      found.forEach((item) => {
        if (!merged.some((x) => x.kind === item.kind && x.id === item.id)) {
          merged.push(item);
        }
      });
      return merged;
    });
  }

  function begin2DEdit() {
    if (!editSnapshotRef.current) editSnapshotRef.current = clone(draft);
  }

  function rotateSelection(delta: number) {
    if (!selected.length) return;
    const next = clone(draft);
    selected.forEach((selection) => rotateObject(next, selection, delta));
    commit(next, `ROTATED ${delta > 0 ? "CW" : "CCW"} · UNSAVED`);
  }

  function updateProperty(path: string, value: string) {
    if (!primary) return;
    const next = clone(draft);
    const obj = findCollectionObject(next, primary.selection.kind, primary.selection.id);
    if (!obj) return;

    if (primary.selection.kind === "navigation-tag" && (path === "id" || path === "uuid")) {
      setErrors(["Navigation tag UUID is immutable."]);
      return;
    }
    if (primary.selection.kind === "navigation-tag" && path === "tag_id") {
      const nextTagId = Number(value);
      const tagIdError = validateNavigationTagIdChange(next.navigation_tags ?? [], obj.uuid, nextTagId);
      if (tagIdError) {
        setErrors([tagIdError]);
        return;
      }
    }

    if (path === "id" || path === "uuid") {
      const arr = getCollection(next, primary.selection.kind);
      if (arr.some((item) => item !== obj && objectId(primary.selection.kind, item) === value)) {
        setErrors([`Duplicate object id: ${value}`]);
        return;
      }
    }

    const oldId = objectId(primary.selection.kind, obj);
    setObjectProperty(obj, path, value);

    if (primary.selection.kind === "navigation-tag" && ["x", "y", "yaw"].includes(path)) {
      if (path === "x" || path === "y") {
        const snapped = snapPoint({ x: Number(obj.x), y: Number(obj.y) }, snapStep);
        obj.x = snapped.x;
        obj.y = snapped.y;
      }
      Object.assign(obj, applyNavigationTagPhysicalOverride(obj));
    }
    if (primary.selection.kind === "navigation-edge" && ["cost", "direction", "speed_limit", "enabled"].includes(path)) {
      obj.placement = "manual";
      obj.locked = true;
      if (path === "direction") obj.bidirectional = obj.direction === "bidirectional";
    }

    if (path === "id" && typeof value === "string" && value !== oldId) {
      setBlocks((current) =>
        current.map((block) => ({
          ...block,
          members: block.members.map((member) =>
            member.kind === primary.selection.kind && member.id === oldId
              ? { ...member, id: value }
              : member,
          ),
        })),
      );
      setSelected((current) =>
        current.map((member) =>
          member.kind === primary.selection.kind && member.id === oldId
            ? { ...member, id: value }
            : member,
        ),
      );
    }

    commit(next, "PROPERTY UPDATED · UNSAVED");
  }

  function duplicateSelections(selections: Selection[]) {
    if (!selections.length) return;
    const next = clone(draft);
    const newSelections: Selection[] = [];

    selections.forEach((selection) => {
      if (selection.kind === "navigation-edge") return;
      if (selection.kind === "column") {
        const index = Number(selection.id.split("-").pop() ?? "-1");
        const point = next.columns?.[index];
        if (point) {
          next.columns?.push([point[0] + 2, point[1] + 2]);
          newSelections.push({
            kind: "column",
            id: `COL-${(next.columns?.length ?? 1) - 1}`,
          });
        }
        return;
      }

      const arr = getCollection(next, selection.kind);
      const obj = arr.find((item) => objectId(selection.kind, item) === selection.id);
      if (!obj) return;

      const cp = clone(obj);
      const baseId = cp.id;
      let i = 1;
      let newId = `${baseId}-COPY`;
      while (arr.some((item) => item.id === newId)) {
        i += 1;
        newId = `${baseId}-COPY-${i}`;
      }
      cp.id = newId;

      if (Array.isArray(cp.position)) {
        cp.position = [
          cp.position[0] + 2,
          cp.position[1],
          cp.position[2] + 2,
        ];
      } else if (Array.isArray(cp.cell)) {
        cp.cell = [cp.cell[0] + 2, cp.cell[1] + 2];
      } else if (Array.isArray(cp.rect)) {
        cp.rect = [
          cp.rect[0] + 2,
          cp.rect[1] + 2,
          cp.rect[2] + 2,
          cp.rect[3] + 2,
        ];
      } else if (Array.isArray(cp.polygon)) {
        cp.polygon = cp.polygon.map((p: number[]) => [
          p[0] + 2,
          p[1] + 2,
        ]);
      } else if (Array.isArray(cp.path)) {
        cp.path = cp.path.map((p: number[]) => [
          p[0] + 2,
          p[1] + 2,
        ]);
      }

      arr.push(cp);
      newSelections.push({ kind: selection.kind, id: newId });
    });

    commit(next, "DUPLICATED · UNSAVED");
    setSelected(newSelections);
  }

  function duplicate() {
    duplicateSelections(selected);
  }

  const clipboardRef = useRef<any[]>([]);
  const editSnapshotRef = useRef<Draft | null>(null);

  function copySelected() {
    clipboardRef.current = selected.map((selection) => ({
      selection,
      obj:
        selection.kind === "column"
          ? null
          : clone(
              getCollection(draft, selection.kind).find(
                (item) => objectId(selection.kind, item) === selection.id,
              ),
            ),
    }));
    setStatus("COPIED");
  }

  function pasteSelected() {
    if (!clipboardRef.current.length) return;
    const next = clone(draft);
    const pasted: Selection[] = [];

    clipboardRef.current.forEach((entry) => {
      const { selection, obj } = entry;

      if (selection.kind === "column") {
        const index = Number(selection.id.split("-").pop() ?? "-1");
        const point = draft.columns?.[index];
        if (point) {
          next.columns?.push([point[0] + 2, point[1] + 2]);
          pasted.push({
            kind: "column",
            id: `COL-${(next.columns?.length ?? 1) - 1}`,
          });
        }
        return;
      }

      if (!obj) return;
      const arr = getCollection(next, selection.kind);
      const cp = clone(obj);
      const baseId = cp.id;
      let i = 1;
      let newId = `${baseId}-COPY`;
      while (arr.some((item) => item.id === newId)) {
        i += 1;
        newId = `${baseId}-COPY-${i}`;
      }
      cp.id = newId;

      if (Array.isArray(cp.position)) {
        cp.position = [
          cp.position[0] + 2,
          cp.position[1],
          cp.position[2] + 2,
        ];
      }
      arr.push(cp);
      pasted.push({ kind: selection.kind, id: newId });
    });

    commit(next, "PASTED · UNSAVED");
    setSelected(pasted);
  }

  function createArray() {
    const first = selected.find((item) => item.kind === "rack");
    if (!first) return;

    const source = getCollection(draft, "rack").find(
      (item) => item.id === first.id,
    );
    if (!source) return;

    const next = clone(draft);
    const arr = next.racks;
    const created: Selection[] = [];

    for (let row = 0; row < arrayRows; row += 1) {
      for (let col = 0; col < arrayCols; col += 1) {
        if (row === 0 && col === 0) continue;

        const cp = clone(source);
        cp.id = nextId(arr, "RACK");
        cp.position = [
          source.position[0] +
            col * (source.size[0] + arrayGapX),
          source.position[1],
          source.position[2] +
            row * (source.size[2] + arrayGapZ),
        ];
        arr.push(cp);
        created.push({ kind: "rack", id: cp.id });
      }
    }

    commit(next, `ARRAY ${arrayRows}×${arrayCols} · UNSAVED`);
    setSelected([first, ...created]);
    setArrayOpen(false);
  }

  type ArrangeMode =
    | "left" | "centerX" | "right"
    | "top" | "centerZ" | "bottom"
    | "distributeX" | "distributeZ";

  function selectionEntries(source: Draft, selections: Selection[]) {
    return selections.map((selection) => {
      if (selection.kind === "column") {
        const index = Number(selection.id.split("-").pop() ?? "-1");
        const point = source.columns?.[index];
        return point ? { selection, box: { x: point[0] - 0.45, z: point[1] - 0.45, w: 0.9, h: 0.9 } } : null;
      }
      const obj = getCollection(source, selection.kind).find((item) => objectId(selection.kind, item) === selection.id);
      return obj ? { selection, box: get2DBox(selection.kind, obj) } : null;
    }).filter(Boolean) as { selection: Selection; box: { x: number; z: number; w: number; h: number; rotation: number } }[];
  }

  function arrange(mode: ArrangeMode) {
    const editable = selected.filter((selection) => !lockedLayers[selection.kind]);
    if ((mode.startsWith("distribute") && editable.length < 3) || (!mode.startsWith("distribute") && editable.length < 2)) return;

    const entries = selectionEntries(draft, editable);
    if (entries.length < (mode.startsWith("distribute") ? 3 : 2)) return;
    const next = clone(draft);

    const targetX = mode === "left" ? Math.min(...entries.map((e) => e.box.x))
      : mode === "centerX" ? (Math.min(...entries.map((e) => e.box.x)) + Math.max(...entries.map((e) => e.box.x + e.box.w))) / 2
      : mode === "right" ? Math.max(...entries.map((e) => e.box.x + e.box.w)) : 0;
    const targetZ = mode === "top" ? Math.min(...entries.map((e) => e.box.z))
      : mode === "centerZ" ? (Math.min(...entries.map((e) => e.box.z)) + Math.max(...entries.map((e) => e.box.z + e.box.h))) / 2
      : mode === "bottom" ? Math.max(...entries.map((e) => e.box.z + e.box.h)) : 0;

    if (mode === "left" || mode === "centerX" || mode === "right") {
      entries.forEach(({ selection, box }) => {
        const cx = mode === "left" ? targetX + box.w / 2 : mode === "right" ? targetX - box.w / 2 : targetX;
        moveObject(next, selection, cx, box.z + box.h / 2);
      });
    } else if (mode === "top" || mode === "centerZ" || mode === "bottom") {
      entries.forEach(({ selection, box }) => {
        const cz = mode === "top" ? targetZ + box.h / 2 : mode === "bottom" ? targetZ - box.h / 2 : targetZ;
        moveObject(next, selection, box.x + box.w / 2, cz);
      });
    } else {
      const axis = mode === "distributeX" ? "x" : "z";
      const ordered = [...entries].sort((a, b) => axis === "x" ? (a.box.x + a.box.w / 2) - (b.box.x + b.box.w / 2) : (a.box.z + a.box.h / 2) - (b.box.z + b.box.h / 2));
      const first = ordered[0].box;
      const last = ordered[ordered.length - 1].box;
      const start = axis === "x" ? first.x + first.w / 2 : first.z + first.h / 2;
      const end = axis === "x" ? last.x + last.w / 2 : last.z + last.h / 2;
      const step = (end - start) / (ordered.length - 1);
      ordered.forEach((entry, index) => {
        if (index === 0 || index === ordered.length - 1) return;
        const center = start + step * index;
        moveObject(next, entry.selection, axis === "x" ? center : entry.box.x + entry.box.w / 2, axis === "z" ? center : entry.box.z + entry.box.h / 2);
      });
    }

    commit(next, `${mode.toUpperCase()} · UNSAVED`);
  }

  function onDrawPoint(point: Point) {
    setDrawingPoints((current) => [...current, point]);
  }

  function editFloorVertex(floorId: number | string, holeIndex: number | null, vertexIndex: number, point: Point) {
    const next = clone(draft);
    const floorItem = next.floors.find((item) => String(item.id) === String(floorId));
    if (!floorItem) return;
    const target = holeIndex == null ? floorItem.boundary : floorItem.holes?.[holeIndex];
    if (!target || vertexIndex < 0 || vertexIndex >= target.length) return;
    const converted = target.map((entry) => Array.isArray(entry) ? { x: entry[0], y: entry[1] } : entry);
    converted[vertexIndex] = point;
    const validation = holeIndex == null ? validateFloorPolygon(converted, `Floor ${floorId}`) : validateHolePolygon(converted, `Floor ${floorId} hole ${holeIndex + 1}`);
    if (validation.length) setErrors(validation); else setErrors([]);
    if (holeIndex == null) floorItem.boundary = converted; else if (floorItem.holes) floorItem.holes[holeIndex] = converted;
    begin2DEdit();
    setDraft(normalizeDraft(next));
    setStatus(validation.length ? "INVALID VERTEX EDIT" : "VERTEX EDITED · UNSAVED");
  }

  function editAisleVertex(aisleId: string, vertexIndex: number, point: Point) {
    const next = clone(draft), aisle = next.aisles?.find((item) => String(item.id) === String(aisleId));
    if (!aisle || vertexIndex < 0 || vertexIndex >= aisle.centerline.length) return;
    const centerline = aisle.centerline.map((entry) => ({ x: entry.x, y: entry.y })); centerline[vertexIndex] = point;
    const validation = validateAisleCenterline(centerline, aisle.width, `Aisle ${aisleId}`);
    if (validation.length) setErrors(validation); else setErrors([]);
    aisle.centerline = centerline;
    begin2DEdit(); setDraft(normalizeDraft(next)); setStatus(validation.length ? "INVALID AISLE EDIT" : "AISLE EDITED · UNSAVED");
  }

  function insertVertex(kind: "floor" | "aisle", id: string, holeIndex: number | null, vertexIndex: number, point: Point) {
    const next = clone(draft);
    if (kind === "aisle") {
      const aisle = next.aisles?.find((item) => String(item.id) === id); if (!aisle) return;
      aisle.centerline.splice(vertexIndex, 0, point);
    } else {
      const floorItem = next.floors.find((item) => String(item.id) === id); if (!floorItem) return;
      const target = holeIndex == null ? floorItem.boundary : floorItem.holes?.[holeIndex]; if (!target) return;
      const converted = target.map((entry) => Array.isArray(entry) ? { x: entry[0], y: entry[1] } : entry); converted.splice(vertexIndex, 0, point);
      if (holeIndex == null) floorItem.boundary = converted; else if (floorItem.holes) floorItem.holes[holeIndex] = converted;
    }
    commit(next, "VERTEX INSERTED · UNSAVED");
  }

  function deleteVertex(kind: "floor" | "aisle", id: string, holeIndex: number | null, vertexIndex: number) {
    const next = clone(draft);
    if (kind === "aisle") {
      const aisle = next.aisles?.find((item) => String(item.id) === id); if (!aisle || aisle.centerline.length <= 2) { setErrors(["An aisle must keep at least 2 centerline points."]); return; }
      aisle.centerline.splice(vertexIndex, 1);
    } else {
      const floorItem = next.floors.find((item) => String(item.id) === id); if (!floorItem) return;
      const target = holeIndex == null ? floorItem.boundary : floorItem.holes?.[holeIndex]; if (!target || target.length <= 3) { setErrors(["A polygon must keep at least 3 vertices."]); return; }
      const converted = target.map((entry) => Array.isArray(entry) ? { x: entry[0], y: entry[1] } : entry); converted.splice(vertexIndex, 1);
      if (holeIndex == null) floorItem.boundary = converted; else if (floorItem.holes) floorItem.holes[holeIndex] = converted;
    }
    commit(next, "VERTEX DELETED · UNSAVED"); setErrors([]);
  }

  function finishDrawing() {
    if (mode === "draw-aisle") {
      const geometryErrors = validateAisleCenterline(drawingPoints, 2, "aisle");
      if (geometryErrors.length) { setErrors(geometryErrors); return; }
      const next = clone(draft), active = next.floors.find((item) => String(item.id) === String(floor)) ?? next.floors[0];
      if (!active) return;
      const aisleId = nextAisleId(next.aisles ?? []);
      next.aisles = [...(next.aisles ?? []), { id: aisleId, floor_id: active.id, centerline: drawingPoints, width: 2, direction: "bidirectional", speed_limit: 0.8, tag_rule: { enabled: true, spacing: 2, start_offset: 0.5, end_offset: 0.5 } }];
      commit(next, "AISLE DRAWN · UNSAVED"); setSelected([{ kind: "aisle", id: aisleId }]); setDrawingPoints([]); setMode("select"); setErrors([]); return;
    }
    if (drawingPoints.length < 3) { setErrors(["A floor/hole polygon requires at least 3 points."]); return; }
    const geometryErrors = mode === "add-hole" ? validateHolePolygon(drawingPoints, "hole") : validateFloorPolygon(drawingPoints, "floor");
    if (geometryErrors.length) { setErrors(geometryErrors); return; }
    const next = clone(draft);
    const active = next.floors.find((item) => String(item.id) === String(floor)) ?? next.floors[0];
    if (!active) return;
    if (mode === "draw-floor") {
      active.boundary = drawingPoints.map((p) => ({ x: p.x, y: p.y }));
    } else if (mode === "add-hole") {
      const boundary = ((active.boundary ?? active.footprint ?? []) as Array<Point | [number, number]>).map((p) => Array.isArray(p) ? { x: p[0], y: p[1] } : p);
      if (!boundary.length || !drawingPoints.every((p) => pointInPolygon(p, boundary))) { setErrors(["Hole must be completely inside the floor boundary."]); return; }
      active.holes = [...(active.holes ?? []), drawingPoints.map((p) => ({ x: p.x, y: p.y }))];
    }
    commit(next, "GEOMETRY DRAWN · UNSAVED");
    setDrawingPoints([]);
    setMode("select");
    setErrors([]);
  }

  function regenerateTags() {
    const next = clone(draft);
    next.navigation_tags = generateAisleTags(next.aisles ?? [], next.navigation_tags ?? []);
    const tagErrors = validateNavigationTagsForLayout(next.floors, next.navigation_tags ?? []);
    if (tagErrors.length) { setErrors(tagErrors); setStatus("TAG REGENERATION ABORTED"); return; }
    setErrors([]); commit(next, "TAGS REGENERATED · UNSAVED");
  }

  function regenerateSelectedAisle() {
    const aisle = selected.find((item) => item.kind === "aisle");
    if (!aisle) return;
    const next = clone(draft); next.navigation_tags = generateAisleTags(next.aisles ?? [], next.navigation_tags ?? [], { onlyAisleId: aisle.id });
    const tagErrors = validateNavigationTagsForLayout(next.floors, next.navigation_tags ?? []);
    if (tagErrors.length) { setErrors(tagErrors); setStatus("TAG REGENERATION ABORTED"); return; }
    setErrors([]); commit(next, "SELECTED AISLE TAGS REGENERATED · UNSAVED");
  }

  function generateNavigationGraph() {
    const next = clone(draft);
    next.navigation_edges = generateNavigationEdges(next.aisles ?? [], next.navigation_tags ?? [], next.navigation_edges ?? []);
    const validation = validateNavigationEdges(next.navigation_tags ?? [], next.navigation_edges ?? [], next.aisles ?? []);
    if (validation.errors.length) {
      setErrors(validation.errors);
      setStatus("GRAPH GENERATION ABORTED");
      return;
    }
    setErrors(validation.warnings);
    commit(next, "NAVIGATION GRAPH GENERATED · UNSAVED");
  }

  function regenerateSelectedAisleGraph() {
    const selection = selected.find((item) => item.kind === "aisle");
    const aisle = selection && (draft.aisles ?? []).find((item) => String(item.id) === selection.id);
    if (!aisle) return;
    const next = clone(draft);
    const untouched = (next.navigation_edges ?? []).filter((edge) => edge.aisle_id !== aisle.id && !(edge.placement === "auto" && !edge.locked));
    const selectedEdges = generateNavigationEdges([aisle], next.navigation_tags ?? [], next.navigation_edges ?? []);
    next.navigation_edges = [...untouched, ...selectedEdges.filter((edge) => edge.aisle_id === aisle.id)];
    const validation = validateNavigationEdges(next.navigation_tags ?? [], next.navigation_edges ?? [], next.aisles ?? []);
    if (validation.errors.length) { setErrors(validation.errors); setStatus("GRAPH REGENERATION ABORTED"); return; }
    setErrors(validation.warnings); commit(next, "SELECTED AISLE GRAPH REGENERATED · UNSAVED");
  }

  function addManualNavigationEdge() {
    const tagSelections = selected.filter((item) => item.kind === "navigation-tag");
    const aisleSelection = selected.find((item) => item.kind === "aisle");
    if (tagSelections.length !== 2 || !aisleSelection) return;
    const aisle = (draft.aisles ?? []).find((item) => String(item.id) === aisleSelection.id);
    const tags = tagSelections.map((selection) => (draft.navigation_tags ?? []).find((tag) => tag.uuid === selection.id)).filter(Boolean) as NonNullable<Draft["navigation_tags"]>[number][];
    if (!aisle || tags.length !== 2 || String(tags[0].floor_id ?? 1) !== String(tags[1].floor_id ?? 1)) return;
    const generated = generateNavigationEdges([aisle], tags, draft.navigation_edges ?? []);
    const candidate = generated.find((edge) => edge.from_tag_uuid === tags[0].uuid && edge.to_tag_uuid === tags[1].uuid) ?? generated[0];
    if (!candidate || (draft.navigation_edges ?? []).some((edge) => edge.from_tag_uuid === candidate.from_tag_uuid && edge.to_tag_uuid === candidate.to_tag_uuid && edge.aisle_id === candidate.aisle_id)) return;
    const next = clone(draft);
    next.navigation_edges = [...(next.navigation_edges ?? []), { ...candidate, placement: "manual", locked: true }];
    const validation = validateNavigationEdges(next.navigation_tags ?? [], next.navigation_edges, next.aisles ?? []);
    if (validation.errors.length) { setErrors(validation.errors); return; }
    setErrors(validation.warnings); commit(next, "MANUAL NAVIGATION EDGE ADDED · UNSAVED");
  }


  function addManualTag(point?: Point) {
    const active = floor === "all" ? draft.floors[0] : draft.floors.find((item) => String(item.id) === String(floor));
    if (!active) return;
    const used = new Set((draft.navigation_tags ?? []).map((tag) => tag.tag_id)); let tagId = Math.max(1000, ...used) + 1; while (used.has(tagId)) tagId += 1;
    const tag = { id: `tag-manual-${tagId}`, uuid: `tag-manual-${tagId}`, tag_id: tagId, floor_id: active.id, x: point?.x ?? draft.size.width / 2, y: point?.y ?? draft.size.depth / 2, z: 0, yaw: 0, placement: "manual" as const, locked: true };
    const next = clone(draft); next.navigation_tags = [...(next.navigation_tags ?? []), tag]; commit(next, "MANUAL TAG ADDED · UNSAVED"); setSelected([{ kind: "navigation-tag", id: tag.uuid }]); setMode("select");
  }

  function validate(draftToCheck = draft) {
    const nextErrors: string[] = [];

    if (
      draftToCheck.size.width <= 0 ||
      draftToCheck.size.depth <= 0 ||
      draftToCheck.size.height <= 0
    ) {
      nextErrors.push("Warehouse dimensions must be positive");
    }

    const floorIds = new Set(draftToCheck.floors.map((item) => String(item.id)));
    draftToCheck.floors.forEach((item) => {
      const boundary = (item.boundary ?? item.footprint ?? []).map((p) => Array.isArray(p) ? { x: p[0], y: p[1] } : p);
      nextErrors.push(...validatePolygon(boundary, `Floor ${item.id}`));
      (item.holes ?? []).forEach((hole, index) => {
        const points = hole.map((p) => Array.isArray(p) ? { x: p[0], y: p[1] } : p);
        nextErrors.push(...validatePolygon(points, `Floor ${item.id} hole ${index + 1}`));
        if (boundary.length && !points.every((point) => pointInPolygon(point, boundary))) nextErrors.push(`Floor ${item.id} hole ${index + 1} is outside floor`);
      });
    });
    (draftToCheck.aisles ?? []).forEach((aisle) => {
      nextErrors.push(...validateAisleCenterline(aisle.centerline, aisle.width, `Aisle ${aisle.id}`));
      if (aisle.floor_id != null && !floorIds.has(String(aisle.floor_id))) nextErrors.push(`Aisle ${aisle.id} has invalid floor reference`);
      const floorItem = draftToCheck.floors.find((item) => String(item.id) === String(aisle.floor_id ?? 1));
      if (floorItem) {
        const boundary = (floorItem.boundary ?? floorItem.footprint ?? []).map((p) => Array.isArray(p) ? { x: p[0], y: p[1] } : p);
        const footprint = buildAisleFootprint(aisle.centerline, aisle.width);
        const holes = (floorItem.holes ?? []).map((hole) => hole.map((entry) => Array.isArray(entry) ? { x: entry[0], y: entry[1] } : entry));
        if (!polygonContainedInFloor(footprint, boundary, holes)) nextErrors.push(`Aisle ${aisle.id} width footprint is outside usable floor or intersects a hole`);
      }
    });
    nextErrors.push(...validateNavigationTagsForLayout(draftToCheck.floors, draftToCheck.navigation_tags ?? []));
    nextErrors.push(...validateNavigationEdges(draftToCheck.navigation_tags ?? [], draftToCheck.navigation_edges ?? [], draftToCheck.aisles ?? []).errors);

    const ids = new Set<string>();
    (Object.keys(KEY_MAP) as Kind[]).forEach((kind) => {
      if (kind === "column") return;
      const arr = getCollection(draftToCheck, kind);
      arr.forEach((obj: any) => {
        const identity = objectId(kind, obj);
        if (ids.has(identity)) nextErrors.push(`Duplicate object id: ${identity}`);
        ids.add(identity);

        const box = get2DBox(kind, obj);
        if (
          box.x < 0 ||
          box.z < 0 ||
          box.x + box.w > draftToCheck.size.width ||
          box.z + box.h > draftToCheck.size.depth
        ) {
          nextErrors.push(`${LABELS[kind]} ${obj.id} is outside warehouse`);
        }
      });
    });

    const rectObjects: { kind: Kind; id: string; box: ReturnType<typeof get2DBox> }[] = [];
    (Object.keys(KEY_MAP) as Kind[]).forEach((kind) => {
      if (kind === "column") return;
      getCollection(draftToCheck, kind).forEach((obj: any) => {
        // Floor containment, aisle footprints and tag placement have dedicated validators above;
        // they are not collidable physical rectangles.
        if (["floor", "aisle", "navigation-tag", "navigation-edge", "zone", "walkway", "camera", "sensor", "location", "spawn"].includes(kind)) return;
        rectObjects.push({
          kind,
          id: obj.id,
          box: get2DBox(kind, obj),
        });
      });
    });

    nextErrors.push(...validatePhysicalObjectOverlaps(rectObjects));

    (draftToCheck.walkways ?? []).forEach((item) => {
      if (item.speed_limit_mps <= 0) {
        nextErrors.push(`Walkway ${item.id} speed limit must be positive`);
      }
    });

    (draftToCheck.restricted_areas ?? []).forEach((item) => {
      if (item.robots_allowed) {
        nextErrors.push(
          `Restricted area ${item.id} allows robots; confirm this intentionally`,
        );
      }
    });

    setErrors(nextErrors);
    return nextErrors;
  }

  async function saveDraft() {
    setSaving(true);
    const response = await apiFetch("/api/layout/draft", {
      method: "PUT",
      headers: layoutRevision > 0 ? { "X-Layout-Revision": String(layoutRevision) } : undefined,
      body: JSON.stringify(draft),
    });
    setSaving(false);

    if (!response.ok) {
      setErrors([await response.text()]);
      return;
    }
    const payload = await response.json().catch(() => ({} as Record<string, unknown>));
    const revision = Number(payload.revision);
    if (Number.isFinite(revision)) useStore.setState({ layoutRevision: revision });
    if (Number.isFinite(Number(payload.published_version))) setPublishedVersion(Number(payload.published_version));
    setStatus("SAVED + SYNCED TO DB");
    setArtifactStatus("");
  }

  function applyCode() {
    try {
      const parsed = JSON.parse(code) as Draft;
      const nextErrors = validate(parsed);
      if (nextErrors.length) return;

      commit(parsed, "CODE APPLIED · UNSAVED");
      setErrors([]);
      setView("2d");
    } catch (error) {
      setErrors([
        error instanceof Error ? error.message : String(error),
      ]);
    }
  }

  async function publish() {
    const nextErrors = validate();
    if (nextErrors.length) return;

    setSaving(true);
    const response = await apiFetch("/api/layout/publish", {
      method: "POST",
      headers: layoutRevision > 0 ? { "X-Layout-Revision": String(layoutRevision) } : undefined,
      body: JSON.stringify(draft),
    });
    setSaving(false);

    if (!response.ok) {
      setErrors([await response.text()]);
      return;
    }
    const payload = await response.json().catch(() => ({} as Record<string, unknown>));
    const revision = Number(payload.revision);
    const version = Number(payload.published_version ?? payload.version);
    if (Number.isFinite(revision)) useStore.setState({ layoutRevision: revision });
    if (Number.isFinite(version)) setPublishedVersion(version);
    setStatus(Number.isFinite(version) ? `PUBLISHED v${version}` : "PUBLISHED");
    setArtifactStatus(payload.artifacts ? "ROS configs + Gazebo generated" : "");
    // The backend replaces the runtime engine on publish; fetch the authoritative FULL state immediately.
    wsSend({ type: "RESYNC" });
  }

  function resetTemplate() {
    editSnapshotRef.current = null;
    setHistory((h) => [...h.slice(-39), clone(draft)]);
    setFuture([]);
    setDraft(normalizeDraft(base));
    setSelected([]);
    setErrors([]);
    setStatus("RESET TO TEMPLATE");
  }

  function on3DDragState(active: boolean) {
    setDragging(active);
  }

  function on3DCommit(point: Vec3) {
    if (!selected.length) return;

    const next = clone(draft);
    const bounds = selectionBounds(next, selected);
    if (!bounds) return;

    selected.forEach((selection) => {
      const obj =
        selection.kind === "column"
          ? null
          : getCollection(next, selection.kind).find(
              (item) => objectId(selection.kind, item) === selection.id,
            );

      if (selection.kind === "column") {
        const index = Number(selection.id.split("-").pop() ?? "-1");
        if (next.columns?.[index]) {
          next.columns[index] = [
            snapPoint({ x: point[0], y: point[2] }, snapStep).x,
            snapPoint({ x: point[0], y: point[2] }, snapStep).y,
          ];
        }
        return;
      }

      if (!obj) return;

      const b = get2DBox(selection.kind, obj);
      const dx = point[0] - bounds.centerX;
      const dz = point[2] - bounds.centerZ;
      moveObject(
        next,
        selection,
        b.x + b.w / 2 + dx,
        b.z + b.h / 2 + dz,
        snapStep,
      );
    });

    commit(next, "MOVED · UNSAVED");
  }

  function on2DMove(selection: Selection, x: number, z: number) {
    if (dragging) return;
    begin2DEdit();

    const next = clone(draft);

    if (
      selected.length > 1 &&
      selected.some(
        (item) =>
          item.kind === selection.kind &&
          item.id === selection.id,
      )
    ) {
      const clicked = getCollection(next, selection.kind).find(
        (item) => objectId(selection.kind, item) === selection.id,
      );

      if (clicked) {
        const clickedBox = get2DBox(selection.kind, clicked);
        const dx = x - (clickedBox.x + clickedBox.w / 2);
        const dz = z - (clickedBox.z + clickedBox.h / 2);

        selected.forEach((item) => {
          if (item.kind === "column") {
            const index = Number(item.id.split("-").pop() ?? "-1");
            if (next.columns?.[index]) {
              next.columns[index] = [
                snapPoint({ x: next.columns[index][0] + dx, y: next.columns[index][1] + dz }, snapStep).x,
                snapPoint({ x: next.columns[index][0] + dx, y: next.columns[index][1] + dz }, snapStep).y,
              ];
            }
            return;
          }

          const obj = getCollection(next, item.kind).find(
            (entry) => objectId(item.kind, entry) === item.id,
          );
          if (!obj) return;

          const box = get2DBox(item.kind, obj);
          moveObject(
            next,
            item,
            box.x + box.w / 2 + dx,
            box.z + box.h / 2 + dz,
            snapStep,
          );
        });
      }
    } else {
      moveObject(next, selection, x, z, snapStep);
    }

    setDraft(normalizeDraft(next));
    setStatus("MOVING · UNSAVED");
  }

  function commit2DPointer() {
    if (!editSnapshotRef.current) return;
    setHistory((current) => [...current.slice(-39), clone(editSnapshotRef.current as Draft)]);
    setFuture([]);
    editSnapshotRef.current = null;
    setStatus("MOVED · UNSAVED");
  }

  function onRotate(selection: Selection, angle: number) {
    begin2DEdit();
    const next = clone(draft);
    setObjectRotation(next, selection, angle);
    setDraft(normalizeDraft(next));
    setStatus("ROTATING · UNSAVED");
  }

  function onResize(
    selection: Selection,
    handle: "nw" | "ne" | "sw" | "se",
    x: number,
    z: number,
  ) {
    begin2DEdit();
    const next = clone(draft);
    resizeObject(next, selection, handle, x, z);
    setDraft(normalizeDraft(next));
    setStatus("RESIZING · UNSAVED");
  }

  function onMeasurePoint(point: { x: number; z: number }) {
    if (settingReferenceOrigin) {
      setBackgroundOffsetX(-point.x); setBackgroundOffsetY(-point.z); setSettingReferenceOrigin(false); setMode("select"); setStatus("REFERENCE ORIGIN SET"); return;
    }
    setMeasurePoints((current) => {
      const next = current.length >= 2 ? [point] : [...current, point];

      if (calibratingBackground && next.length === 2) {
        const distance = Math.hypot(
          next[1].x - next[0].x,
          next[1].z - next[0].z,
        );

        const raw = window.prompt(
          "Real distance represented by this line (meters):",
          distance > 0 ? distance.toFixed(2) : "10",
        );

        const realDistance = raw ? Number(raw) : NaN;

        if (Number.isFinite(realDistance) && realDistance > 0 && distance > 0) {
          setBackgroundScale((currentScale) =>
            currentScale * (realDistance / distance),
          );
          setStatus("FLOOR PLAN CALIBRATED");
        }

        setCalibratingBackground(false);
      }

      return next;
    });
  }

  function rotateView(delta: number) {
    setViewRotation((rotation) => (rotation + delta + 360) % 360);
  }

  function toggleLayer(kind: Kind) {
    setVisibleLayers((current) => ({
      ...current,
      [kind]: !current[kind],
    }));
    setSelected((current) =>
      current.filter((selection) => selection.kind !== kind),
    );
  }

  function toggleLayerLock(kind: Kind) {
    setLockedLayers((current) => ({
      ...current,
      [kind]: !current[kind],
    }));
    setSelected((current) =>
      current.filter((selection) => selection.kind !== kind),
    );
    setStatus(`${LABELS[kind].toUpperCase()} LAYER ${lockedLayers[kind] ? "UNLOCKED" : "LOCKED"}`);
  }

  function moveLayer(kind: Kind, delta: -1 | 1) {
    setLayerOrder((current) => {
      const next = [...current];
      const index = next.indexOf(kind);
      if (index < 0) return current;
      const target = index + delta;
      if (target < 0 || target >= next.length) return current;
      [next[index], next[target]] = [next[target], next[index]];
      return next;
    });
    setStatus(`${LAYER_LABELS[kind].toUpperCase()} LAYER REORDERED`);
  }

  function isolateLayer(kind: Kind) {
    setVisibleLayers(() =>
      Object.fromEntries(
        Object.keys(COLORS).map((key) => [key, key === kind]),
      ) as Record<Kind, boolean>,
    );
    setSelected((current) =>
      current.filter((selection) => selection.kind === kind),
    );
    setStatus(`ISOLATING ${LAYER_LABELS[kind].toUpperCase()}`);
  }

  function showAllLayers() {
    setVisibleLayers(
      Object.fromEntries(
        Object.keys(COLORS).map((key) => [key, true]),
      ) as Record<Kind, boolean>,
    );
    setStatus("ALL LAYERS VISIBLE");
  }

  function lockAllLayers() {
    setLockedLayers(
      Object.fromEntries(
        Object.keys(COLORS).map((key) => [key, true]),
      ) as Record<Kind, boolean>,
    );
    setSelected([]);
    setStatus("ALL LAYERS LOCKED");
  }

  function unlockAllLayers() {
    setLockedLayers(
      Object.fromEntries(
        Object.keys(COLORS).map((key) => [key, false]),
      ) as Record<Kind, boolean>,
    );
    setStatus("ALL LAYERS UNLOCKED");
  }

  function createBlock() {
    if (selected.length < 2) return;

    if (selected.some((selection) => lockedLayers[selection.kind])) {
      setStatus("UNLOCK SELECTED LAYERS BEFORE CREATING A BLOCK");
      return;
    }

    const alreadyGrouped = selected.filter((selection) =>
      blocks.some((block) =>
        block.members.some((member) => selectionKey(member) === selectionKey(selection)),
      ),
    );

    if (alreadyGrouped.length) {
      setStatus("SOME OBJECTS ALREADY BELONG TO A BLOCK");
      return;
    }

    const id = `BLOCK-${String(blocks.length + 1).padStart(3, "0")}`;
    const name = window.prompt("Block name:", `Block ${blocks.length + 1}`)?.trim();
    if (!name) return;

    const nextBlock: EditorBlock = {
      id,
      name,
      members: clone(selected),
    };

    setBlocks((current) => [...current, nextBlock]);
    setActiveBlockId(null);
    setStatus(`${name.toUpperCase()} CREATED`);
  }

  function getActiveBlock() {
    return activeBlockId
      ? blocks.find((block) => block.id === activeBlockId) ?? null
      : null;
  }

  function enterBlock() {
    if (selected.length !== 1) return;
    const block = findBlockForSelection(selected[0]);
    if (!block) return;
    setActiveBlockId(block.id);
    setSelected([selected[0]]);
    setStatus(`EDITING BLOCK · ${block.name}`);
  }

  function exitBlock() {
    if (!activeBlockId) return;
    const block = getActiveBlock();
    setActiveBlockId(null);
    if (block) setSelected(block.members);
    setStatus("BLOCK EDIT MODE EXITED");
  }

  function ungroupBlock() {
    const target = activeBlockId
      ? getActiveBlock()
      : selected.length === 1
        ? findBlockForSelection(selected[0])
        : null;

    if (!target) return;

    setBlocks((current) => current.filter((block) => block.id !== target.id));
    setActiveBlockId(null);
    setSelected(target.members);
    setStatus(`${target.name.toUpperCase()} UNGROUPED`);
  }

  function renameBlock() {
    const target = activeBlockId
      ? getActiveBlock()
      : selected.length === 1
        ? findBlockForSelection(selected[0])
        : null;

    if (!target) return;
    const name = window.prompt("Block name:", target.name)?.trim();
    if (!name || name === target.name) return;

    setBlocks((current) =>
      current.map((block) => block.id === target.id ? { ...block, name } : block),
    );
    setStatus("BLOCK RENAMED");
  }

  function focusContextSelection(selection: Selection | null) {
    if (!selection) return;
    setSelected([selection]);
    setContextMenu(null);
    setStatus(`FOCUS ${selection.id}`);
  }

  function reorderObjects(selections: Selection[], direction: "front" | "back") {
    const next = clone(draft);

    const editableSelections = selections.filter((selection) => !lockedLayers[selection.kind]);
    const grouped = new Map<Kind, string[]>();
    editableSelections.forEach((selection) => {
      const key = selection.kind;
      const list = grouped.get(key) ?? [];
      list.push(selection.id);
      grouped.set(key, list);
    });

    grouped.forEach((ids, kind) => {
      if (kind === "column") return;
      const arr = getCollection(next, kind);
      if (!Array.isArray(arr)) return;

      const selectedSet = new Set(ids);
      const selectedObjects = arr.filter((obj: any) => selectedSet.has(String(obj.id)));
      const remaining = arr.filter((obj: any) => !selectedSet.has(String(obj.id)));

      (next as any)[KEY_MAP[kind] as keyof Draft] =
        direction === "front"
          ? [...remaining, ...selectedObjects]
          : [...selectedObjects, ...remaining];
    });

    commit(
      next,
      direction === "front" ? "BROUGHT TO FRONT · UNSAVED" : "SENT TO BACK · UNSAVED",
    );
  }

  function runContextAction(action: ContextAction) {
    const selection = contextMenu?.selection;
    if (!selection) { setContextMenu(null); return; }
    setSelected([selection]);
    if (action === "focus") {
      focusContextSelection(selection);
      return;
    }
    if (action === "rotateLeft") rotateObjectBy([selection], -15);
    if (action === "rotateRight") rotateObjectBy([selection], 15);
    if (action === "duplicate") duplicateSelections([selection]);
    if (action === "delete") removeSelections([selection]);
    if (action === "bringFront") reorderObjects([selection], "front");
    if (action === "sendBack") reorderObjects([selection], "back");
    if (action === "group") {
      setContextMenu(null);
      createBlock();
      return;
    }
    if (action === "enterBlock") {
      setContextMenu(null);
      enterBlock();
      return;
    }
    if (action === "ungroup") {
      setContextMenu(null);
      ungroupBlock();
      return;
    }
    if (action === "renameBlock") {
      setContextMenu(null);
      renameBlock();
      return;
    }
    setContextMenu(null);
  }

  function rotateObjectBy(selections: Selection[], delta: number) {
    const next = clone(draft);
    selections.forEach((selection) => rotateObject(next, selection, delta));
    commit(next, `ROTATED ${delta > 0 ? "CW" : "CCW"} · UNSAVED`);
  }

  function importBackground(event: React.ChangeEvent<HTMLInputElement>) {
    const file = event.target.files?.[0];
    if (!file) return;

    if (!file.type.startsWith("image/")) {
      setErrors(["Floor plan must be an image file (PNG/JPG/SVG)."]);
      return;
    }

    const reader = new FileReader();
    reader.onload = () => {
      setBackground(typeof reader.result === "string" ? reader.result : null);
      setStatus("FLOOR PLAN LOADED");
    };
    reader.readAsDataURL(file);
    event.target.value = "";
  }

  const measurement =
    measurePoints.length === 2
      ? Math.hypot(
          measurePoints[1].x - measurePoints[0].x,
          measurePoints[1].z - measurePoints[0].z,
        )
      : 0;

  const primaryBox = primary
    ? get2DBox(primary.selection.kind, primary.obj)
    : null;

  const resultSearch =
    search.trim().length === 0
      ? []
      : (Object.keys(KEY_MAP) as Kind[])
          .flatMap((kind) =>
            getCollection(draft, kind)
              .filter((obj: any) =>
                String(obj.id)
                  .toLowerCase()
                  .includes(search.toLowerCase()),
              )
              .slice(0, 8)
              .map((obj: any) => ({
                kind,
                id: obj.id,
              })),
          );

  return (
    <div
      className="flex h-screen min-h-0 flex-col overflow-hidden bg-[#05080f] text-slate-200"
      onPointerDownCapture={(event) => {
        const target = event.target as HTMLElement;
        if (!target.closest("[data-context-menu]") && contextMenu && event.button !== 2) setContextMenu(null);
      }}
    >
      {/* Header */}
      <header className="flex min-h-[72px] shrink-0 items-center justify-between border-b border-white/[0.08] bg-[#080d16] px-4 lg:px-5">
        <div className="flex min-w-0 items-center gap-4">
          <div className="min-w-0">
            <div className="text-[9px] font-bold uppercase tracking-[0.18em] text-slate-600">
              ADMIN / CONFIGURATION
            </div>
            <div className="mt-0.5 truncate text-lg font-semibold tracking-tight text-white">
              Warehouse Layout & Simulation Designer
            </div>
          </div>

          <div className="hidden items-center gap-2 rounded-lg border border-white/[0.06] bg-white/[0.025] px-2.5 py-1.5 xl:flex">
            <span className="h-1.5 w-1.5 rounded-full bg-emerald-400 shadow-[0_0_8px_rgba(52,211,153,.7)]" />
            <span className="text-[9px] font-semibold uppercase tracking-[0.12em] text-emerald-300/80">
              {status} · draft r{layoutRevision}{publishedVersion !== null ? ` · published v${publishedVersion}` : ""}{activeWarehouseId ? ` · WH#${activeWarehouseId}` : ""}
              {artifactStatus ? ` · ${artifactStatus}` : ""}
            </span>
          </div>

          {activeBlockId && (
            <button
              type="button"
              onClick={exitBlock}
              className="hidden items-center gap-2 rounded-lg border border-violet-400/15 bg-violet-400/[0.05] px-2.5 py-1.5 text-[9px] text-violet-300 xl:flex"
              title="Exit block editing"
            >
              <span className="h-1.5 w-1.5 rounded-full bg-violet-400" />
              {getActiveBlock()?.name ?? "BLOCK"}
              <span className="text-violet-400/60">· editing</span>
            </button>
          )}
        </div>

        <div className="flex items-center gap-1.5">
          <button
            type="button"
            onClick={undo}
            disabled={!history.length}
            className="rounded-lg border border-white/[0.07] bg-white/[0.025] px-2.5 py-2 text-xs text-slate-400 transition hover:bg-white/[0.06] disabled:cursor-not-allowed disabled:opacity-30"
            title="Undo"
          >
            ↶
          </button>
          <button
            type="button"
            onClick={redo}
            disabled={!future.length}
            className="rounded-lg border border-white/[0.07] bg-white/[0.025] px-2.5 py-2 text-xs text-slate-400 transition hover:bg-white/[0.06] disabled:cursor-not-allowed disabled:opacity-30"
            title="Redo"
          >
            ↷
          </button>

          <div className="mx-1 hidden h-7 w-px bg-white/[0.08] sm:block" />

          <button
            type="button"
            onClick={resetTemplate}
            className="rounded-lg border border-white/[0.07] bg-white/[0.025] px-3 py-2 text-[10px] font-medium text-slate-400 transition hover:bg-white/[0.06] hover:text-white"
          >
            Reset
          </button>
          <button
            type="button"
            onClick={() => validate()}
            className="rounded-lg border border-amber-400/10 bg-amber-400/[0.05] px-3 py-2 text-[10px] font-semibold text-amber-300 transition hover:bg-amber-400/[0.09]"
          >
            Validate
          </button>
          <button
            type="button"
            disabled={saving}
            onClick={() => void saveDraft()}
            className="rounded-lg border border-sky-400/15 bg-sky-400/[0.07] px-3 py-2 text-[10px] font-semibold text-sky-300 transition hover:bg-sky-400/[0.11] disabled:opacity-50"
          >
            {saving ? "Saving…" : "Save & Sync"}
          </button>
          <button
            type="button"
            disabled={saving}
            onClick={() => void publish()}
            className="rounded-lg border border-emerald-400/15 bg-emerald-400/[0.08] px-3 py-2 text-[10px] font-semibold text-emerald-300 transition hover:bg-emerald-400/[0.12] disabled:opacity-50"
          >
            Publish
          </button>
          <button
            type="button"
            onClick={() => { window.history.pushState({}, "", "/admin/warehouse"); window.dispatchEvent(new PopStateEvent("popstate")); }}
            className="ml-1 rounded-lg border border-white/[0.07] px-3 py-2 text-[10px] text-slate-400 transition hover:bg-white/[0.04] hover:text-white"
          >
            Warehouse Data
          </button>
          <button
            type="button"
            onClick={() => { window.history.pushState({}, "", "/"); window.dispatchEvent(new PopStateEvent("popstate")); }}
            className="rounded-lg border border-emerald-400/15 bg-emerald-400/[0.05] px-3 py-2 text-[10px] text-emerald-300 transition hover:bg-emerald-400/[0.1]"
          >
            Live Map
          </button>
        </div>
      </header>

      {/* Main */}
      <div className="grid min-h-0 flex-1 grid-cols-[230px_minmax(0,1fr)_300px]">
        {/* Left */}
        <aside className="min-h-0 overflow-y-auto border-r border-white/[0.08] bg-[#080d16]">
          <div className="border-b border-white/[0.07] p-3">
            <div className="grid grid-cols-3 gap-1 rounded-lg border border-white/[0.06] bg-black/10 p-1">
              {([
                ["2d", "2D MAP"],
                ["3d", "3D VIEW"],
                ["code", "CODE"],
              ] as const).map(([key, label]) => (
                <button
                  key={key}
                  type="button"
                  onClick={() => setView(key)}
                  className={[
                    "rounded-md px-2 py-2 text-[9px] font-semibold transition",
                    view === key
                      ? "bg-sky-400/10 text-sky-300"
                      : "text-slate-600 hover:text-slate-300",
                  ].join(" ")}
                >
                  {label}
                </button>
              ))}
            </div>
          </div>

          <div className="space-y-4 p-3">
            <div>
              <div className="mb-1 px-1 text-[9px] font-bold uppercase tracking-[0.16em] text-slate-600">
                Tools
              </div>
              <ToolButton
                active={mode === "select"}
                label="Select / Multi-select"
                icon="⌖"
                onClick={() => {
                  setMode("select");
                  setMeasurePoints([]);
                }}
              />
              <ToolButton
                active={mode === "move"}
                label="Move"
                icon="✥"
                onClick={() => setMode("move")}
              />
              <ToolButton
                active={mode === "measure"}
                label="Measure"
                icon="↔"
                onClick={() => {
                  setMode("measure");
                  setMeasurePoints([]);
                }}
              />
              <ToolButton active={mode === "draw-floor"} label="Draw Floor" icon="⌂" onClick={() => { setMode("draw-floor"); setDrawingPoints([]); setErrors([]); }} />
              <ToolButton active={mode === "edit-vertex"} label="Edit Vertex" icon="◇" onClick={() => setMode("edit-vertex")} />
              <ToolButton active={mode === "add-hole"} label="Add Hole" icon="□" onClick={() => { setMode("add-hole"); setDrawingPoints([]); setErrors([]); }} />
              <ToolButton active={mode === "draw-aisle"} label="Draw Aisle" icon="╱" onClick={() => { setMode("draw-aisle"); setDrawingPoints([]); setErrors([]); }} />
              <ToolButton active={mode === "edit-aisle"} label="Edit Aisle" icon="✎" onClick={() => setMode("edit-aisle")} />
              <ToolButton active={mode === "add-manual-tag"} label="Add Manual Tag" icon="●" onClick={() => { setMode("add-manual-tag"); setStatus("MANUAL TAG · CLICK MAP"); }} />
              <button type="button" onClick={regenerateTags} className="mt-1 w-full rounded-md border border-violet-400/20 bg-violet-400/[0.06] px-2 py-2 text-left text-[9px] font-semibold text-violet-300 hover:bg-violet-400/[0.12]">Regenerate All Tags</button>
              <button type="button" disabled={!selected.some((item) => item.kind === "aisle")} onClick={regenerateSelectedAisle} className="mt-1 w-full rounded-md border border-violet-400/15 px-2 py-2 text-left text-[9px] text-violet-300 hover:bg-violet-400/[0.08] disabled:opacity-30">Regenerate Selected Aisle</button>
              <button type="button" onClick={generateNavigationGraph} className="mt-1 w-full rounded-md border border-fuchsia-400/20 bg-fuchsia-400/[0.06] px-2 py-2 text-left text-[9px] font-semibold text-fuchsia-300 hover:bg-fuchsia-400/[0.12]">Generate Navigation Graph</button>
              <button type="button" disabled={!selected.some((item) => item.kind === "aisle")} onClick={regenerateSelectedAisleGraph} className="mt-1 w-full rounded-md border border-fuchsia-400/15 px-2 py-2 text-left text-[9px] text-fuchsia-300 hover:bg-fuchsia-400/[0.08] disabled:opacity-30">Regenerate Selected Aisle Graph</button>
              <button type="button" disabled={selected.filter((item) => item.kind === "navigation-tag").length !== 2 || !selected.some((item) => item.kind === "aisle")} onClick={addManualNavigationEdge} className="mt-1 w-full rounded-md border border-fuchsia-400/15 px-2 py-2 text-left text-[9px] text-fuchsia-300 hover:bg-fuchsia-400/[0.08] disabled:opacity-30">Add Manual Edge (2 Tags)</button>
              <div className="mt-1 grid grid-cols-2 gap-1">
                <button
                  type="button"
                  onClick={() => rotateView(-90)}
                  className="rounded-md border border-white/[0.06] bg-white/[0.02] px-2 py-2 text-[9px] text-slate-500 hover:bg-white/[0.05] hover:text-white"
                >
                  ↺ View
                </button>
                <button
                  type="button"
                  onClick={() => rotateView(90)}
                  className="rounded-md border border-white/[0.06] bg-white/[0.02] px-2 py-2 text-[9px] text-slate-500 hover:bg-white/[0.05] hover:text-white"
                >
                  ↻ View
                </button>
              </div>
            </div>

            <div>
              <div className="mb-1 px-1 text-[9px] font-bold uppercase tracking-[0.16em] text-slate-600">
                Build
              </div>
              {GROUPS.map((group) => (
                <div key={group.title} className="mb-2">
                  <div className="px-1 pb-1 text-[8px] font-semibold uppercase tracking-[0.14em] text-slate-700">
                    {group.title}
                  </div>
                  {group.kinds.map((kind) => (
                    <button
                      type="button"
                      key={kind}
                      onClick={() => add(kind)}
                      className="group flex w-full items-center gap-2 rounded-lg px-2 py-2 text-left hover:bg-white/[0.04]"
                    >
                      <span
                        className="grid h-7 w-7 shrink-0 place-items-center rounded-md border text-[10px]"
                        style={{
                          borderColor: `${COLORS[kind]}33`,
                          background: `${COLORS[kind]}10`,
                          color: COLORS[kind],
                        }}
                      >
                        {ICONS[kind]}
                      </span>
                      <span className="min-w-0 flex-1 truncate text-[10px] text-slate-400 group-hover:text-slate-200">
                        {LABELS[kind]}
                      </span>
                      <span className="text-slate-700">+</span>
                    </button>
                  ))}
                </div>
              ))}
            </div>

            <div>
              <div className="mb-1 px-1 text-[9px] font-bold uppercase tracking-[0.16em] text-slate-600">
                Reference
              </div>
              <button
                type="button"
                onClick={() => fileInputRef.current?.click()}
                className="flex w-full items-center gap-2 rounded-lg border border-white/[0.06] bg-white/[0.02] px-2 py-2 text-left text-[10px] text-slate-400 hover:bg-white/[0.04] hover:text-white"
              >
                <span className="grid h-7 w-7 place-items-center rounded-md bg-slate-400/5">
                  ▧
                </span>
                Import floor plan
              </button>
              <input
                ref={fileInputRef}
                type="file"
                accept="image/png,image/jpeg,image/svg+xml"
                className="hidden"
                onChange={importBackground}
              />

              {background && (
                <div className="mt-2 rounded-lg border border-white/[0.06] bg-white/[0.02] p-2">
                  <div className="mb-2 flex items-center justify-between">
                    <span className="text-[9px] font-medium text-slate-500">
                      Background opacity
                    </span>
                    <span className="font-mono text-[9px] text-slate-400">
                      {Math.round(backgroundOpacity * 100)}%
                    </span>
                  </div>
                  <input
                    type="range"
                    min="0"
                    max="1"
                    step="0.05"
                    value={backgroundOpacity}
                    onChange={(e) =>
                      setBackgroundOpacity(Number(e.target.value))
                    }
                    className="w-full accent-sky-400"
                  />
                  <div className="mt-2 flex items-center justify-between text-[9px] text-slate-600">
                    <span>Scale</span>
                    <span className="font-mono text-slate-400">{backgroundScale.toFixed(2)}×</span>
                  </div>
                  <input type="range" min="0.1" max="3" step="0.05" value={backgroundScale} onChange={(e) => setBackgroundScale(Number(e.target.value))} className="w-full accent-sky-400" />
                  <div className="mt-2 grid grid-cols-3 gap-1"><PropertyInput label="Translate X" type="number" step="0.1" value={backgroundOffsetX} onChange={(value) => setBackgroundOffsetX(Number(value) || 0)} /><PropertyInput label="Translate Y" type="number" step="0.1" value={backgroundOffsetY} onChange={(value) => setBackgroundOffsetY(Number(value) || 0)} /><PropertyInput label="Rotate" type="number" step="1" value={backgroundRotation} onChange={(value) => setBackgroundRotation(Number(value) || 0)} /></div>
                  <button
                    type="button"
                    onClick={() => {
                      setCalibratingBackground(true);
                      setMeasurePoints([]);
                      setMode("measure");
                      setStatus("CALIBRATE · PICK TWO POINTS");
                    }}
                    className="mt-2 w-full rounded-md border border-sky-400/10 bg-sky-400/[0.04] px-2 py-1.5 text-[9px] font-medium text-sky-300 hover:bg-sky-400/[0.08]"
                  >
                    Calibrate with 2 points
                  </button>
                  <button type="button" onClick={() => { setSettingReferenceOrigin(true); setMode("measure"); setStatus("REFERENCE ORIGIN · PICK (0,0)"); }} className="mt-2 w-full rounded-md border border-sky-400/10 px-2 py-1.5 text-[9px] text-sky-300 hover:bg-sky-400/[0.08]">Set Reference Origin</button>
                  <button
                    type="button"
                    onClick={() => {
                      setBackground(null);
                      setBackgroundScale(1);
                      setBackgroundOffsetX(0); setBackgroundOffsetY(0); setBackgroundRotation(0);
                      setCalibratingBackground(false);
                    }}
                    className="mt-2 w-full rounded-md px-2 py-1.5 text-[9px] text-rose-300 hover:bg-rose-500/[0.07]"
                  >
                    Remove reference
                  </button>
                </div>
              )}
            </div>

            <div>
              <div className="mb-1 flex items-center justify-between px-1">
                <span className="text-[9px] font-bold uppercase tracking-[0.16em] text-slate-600">
                  Layers
                </span>
                <span className="text-[8px] text-slate-700">
                  {layerOrder.length}
                </span>
              </div>

              <div className="mb-2 grid grid-cols-2 gap-1">
                <button
                  type="button"
                  onClick={showAllLayers}
                  className="rounded-md border border-white/[0.06] px-2 py-1.5 text-[8px] text-slate-500 hover:bg-white/[0.04] hover:text-slate-200"
                >
                  Show all
                </button>
                <button
                  type="button"
                  onClick={unlockAllLayers}
                  className="rounded-md border border-white/[0.06] px-2 py-1.5 text-[8px] text-slate-500 hover:bg-white/[0.04] hover:text-slate-200"
                >
                  Unlock all
                </button>
                <button
                  type="button"
                  onClick={lockAllLayers}
                  className="rounded-md border border-amber-400/[0.10] bg-amber-400/[0.03] px-2 py-1.5 text-[8px] text-amber-300/80 hover:bg-amber-400/[0.07] hover:text-amber-200"
                >
                  Lock all
                </button>
              </div>

              <div className="mb-2 rounded-md border border-white/[0.05] bg-white/[0.015] px-2 py-1.5 text-[8px] leading-4 text-slate-700">
                <div>● isolate · 🔒 lock · ↑↓ reorder</div>
                <div>Alt+Click cycles objects under cursor.</div>
              </div>
              <div className="space-y-0.5">
                {layerOrder.map((kind, index) => (
                  <LayerRow
                    key={kind}
                    kind={kind}
                    visible={visibleLayers[kind]}
                    locked={lockedLayers[kind]}
                    count={getCollection(draft, kind).length}
                    isTop={index === layerOrder.length - 1}
                    isBottom={index === 0}
                    onToggle={() => toggleLayer(kind)}
                    onToggleLock={() => toggleLayerLock(kind)}
                    onMove={(delta) => moveLayer(kind, delta)}
                    onIsolate={() => isolateLayer(kind)}
                  />
                ))}
              </div>
            </div>

            <div>
              <div className="mb-1 flex items-center justify-between px-1">
                <span className="text-[9px] font-bold uppercase tracking-[0.16em] text-slate-600">
                  Blocks
                </span>
                <span className="text-[8px] text-slate-700">
                  {blocks.length}
                </span>
              </div>

              <div className="space-y-1">
                {blocks.length === 0 ? (
                  <div className="rounded-lg border border-dashed border-white/[0.06] p-2 text-[9px] text-slate-700">
                    Select 2+ objects and create a block.
                  </div>
                ) : (
                  blocks.map((block) => {
                    const active = activeBlockId === block.id;
                    return (
                      <div
                        key={block.id}
                        className={[
                          "rounded-lg border p-2 transition-colors",
                          active
                            ? "border-violet-400/20 bg-violet-400/[0.06]"
                            : "border-white/[0.06] bg-white/[0.015]",
                        ].join(" ")}
                      >
                        <div className="flex items-center gap-2">
                          <button
                            type="button"
                            onClick={() => {
                              setActiveBlockId(block.id);
                              setSelected(block.members);
                              setStatus(`BLOCK SELECTED · ${block.name}`);
                            }}
                            className="min-w-0 flex-1 text-left"
                          >
                            <div className="truncate text-[10px] font-semibold text-slate-300">
                              {block.name}
                            </div>
                            <div className="mt-0.5 text-[8px] text-slate-600">
                              {block.members.length} members · {block.id}
                            </div>
                          </button>
                          <button
                            type="button"
                            onClick={() => {
                              setActiveBlockId(block.id);
                              setSelected(block.members);
                              setStatus(`EDITING BLOCK · ${block.name}`);
                            }}
                            className="rounded-md px-1.5 py-1 text-[9px] text-violet-300 hover:bg-violet-400/10"
                            title="Edit block"
                          >
                            ↗
                          </button>
                        </div>
                      </div>
                    );
                  })
                )}
              </div>

              <div className="mt-2 grid grid-cols-2 gap-1">
                <button
                  type="button"
                  disabled={selected.length < 2}
                  onClick={createBlock}
                  className="rounded-md border border-violet-400/15 bg-violet-400/[0.05] px-2 py-1.5 text-[8px] font-semibold text-violet-300 hover:bg-violet-400/[0.09] disabled:opacity-30"
                >
                  Create block
                </button>
                <button
                  type="button"
                  disabled={!activeBlockId}
                  onClick={exitBlock}
                  className="rounded-md border border-white/[0.06] px-2 py-1.5 text-[8px] text-slate-500 hover:bg-white/[0.04] disabled:opacity-30"
                >
                  Exit block
                </button>
              </div>
            </div>

            <div>
              <div className="mb-1 px-1 text-[9px] font-bold uppercase tracking-[0.16em] text-slate-600">
                Navigation
              </div>
              <div className="rounded-lg border border-white/[0.06] bg-white/[0.018] p-2.5 text-[9px] leading-5 text-slate-500">
                <div>
                  Grid <span className="float-right font-mono text-slate-400">{draft.grid.cell_size} m</span>
                </div>
                <label className="flex items-center justify-between gap-2">
                  <span>Snap</span>
                  <select value={snapStep ?? "off"} onChange={(event) => setSnapStep(event.target.value === "off" ? null : Number(event.target.value))} className="rounded border border-white/[0.08] bg-[#070d16] px-1 py-0.5 font-mono text-[9px] text-emerald-300 outline-none">
                    <option value="off">OFF</option><option value="0.01">0.01 m</option><option value="0.05">0.05 m</option><option value="0.1">0.10 m</option><option value="0.5">0.50 m</option><option value="1">1.00 m</option>
                  </select>
                </label>
                <div>
                  Origin <span className="float-right font-mono text-slate-400">0, 0</span>
                </div>
                <div>
                  Orientation <span className="float-right font-mono text-slate-400">{viewRotation}°</span>
                </div>
              </div>
            </div>
          </div>
        </aside>

        {/* Center */}
        <main className="relative min-w-0 bg-[#05080f]">
          {/* toolbar */}
          <div className="absolute inset-x-0 top-0 z-20 flex h-12 items-center justify-between border-b border-white/[0.06] bg-[#080d16]/90 px-3 backdrop-blur-xl">
            <div className="flex items-center gap-1.5">
              <button
                type="button"
                onClick={() => setMode("select")}
                className={[
                  "rounded-md border px-2.5 py-1.5 text-[9px] font-semibold",
                  mode === "select"
                    ? "border-sky-400/20 bg-sky-400/[0.08] text-sky-300"
                    : "border-white/[0.06] text-slate-500",
                ].join(" ")}
              >
                SELECT
              </button>
              <button
                type="button"
                onClick={() => setMode("move")}
                disabled={!selected.length}
                className={[
                  "rounded-md border px-2.5 py-1.5 text-[9px] font-semibold",
                  mode === "move"
                    ? "border-sky-400/20 bg-sky-400/[0.08] text-sky-300"
                    : "border-white/[0.06] text-slate-500",
                  !selected.length ? "opacity-30" : "",
                ].join(" ")}
              >
                MOVE
              </button>
              <button
                type="button"
                onClick={() => rotateSelection(-15)}
                disabled={!selected.length}
                className="rounded-md border border-white/[0.06] px-2.5 py-1.5 text-[9px] text-slate-500 hover:bg-white/[0.04] hover:text-white disabled:opacity-30"
              >
                ↺ 15°
              </button>
              <button
                type="button"
                onClick={() => rotateSelection(15)}
                disabled={!selected.length}
                className="rounded-md border border-white/[0.06] px-2.5 py-1.5 text-[9px] text-slate-500 hover:bg-white/[0.04] hover:text-white disabled:opacity-30"
              >
                ↻ 15°
              </button>
              <button
                type="button"
                onClick={() => reorderObjects(selected, "front")}
                disabled={!selected.length}
                className="rounded-md border border-white/[0.06] px-2.5 py-1.5 text-[9px] text-slate-500 hover:bg-white/[0.04] hover:text-white disabled:opacity-30"
              >
                Front
              </button>
              <button
                type="button"
                onClick={() => reorderObjects(selected, "back")}
                disabled={!selected.length}
                className="rounded-md border border-white/[0.06] px-2.5 py-1.5 text-[9px] text-slate-500 hover:bg-white/[0.04] hover:text-white disabled:opacity-30"
              >
                Back
              </button>
              <button
                type="button"
                onClick={() => createBlock()}
                disabled={selected.length < 2}
                className="rounded-md border border-violet-400/10 bg-violet-400/[0.03] px-2.5 py-1.5 text-[9px] text-violet-300 hover:bg-violet-400/[0.07] disabled:opacity-30"
              >
                Block
              </button>
              <button
                type="button"
                onClick={() => duplicate()}
                disabled={!selected.length}
                className="rounded-md border border-white/[0.06] px-2.5 py-1.5 text-[9px] text-slate-500 hover:bg-white/[0.04] hover:text-white disabled:opacity-30"
              >
                Duplicate
              </button>
              <button
                type="button"
                onClick={() => {
                  if (selected.some((x) => x.kind === "rack")) {
                    setArrayOpen(true);
                  }
                }}
                disabled={!selected.some((x) => x.kind === "rack")}
                className="rounded-md border border-white/[0.06] px-2.5 py-1.5 text-[9px] text-slate-500 hover:bg-white/[0.04] hover:text-white disabled:opacity-30"
              >
                Array
              </button>
              <div className="relative">
                <details className="group">
                  <summary className="list-none cursor-pointer rounded-md border border-white/[0.06] px-2.5 py-1.5 text-[9px] text-slate-400 hover:bg-white/[0.04] hover:text-white">Arrange ▾</summary>
                  <div className="absolute left-0 top-full z-50 mt-1 w-[190px] rounded-lg border border-white/[0.08] bg-[#0b1220]/[0.98] p-1.5 shadow-2xl backdrop-blur-xl">
                    <div className="px-2 py-1 text-[8px] font-bold uppercase tracking-[0.14em] text-slate-600">Align</div>
                    {[["left","Left"],["centerX","Center X"],["right","Right"],["top","Top"],["centerZ","Center Z"],["bottom","Bottom"]].map(([value,label]) => (
                      <button key={value} type="button" disabled={selected.length < 2} onClick={(e) => { arrange(value as ArrangeMode); (e.currentTarget.closest("details") as HTMLDetailsElement | null)?.removeAttribute("open"); }} className="block w-full rounded-md px-2 py-1.5 text-left text-[9px] text-slate-400 hover:bg-white/[0.05] hover:text-white disabled:opacity-30">{label}</button>
                    ))}
                    <div className="my-1 h-px bg-white/[0.06]" />
                    <div className="px-2 py-1 text-[8px] font-bold uppercase tracking-[0.14em] text-slate-600">Distribute</div>
                    <button type="button" disabled={selected.length < 3} onClick={(e) => { arrange("distributeX"); (e.currentTarget.closest("details") as HTMLDetailsElement | null)?.removeAttribute("open"); }} className="block w-full rounded-md px-2 py-1.5 text-left text-[9px] text-slate-400 hover:bg-white/[0.05] hover:text-white disabled:opacity-30">Horizontal</button>
                    <button type="button" disabled={selected.length < 3} onClick={(e) => { arrange("distributeZ"); (e.currentTarget.closest("details") as HTMLDetailsElement | null)?.removeAttribute("open"); }} className="block w-full rounded-md px-2 py-1.5 text-left text-[9px] text-slate-400 hover:bg-white/[0.05] hover:text-white disabled:opacity-30">Vertical</button>
                  </div>
                </details>
              </div>

              <button
                type="button"
                onClick={() => setSmartGuidesEnabled((value) => !value)}
                className={[
                  "rounded-md border px-2.5 py-1.5 text-[9px]",
                  smartGuidesEnabled
                    ? "border-sky-400/20 bg-sky-400/[0.08] text-sky-300"
                    : "border-white/[0.06] text-slate-600",
                ].join(" ")}
                title="Snap to nearby object edges and centers"
              >
                Guides {smartGuidesEnabled ? "ON" : "OFF"}
              </button>

              <button
                type="button"
                onClick={remove}
                disabled={!selected.length}
                className="rounded-md border border-rose-400/10 bg-rose-400/[0.03] px-2.5 py-1.5 text-[9px] text-rose-300 hover:bg-rose-400/[0.07] disabled:opacity-30"
              >
                Delete
              </button>
            </div>

            <div className="flex items-center gap-2">
              {search.length > 0 && resultSearch.length > 0 && (
                <div className="hidden max-w-[250px] items-center gap-1.5 md:flex">
                  {resultSearch.slice(0, 2).map((item) => (
                    <button
                      key={`${item.kind}-${item.id}`}
                      type="button"
                      onClick={() => selectObject(item, false)}
                      className="rounded-md border border-white/[0.06] bg-white/[0.025] px-2 py-1 text-[9px] text-slate-400 hover:text-white"
                    >
                      {item.id}
                    </button>
                  ))}
                </div>
              )}

              <select
                value={floor}
                onChange={(e) =>
                  setFloor(
                    e.target.value === "all"
                      ? "all"
                      : (draft.floors.find((item) => String(item.id) === e.target.value)?.id ?? e.target.value),
                  )
                }
                className="rounded-md border border-white/[0.08] bg-[#0b1220] px-2.5 py-1.5 text-[9px] text-slate-300 outline-none"
              >
                {draft.floors.map((item) => (
                  <option key={item.id} value={item.id}>
                    {item.name}
                  </option>
                ))}
                <option value="all">All Floors</option>
              </select>

              <div className="hidden items-center gap-1.5 rounded-md border border-white/[0.06] bg-white/[0.02] px-2 py-1.5 lg:flex">
                <span className="h-1.5 w-1.5 rounded-full bg-sky-400" />
                <span className="text-[9px] text-slate-500">
                  {dragging ? "DRAGGING · CAMERA LOCKED" : mode === "measure" ? "MEASURE MODE" : "READY"}
                </span>
              </div>
            </div>
          </div>

          <div className="absolute inset-0 pt-12">
            {view === "2d" && (
              <Editor2DMap
                draft={draft}
                selected={selected}
                floor={floor}
                viewRotation={viewRotation}
                visibleLayers={visibleLayers}
                lockedLayers={lockedLayers}
                layerOrder={layerOrder}
                blocks={blocks}
                activeBlockId={activeBlockId}
                background={background}
                backgroundOpacity={backgroundOpacity}
                backgroundScale={backgroundScale}
                backgroundOffsetX={backgroundOffsetX}
                backgroundOffsetY={backgroundOffsetY}
                backgroundRotation={backgroundRotation}
                mode={mode}
                measurePoints={measurePoints}
                drawingPoints={drawingPoints}
                onSelect={selectObject}
                onMove={on2DMove}
                onCommitMove={commit2DPointer}
                onResize={onResize}
                onRotate={onRotate}
                onMeasurePoint={onMeasurePoint}
                onDrawPoint={onDrawPoint}
                onFinishDrawing={finishDrawing}
                onAddManualTag={addManualTag}
                onEditVertex={editFloorVertex}
                onEditAisleVertex={editAisleVertex}
                onInsertVertex={insertVertex}
                onDeleteVertex={deleteVertex}
                snapStep={snapStep}
                onBoxSelect={onBoxSelect}
                onContextMenu={(selection, x, y) => setContextMenu({ selection, x, y })}
                smartGuidesEnabled={smartGuidesEnabled}
              />
            )}

            {view === "3d" && (
              <EditorScene3D
                draft={draft}
                selected={selected}
                onSelect={selectObject}
                onCommit={on3DCommit}
                onDragState={on3DDragState}
                floor={floor}
                visibleLayers={visibleLayers}
                lockedLayers={lockedLayers}
                layerOrder={layerOrder}
                snapStep={snapStep}
              />
            )}

            {view === "code" && (
              <div className="flex h-full flex-col gap-2 bg-[#070d16] p-3">
                <div className="flex items-center justify-between rounded-lg border border-white/[0.06] bg-black/10 px-3 py-2">
                  <div>
                    <div className="text-[10px] font-semibold text-slate-300">
                      warehouse.layout.json
                    </div>
                    <div className="mt-0.5 text-[9px] text-slate-600">
                      Declarative layout · validated before apply
                    </div>
                  </div>
                  <div className="flex items-center gap-1.5">
                    <button
                      type="button"
                      onClick={() => setCode(JSON.stringify(draft, null, 2))}
                      className="rounded-md border border-white/[0.06] px-2.5 py-1.5 text-[9px] text-slate-500 hover:bg-white/[0.04] hover:text-white"
                    >
                      Format
                    </button>
                    <button
                      type="button"
                      onClick={applyCode}
                      className="rounded-md border border-sky-400/15 bg-sky-400/[0.07] px-2.5 py-1.5 text-[9px] font-semibold text-sky-300"
                    >
                      Apply JSON
                    </button>
                  </div>
                </div>

                <textarea
                  spellCheck={false}
                  value={code}
                  onChange={(e) => setCode(e.target.value)}
                  className="min-h-0 flex-1 resize-none rounded-lg border border-white/[0.06] bg-[#05080f] p-4 font-mono text-[11px] leading-5 text-slate-300 outline-none focus:border-sky-400/25"
                />
              </div>
            )}
          </div>

          {/* lower status */}
          <div className="absolute bottom-3 left-3 z-20 flex flex-wrap items-center gap-2">
            <div className="rounded-md border border-white/[0.07] bg-[#080d16]/90 px-2.5 py-1.5 text-[9px] text-slate-500 backdrop-blur-xl">
              {view === "2d" ? "TOP VIEW" : view === "3d" ? "PERSPECTIVE" : "JSON"}
            </div>
            {view === "2d" && (
              <div className="rounded-md border border-white/[0.07] bg-[#080d16]/90 px-2.5 py-1.5 text-[9px] text-slate-500 backdrop-blur-xl">
                Grid {draft.grid.cell_size}m · Snap {snapStep == null ? "OFF" : `${snapStep}m`} · {viewRotation}°
              </div>
            )}
            {measurement > 0 && (
              <div className="rounded-md border border-amber-400/15 bg-amber-400/[0.06] px-2.5 py-1.5 text-[9px] text-amber-300 backdrop-blur-xl">
                Measure {measurement.toFixed(2)} m
              </div>
            )}
          </div>

          {errors.length > 0 && (
            <div className="absolute bottom-3 right-3 z-30 max-h-[220px] w-[min(520px,calc(100%-24px))] overflow-auto rounded-xl border border-rose-400/20 bg-[#260c12]/95 p-3 shadow-2xl backdrop-blur-xl">
              <div className="mb-2 text-[10px] font-bold uppercase tracking-[0.12em] text-rose-300">
                Layout validation
              </div>
              <div className="space-y-1 text-[10px] leading-4 text-rose-200/85">
                {errors.map((error) => (
                  <div key={error}>• {error}</div>
                ))}
              </div>
            </div>
          )}

          {contextMenu && (
            <div
              data-context-menu
              className="fixed z-[200] w-52 overflow-hidden rounded-xl border border-white/[0.10] bg-[#0b1220]/95 p-1.5 text-slate-200 shadow-2xl backdrop-blur-xl"
              style={{ left: Math.min(contextMenu.x, window.innerWidth - 228), top: Math.min(contextMenu.y, window.innerHeight - 260) }}
              onPointerDown={(event) => event.stopPropagation()}
            >
              <div className="px-2.5 pb-1.5 pt-1 text-[9px] font-bold uppercase tracking-[0.16em] text-slate-600">
                {contextMenu.selection ? contextMenu.selection.id : "Canvas"}
              </div>
              {([
                ["focus", "Focus selection"],
                ["bringFront", "Bring to front"],
                ["sendBack", "Send to back"],
                ["rotateLeft", "Rotate left 15°"],
                ["rotateRight", "Rotate right 15°"],
                ["duplicate", "Duplicate"],
                ...(selected.length > 1 ? [["group", "Create block"] as [ContextAction, string]] : []),
                ...(contextMenu.selection && findBlockForSelection(contextMenu.selection)
                  ? [
                      ["enterBlock", "Edit block"] as [ContextAction, string],
                      ["renameBlock", "Rename block"] as [ContextAction, string],
                      ["ungroup", "Ungroup block"] as [ContextAction, string],
                    ]
                  : []),
                ["delete", "Delete"],
              ] as [ContextAction, string][]).map(([action, label]) => (
                <button
                  key={action}
                  type="button"
                  className={[
                    "flex w-full items-center rounded-lg px-2.5 py-2 text-left text-[11px] transition-colors",
                    action === "delete" ? "text-rose-300 hover:bg-rose-500/10" : "text-slate-300 hover:bg-white/[0.06] hover:text-white",
                  ].join(" ")}
                  onClick={() => runContextAction(action)}
                >
                  {label}
                </button>
              ))}
            </div>
          )}
        </main>

        {/* Right */}
        <aside className="min-h-0 overflow-y-auto border-l border-white/[0.08] bg-[#080d16]">
          <div className="border-b border-white/[0.07] p-3">
            <div className="relative">
              <input
                value={search}
                onChange={(e) => setSearch(e.target.value)}
                placeholder="Search object ID…"
                className="w-full rounded-lg border border-white/[0.08] bg-white/[0.025] px-3 py-2.5 text-[10px] text-slate-200 outline-none placeholder:text-slate-700 focus:border-sky-400/25"
              />
              {search && resultSearch.length > 0 && (
                <div className="absolute left-0 right-0 top-[calc(100%+4px)] z-50 overflow-hidden rounded-lg border border-white/[0.08] bg-[#0b1220] shadow-xl">
                  {resultSearch.map((item) => (
                    <button
                      type="button"
                      key={`${item.kind}-${item.id}`}
                      onClick={() => {
                        selectObject(item, false);
                        setSearch("");
                      }}
                      className="flex w-full items-center gap-2 px-3 py-2 text-left hover:bg-white/[0.04]"
                    >
                      <span
                        className="h-2 w-2 rounded-full"
                        style={{ background: COLORS[item.kind] }}
                      />
                      <span className="text-[10px] text-slate-300">
                        {item.id}
                      </span>
                    </button>
                  ))}
                </div>
              )}
            </div>
          </div>

          <div className="space-y-4 p-3">
            {/* selection */}
            <div>
              <div className="mb-2 flex items-center justify-between">
                <span className="text-[9px] font-bold uppercase tracking-[0.16em] text-slate-600">
                  Selection
                </span>
                <span className="font-mono text-[9px] text-slate-500">
                  {selected.length}
                </span>
              </div>

              {primary ? (
                <div className="rounded-xl border border-white/[0.07] bg-white/[0.025] p-3">
                  <div className="flex items-center gap-2">
                    <span
                      className="grid h-9 w-9 place-items-center rounded-lg border text-xs"
                      style={{
                        borderColor: `${COLORS[primary.selection.kind]}33`,
                        background: `${COLORS[primary.selection.kind]}10`,
                        color: COLORS[primary.selection.kind],
                      }}
                    >
                      {ICONS[primary.selection.kind]}
                    </span>
                    <div className="min-w-0 flex-1">
                      <div className="text-[12px] font-semibold text-white">
                        {primary.obj.id ?? primary.obj.uuid}
                      </div>
                      <div className="mt-0.5 text-[9px] uppercase tracking-[0.08em] text-slate-600">
                        {LABELS[primary.selection.kind]}
                      </div>
                    </div>
                    {selected.length > 1 && (
                      <span className="rounded-md bg-sky-400/10 px-1.5 py-1 text-[9px] text-sky-300">
                        +{selected.length - 1}
                      </span>
                    )}
                  </div>
                </div>
              ) : (
                <div className="rounded-xl border border-dashed border-white/[0.07] p-4 text-center text-[10px] text-slate-600">
                  Select an object in the map.
                  <div className="mt-1 text-[9px] text-slate-700">
                    Shift/Ctrl + click for multi-select.
                  </div>
                </div>
              )}
            </div>

            {/* Transform */}
            {primary && (
              <div>
                <div className="mb-2 text-[9px] font-bold uppercase tracking-[0.16em] text-slate-600">
                  Transform & Properties
                </div>

                <div className="space-y-2 rounded-xl border border-white/[0.07] bg-white/[0.018] p-3">
                  {primary.selection.kind === "navigation-tag" ? (
                    <div className="text-[9px] text-slate-500">UUID <span className="float-right max-w-[72%] truncate font-mono text-slate-300" title={primary.obj.uuid}>{primary.obj.uuid}</span></div>
                  ) : (
                    <PropertyInput
                      label="ID"
                      value={objectId(primary.selection.kind, primary.obj)}
                      onChange={(value) => updateProperty("id", value)}
                    />
                  )}

                  {primary.selection.kind === "aisle" && (
                    <>
                      <PropertyInput label="Width" type="number" step="0.01" value={primary.obj.width} onChange={(value) => updateProperty("width", value)} />
                      <label className="block"><span className="mb-1 block text-[9px] font-medium uppercase tracking-[0.08em] text-slate-500">Direction</span><select value={primary.obj.direction ?? "bidirectional"} onChange={(event) => updateProperty("direction", event.target.value)} className="w-full rounded-md border border-white/[0.08] bg-[#070d16] px-2.5 py-2 text-[11px] text-slate-200 outline-none"><option value="bidirectional">Bidirectional</option><option value="forward">Forward</option><option value="reverse">Reverse</option></select></label>
                      <PropertyInput label="Speed Limit" type="number" step="0.01" value={primary.obj.speed_limit ?? 0.8} onChange={(value) => updateProperty("speed_limit", value)} />
                      <div className="grid grid-cols-2 gap-2"><PropertyInput label="Tag Spacing" type="number" step="0.01" value={primary.obj.tag_rule?.spacing ?? 2} onChange={(value) => updateProperty("tag_rule.spacing", value)} /><PropertyInput label="Start Offset" type="number" step="0.01" value={primary.obj.tag_rule?.start_offset ?? 0} onChange={(value) => updateProperty("tag_rule.start_offset", value)} /></div>
                      <PropertyInput label="End Offset" type="number" step="0.01" value={primary.obj.tag_rule?.end_offset ?? 0} onChange={(value) => updateProperty("tag_rule.end_offset", value)} />
                      <label className="flex items-center justify-between text-[10px] text-slate-400"><span>Auto Tags</span><input type="checkbox" checked={primary.obj.tag_rule?.enabled !== false} onChange={(event) => updateProperty("tag_rule.enabled", event.target.checked ? "true" : "false")} /></label>
                    </>
                  )}

                  {primary.selection.kind === "navigation-tag" && (
                    <>
                      <PropertyInput label="Tag ID" type="number" step="1" value={primary.obj.tag_id} onChange={(value) => updateProperty("tag_id", value)} />
                      <div className="grid grid-cols-2 gap-2"><PropertyInput label="X" type="number" step="0.01" value={primary.obj.x} onChange={(value) => updateProperty("x", value)} /><PropertyInput label="Y" type="number" step="0.01" value={primary.obj.y} onChange={(value) => updateProperty("y", value)} /></div>
                      <div className="grid grid-cols-2 gap-2"><PropertyInput label="Z" type="number" step="0.01" value={primary.obj.z ?? 0} onChange={(value) => updateProperty("z", value)} /><PropertyInput label="Yaw" type="number" step="0.01" value={primary.obj.yaw ?? 0} onChange={(value) => updateProperty("yaw", value)} /></div>
                      <label className="flex items-center justify-between text-[10px] text-slate-400"><span>Locked</span><input type="checkbox" checked={Boolean(primary.obj.locked)} onChange={(event) => updateProperty("locked", event.target.checked ? "true" : "false")} /></label>
                      <div className="text-[9px] text-slate-500">Placement <span className="float-right font-mono text-slate-300">{String(primary.obj.placement ?? "auto").toUpperCase()}</span></div>
                      <div className="text-[9px] text-slate-500">Generated From <span className="float-right font-mono text-slate-300">{primary.obj.generated_from ?? "—"}</span></div>
                    </>
                  )}

                  {primary.selection.kind === "navigation-edge" && (
                    <>
                      <div className="text-[9px] text-slate-500">From <span className="float-right font-mono text-slate-300">{primary.obj.from_tag_id ?? primary.obj.from_tag_uuid}</span></div>
                      <div className="text-[9px] text-slate-500">To <span className="float-right font-mono text-slate-300">{primary.obj.to_tag_id ?? primary.obj.to_tag_uuid}</span></div>
                      <div className="text-[9px] text-slate-500">Aisle <span className="float-right font-mono text-slate-300">{primary.obj.aisle_id}</span></div>
                      <PropertyInput label="Distance" type="number" step="0.01" value={primary.obj.distance} onChange={(value) => updateProperty("distance", value)} />
                      <PropertyInput label="Cost" type="number" step="0.01" value={primary.obj.cost} onChange={(value) => updateProperty("cost", value)} />
                      <label className="block"><span className="mb-1 block text-[9px] font-medium uppercase tracking-[0.08em] text-slate-500">Direction</span><select value={primary.obj.direction} onChange={(event) => updateProperty("direction", event.target.value)} className="w-full rounded-md border border-white/[0.08] bg-[#070d16] px-2.5 py-2 text-[11px] text-slate-200 outline-none"><option value="bidirectional">Bidirectional</option><option value="forward">Forward</option><option value="reverse">Reverse</option></select></label>
                      <PropertyInput label="Speed Limit" type="number" step="0.01" value={primary.obj.speed_limit ?? ""} onChange={(value) => updateProperty("speed_limit", value)} />
                      <label className="flex items-center justify-between text-[10px] text-slate-400"><span>Enabled</span><input type="checkbox" checked={primary.obj.enabled !== false} onChange={(event) => updateProperty("enabled", event.target.checked ? "true" : "false")} /></label>
                      <div className="text-[9px] text-slate-500">Placement <span className="float-right font-mono text-slate-300">{String(primary.obj.placement ?? "auto").toUpperCase()}</span></div>
                      <label className="flex items-center justify-between text-[10px] text-slate-400"><span>Locked</span><input type="checkbox" checked={Boolean(primary.obj.locked)} onChange={(event) => updateProperty("locked", event.target.checked ? "true" : "false")} /></label>
                    </>
                  )}

                  {primary.selection.kind === "rack" && (
                    <>
                      <div className="grid grid-cols-2 gap-2">
                        <PropertyInput
                          label="X"
                          type="number"
                          step="0.5"
                          value={primary.obj.position[0]}
                          onChange={(value) =>
                            updateProperty("position.0", value)
                          }
                        />
                        <PropertyInput
                          label="Z"
                          type="number"
                          step="0.5"
                          value={primary.obj.position[2]}
                          onChange={(value) =>
                            updateProperty("position.2", value)
                          }
                        />
                      </div>
                      <div className="grid grid-cols-2 gap-2">
                        <PropertyInput
                          label="Width"
                          type="number"
                          step="0.1"
                          value={primary.obj.size[0]}
                          onChange={(value) =>
                            updateProperty("size.0", value)
                          }
                        />
                        <PropertyInput
                          label="Depth"
                          type="number"
                          step="0.1"
                          value={primary.obj.size[2]}
                          onChange={(value) =>
                            updateProperty("size.2", value)
                          }
                        />
                      </div>
                      <div className="grid grid-cols-2 gap-2">
                        <PropertyInput
                          label="Height"
                          type="number"
                          step="0.1"
                          value={primary.obj.size[1]}
                          onChange={(value) =>
                            updateProperty("size.1", value)
                          }
                        />
                        <PropertyInput
                          label="Rotation"
                          type="number"
                          step="15"
                          value={primary.obj.rotation}
                          onChange={(value) =>
                            updateProperty("rotation", value)
                          }
                        />
                      </div>
                    </>
                  )}

                  {["charger", "spawn"].includes(
                    primary.selection.kind,
                  ) && (
                    <>
                      <div className="grid grid-cols-2 gap-2">
                        <PropertyInput
                          label="X"
                          type="number"
                          step="0.5"
                          value={primary.obj.position[0]}
                          onChange={(value) =>
                            updateProperty("position.0", value)
                          }
                        />
                        <PropertyInput
                          label="Z"
                          type="number"
                          step="0.5"
                          value={primary.obj.position[2]}
                          onChange={(value) =>
                            updateProperty("position.2", value)
                          }
                        />
                      </div>
                      <PropertyInput
                        label={primary.selection.kind === "spawn" ? "Heading" : "Heading"}
                        type="number"
                        step="15"
                        value={primary.obj.heading}
                        onChange={(value) =>
                          updateProperty("heading", value)
                        }
                      />
                    </>
                  )}

                  {primary.selection.kind === "charger" && (
                    <PropertyInput
                      label="Power kW"
                      type="number"
                      step="0.5"
                      value={primary.obj.power_kw}
                      onChange={(value) =>
                        updateProperty("power_kw", value)
                      }
                    />
                  )}

                  {primary.selection.kind === "zone" && (
                    <PropertyInput
                      label="Name"
                      value={primary.obj.name}
                      onChange={(value) =>
                        updateProperty("name", value)
                      }
                    />
                  )}

                  {primary.selection.kind === "dock" && (
                    <PropertyInput
                      label="Type"
                      value={primary.obj.kind}
                      onChange={(value) =>
                        updateProperty("kind", value)
                      }
                    />
                  )}

                  {primary.selection.kind === "restricted" && (
                    <div className="rounded-lg border border-rose-400/10 bg-rose-400/[0.04] p-2">
                      <div className="text-[9px] font-semibold text-rose-300">
                        Navigation restriction
                      </div>
                      <div className="mt-1 text-[9px] text-slate-500">
                        Robots should normally be blocked in this area.
                      </div>
                    </div>
                  )}

                  {primaryBox && (
                    <div className="grid grid-cols-2 gap-2 pt-1 text-[9px] text-slate-500">
                      <div>
                        Position
                        <div className="mt-0.5 font-mono text-slate-300">
                          {primaryBox.x.toFixed(1)}, {primaryBox.z.toFixed(1)}
                        </div>
                      </div>
                      <div>
                        Size
                        <div className="mt-0.5 font-mono text-slate-300">
                          {primaryBox.w.toFixed(1)} × {primaryBox.h.toFixed(1)}
                        </div>
                      </div>
                    </div>
                  )}
                </div>
              </div>
            )}

            {/* Array */}
            {selected.some((item) => item.kind === "rack") && (
              <div className="rounded-xl border border-violet-400/10 bg-violet-400/[0.03] p-3">
                <div className="flex items-center justify-between">
                  <div>
                    <div className="text-[10px] font-semibold text-violet-200">
                      Rack Array
                    </div>
                    <div className="mt-0.5 text-[9px] text-slate-600">
                      Create a repeated rack pattern.
                    </div>
                  </div>
                  <button
                    type="button"
                    onClick={() => setArrayOpen(true)}
                    className="rounded-md border border-violet-400/15 bg-violet-400/[0.06] px-2 py-1.5 text-[9px] text-violet-300"
                  >
                    Configure
                  </button>
                </div>
              </div>
            )}

            {/* Layout info */}
            <div>
              <div className="mb-2 text-[9px] font-bold uppercase tracking-[0.16em] text-slate-600">
                Warehouse
              </div>
              <div className="space-y-1.5 rounded-xl border border-white/[0.07] bg-white/[0.018] p-3 text-[9px] text-slate-500">
                <div className="flex justify-between">
                  <span>Dimensions</span>
                  <span className="font-mono text-slate-300">
                    {draft.size.width} × {draft.size.depth} × {draft.size.height} m
                  </span>
                </div>
                <div className="flex justify-between">
                  <span>Floors</span>
                  <span className="font-mono text-slate-300">
                    {draft.floors.length}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span>Grid</span>
                  <span className="font-mono text-slate-300">
                    {draft.grid.cols} × {draft.grid.rows}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span>Selection</span>
                  <span className="font-mono text-slate-300">
                    {selected.length}
                  </span>
                </div>
              </div>
            </div>

            {/* shortcuts */}
            <div>
              <div className="mb-2 text-[9px] font-bold uppercase tracking-[0.16em] text-slate-600">
                Shortcuts
              </div>
              <div className="rounded-xl border border-white/[0.07] bg-white/[0.018] p-3 text-[9px] leading-5 text-slate-500">
                <div><kbd className="text-slate-300">Shift</kbd> + Click · Multi-select</div>
                <div><kbd className="text-slate-300">Drag</kbd> · Move object</div>
                <div><kbd className="text-slate-300">Ctrl/Cmd+D</kbd> · Duplicate</div>
                <div><kbd className="text-slate-300">Ctrl/Cmd+G</kbd> · Create block</div>
                <div><kbd className="text-slate-300">Ctrl/Cmd+Shift+G</kbd> · Ungroup block</div>
                <div><kbd className="text-slate-300">Alt+Click</kbd> · Cycle overlapped objects</div>
                <div><kbd className="text-slate-300">Ctrl/Cmd+C/V</kbd> · Copy / paste</div>
                <div><kbd className="text-slate-300">Ctrl/Cmd+Z</kbd> · Undo</div>
                <div><kbd className="text-slate-300">Delete</kbd> · Remove</div>
                <div><kbd className="text-slate-300">Enter</kbd> · Edit/exit block</div>
                <div><kbd className="text-slate-300">Esc</kbd> · Clear selection</div>
              </div>
            </div>

            {/* actions */}
            {selected.length > 0 && (
              <div className="grid grid-cols-2 gap-2">
                <button
                  type="button"
                  onClick={duplicate}
                  className="rounded-lg border border-white/[0.07] bg-white/[0.025] px-3 py-2 text-[10px] font-semibold text-slate-300 hover:bg-white/[0.05]"
                >
                  Duplicate
                </button>
                <button
                  type="button"
                  onClick={remove}
                  className="rounded-lg border border-rose-400/10 bg-rose-400/[0.04] px-3 py-2 text-[10px] font-semibold text-rose-300 hover:bg-rose-400/[0.08]"
                >
                  Delete
                </button>
              </div>
            )}
          </div>
        </aside>
      </div>

      {/* Array modal */}
      {arrayOpen && (
        <div className="fixed inset-0 z-[200] grid place-items-center bg-black/60 p-4 backdrop-blur-sm">
          <div className="w-[min(420px,100%)] rounded-2xl border border-white/[0.10] bg-[#0b1220] p-4 shadow-2xl">
            <div className="mb-4">
              <div className="text-sm font-semibold text-white">
                Create Rack Array
              </div>
              <div className="mt-1 text-[10px] text-slate-500">
                Generate a repeated storage pattern from the selected rack.
              </div>
            </div>

            <div className="grid grid-cols-2 gap-3">
              <PropertyInput
                label="Rows"
                type="number"
                value={arrayRows}
                onChange={(value) =>
                  setArrayRows(Math.max(1, Number(value) || 1))
                }
              />
              <PropertyInput
                label="Columns"
                type="number"
                value={arrayCols}
                onChange={(value) =>
                  setArrayCols(Math.max(1, Number(value) || 1))
                }
              />
              <PropertyInput
                label="Gap X (m)"
                type="number"
                step="0.1"
                value={arrayGapX}
                onChange={(value) =>
                  setArrayGapX(Math.max(0, Number(value) || 0))
                }
              />
              <PropertyInput
                label="Gap Z (m)"
                type="number"
                step="0.1"
                value={arrayGapZ}
                onChange={(value) =>
                  setArrayGapZ(Math.max(0, Number(value) || 0))
                }
              />
            </div>

            <div className="mt-4 rounded-lg border border-white/[0.06] bg-white/[0.02] p-3 text-[10px] text-slate-500">
              Total objects:{" "}
              <span className="font-mono text-slate-300">
                {arrayRows * arrayCols}
              </span>
            </div>

            <div className="mt-4 flex justify-end gap-2">
              <button
                type="button"
                onClick={() => setArrayOpen(false)}
                className="rounded-lg border border-white/[0.07] px-3 py-2 text-[10px] text-slate-500 hover:bg-white/[0.04] hover:text-white"
              >
                Cancel
              </button>
              <button
                type="button"
                onClick={createArray}
                className="rounded-lg border border-violet-400/15 bg-violet-400/[0.08] px-3 py-2 text-[10px] font-semibold text-violet-300"
              >
                Create Array
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
