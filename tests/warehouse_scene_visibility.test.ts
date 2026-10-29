import { existsSync, readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";

const scenePath = new URL("../src/components/scene/Scene3D.tsx", import.meta.url);
const actorPath = new URL("../src/components/scene/People.tsx", import.meta.url);

describe("warehouse 3D scene visibility", () => {
  it("omits decorative workers and forklifts while retaining warehouse and robot renderers", () => {
    const scene = readFileSync(scenePath, "utf8");
    expect(existsSync(actorPath)).toBe(false);
    expect(scene).not.toMatch(/from\s+["']\.\/People["']/);
    expect(scene).not.toMatch(/<People\b/);
    expect(scene).toContain("<WarehouseShell");
    expect(scene).toContain("<RackInstances");
    expect(scene).toContain("<Robots");
    expect(scene).toContain("<Fixtures");
  });
});
