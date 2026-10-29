// @vitest-environment jsdom
import { describe, expect, it } from "vitest";
import { validateRosMapFiles } from "../src/pages/mapping/mapImportValidation";

const yamlText = (overrides: Partial<Record<string, string>> = {}) => [
  `image: ${overrides.image ?? "floor.pgm"}`,
  `mode: ${overrides.mode ?? "trinary"}`,
  `resolution: ${overrides.resolution ?? "0.05"}`,
  `origin: ${overrides.origin ?? "[-1.0, -2.0, 0.0]"}`,
  `negate: ${overrides.negate ?? "0"}`,
  `occupied_thresh: ${overrides.occupied_thresh ?? "0.65"}`,
  `free_thresh: ${overrides.free_thresh ?? "0.25"}`,
].join("\n");

function files(yaml = yamlText(), pgmBytes = new Uint8Array([
  ...new TextEncoder().encode("P5\n2 2\n255\n"), 0, 127, 255, 10,
])): File[] {
  return [new File([yaml], "floor.yaml", { type: "text/yaml" }),
    new File([pgmBytes], "floor.pgm", { type: "image/x-portable-graymap" })];
}

describe("ROS map file staging validation", () => {
  it("accepts a matching trinary YAML and 8-bit P5 image and returns preview metadata", async () => {
    const result = await validateRosMapFiles(files());
    expect(result.ok).toBe(true);
    if (!result.ok) return;
    expect(result.map).toMatchObject({ width: 2, height: 2, resolution: 0.05, origin: [-1, -2, 0] });
    expect(Array.from(result.map.pixels)).toEqual([0, 127, 255, 10]);
  });

  it("accepts P2 images and rejects unsupported extensions or missing pairs", async () => {
    const ascii = new TextEncoder().encode("P2\n2 1\n255\n0 255\n");
    const validAscii = await validateRosMapFiles([
      new File([yamlText({ image: "floor.pgm" })], "floor.yaml"), new File([ascii], "floor.pgm"),
    ]);
    expect(validAscii.ok).toBe(true);
    expect((await validateRosMapFiles([new File(["x"], "floor.png")] )).ok).toBe(false);
    expect((await validateRosMapFiles([new File([yamlText()], "floor.yaml")])).ok).toBe(false);
  });

  it.each([
    [yamlText({ image: "other.pgm" }), undefined, "reference"],
    [yamlText({ mode: "scale" }), undefined, "trinary"],
    [yamlText({ resolution: "0" }), undefined, "greater than zero"],
    [yamlText({ origin: "[0, 1]" }), undefined, "three finite map coordinates"],
    [yamlText({ occupied_thresh: "0.2", free_thresh: "0.3" }), undefined, "thresholds"],
    [yamlText(), new Uint8Array([...new TextEncoder().encode("P5\n3 2\n255\n"), 0, 1, 2]), "raster length"],
    [yamlText(), new Uint8Array([...new TextEncoder().encode("P5\n2 2\n65535\n"), 0, 1, 2, 3]), "8-bit"],
  ])("rejects incompatible map metadata or raster content", async (yaml, pgmBytes, errorText) => {
    const result = await validateRosMapFiles(files(yaml, pgmBytes));
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.error.toLowerCase()).toContain(String(errorText).toLowerCase());
  });
});
