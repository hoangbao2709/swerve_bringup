// @vitest-environment jsdom
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { MapImportStager } from "../src/pages/mapping/MapImportStager";

let root: Root;
let container: HTMLDivElement;

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  container = document.createElement("div");
  document.body.appendChild(container);
  root = createRoot(container);
  HTMLCanvasElement.prototype.getContext = () => null;
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
});

describe("new map staging UI", () => {
  it("validates and stages a local pair without claiming import or activation", async () => {
    act(() => root.render(<MapImportStager />));
    const yaml = new File([[
      "image: floor.pgm", "mode: trinary", "resolution: 0.05", "origin: [0, 0, 0]",
      "negate: 0", "occupied_thresh: 0.65", "free_thresh: 0.25",
    ].join("\n")], "floor.yaml");
    const pgm = new File([new Uint8Array([...new TextEncoder().encode("P5\n2 2\n255\n"), 0, 80, 160, 255])], "floor.pgm");
    const input = container.querySelector<HTMLInputElement>('input[aria-label="Select ROS map YAML and PGM files"]');
    expect(input).not.toBeNull();
    if (!input) return;
    Object.defineProperty(input, "files", { configurable: true, value: [yaml, pgm] });
    await act(async () => {
      input.dispatchEvent(new Event("change", { bubbles: true }));
      await new Promise((resolve) => window.setTimeout(resolve, 0));
    });

    expect(container.textContent).toContain("MAP PAIR VALIDATED");
    expect(container.textContent).toContain("2 × 2 cells");
    expect(container.querySelector('canvas[role="img"][aria-label^="Local PGM map preview"]')).not.toBeNull();
    expect(container.querySelector<HTMLButtonElement>('button[title="This installation has no map-import API"]')?.disabled).toBe(true);

    act(() => Array.from(container.querySelectorAll<HTMLButtonElement>("button"))
      .find((button) => button.textContent?.trim() === "STAGE FILES FOR REVIEW")?.click());
    expect(container.textContent).toContain("Files staged in this browser. Robot map unchanged.");
    expect(container.querySelector<HTMLButtonElement>('button[title="This installation has no map-import API"]')?.disabled).toBe(true);

    act(() => Array.from(container.querySelectorAll<HTMLButtonElement>("button"))
      .find((button) => button.textContent?.trim() === "CLEAR STAGING")?.click());
    expect(container.textContent).not.toContain("MAP PAIR VALIDATED");
  });
});
