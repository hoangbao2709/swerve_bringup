// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import { useStore } from "../src/state/store";
import { AppLayout } from "../src/app/AppLayout";
import { ControlPage } from "../src/pages/control/ControlPage";

const initialState = useStore.getState();
let root: Root;
let container: HTMLDivElement;

function renderControl() {
  act(() => root.render(<AppLayout pageTitle="CONTROL" isControlPage><ControlPage /></AppLayout>));
}

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  window.history.replaceState({}, "", "/control");
  useStore.setState({ ...initialState, twin: { ...initialState.twin, robots: {} }, selectedRobot: null,
    localRobotId: null, localIdentityError: null });
  container = document.createElement("div");
  document.body.appendChild(container);
  root = createRoot(container);
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
  useStore.setState(initialState);
});

describe("shared application shell for the local CONTROL route", () => {
  it("prioritizes robot operation and presents monitoring destinations separately", () => {
    renderControl();
    const navigation = container.querySelector(".wt-sidebar-nav");
    expect(navigation?.textContent).toContain("CONTROL");
    expect(navigation?.textContent).toContain("ROBOT OPERATION");
    expect(navigation?.textContent).toContain("MONITORING");
    expect(navigation?.textContent).toContain("INTEGRATION");
    expect(navigation?.querySelector('.wt-sidebar-item[href="/system"]')?.textContent).toContain("SYSTEM OVERVIEW");
    expect(navigation?.querySelector('.wt-sidebar-item[href="/diagnosis"]')?.textContent).toContain("DIAGNOSIS");
    expect(navigation?.textContent).toContain("MAPPING");
    expect(navigation?.textContent).toContain("LOCALIZATION");
    expect(navigation?.querySelector('[aria-label="MAPS"]')).toBeNull();
    expect(navigation?.textContent).toContain("VDA5050");
    expect(navigation?.querySelectorAll(".wt-sidebar-item")).toHaveLength(7);
    expect(navigation?.querySelectorAll("a.wt-sidebar-item")).toHaveLength(7);
    expect(navigation?.querySelector("button[aria-label='Toggle System pages']")).toBeNull();
    expect(Array.from(navigation?.querySelectorAll("section[aria-label$='navigation'] h2") ?? [])
      .map((heading) => heading.textContent?.trim())).toEqual(["ROBOT OPERATION", "MONITORING", "INTEGRATION"]);
    expect(container.querySelector('.wt-sidebar-item[href="/control"]')?.getAttribute("aria-current")).toBe("page");
    expect(navigation?.querySelectorAll('.wt-sidebar-item[aria-current="page"]')).toHaveLength(1);
    expect(container.textContent).toContain("WAITING FOR LOCAL ROBOT CONFIGURATION");
  });

  it("derives the active page from the browser pathname", () => {
    window.history.replaceState({}, "", "/localization");
    renderControl();
    expect(container.querySelector('.wt-sidebar-item[href="/localization"]')?.getAttribute("aria-current"))
      .toBe("page");
    expect(container.querySelector('.wt-sidebar-item[href="/control"]')?.getAttribute("aria-current"))
      .toBeNull();
    expect(container.querySelectorAll('.wt-sidebar-item[aria-current="page"]')).toHaveLength(1);
  });

  it("keeps deployment identity authoritative and never displays robot ID in the global header", () => {
    useStore.setState({
      twin: { ...initialState.twin, robots: { R02: { id: "R02" } as never } },
      selectedRobot: "R02", localRobotId: "R01",
    });
    renderControl();
    expect(container.querySelector(".local-app-header-title")?.textContent).toBe("CONTROL");
    expect(container.querySelector(".local-app-header")?.textContent).not.toMatch(/R01|R02|LOCAL ROBOT/i);
    expect(container.querySelectorAll(".wt-sidebar-item")).toHaveLength(7);
  });
});
