// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from "vitest";
import type { RobotDetailPathPreview } from "../src/schema/twin_state";
import { wsConnect, wsDisconnect } from "../src/services/ws";
import { useStore } from "../src/state/store";

class TestWebSocket {
  static readonly OPEN = 1;
  readyState = 0;
  onopen: ((event: Event) => void) | null = null;
  onclose: ((event: CloseEvent) => void) | null = null;
  onerror: ((event: Event) => void) | null = null;
  onmessage: ((event: MessageEvent) => void) | null = null;
  static instances: TestWebSocket[] = [];
  constructor() { TestWebSocket.instances.push(this); }
  close() { this.readyState = 3; }
  send() {}
}

afterEach(() => {
  wsDisconnect();
  useStore.setState((state) => ({ robotDetail: { ...state.robotDetail, R01: { ...state.robotDetail.R01, pathPreview: null } } }));
  vi.unstubAllGlobals();
  TestWebSocket.instances = [];
});

describe("PATH_PREVIEW_RESULT WebSocket delivery", () => {
  it("stores the backend result under the matching robot detail state unchanged", () => {
    vi.stubGlobal("WebSocket", TestWebSocket);
    wsConnect(() => undefined);
    const socket = TestWebSocket.instances[0];
    expect(socket?.onmessage).toBeTypeOf("function");

    const result: RobotDetailPathPreview = {
      robot_id: "R01", request_id: "shelf-preview-1103", status: "VALID", source_type: "TAG", source_id: "1103",
      tag_id: 1103, tag_revision: "tag-1103-r23", registry_revision: "registry-r23",
      registration_revision: 23, active_map_id: "SLAM-session-23", active_map_revision: "session-23",
      goal: { x: 10.5, y: 16.5, yaw: Math.PI }, source_goal: { x: 10.5, y: 16.5, yaw: Math.PI },
      route_revision: "route-r23", route_nodes: [1102, 1103],
      route_segments: [{ from: 1102, to: 1103, axis: "X", length_m: 1 }],
      path: [{ x: 1, y: 1 }, { x: 10.5, y: 16.5 }],
      active_path: [{ x: 1, y: 1 }, { x: 10.5, y: 16.5 }],
      canonical_path: [{ x: 1, y: 1 }, { x: 10.5, y: 16.5 }],
      active_route_points: [{ x: 10.5, y: 16.5, yaw: Math.PI, kind: "TAG_SERVICE", tag_id: 1103 }],
      canonical_route_points: [{ x: 10.5, y: 16.5, yaw: Math.PI, kind: "TAG_SERVICE", tag_id: 1103 }],
      graph_revision: "graph-r23", path_length_m: 10, timestamp: new Date().toISOString(),
    };
    const message = { type: "PATH_PREVIEW_RESULT", ...result };
    socket!.onmessage!(new MessageEvent("message", { data: JSON.stringify(message) }));

    expect(useStore.getState().robotDetail.R01?.pathPreview).toEqual(message);
  });
});
