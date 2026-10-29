import { API_URL } from "./ws";
import { useStore } from "../state/store";

/** Existing backend transport and single-local-robot identity guard. */
export async function backendApiFetch(path: string, init: RequestInit = {}): Promise<Response> {
  const target = path.match(/^\/api\/robots\/([^/]+)/);
  if (target && decodeURIComponent(target[1]) !== useStore.getState().localRobotId) {
    throw new Error("LOCAL_ROBOT_ID_MISMATCH: request must target the configured local robot");
  }
  const headers = new Headers(init.headers ?? {});
  if (init.body && !headers.has("content-type") && !(init.body instanceof FormData)) {
    headers.set("content-type", "application/json");
  }
  // Same-origin credentials let the configured authenticated gateway attach
  // its operator session in PROTECTED_LAN mode. They are never sent to a
  // different origin (and LOCAL_LOOPBACK still rejects non-loopback origins).
  return fetch(`${API_URL}${path}`, { ...init, headers, credentials: "same-origin" });
}
