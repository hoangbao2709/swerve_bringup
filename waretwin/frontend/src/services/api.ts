import { API_URL } from "./ws";
import { useStore } from "../state/store";
import type { TagNavigationState } from "../schema/twin_state";

export async function navigationApi(path: string, init: RequestInit = {}) {
  const response = await apiFetch(`/api/navigation/${path}`, init);
  if (!response.ok) { const body = await response.json().catch(() => ({})); throw new Error(body.detail || response.statusText); }
  return response.json();
}
export const startTagMission = (robot_id: string, target_tag_id: number) => {
  const state = useStore.getState();
  if (state.runtimeMode !== "LOCAL_SIM" && state.mapSync.status !== "SYNCED") {
    return Promise.reject(new Error(`Cannot start mission: map revision mismatch (${state.mapSync.status})`));
  }
  return navigationApi("missions/start", { method: "POST", body: JSON.stringify({ robot_id, target_tag_id }) }) as Promise<TagNavigationState>;
};
export const missionAction = (id: number, action: "pause" | "resume" | "cancel" | "replan") => navigationApi(`missions/${id}/${action}`, { method: "POST" });
export const emergencyStop = (robotId: string) => apiFetch(`/api/robots/${encodeURIComponent(robotId)}/emergency-stop`, { method: "POST" }).then(async r => { if (!r.ok) throw new Error((await r.json()).detail); return r.json(); });

export async function apiFetch(path: string, init: RequestInit = {}): Promise<Response> {
  const token = useStore.getState().authToken;
  const headers = new Headers(init.headers ?? {});
  if (token) headers.set("authorization", `Bearer ${token}`);
  if (init.body && !headers.has("content-type") && !(init.body instanceof FormData)) {
    headers.set("content-type", "application/json");
  }
  return fetch(`${API_URL}${path}`, { ...init, headers, credentials: "omit" });
}
