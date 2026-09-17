import { API_URL } from "./ws";
import { useStore } from "../state/store";

export async function apiFetch(path: string, init: RequestInit = {}): Promise<Response> {
  const token = useStore.getState().authToken;
  const headers = new Headers(init.headers ?? {});
  if (token) headers.set("authorization", `Bearer ${token}`);
  if (init.body && !headers.has("content-type") && !(init.body instanceof FormData)) {
    headers.set("content-type", "application/json");
  }
  return fetch(`${API_URL}${path}`, { ...init, headers, credentials: "omit" });
}

