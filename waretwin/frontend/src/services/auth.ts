import { apiFetch } from "./api";
import { useStore, type AuthUser } from "../state/store";
import { wsDisconnect } from "./ws";

const STORAGE_KEY = "waretwin.auth";

type StoredSession = { token: string; user: AuthUser };
type LoginResponse = { access_token: string; token_type: "bearer"; user: AuthUser };

async function responseMessage(response: Response): Promise<string> {
  try {
    const body = await response.json() as { detail?: unknown; error?: { message?: unknown }; message?: unknown };
    const message = body.detail ?? body.error?.message ?? body.message;
    if (typeof message === "string" && message.trim()) return message.trim();
  } catch {
    // Fall back to the HTTP status text for non-JSON responses.
  }
  return response.statusText || `HTTP ${response.status}`;
}

function readStoredSession(): StoredSession | null {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return null;
    const parsed = JSON.parse(raw) as StoredSession;
    if (!parsed?.token || !parsed?.user) return null;
    return parsed;
  } catch {
    return null;
  }
}

function writeStoredSession(session: StoredSession | null) {
  if (!session) {
    localStorage.removeItem(STORAGE_KEY);
    return;
  }
  localStorage.setItem(STORAGE_KEY, JSON.stringify(session));
}

export async function bootstrapAuth() {
  const session = readStoredSession();
  if (!session) {
    useStore.getState().setAuth({ status: "guest", token: null, user: null });
    return null;
  }

  useStore.getState().setAuth({ status: "loading", token: session.token, user: session.user });
  try {
    const r = await apiFetch("/api/auth/me");
    if (!r.ok) {
      writeStoredSession(null);
      useStore.getState().clearAuth();
      return null;
    }
    const user = (await r.json()) as AuthUser;
    const next = { token: session.token, user };
    writeStoredSession(next);
    useStore.getState().setAuth({ status: "authenticated", token: session.token, user });
    return next;
  } catch {
    // Never leave the UI stuck on “Restoring session…” when the backend is down.
    writeStoredSession(null);
    useStore.getState().clearAuth();
    return null;
  }
}

export async function login(username: string, password: string) {
  const r = await apiFetch("/api/auth/login", { method: "POST", body: JSON.stringify({ username, password }) });
  if (!r.ok) throw new Error(await responseMessage(r));
  const data = (await r.json()) as LoginResponse;
  if (!data || typeof data.access_token !== "string" || !data.access_token.trim() || !data.user?.username) {
    throw new Error("The backend returned an invalid login response.");
  }
  const session = { token: data.access_token, user: data.user };
  writeStoredSession(session);
  useStore.getState().setAuth({ status: "authenticated", token: data.access_token, user: data.user });
  return session;
}

export async function logout() {
  try {
    const token = useStore.getState().authToken;
    if (token) await apiFetch("/api/auth/logout", { method: "POST" });
  } catch {
    // Ignore network errors on logout; local session still gets cleared.
  } finally {
    writeStoredSession(null);
    useStore.getState().clearAuth();
    wsDisconnect();
  }
}

