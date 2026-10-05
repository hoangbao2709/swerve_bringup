// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from "vitest";

import { apiFetch } from "../src/services/api";
import { WS_URL, wsConnect, wsDisconnect } from "../src/services/ws";

afterEach(() => {
  wsDisconnect();
  vi.unstubAllGlobals();
});

describe("local browser runtime connections", () => {
  it("calls REST without cookies or a user Authorization header", async () => {
    const fetchMock = vi.fn().mockResolvedValue({ ok: true, status: 200 } as Response);
    vi.stubGlobal("fetch", fetchMock);

    await apiFetch("/api/health", { method: "POST", body: JSON.stringify({ probe: true }) });

    const init = fetchMock.mock.calls[0]?.[1] as RequestInit | undefined;
    expect(init).toBeDefined();
    expect(init?.credentials).toBe("omit");
    expect(new Headers(init?.headers).has("authorization")).toBe(false);
  });

  it("opens the browser WebSocket without a user token in its URL", () => {
    const urls: string[] = [];
    class LocalSocket {
      static readonly OPEN = 1;
      readyState = 0;
      onopen: ((event: Event) => void) | null = null;
      onclose: ((event: CloseEvent) => void) | null = null;
      onerror: ((event: Event) => void) | null = null;
      onmessage: ((event: MessageEvent) => void) | null = null;
      constructor(url: string) { urls.push(url); }
      close() { this.readyState = 3; }
      send() {}
    }
    vi.stubGlobal("WebSocket", LocalSocket);

    wsConnect(() => undefined);

    expect(urls).toEqual([WS_URL]);
    expect(urls[0]).not.toMatch(/[?&]token=/i);
  });
});
