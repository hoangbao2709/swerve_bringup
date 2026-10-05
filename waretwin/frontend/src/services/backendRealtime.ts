import { useEffect } from "react";
import { useStore } from "../state/store";
import { wsConnect, wsDisconnect } from "./ws";

/** The authenticated backend WebSocket is the only frontend runtime source. */
export function useBackendRealtime() {
  const authStatus = useStore((state) => state.authStatus);
  useEffect(() => {
    if (authStatus !== "authenticated") {
      wsDisconnect();
      return;
    }

    wsConnect((connection) => {
      const state = useStore.getState();
      const websocketStates = {
        connecting: "CONNECTING",
        online: "CONNECTED",
        reconnecting: "RECONNECTING",
        offline: "DISCONNECTED",
        error: "ERROR",
        unauthorized: "DISCONNECTED",
      } as const;
      state.setWebsocketState(websocketStates[connection]);
      if (connection === "unauthorized") state.clearAuth();
    });
    return () => wsDisconnect();
  }, [authStatus]);
}
