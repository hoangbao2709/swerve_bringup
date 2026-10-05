/** Backend WebSocket connection and existing panel actions.
 * This module has no simulator, local motion loop, or fake-data fallback. */
import { useEffect } from "react";
import { useStore } from "../state/store";
import { wsConnect, wsDisconnect, wsSend } from "./ws";
import type { ScenarioInjection, TaskPriority, TaskType } from "../schema/twin_state";

function backendSend(message: Parameters<typeof wsSend>[0], errorText: string): boolean {
  const state = useStore.getState();
  if (state.source !== "online" || !wsSend(message)) {
    state.setNotice(errorText);
    return false;
  }
  return true;
}

/** Compatibility for retained Overview dialogs; commands are backend-only. */
export const backendActions = {
  play(speed?: 1 | 2 | 5 | 10) {
    const state = useStore.getState();
    backendSend({ type: "SIM_CONTROL", action: "PLAY", speed: speed ?? (state.speed === 0 ? 1 : state.speed) }, "Backend offline — play command was not sent.");
  },
  pause() {
    backendSend({ type: "SIM_CONTROL", action: "PAUSE" }, "Backend offline — pause command was not sent.");
  },
  reset() {
    backendSend({ type: "SIM_CONTROL", action: "RESET" }, "Backend offline — reset command was not sent.");
  },
  inject(injection: ScenarioInjection) {
    backendSend({ type: "INJECT", injection }, "Backend offline — scenario was not applied.");
  },
  createTask(task: { type: TaskType; priority: TaskPriority; source: string; destination: string; load_units?: number; robot_id?: string | null }) {
    backendSend({ type: "CREATE_TASK", task: { load_units: 1, ...task } }, "Backend offline — task was not created.");
  },
  assignTask(task_id: string, robot_id: string) {
    backendSend({ type: "ASSIGN_TASK", task_id, robot_id }, "Backend offline — assignment was not sent.");
  },
  clearInjection(kind: ScenarioInjection["kind"], target_id: string) {
    backendSend({ type: "CLEAR_INJECTION", kind, target_id }, "Backend offline — clear command was not sent.");
  },
  ackAlert(alert_id: string) {
    backendSend({ type: "ACK_ALERT", alert_id }, "Backend offline — alert acknowledgement was not sent.");
  },
};

export function useBackendRealtime() {
  useEffect(() => {
    const state = useStore.getState();
    state.setSource("connecting");
    wsConnect((connection) => {
      const current = useStore.getState();
      const states = {
        connecting: "CONNECTING", online: "CONNECTED", reconnecting: "RECONNECTING",
        offline: "DISCONNECTED", error: "ERROR",
      } as const;
      current.setWebsocketState(states[connection]);
      if (connection === "online") {
        current.setSource("online");
        current.setHeat(null);
      } else if (connection === "offline" || connection === "error") {
        current.setSource("offline");
        current.setHeat(null);
      } else {
        current.setSource("connecting");
      }
    });
    return () => wsDisconnect();
  }, []);
}
