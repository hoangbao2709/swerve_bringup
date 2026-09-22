/**
 * Data-source runner.
 * - Demo mode: local TypeScript SimEngine.
 * - Backend mode: Django/Channels is authoritative. No local simulation fallback.
 */
import { useEffect } from "react";
import { SimEngine, SIM } from "./engine";
import { layout, useStore } from "../state/store";
import { wsConnect, wsDisconnect, wsSend } from "../services/ws";
import type { ScenarioInjection, TaskPriority, TaskType } from "../schema/twin_state";
import { DEMO_MODE, RUNTIME_MODE } from "../config";

// This can only be true for LOCAL_SIM.  GAZEBO_ROS and REAL_ROBOT always keep
// the last backend snapshot while disconnected and never instantiate a motion
// fallback loop.
const LOCAL_SIM_ENABLED = DEMO_MODE && RUNTIME_MODE === "LOCAL_SIM";

let engine: SimEngine | null = null;
export function getEngine(): SimEngine {
  if (!engine) engine = new SimEngine(layout, { seed: useStore.getState().seed });
  return engine;
}
export function resetEngine(seed?: number) {
  engine = new SimEngine(layout, { seed: seed ?? useStore.getState().seed });
  useStore.getState().setTwin(engine.snapshot());
  return engine;
}

function backendSend(message: Parameters<typeof wsSend>[0], errorText: string): boolean {
  const st = useStore.getState();
  if (st.source !== "online" || !wsSend(message)) {
    st.setNotice(errorText);
    return false;
  }
  return true;
}

/** Unified control path: demo uses local engine; backend mode sends commands only to Django. */
export const simControl = {
  play(speed?: 1 | 2 | 5 | 10) {
    const st = useStore.getState();
    if (!DEMO_MODE) {
      backendSend({ type: "SIM_CONTROL", action: "PLAY", speed: speed ?? (st.speed === 0 ? 1 : st.speed) }, "Backend offline — play command was not sent.");
      return;
    }
    st.setPaused(false);
    if (speed) st.setSpeed(speed); else if (st.speed === 0) st.setSpeed(1);
  },
  pause() {
    const st = useStore.getState();
    if (!DEMO_MODE) {
      backendSend({ type: "SIM_CONTROL", action: "PAUSE" }, "Backend offline — pause command was not sent.");
      return;
    }
    st.setPaused(true);
  },
  reset() {
    if (!DEMO_MODE) {
      backendSend({ type: "SIM_CONTROL", action: "RESET" }, "Backend offline — reset command was not sent.");
      return;
    }
    resetEngine();
  },
  inject(injection: ScenarioInjection) {
    const st = useStore.getState();
    if (!DEMO_MODE) {
      backendSend({ type: "INJECT", injection }, "Backend offline — scenario was not applied.");
      return;
    }
    getEngine().inject(injection);
    st.setTwin(getEngine().snapshot());
  },
  createTask(task: { type: TaskType; priority: TaskPriority; source: string; destination: string; load_units?: number; robot_id?: string | null }) {
    const st = useStore.getState();
    if (!DEMO_MODE) {
      backendSend({ type: "CREATE_TASK", task: { load_units: 1, ...task } }, "Backend offline — task was not created.");
      return;
    }
    try {
      const created = getEngine().createTask(task);
      if (task.robot_id) getEngine().assignTask(created.id, task.robot_id);
      st.setTwin(getEngine().snapshot());
    } catch (e) { st.setNotice(`Task rejected: ${(e as Error).message}`); }
  },
  assignTask(task_id: string, robot_id: string) {
    const st = useStore.getState();
    if (!DEMO_MODE) {
      backendSend({ type: "ASSIGN_TASK", task_id, robot_id }, "Backend offline — assignment was not sent.");
      return;
    }
    try { getEngine().assignTask(task_id, robot_id); st.setTwin(getEngine().snapshot()); }
    catch (e) { st.setNotice(`Assignment rejected: ${(e as Error).message}`); }
  },
  clearInjection(kind: ScenarioInjection["kind"], target_id: string) {
    const st = useStore.getState();
    if (!DEMO_MODE) {
      backendSend({ type: "CLEAR_INJECTION", kind, target_id }, "Backend offline — clear command was not sent.");
      return;
    }
    getEngine().clearInjection(kind, target_id);
    st.setTwin(getEngine().snapshot());
  },
  ackAlert(alert_id: string) {
    const st = useStore.getState();
    if (!DEMO_MODE) {
      backendSend({ type: "ACK_ALERT", alert_id }, "Backend offline — alert acknowledgement was not sent.");
      return;
    }
    getEngine().ackAlert(alert_id);
    st.setTwin(getEngine().snapshot());
  },
};

export function useBackendRealtime() {
  const authStatus = useStore((s) => s.authStatus);
  useEffect(() => {
    if (LOCAL_SIM_ENABLED) return;
    const st = useStore.getState();
    if (authStatus !== "authenticated") {
      wsDisconnect();
      return;
    }
    st.setSource("connecting");
    wsConnect((conn) => {
      const s = useStore.getState();
      const wsStates = { connecting: "CONNECTING", online: "CONNECTED", reconnecting: "RECONNECTING", offline: "DISCONNECTED", error: "ERROR", unauthorized: "DISCONNECTED" } as const;
      s.setWebsocketState(wsStates[conn]);
      if (conn === "online") {
        s.setSource("online");
        s.setHeat(null);
      } else if (conn === "unauthorized") {
        s.setSource("unauthorized");
        s.clearAuth();
      } else if (conn === "offline" || conn === "error") {
        s.setSource("offline");
        s.setHeat(null);
      } else {
        s.setSource("connecting");
      }
    });
    return () => wsDisconnect();
  }, [authStatus]);
}

export function useSimulationRunner() {
  useEffect(() => {
    const st = useStore.getState();
    if (st.authStatus !== "authenticated") {
      wsDisconnect();
      return;
    }

    if (LOCAL_SIM_ENABLED) {
      let raf = 0, last = performance.now(), acc = 0;
      const MAX_TICKS_PER_FRAME = 40;
      st.setTwin(getEngine().snapshot());
      st.setSource("local");
      const loop = (now: number) => {
        raf = requestAnimationFrame(loop);
        const dt = Math.min(0.25, (now - last) / 1000); last = now;
        const s = useStore.getState();
        if (s.paused || s.speed === 0) return;
        const eng = getEngine();
        acc += dt * s.speed;
        let n = 0;
        while (acc >= SIM.TICK_S && n < MAX_TICKS_PER_FRAME) { eng.step(); acc -= SIM.TICK_S; n++; }
        if (n > 0) { eng.state.sim.speed = s.speed; eng.state.sim.mode = "LIVE"; s.setTwin(eng.snapshot()); }
      };
      raf = requestAnimationFrame(loop);
      return () => { cancelAnimationFrame(raf); wsDisconnect(); };
    }

    // Backend mode connection is owned globally by App so admin pages receive
    // the same LAYOUT_UPDATED events as the live console.
    return;
  }, []);
}
