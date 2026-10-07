/// <reference lib="webworker" />

import { ManualCommandWorkerSession } from "./manualCommandWorkerSession";
import type { ManualWorkerSocket } from "./manualCommandWorkerSession";
import type { ActiveManualCommand } from "./manualCommand";
type WorkerRequest =
  | { type: "CONNECT"; url: string }
  | { type: "HOLD"; robot_id: string; action: ActiveManualCommand }
  | { type: "STOP"; robot_id: string }
  | { type: "DISCONNECT"; robot_id: string };

const scope = self as DedicatedWorkerGlobalScope;
let socket: WebSocket | null = null;
let socketUrl = "";
let robotId = "";
let reconnectTimer: number | null = null;
let reconnectDelayMs = 250;
let disconnecting = false;

const session = new ManualCommandWorkerSession(
  () => socket as ManualWorkerSocket | null,
  {
    setInterval: (callback, delayMs) => self.setInterval(callback, delayMs),
    clearInterval: (timer) => self.clearInterval(timer as number),
  },
  (message) => scope.postMessage({ type: "ERROR", message }),
);

function clearReconnectTimer() {
  if (reconnectTimer === null) return;
  clearTimeout(reconnectTimer);
  reconnectTimer = null;
}

function scheduleReconnect() {
  if (disconnecting || !socketUrl || reconnectTimer !== null) return;
  const delay = reconnectDelayMs;
  reconnectDelayMs = Math.min(5000, reconnectDelayMs * 2);
  reconnectTimer = self.setTimeout(() => {
    reconnectTimer = null;
    openSocket();
  }, delay);
}

function openSocket() {
  if (disconnecting || !socketUrl) return;
  const separator = socketUrl.includes("?") ? "&" : "?";
  const url = `${socketUrl}${separator}control_only=1`;
  const connection = new WebSocket(url);
  socket = connection;
  connection.onopen = () => {
    if (socket !== connection) return;
    reconnectDelayMs = 250;
    session.onOpen();
    scope.postMessage({ type: "READY" });
  };
  connection.onmessage = (event) => {
    if (socket !== connection) return;
    try {
      const message = JSON.parse(String(event.data)) as { type?: string; code?: string; message?: string };
      if (message.type === "ERROR") {
        session.onBackendError(message.message || message.code || "manual command was rejected");
      }
    } catch {
      // No other server frame is required on this restricted channel.
    }
  };
  connection.onerror = () => {
    if (socket === connection) session.onTransportError("manual command channel is unavailable; motion latch cleared");
  };
  connection.onclose = () => {
    if (socket !== connection) return;
    socket = null;
    session.onClose();
    scheduleReconnect();
  };
}

scope.onmessage = (event: MessageEvent<WorkerRequest>) => {
  const request = event.data;
  if (request.type === "CONNECT") {
    if (socketUrl && socketUrl !== request.url) {
      disconnecting = true;
      clearReconnectTimer();
      session.disconnect(robotId);
    }
    socketUrl = request.url;
    disconnecting = false;
    clearReconnectTimer();
    if (!socket || socket.readyState === WebSocket.CLOSED) openSocket();
    return;
  }
  if (request.type === "HOLD") {
    robotId = request.robot_id;
    session.setAction(request.robot_id, request.action);
    return;
  }
  if (request.type === "STOP") {
    robotId = request.robot_id;
    session.stop(request.robot_id);
    return;
  }
  if (request.type === "DISCONNECT") {
    robotId = request.robot_id;
    disconnecting = true;
    clearReconnectTimer();
    session.disconnect(request.robot_id);
  }
};
