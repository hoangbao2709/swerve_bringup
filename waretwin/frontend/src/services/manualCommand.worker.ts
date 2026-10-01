/// <reference lib="webworker" />

type MotionAction = "FORWARD" | "BACKWARD" | "LEFT" | "RIGHT" | "ROTATE_LEFT" | "ROTATE_RIGHT";
type WorkerRequest =
  | { type: "CONNECT"; url: string; token: string }
  | { type: "HOLD"; robot_id: string; action: MotionAction }
  | { type: "STOP"; robot_id: string }
  | { type: "DISCONNECT"; robot_id: string };

const scope = self as DedicatedWorkerGlobalScope;
let socket: WebSocket | null = null;
let socketUrl = "";
let token = "";
let robotId = "";
let activeAction: MotionAction | null = null;
let refreshTimer: number | null = null;
let reconnectTimer: number | null = null;
let reconnectDelayMs = 250;
let disconnecting = false;

function stopRefresh() {
  if (refreshTimer !== null) {
    clearInterval(refreshTimer);
    refreshTimer = null;
  }
}

function notifyError(message: string) {
  scope.postMessage({ type: "ERROR", message });
}

function send(action: MotionAction | "STOP") {
  if (!socket || socket.readyState !== WebSocket.OPEN) return false;
  if (socket.bufferedAmount > 8192) {
    stopRefresh();
    activeAction = null;
    notifyError("manual command channel is backpressured; command hold stopped");
    return false;
  }
  socket.send(JSON.stringify({ type: "ROBOT_MANUAL", robot_id: robotId, action }));
  return true;
}

function startRefresh() {
  stopRefresh();
  if (!activeAction || !socket || socket.readyState !== WebSocket.OPEN) return;
  if (!send(activeAction)) return;
  refreshTimer = self.setInterval(() => {
    if (!activeAction || !send(activeAction)) {
      stopRefresh();
      activeAction = null;
    }
  }, 100);
}

function openSocket() {
  if (disconnecting || !socketUrl || !token) return;
  const separator = socketUrl.includes("?") ? "&" : "?";
  const url = `${socketUrl}${separator}token=${encodeURIComponent(token)}&control_only=1`;
  const connection = new WebSocket(url);
  socket = connection;
  connection.onopen = () => {
    reconnectDelayMs = 250;
    scope.postMessage({ type: "READY" });
    startRefresh();
  };
  connection.onmessage = (event) => {
    try {
      const message = JSON.parse(String(event.data)) as { type?: string; code?: string; message?: string };
      if (message.type === "ERROR") {
        stopRefresh();
        activeAction = null;
        notifyError(message.message || message.code || "manual command was rejected");
      }
    } catch {
      // No other server frame is required on this restricted channel.
    }
  };
  connection.onerror = () => {
    notifyError("manual command channel is unavailable");
  };
  connection.onclose = () => {
    if (socket === connection) socket = null;
    stopRefresh();
    if (activeAction) {
      activeAction = null;
      notifyError("manual refresh channel disconnected; the dead-man stop is active");
    }
    if (!disconnecting) {
      const delay = reconnectDelayMs;
      reconnectDelayMs = Math.min(5000, reconnectDelayMs * 2);
      reconnectTimer = self.setTimeout(openSocket, delay);
    }
  };
}

scope.onmessage = (event: MessageEvent<WorkerRequest>) => {
  const request = event.data;
  if (request.type === "CONNECT") {
    socketUrl = request.url;
    token = request.token;
    disconnecting = false;
    if (!socket || socket.readyState === WebSocket.CLOSED) openSocket();
    return;
  }
  if (request.type === "HOLD") {
    robotId = request.robot_id;
    activeAction = request.action;
    if (socket?.readyState === WebSocket.OPEN) startRefresh();
    return;
  }
  if (request.type === "STOP") {
    robotId = request.robot_id;
    stopRefresh();
    activeAction = null;
    send("STOP");
    return;
  }
  if (request.type === "DISCONNECT") {
    robotId = request.robot_id;
    disconnecting = true;
    stopRefresh();
    activeAction = null;
    if (reconnectTimer !== null) {
      clearTimeout(reconnectTimer);
      reconnectTimer = null;
    }
    send("STOP");
    socket?.close();
    socket = null;
  }
};
