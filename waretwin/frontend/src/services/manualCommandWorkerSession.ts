import { MANUAL_COMMAND_REFRESH_MS, type ActiveManualCommand } from "./manualCommand";

export type ManualWorkerAction = ActiveManualCommand | "STOP";

export interface ManualWorkerSocket {
  readonly readyState: number;
  readonly bufferedAmount: number;
  send(data: string): void;
  close(): void;
}

export interface ManualWorkerTimers {
  setInterval(callback: () => void, delayMs: number): unknown;
  clearInterval(timer: unknown): void;
}

const SOCKET_OPEN = 1;
const SOCKET_CLOSING = 2;
const SOCKET_CLOSED = 3;
const MAX_BUFFERED_BYTES = 8192;

/** Worker-owned manual lease refresh and fail-closed socket lifecycle. */
export class ManualCommandWorkerSession {
  private activeAction: ActiveManualCommand | null = null;
  private robotId = "";
  private refreshTimer: unknown | null = null;
  private errorReportedForConnection = false;
  private closingAfterFailure = false;

  constructor(
    private readonly getSocket: () => ManualWorkerSocket | null,
    private readonly timers: ManualWorkerTimers,
    private readonly reportError: (message: string) => void,
  ) {}

  get activeManualCommand(): ActiveManualCommand | null {
    return this.activeAction;
  }

  onOpen() {
    this.errorReportedForConnection = false;
    this.closingAfterFailure = false;
    this.refreshNow();
  }

  onClose() {
    this.stopRefresh();
    const lostAction = this.activeAction !== null;
    this.activeAction = null;
    this.closingAfterFailure = false;
    if (lostAction) this.reportOnce("manual refresh channel disconnected; the backend watchdog will stop motion");
  }

  setAction(robotId: string, action: ManualWorkerAction) {
    this.robotId = robotId;
    if (action === "STOP") {
      this.stop(robotId);
      return;
    }
    this.activeAction = action;
    this.refreshNow();
  }

  stop(robotId: string) {
    this.robotId = robotId;
    this.stopRefresh();
    this.activeAction = null;
    if (this.closingAfterFailure) return;
    this.send("STOP");
  }

  disconnect(robotId: string) {
    this.stop(robotId);
    const socket = this.getSocket();
    if (!socket || socket.readyState === SOCKET_CLOSING || socket.readyState === SOCKET_CLOSED) return;
    try {
      socket.close();
    } catch {
      // The Django disconnect hook and 0.40 s ROS lease remain the final stop barriers.
    }
  }

  onTransportError(message: string) {
    this.failClosed(message);
  }

  onBackendError(message: string) {
    this.failClosed(message);
  }

  private refreshNow() {
    this.stopRefresh();
    if (!this.activeAction) return;
    const socket = this.getSocket();
    if (!socket) return; // The first user command may wait for the initial connect.
    if (socket.readyState === SOCKET_CLOSED || socket.readyState === SOCKET_CLOSING) {
      this.failClosed("manual command channel closed before refresh; motion latch cleared");
      return;
    }
    if (socket.readyState !== SOCKET_OPEN) return;
    if (!this.send(this.activeAction)) return;
    this.refreshTimer = this.timers.setInterval(() => {
      const action = this.activeAction;
      if (!action || !this.send(action)) this.stopRefresh();
    }, MANUAL_COMMAND_REFRESH_MS);
  }

  private send(action: ManualWorkerAction): boolean {
    if (this.closingAfterFailure) return false;
    const socket = this.getSocket();
    if (!socket || socket.readyState !== SOCKET_OPEN) return false;
    if (socket.bufferedAmount > MAX_BUFFERED_BYTES) {
      this.failClosed("manual command channel backpressured; refresh stopped and socket closed");
      return false;
    }
    try {
      socket.send(JSON.stringify({ type: "ROBOT_MANUAL", robot_id: this.robotId, action }));
      return true;
    } catch {
      this.failClosed("manual command send failed; refresh stopped and socket closed");
      return false;
    }
  }

  private failClosed(message: string) {
    this.stopRefresh();
    this.activeAction = null;
    this.closingAfterFailure = true;
    this.reportOnce(message);
    const socket = this.getSocket();
    if (!socket || socket.readyState === SOCKET_CLOSING || socket.readyState === SOCKET_CLOSED) return;
    try {
      // Django's disconnect path clears MANUAL ownership; its 0.40 s watchdog
      // is still the fallback if the close handshake cannot complete.
      socket.close();
    } catch {
      // Do not retry from the ERROR feedback path; the command lease expires independently.
    }
  }

  private stopRefresh() {
    if (this.refreshTimer === null) return;
    this.timers.clearInterval(this.refreshTimer);
    this.refreshTimer = null;
  }

  private reportOnce(message: string) {
    if (this.errorReportedForConnection) return;
    this.errorReportedForConnection = true;
    this.reportError(message);
  }
}
