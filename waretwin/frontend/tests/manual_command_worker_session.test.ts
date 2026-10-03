import { describe, expect, it, vi } from "vitest";
import { MANUAL_COMMAND_REFRESH_MS } from "../src/services/manualCommand";
import {
  ManualCommandWorkerSession,
  type ManualWorkerSocket,
  type ManualWorkerTimers,
} from "../src/services/manualCommandWorkerSession";

class FakeSocket implements ManualWorkerSocket {
  readyState = 1;
  bufferedAmount = 0;
  sent: Array<{ type: string; robot_id: string; action: string }> = [];
  closeCalls = 0;
  throwOnSend = false;

  send(data: string) {
    if (this.throwOnSend) throw new Error("closed socket");
    this.sent.push(JSON.parse(data));
  }

  close() {
    this.closeCalls += 1;
    this.readyState = 2;
  }
}

class FakeTimers implements ManualWorkerTimers {
  private nextId = 1;
  readonly intervals = new Map<number, { callback: () => void; delayMs: number }>();

  setInterval(callback: () => void, delayMs: number) {
    const id = this.nextId++;
    this.intervals.set(id, { callback, delayMs });
    return id;
  }

  clearInterval(timer: unknown) {
    this.intervals.delete(timer as number);
  }

  tick() {
    for (const timer of Array.from(this.intervals.values())) timer.callback();
  }
}

function setup(initialSocket = new FakeSocket()) {
  let socket: FakeSocket | null = initialSocket;
  const timers = new FakeTimers();
  const errors = vi.fn();
  const session = new ManualCommandWorkerSession(() => socket, timers, errors);
  return {
    session,
    socket: () => socket,
    replaceSocket: (next: FakeSocket | null) => { socket = next; },
    timers,
    errors,
  };
}

describe("manual worker refresh state machine", () => {
  it("sends HOLD immediately and refreshes at the single configured period", () => {
    const { session, socket, timers } = setup();
    session.setAction("R01", "FORWARD");
    expect(socket()?.sent).toEqual([{ type: "ROBOT_MANUAL", robot_id: "R01", action: "FORWARD" }]);
    expect(Array.from(timers.intervals.values()).map(({ delayMs }) => delayMs)).toEqual([MANUAL_COMMAND_REFRESH_MS]);
    timers.tick();
    expect(socket()?.sent.map(({ action }) => action)).toEqual(["FORWARD", "FORWARD"]);
  });

  it("switches the refreshed action directly without inserting STOP", () => {
    const { session, socket, timers } = setup();
    session.setAction("R01", "FORWARD");
    session.setAction("R01", "LEFT");
    timers.tick();
    expect(socket()?.sent.map(({ action }) => action)).toEqual(["FORWARD", "LEFT", "LEFT"]);
    expect(session.activeManualCommand).toBe("LEFT");
  });

  it("STOP clears active motion, sends zero, and cancels future refreshes", () => {
    const { session, socket, timers } = setup();
    session.setAction("R01", "FORWARD");
    session.stop("R01");
    timers.tick();
    expect(socket()?.sent.map(({ action }) => action)).toEqual(["FORWARD", "STOP"]);
    expect(session.activeManualCommand).toBeNull();
    expect(timers.intervals.size).toBe(0);
  });

  it("DISCONNECT best-effort sends STOP, cancels refresh, and closes the control channel", () => {
    const { session, socket, timers } = setup();
    session.setAction("R01", "RIGHT");
    session.disconnect("R01");
    timers.tick();
    expect(socket()?.sent.map(({ action }) => action)).toEqual(["RIGHT", "STOP"]);
    expect(socket()?.closeCalls).toBe(1);
    expect(session.activeManualCommand).toBeNull();
    expect(timers.intervals.size).toBe(0);
  });

  it("a socket close clears the command and reconnect starts stopped", () => {
    const state = setup();
    const originalSocket = state.socket()!;
    state.session.setAction("R01", "ROTATE_LEFT");
    originalSocket.readyState = 3;
    state.session.onClose();
    expect(state.session.activeManualCommand).toBeNull();
    expect(state.errors).toHaveBeenCalledTimes(1);

    const reconnectedSocket = new FakeSocket();
    state.replaceSocket(reconnectedSocket);
    state.session.onOpen();
    state.timers.tick();
    expect(reconnectedSocket.sent).toEqual([]);
    expect(state.timers.intervals.size).toBe(0);
  });

  it("backpressure closes the channel and ERROR-to-STOP feedback reports only once", () => {
    const state = setup();
    state.socket()!.bufferedAmount = 8193;
    state.session.setAction("R01", "FORWARD");
    expect(state.session.activeManualCommand).toBeNull();
    expect(state.socket()!.closeCalls).toBe(1);
    expect(state.errors).toHaveBeenCalledTimes(1);

    // This is the main-thread ERROR handler's best-effort STOP reply.
    state.session.stop("R01");
    state.socket()!.readyState = 3;
    state.session.onClose();
    expect(state.errors).toHaveBeenCalledTimes(1);
    expect(state.timers.intervals.size).toBe(0);

    const reconnectedSocket = new FakeSocket();
    state.replaceSocket(reconnectedSocket);
    state.session.onOpen();
    expect(reconnectedSocket.sent).toEqual([]);
    expect(state.session.activeManualCommand).toBeNull();
  });

  it("a send exception also clears motion and closes rather than silently retrying", () => {
    const state = setup();
    state.socket()!.throwOnSend = true;
    state.session.setAction("R01", "BACKWARD");
    expect(state.session.activeManualCommand).toBeNull();
    expect(state.socket()!.closeCalls).toBe(1);
    expect(state.errors).toHaveBeenCalledTimes(1);
    expect(state.timers.intervals.size).toBe(0);
  });
});
