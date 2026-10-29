import { useMemo, useState } from "react";
import { schedulerApi, type ConveyorCommand } from "../../services/scheduler";
import { useStore } from "../../state/store";

const ACTIONS: Array<[ConveyorCommand, string]> = [
  ["START", "Start"],
  ["STOP", "Stop"],
  ["MAINTENANCE", "Maintenance"],
  ["CLEAR_FAULT", "Clear fault"],
  ["EMERGENCY_STOP", "Emergency stop"],
];

export function ConveyorWindow({ conveyorId }: { conveyorId: string }) {
  const conveyor = useStore((s) => s.twin.conveyors[conveyorId]);
  const robots = useStore((s) => s.twin.robots);
  const [speed, setSpeed] = useState(String(conveyor?.speed_mps ?? 0.5));
  const [robotId, setRobotId] = useState(Object.keys(robots)[0] ?? "");
  const [busy, setBusy] = useState(false);
  const [message, setMessage] = useState("");

  const items = useMemo(() => conveyor?.items ?? [], [conveyor?.items]);
  if (!conveyor) return <div className="conveyor-window">Conveyor {conveyorId} is not available.</div>;

  async function command(action: ConveyorCommand, body: Record<string, unknown> = {}) {
    setBusy(true);
    setMessage("");
    try {
      await schedulerApi.conveyorCommand(conveyorId, action, body);
      setMessage(`${action} accepted by PLC`);
    } catch (error) {
      setMessage(error instanceof Error ? error.message : String(error));
    } finally {
      setBusy(false);
    }
  }

  async function handshake(itemId: string, phase: string) {
    setBusy(true);
    try {
      await schedulerApi.conveyorHandshake(conveyorId, { robot_id: robotId, item_id: itemId, phase });
      setMessage(`${phase} confirmed for ${itemId}`);
    } catch (error) {
      setMessage(error instanceof Error ? error.message : String(error));
    } finally {
      setBusy(false);
    }
  }

  return (
    <div className="conveyor-window">
      <div className="modal-h conveyor-window-head">
        <strong>{conveyorId} · PLC CONTROL</strong>
        <span className="spacer" />
        <span className={`conveyor-status ${conveyor.status.toLowerCase()}`}>{conveyor.mode ?? conveyor.status}</span>
      </div>
      <div className="conveyor-window-body">
        <div className="conveyor-command-grid">
          {ACTIONS.map(([action, label]) => (
            <button key={action} type="button" className={action === "EMERGENCY_STOP" ? "danger" : "btn"} disabled={busy} onClick={() => void command(action)}>{label}</button>
          ))}
        </div>
        <div className="conveyor-speed-row">
          <label>Speed (m/s)<input type="number" min="0" max="5" step="0.05" value={speed} onChange={(e) => setSpeed(e.target.value)} /></label>
          <button className="btn" type="button" disabled={busy} onClick={() => void command("SET_SPEED", { speed_mps: Number(speed) })}>Apply speed</button>
        </div>
        <div className="conveyor-kpis">
          <span>Items <b>{conveyor.items_on_belt}</b></span>
          <span>Throughput <b>{conveyor.throughput_per_min}/min</b></span>
          <span>Entry <b>{conveyor.sensors?.entry ? "ON" : "OFF"}</b></span>
          <span>Exit <b>{conveyor.sensors?.exit ? "ON" : "OFF"}</b></span>
          <span>Jam <b>{conveyor.sensors?.jam ? "YES" : "NO"}</b></span>
        </div>
        {conveyor.fault_code && <div className="conveyor-fault">FAULT: {conveyor.fault_code}</div>}
        {message && <div className="conveyor-message">{message}</div>}
        <div className="conveyor-handshake-head"><b>Tracked items / robot handshake</b><label>Robot<select value={robotId} onChange={(e) => setRobotId(e.target.value)}>{Object.keys(robots).map((id) => <option key={id}>{id}</option>)}</select></label></div>
        <div className="conveyor-item-list">
          {items.length === 0 && <div className="conveyor-empty">No tracked item</div>}
          {items.map((item) => (
            <div className="conveyor-item-row" key={item.item_id}>
              <div><b>{item.item_id}</b><small>{item.order_id ?? "No order"} · {item.status} · {item.position_m.toFixed(2)}m</small></div>
              <div className="conveyor-item-actions">
                <button className="btn" disabled={busy || item.status !== "AT_EXIT"} onClick={() => void handshake(item.item_id, "REQUEST_PICKUP")}>Request</button>
                <button className="btn" disabled={busy || item.status !== "AT_EXIT"} onClick={() => void handshake(item.item_id, "PICKUP_CONFIRMED")}>Confirm pickup</button>
                <button className="btn" disabled={busy || item.status !== "HANDED_TO_ROBOT"} onClick={() => void handshake(item.item_id, "DELIVERY_CONFIRMED")}>Confirm delivery</button>
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
