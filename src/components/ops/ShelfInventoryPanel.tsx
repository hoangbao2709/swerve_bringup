import { useEffect, useMemo, useState } from "react";
import { schedulerApi, type InventoryDispatchResult, type ShelfInventoryItem, type ShelfInventoryResponse } from "../../services/scheduler";
import { useStore } from "../../state/store";
import type { FloorId } from "../../layout/types";

type Props = {
  rackId: string;
  zone: string;
  floor: FloorId;
  position: [number, number, number];
  load: number;
  percentLabel: string;
  onClose?: () => void;
};

function errorText(error: unknown): string {
  return error instanceof Error ? error.message : String(error || "Unknown error");
}

function itemTitle(item: ShelfInventoryItem): string {
  return item.item_code || item.origin_order_no || item.item_uid;
}

export function ShelfInventoryPanel({ rackId, zone, floor, position, load, percentLabel, onClose }: Props) {
  const layoutRevision = useStore((s) => s.layoutRevision);
  const [data, setData] = useState<ShelfInventoryResponse | null>(null);
  const [loading, setLoading] = useState(true);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const [ok, setOk] = useState("");
  const [selectedId, setSelectedId] = useState<number | null>(null);
  const [destination, setDestination] = useState("");
  const [priority, setPriority] = useState("NORMAL");
  const [preferUnloaded, setPreferUnloaded] = useState(false);
  const [lastDispatch, setLastDispatch] = useState<InventoryDispatchResult | null>(null);

  async function refresh(quiet = false) {
    if (!quiet) setLoading(true);
    setError("");
    try {
      const next = await schedulerApi.shelfInventory(rackId);
      setData(next);
      const selectable = next.items.find((x) => x.status === "STORED");
      setSelectedId((prev) => next.items.some((x) => x.id === prev && x.status === "STORED") ? prev : selectable?.id ?? null);
      setDestination((prev) => next.destination_shelves.some((x) => x.shelf_code === prev && x.enabled) ? prev : next.destination_shelves.find((x) => x.enabled)?.shelf_code ?? "");
    } catch (e) {
      setError(errorText(e));
    } finally {
      if (!quiet) setLoading(false);
    }
  }

  useEffect(() => {
    setSelectedId(null);
    setDestination("");
    setLastDispatch(null);
    setOk("");
    void refresh();
    // A completed movement emits LAYOUT_UPDATED, which increments layoutRevision.
    // Refetching here keeps exact item location/status aligned with the 2D/3D count.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rackId, layoutRevision]);

  const selected = useMemo(() => data?.items.find((x) => x.id === selectedId) ?? null, [data, selectedId]);
  const freeDestinations = useMemo(() => data?.destination_shelves.filter((x) => x.enabled && x.available_slots > 0) ?? [], [data]);

  async function dispatch(action: "OUTBOUND" | "TRANSFER") {
    if (!selected || selected.status !== "STORED") return;
    if (action === "TRANSFER" && !destination) {
      setError("Chọn shelf đích trước khi chuyển hàng.");
      return;
    }
    setBusy(true);
    setError("");
    setOk("");
    try {
      const result = await schedulerApi.dispatchInventoryItem(selected.id, {
        action,
        destination_shelf_code: action === "TRANSFER" ? destination : undefined,
        priority,
        prefer_unloaded_robot: preferUnloaded,
      });
      setLastDispatch(result);
      const target = action === "OUTBOUND" ? result.order.destination.code : result.order.destination.resource_id || result.order.destination.code;
      setOk(`${selected.item_code || selected.item_uid} → ${target}. Robot ${result.schedule.robot_id} đã được tự động chọn.`);
      await refresh(true);
    } catch (e) {
      setError(errorText(e));
    } finally {
      setBusy(false);
    }
  }

  return (
    <div className="shelf-inventory-card" role="dialog" aria-label={`Shelf ${rackId} inventory`}>
      <div className="modal-h shelf-selection-head shelf-inventory-head">
        <div><span>SHELF INVENTORY</span><small>Chọn từng đơn hàng để robot xử lý</small></div>
      </div>
      <div className="shelf-selection-id">{rackId}</div>
      <div className="shelf-selection-meta shelf-inventory-meta">
        <span>ZONE <b>{zone || "—"}</b></span>
        <span>FLOOR <b>{floor}</b></span>
        <span>ORDERS <b>{data?.shelf.current_load ?? load}/8</b></span>
        <span>LOAD <b>{data ? data.shelf.percent : percentLabel}%</b></span>
      </div>
      <div className="shelf-selection-position shelf-inventory-position">
        <span><small>X</small><b>{position[0].toFixed(3)} m</b></span>
        <span><small>Y</small><b>{position[2].toFixed(3)} m</b></span>
        <span><small>Z</small><b>{position[1].toFixed(3)} m</b></span>
      </div>

      {error && <div className="shelf-action-msg error">{error}</div>}
      {ok && <div className="shelf-action-msg ok">{ok}</div>}

      <div className="shelf-items-title">
        <div><b>ĐƠN HÀNG TRÊN KỆ</b><small>{data?.items.length ?? 0} item được theo dõi</small></div>
        <button className="btn" type="button" disabled={loading || busy} onClick={() => void refresh()}>{loading ? "Loading…" : "↻ Refresh"}</button>
      </div>

      <div className="shelf-item-list">
        {loading && !data ? <div className="shelf-items-empty">Đang tải dữ liệu…</div> : null}
        {!loading && data?.items.length === 0 ? <div className="shelf-items-empty">Shelf hiện không có đơn hàng.</div> : null}
        {data?.items.map((item, index) => {
          const reserved = item.status !== "STORED";
          return (
            <button
              key={item.id}
              type="button"
              className={`shelf-item-row ${selectedId === item.id ? "selected" : ""} ${reserved ? "reserved" : ""}`}
              onClick={() => !reserved && setSelectedId(item.id)}
              disabled={reserved}
            >
              <span className="shelf-item-slot">{index + 1}</span>
              <span className="shelf-item-main">
                <b>{itemTitle(item)}</b>
                <small>{item.item_name || "Warehouse item"}</small>
                <em>{item.origin_order_no ? `Inbound: ${item.origin_order_no}` : "Demo / initial stock"}{item.external_ref ? ` · Ref: ${item.external_ref}` : ""}</em>
              </span>
              <span className="shelf-item-side">
                <b>{item.quantity} qty</b>
                <small>{item.payload_weight_kg ? `${item.payload_weight_kg.toFixed(2)} kg` : `${item.load_units} load unit`}</small>
                <i className={reserved ? "busy" : "ready"}>{reserved ? `RESERVED ${item.reserved_by_order_no || ""}` : "READY"}</i>
              </span>
            </button>
          );
        })}
      </div>

      <div className="shelf-action-panel">
        <div className="shelf-action-panel-head">
          <div><b>DI CHUYỂN ĐƠN ĐÃ CHỌN</b><small>{selected ? itemTitle(selected) : "Chọn một item READY ở trên"}</small></div>
          <label className="shelf-priority-select">Priority
            <select value={priority} onChange={(e) => setPriority(e.target.value)} disabled={busy}>
              {['LOW', 'NORMAL', 'HIGH', 'CRITICAL'].map((x) => <option key={x}>{x}</option>)}
            </select>
          </label>
        </div>

        <button
          type="button"
          className={`unloaded-robot-toggle ${preferUnloaded ? "on" : ""}`}
          onClick={() => setPreferUnloaded((v) => !v)}
          disabled={busy}
          title="When enabled, robots already carrying/delivering goods are excluded. Idle robots are preferred; a robot going to pick another order may be queued if it is not carrying goods yet."
        >
          <span className="toggle-dot" />
          <span><b>Ưu tiên robot không chở hàng</b><small>{preferUnloaded ? "BẬT · loại robot đang TRANSPORTING / DELIVERING" : "TẮT · scheduler chọn robot theo lịch sớm nhất"}</small></span>
        </button>

        <div className="shelf-action-grid">
          <div className="shelf-action-box outbound">
            <div><b>OUTBOUND</b><small>Shelf hiện tại → cửa outbound gần nhất</small></div>
            <button className="btn primary" type="button" disabled={!selected || busy} onClick={() => void dispatch("OUTBOUND")}>
              {busy ? "Đang schedule…" : "Đưa ra OUTBOUND"}
            </button>
          </div>
          <div className="shelf-action-box transfer">
            <div><b>TRANSFER SHELF</b><small>Chọn bất kỳ shelf còn chỗ trong kho</small></div>
            <select value={destination} onChange={(e) => setDestination(e.target.value)} disabled={busy || freeDestinations.length === 0}>
              {freeDestinations.length === 0 ? <option value="">Không có shelf còn chỗ</option> : null}
              {freeDestinations.map((x) => (
                <option value={x.shelf_code} key={x.shelf_id}>{x.shelf_code} · {x.current_load}/8 · trống {x.available_slots}</option>
              ))}
            </select>
            <button className="btn" type="button" disabled={!selected || !destination || busy} onClick={() => void dispatch("TRANSFER")}>
              {busy ? "Đang schedule…" : "Chuyển tới shelf này"}
            </button>
          </div>
        </div>

        {lastDispatch && (
          <div className="shelf-robot-result">
            <span>ROBOT</span><b>{lastDispatch.schedule.robot_id}</b>
            <span>FSM</span><b>{lastDispatch.selected_robot.telemetry.fsm}</b>
            <span>LOAD</span><b>{lastDispatch.selected_robot.telemetry.load?.current ?? 0}/{lastDispatch.selected_robot.telemetry.load?.capacity ?? lastDispatch.selected_robot.payload_capacity}</b>
            <span>SCHEDULE</span><b>{lastDispatch.schedule.schedule_id}</b>
          </div>
        )}
      </div>
    </div>
  );
}
