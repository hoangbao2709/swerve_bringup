import { useEffect, useMemo, useState } from "react";
import { useStore } from "../../state/store";
import { onScheduleUpdated } from "../../services/ws";
import { schedulerApi, type ManagedRobot, type RobotSchedule, type SchedulePreview, type SchedulerOverview, type WarehouseOrder, type WorkPoint } from "../../services/scheduler";

const PRIORITIES = ["LOW","NORMAL","HIGH","CRITICAL"];
const ORDER_TYPES = ["MOVE","PICK","REPLENISH","INBOUND","OUTBOUND","TRANSFER"];

function localDatetimeValue(d = new Date(Date.now() + 10_000)) {
  const x = new Date(d.getTime() - d.getTimezoneOffset() * 60000);
  return x.toISOString().slice(0, 16);
}
function fmt(dt?: string | null) { return dt ? new Date(dt).toLocaleString() : "—"; }
function minutes(s: number) { return `${Math.max(1, Math.round(s / 60))} min`; }

export function SchedulerModal() {
  const setModal = useStore((s) => s.setModal);
  const [tab, setTab] = useState<"plan"|"orders"|"timeline">("plan");
  const [workpoints, setWorkpoints] = useState<WorkPoint[]>([]);
  const [robots, setRobots] = useState<ManagedRobot[]>([]);
  const [orders, setOrders] = useState<WarehouseOrder[]>([]);
  const [schedules, setSchedules] = useState<RobotSchedule[]>([]);
  const [overview, setOverview] = useState<SchedulerOverview | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const [message, setMessage] = useState("");

  const [orderId, setOrderId] = useState<number | null>(null);
  const [sourceId, setSourceId] = useState<number | null>(null);
  const [destinationId, setDestinationId] = useState<number | null>(null);
  const [priority, setPriority] = useState("HIGH");
  const [orderType, setOrderType] = useState("MOVE");
  const [loadUnits, setLoadUnits] = useState(1);
  const [quantity, setQuantity] = useState(1);
  const [payloadWeight, setPayloadWeight] = useState(0);
  const [externalRef, setExternalRef] = useState("");
  const [dueAt, setDueAt] = useState("");
  const [notes, setNotes] = useState("");
  const [plannedStart, setPlannedStart] = useState(localDatetimeValue());
  const [mode, setMode] = useState("AUTO");
  const [robotId, setRobotId] = useState("");
  const [via, setVia] = useState<number[]>([]);
  const [viaPick, setViaPick] = useState<number | "">("");
  const [preview, setPreview] = useState<SchedulePreview | null>(null);
  const [editingOrderId, setEditingOrderId] = useState<number | null>(null);
  const [editPriority, setEditPriority] = useState("NORMAL");
  const [editDueAt, setEditDueAt] = useState("");
  const [editNotes, setEditNotes] = useState("");
  const [editExternalRef, setEditExternalRef] = useState("");

  const reload = async () => {
    try {
      const [w, r, o, s, ov] = await Promise.all([schedulerApi.workpoints(), schedulerApi.robots(), schedulerApi.orders(), schedulerApi.schedules(), schedulerApi.overview()]);
      setWorkpoints(w); setRobots(r); setOrders(o); setSchedules(s); setOverview(ov);
      if (!sourceId && w.length) setSourceId(w.find((x) => x.kind === "SHELF")?.id ?? w[0].id);
      if (!destinationId && w.length) setDestinationId(w.find((x) => x.kind === "OUTBOUND")?.id ?? w.find((x) => x.kind === "CONVEYOR_IN")?.id ?? w[w.length - 1].id);
    } catch (e) { setError(e instanceof Error ? e.message : String(e)); }
  };
  useEffect(() => { void reload(); return onScheduleUpdated(() => { void reload(); }); }, []);

  const selectedOrder = orders.find((o) => o.id === orderId) ?? null;
  useEffect(() => {
    if (!selectedOrder) return;
    setSourceId(selectedOrder.source.id); setDestinationId(selectedOrder.destination.id);
    setPriority(selectedOrder.priority); setOrderType(selectedOrder.type); setLoadUnits(selectedOrder.load_units);
    setQuantity(selectedOrder.quantity); setPayloadWeight(selectedOrder.payload_weight_kg); setExternalRef(selectedOrder.external_ref || "");
    setDueAt(selectedOrder.due_at ? localDatetimeValue(new Date(selectedOrder.due_at)) : ""); setNotes(selectedOrder.notes || ""); setPreview(null);
  }, [selectedOrder?.id]);

  const selectableVia = useMemo(() => workpoints.filter((w) => w.enabled && w.id !== sourceId && w.id !== destinationId && !via.includes(w.id)), [workpoints, sourceId, destinationId, via]);
  const wpById = useMemo(() => Object.fromEntries(workpoints.map((w) => [w.id, w])) as Record<number, WorkPoint>, [workpoints]);

  async function ensureOrder(): Promise<WarehouseOrder | null> {
    if (selectedOrder) return selectedOrder;
    if (!sourceId || !destinationId) { setError("Select source and destination."); return null; }
    const created = await schedulerApi.createOrder({
      source_id: sourceId, destination_id: destinationId, priority, type: orderType,
      load_units: loadUnits, quantity, payload_weight_kg: payloadWeight, external_ref: externalRef,
      due_at: dueAt ? new Date(dueAt).toISOString() : null, notes,
    });
    setOrderId(created.id); setOrders((x) => [created, ...x]);
    return created;
  }

  async function doPreview() {
    setBusy(true); setError(""); setMessage("");
    try {
      const order = await ensureOrder(); if (!order) return;
      const p = await schedulerApi.preview({ order_id: order.id, planned_start: new Date(plannedStart).toISOString(), route_workpoint_ids: via, robot_id: mode === "AUTO" ? "" : robotId });
      setPreview(p);
      if (!robotId && p.recommended) setRobotId(p.recommended.robot_id);
      setMessage(`Order ${order.order_no} saved. Scheduler recommendation is ready.`);
    } catch (e) { setError(e instanceof Error ? e.message : String(e)); }
    finally { setBusy(false); }
  }

  async function createPlan() {
    if (!preview || !orderId) { await doPreview(); return; }
    setBusy(true); setError("");
    try {
      const s = await schedulerApi.createSchedule({ order_id: orderId, planned_start: new Date(plannedStart).toISOString(), mode, robot_id: mode === "AUTO" ? "" : robotId, route_workpoint_ids: via });
      setMessage(`${s.schedule_id} created for ${s.robot_id}. The simulated robot will execute this route when due.`);
      setPreview(null); setOrderId(null); setVia([]); setRobotId(""); setPlannedStart(localDatetimeValue());
      await reload(); setTab("timeline");
    } catch (e) { setError(e instanceof Error ? e.message : String(e)); }
    finally { setBusy(false); }
  }

  async function cancelSchedule(id: number) {
    if (!confirm("Cancel this schedule?")) return;
    try { await schedulerApi.cancelSchedule(id); await reload(); }
    catch (e) { setError(e instanceof Error ? e.message : String(e)); }
  }

  async function cancelOrder(id: number) {
    if (!confirm("Cancel this order and any active schedule?")) return;
    try { await schedulerApi.cancelOrder(id); if (orderId === id) { setOrderId(null); setPreview(null); } await reload(); }
    catch (e) { setError(e instanceof Error ? e.message : String(e)); }
  }

  function beginEditOrder(o: WarehouseOrder) {
    setEditingOrderId(o.id); setEditPriority(o.priority); setEditNotes(o.notes || ""); setEditExternalRef(o.external_ref || "");
    setEditDueAt(o.due_at ? localDatetimeValue(new Date(o.due_at)) : ""); setError("");
  }

  async function saveOrderEdit() {
    if (!editingOrderId) return;
    setBusy(true); setError("");
    try {
      await schedulerApi.updateOrder(editingOrderId, { priority: editPriority, due_at: editDueAt ? new Date(editDueAt).toISOString() : null, notes: editNotes, external_ref: editExternalRef });
      setEditingOrderId(null); setMessage("Order updated in database."); await reload();
    } catch (e) { setError(e instanceof Error ? e.message : String(e)); }
    finally { setBusy(false); }
  }

  async function deleteOrder(id: number) {
    if (!confirm("Delete this order permanently? Completed history and active schedules cannot be deleted.")) return;
    try { await schedulerApi.deleteOrder(id); if (orderId === id) setOrderId(null); setEditingOrderId(null); await reload(); }
    catch (e) { setError(e instanceof Error ? e.message : String(e)); }
  }

  const activeSchedules = schedules.filter((s) => ["PLANNED","QUEUED","RUNNING"].includes(s.status));
  const selectedCandidate = preview?.candidates.find((c) => c.robot_id === robotId) ?? null;
  const manualRobotInvalid = mode !== "AUTO" && (!robotId || !selectedCandidate?.eligible);
  const grouped = robots.map((r) => ({ robot: r, rows: activeSchedules.filter((s) => s.robot_id === r.robot_id).sort((a,b) => +new Date(a.planned_start) - +new Date(b.planned_start)) }));

  return <div className="scheduler-wrap">
    <div className="scheduler-head">
      <div><small>FLEET ORCHESTRATION</small><h2>Robot Scheduler & Orders</h2><p>Persist orders, select robot work-points, reserve shared resources and execute planned routes in the simulation.</p></div>
      <div className="scheduler-head-actions"><button className="btn" onClick={() => void reload()}>Refresh</button><button className="icon-btn" onClick={() => setModal(null)}>✕</button></div>
    </div>

    <div className="scheduler-kpis">
      <div><small>ORDERS</small><b>{overview?.orders.total ?? orders.length}</b><span>{overview?.orders.running ?? 0} running</span></div>
      <div><small>ACTIVE SCHEDULES</small><b>{overview?.schedules.active ?? activeSchedules.length}</b><span>{overview?.schedules.running ?? 0} executing</span></div>
      <div><small>ROBOTS</small><b>{overview?.robots ?? robots.length}</b><span>{robots.filter((r) => r.telemetry.fsm === "IDLE").length} idle</span></div>
      <div><small>WORK POINTS</small><b>{overview?.workpoints ?? workpoints.length}</b><span>{new Set(workpoints.map((w) => w.kind)).size} types</span></div>
    </div>

    <div className="scheduler-tabs">
      <button className={tab === "plan" ? "active" : ""} onClick={() => setTab("plan")}>Create schedule</button>
      <button className={tab === "orders" ? "active" : ""} onClick={() => setTab("orders")}>Orders <i>{orders.length}</i></button>
      <button className={tab === "timeline" ? "active" : ""} onClick={() => setTab("timeline")}>Timeline <i>{activeSchedules.length}</i></button>
    </div>
    {error && <div className="scheduler-alert error" onClick={() => setError("")}>{error}</div>}
    {message && <div className="scheduler-alert ok" onClick={() => setMessage("")}>{message}</div>}

    {tab === "plan" && <div className="scheduler-plan">
      <section className="scheduler-form-card">
        <div className="scheduler-section-title">1. Order</div>
        <label>Existing order<select value={orderId ?? ""} onChange={(e) => { setOrderId(e.target.value ? Number(e.target.value) : null); setPreview(null); }}><option value="">New order</option>{orders.filter((o) => o.status === "NEW").map((o) => <option key={o.id} value={o.id}>{o.order_no} · {o.status}</option>)}</select></label>
        <div className="scheduler-grid2">
          <label>Order type<select disabled={!!selectedOrder} value={orderType} onChange={(e) => setOrderType(e.target.value)}>{ORDER_TYPES.map((x) => <option key={x}>{x}</option>)}</select></label>
          <label>Priority<select disabled={!!selectedOrder} value={priority} onChange={(e) => setPriority(e.target.value)}>{PRIORITIES.map((x) => <option key={x}>{x}</option>)}</select></label>
          <label>Load units<input disabled={!!selectedOrder} type="number" min={1} max={4} value={loadUnits} onChange={(e) => setLoadUnits(Number(e.target.value))}/></label>
          <label>Quantity<input disabled={!!selectedOrder} type="number" min={1} value={quantity} onChange={(e) => setQuantity(Number(e.target.value))}/></label>
          <label>Payload kg<input disabled={!!selectedOrder} type="number" min={0} step="0.1" value={payloadWeight} onChange={(e) => setPayloadWeight(Number(e.target.value))}/></label>
          <label>Due time<input disabled={!!selectedOrder} type="datetime-local" value={dueAt} onChange={(e) => setDueAt(e.target.value)}/></label>
        </div>
        <label>External / ERP reference<input disabled={!!selectedOrder} value={externalRef} onChange={(e) => setExternalRef(e.target.value)} placeholder="Optional order reference"/></label>
        <label>Notes<textarea disabled={!!selectedOrder} rows={2} value={notes} onChange={(e) => setNotes(e.target.value)} placeholder="Handling notes, product or pallet information…"/></label>
        <label>From<select disabled={!!selectedOrder} value={sourceId ?? ""} onChange={(e) => { setSourceId(Number(e.target.value)); setPreview(null); }}>{workpoints.map((w) => <option key={w.id} value={w.id}>[{w.kind}] {w.code} · F{w.floor} · ({w.x.toFixed(1)}, {w.y.toFixed(1)})</option>)}</select></label>
        <label>To<select disabled={!!selectedOrder} value={destinationId ?? ""} onChange={(e) => { setDestinationId(Number(e.target.value)); setPreview(null); }}>{workpoints.map((w) => <option key={w.id} value={w.id}>[{w.kind}] {w.code} · F{w.floor} · ({w.x.toFixed(1)}, {w.y.toFixed(1)})</option>)}</select></label>

        <div className="scheduler-section-title">2. Route</div>
        <div className="route-chain"><span>{sourceId ? wpById[sourceId]?.code : "FROM"}</span>{via.map((id, i) => <span key={id} className="via">→ <b>{wpById[id]?.code}</b><button onClick={() => { setVia((x) => x.filter((_,j) => j !== i)); setPreview(null); }}>×</button></span>)}<span>→ {destinationId ? wpById[destinationId]?.code : "TO"}</span></div>
        <div className="scheduler-inline"><select value={viaPick} onChange={(e) => setViaPick(e.target.value ? Number(e.target.value) : "")}><option value="">Add intermediate work-point…</option>{selectableVia.map((w) => <option key={w.id} value={w.id}>[{w.kind}] {w.code}</option>)}</select><button className="btn" disabled={!viaPick} onClick={() => { if (viaPick) { setVia((x) => [...x, Number(viaPick)]); setViaPick(""); setPreview(null); } }}>Add stop</button></div>

        <div className="scheduler-section-title">3. Time & dispatch</div>
        <div className="scheduler-grid2">
          <label>Planned start<input type="datetime-local" value={plannedStart} onChange={(e) => { setPlannedStart(e.target.value); setPreview(null); }}/></label>
          <label>Mode<select value={mode} onChange={(e) => setMode(e.target.value)}><option value="AUTO">AUTO · best robot</option><option value="SEMI_AUTO">SEMI-AUTO · recommendation</option><option value="MANUAL">MANUAL · selected robot</option></select></label>
        </div>
        {mode !== "AUTO" && <label>Robot<select value={robotId} onChange={(e) => setRobotId(e.target.value)}><option value="">Select robot</option>{robots.map((r) => <option key={r.id} value={r.robot_id}>{r.robot_id} · {r.telemetry.battery.toFixed(0)}% · {r.telemetry.fsm}</option>)}</select></label>}
        <div className="scheduler-form-actions"><button className="btn" disabled={busy} onClick={() => void doPreview()}>{busy ? "Calculating…" : "Validate & preview"}</button><button className="btn primary" disabled={busy || !preview || !!preview?.resource_conflicts?.length || manualRobotInvalid} onClick={() => void createPlan()}>Create schedule</button></div>
      </section>

      <section className="scheduler-recommend">
        <div className="scheduler-section-title">Recommendation</div>
        {!preview && <div className="scheduler-empty">Choose the order, route and planned start, then validate. The scheduler evaluates battery, distance, availability, workload and conflicts.</div>}
        {preview && <>
          <div className="route-summary"><small>PLANNED ROUTE</small><b>{preview.route.map((x) => x.code).join("  →  ")}</b></div>
          {!!preview.resource_conflicts?.length && <div className="scheduler-resource-conflict"><b>Shared resource conflict</b>{preview.resource_conflicts.map((c) => <span key={`${c.resource_type}-${c.resource_id}`}>{c.workpoint} conflicts with {c.conflicting_schedule} · {fmt(c.starts_at)}</span>)}</div>}
          {preview.candidates.slice(0, 8).map((c, i) => <div className={`candidate ${c.eligible ? "" : "rejected"} ${preview.recommended?.robot_id === c.robot_id ? "recommended" : ""}`} key={c.robot_id}>
            <div><b>{c.robot_id}</b><small>{i === 0 && c.eligible ? "RECOMMENDED" : c.eligible ? "ELIGIBLE" : "REJECTED"}</small></div>
            <span>Score <b>{c.score}</b></span><span>{c.distance_m.toFixed(1)} m</span><span>{minutes(c.duration_s)}</span><span>{c.battery.toFixed(0)}%</span>
            <em>{c.rejected_reason ?? c.reasons.join(" · ")}</em>
          </div>)}
        </>}
      </section>
    </div>}

    {tab === "orders" && <div className="scheduler-orders-manage">
      <div className="scheduler-table-wrap"><table className="scheduler-table"><thead><tr><th>Order</th><th>Type</th><th>Route</th><th>Priority</th><th>Status</th><th>Load</th><th>Due</th><th>Created</th><th>Action</th></tr></thead><tbody>{orders.map((o) => { const active = activeSchedules.some((s) => s.order_id === o.id); return <tr key={o.id}><td><b>{o.order_no}</b><small>{o.external_ref || ""}</small></td><td>{o.type}</td><td>{o.source.code} → {o.destination.code}</td><td><span className={`pill ${o.priority.toLowerCase()}`}>{o.priority}</span></td><td>{o.status}</td><td>{o.load_units}</td><td>{fmt(o.due_at)}</td><td>{fmt(o.created_at)}</td><td><div style={{display:"flex",gap:4,flexWrap:"wrap"}}>{o.status === "NEW" && !active && <button className="btn" onClick={() => { setOrderId(o.id); setTab("plan"); }}>Plan</button>}{o.status === "NEW" && !active && <button className="btn" onClick={() => beginEditOrder(o)}>Edit</button>}{!["COMPLETED","FAILED","CANCELLED"].includes(o.status) && <button className="btn" onClick={() => void cancelOrder(o.id)}>Cancel</button>}{!active && o.status !== "COMPLETED" && <button className="btn danger" onClick={() => void deleteOrder(o.id)}>Delete</button>}</div></td></tr>})}</tbody></table></div>
      {editingOrderId && (() => { const o = orders.find((x) => x.id === editingOrderId); return o ? <aside className="scheduler-order-editor"><div className="scheduler-section-title">Edit {o.order_no}</div><label>Priority<select value={editPriority} onChange={(e) => setEditPriority(e.target.value)}>{PRIORITIES.map((x) => <option key={x}>{x}</option>)}</select></label><label>Due<input type="datetime-local" value={editDueAt} onChange={(e) => setEditDueAt(e.target.value)}/></label><label>External reference<input value={editExternalRef} onChange={(e) => setEditExternalRef(e.target.value)}/></label><label>Notes<textarea rows={5} value={editNotes} onChange={(e) => setEditNotes(e.target.value)}/></label><div className="scheduler-form-actions"><button className="btn" onClick={() => setEditingOrderId(null)}>Close</button><button className="btn primary" disabled={busy} onClick={() => void saveOrderEdit()}>Save changes</button></div></aside> : null; })()}
    </div>}

    {tab === "timeline" && <div className="scheduler-timeline">
      {grouped.map(({robot,rows}) => <div className="timeline-row" key={robot.robot_id}><div className="timeline-robot"><b>{robot.robot_id}</b><span>{robot.telemetry.fsm} · {robot.telemetry.battery.toFixed(0)}%</span></div><div className="timeline-items">{rows.length ? rows.map((s) => <div className={`timeline-item ${s.status.toLowerCase()}`} key={s.id}><div><b>{s.order_no}</b><span>{s.schedule_id}</span></div><div>{fmt(s.planned_start)} → {s.planned_end ? new Date(s.planned_end).toLocaleTimeString([], {hour:"2-digit",minute:"2-digit"}) : "—"}</div><div>{s.status} · {s.estimated_distance_m.toFixed(1)}m</div>{!['COMPLETED','FAILED','CANCELLED'].includes(s.status) && <button onClick={() => void cancelSchedule(s.id)}>Cancel</button>}</div>) : <span className="timeline-empty">No planned work</span>}</div></div>)}
    </div>}
  </div>;
}
