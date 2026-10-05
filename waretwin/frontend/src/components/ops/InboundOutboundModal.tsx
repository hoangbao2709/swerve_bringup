import { useEffect, useMemo, useState } from "react";

import { useStore } from "../../state/store";
import { onScheduleUpdated } from "../../services/ws";

import {
  schedulerApi,
  type OrderImportResult,
  type WarehouseOrder,
  type WorkPoint,
} from "../../services/scheduler";

import {
  warehouseApi,
  type ShelfRecord,
} from "../../services/warehouse";

const ACTIVE = new Set([
  "NEW",
  "PLANNED",
  "QUEUED",
  "RUNNING",
]);

const DONE = new Set([
  "COMPLETED",
  "CANCELLED",
  "FAILED",
]);

type FlowMode = "INBOUND" | "OUTBOUND";

function flowKind(point: WorkPoint): string {
  const kind = String(point.kind || "").toUpperCase();

  if (kind !== "DOCK") {
    return kind;
  }

  const code = String(point.code || "").toUpperCase();

  if (code.startsWith("INBOUND")) {
    return "INBOUND";
  }

  if (code.startsWith("OUTBOUND")) {
    return "OUTBOUND";
  }

  const meta = point.metadata || {};

  return String(
    (meta as Record<string, unknown>).dock_kind || kind,
  ).toUpperCase();
}

function shelfRackId(point: WorkPoint): string {
  return String(point.resource_id || point.code || "");
}

function fmtDate(
  value: string | null | undefined,
) {
  if (!value) {
    return "—";
  }

  const date = new Date(value);

  if (Number.isNaN(date.getTime())) {
    return "—";
  }

  return date.toLocaleString([], {
    month: "2-digit",
    day: "2-digit",
    hour: "2-digit",
    minute: "2-digit",
  });
}

function statusClass(status: string) {
  switch (status) {
    case "COMPLETED":
      return "success";

    case "RUNNING":
      return "running";

    case "FAILED":
    case "CANCELLED":
      return "danger";

    default:
      return "waiting";
  }
}

function fileLabel(file: File | null) {
  if (!file) {
    return "Chưa chọn file";
  }

  const kb = Math.max(
    1,
    Math.round(file.size / 1024),
  );

  return `${file.name} Â· ${kb} KB`;
}

function downloadJsonExample(
  flow: FlowMode,
) {
  const common = {
    order_no:
      flow === "INBOUND"
        ? "IN-0001"
        : "OUT-0001",

    external_ref:
      flow === "INBOUND"
        ? "ASN-001"
        : "SO-001",

    item_code: "SKU-001",
    item_name: "Demo item",
    quantity: 1,
    priority: "NORMAL",
    payload_weight_kg: 2.5,

    due_at: new Date(
      Date.now() + 60 * 60 * 1000,
    ).toISOString(),

    notes: "Imported from file",
  };

  const row =
    flow === "INBOUND"
      ? {
          ...common,
          dock_code: "INBOUND-1",
          shelf_code: "",
        }
      : {
          ...common,
          shelf_code: "RACK-A101",
          dock_code: "OUTBOUND-1",
        };

  const blob = new Blob(
    [
      JSON.stringify(
        {
          orders: [row],
        },
        null,
        2,
      ),
    ],
    {
      type: "application/json",
    },
  );

  const url =
    URL.createObjectURL(blob);

  const anchor =
    document.createElement("a");

  anchor.href = url;

  anchor.download =
    `${flow.toLowerCase()}_orders_example.json`;

  anchor.click();

  URL.revokeObjectURL(url);
}

export function InboundOutboundModal() {
  const openWindow = useStore((state) => state.openWindow);


  const [mode, setMode] =
    useState<FlowMode>("INBOUND");

  const [
    workpoints,
    setWorkpoints,
  ] = useState<WorkPoint[]>([]);

  const [
    orders,
    setOrders,
  ] = useState<WarehouseOrder[]>([]);

  const [
    shelves,
    setShelves,
  ] = useState<ShelfRecord[]>([]);

  const [
    inboundFile,
    setInboundFile,
  ] = useState<File | null>(null);

  const [
    outboundFile,
    setOutboundFile,
  ] = useState<File | null>(null);

  const [
    importResult,
    setImportResult,
  ] = useState<OrderImportResult | null>(
    null,
  );

  const [
    status,
    setStatus,
  ] = useState("ACTIVE");

  const [
    search,
    setSearch,
  ] = useState("");

  const [
    busy,
    setBusy,
  ] = useState(false);

  const [
    loading,
    setLoading,
  ] = useState(true);

  const [
    message,
    setMessage,
  ] = useState<{
    kind: "ok" | "error";
    text: string;
  } | null>(null);

  const load = async () => {
    try {
      const overview =
        await schedulerApi.overview();

      const [
        workpointRows,
        orderRows,
        shelfRows,
      ] = await Promise.all([
        schedulerApi.workpoints(),

        schedulerApi.orders(),

        warehouseApi.shelves({
          warehouse:
            overview.warehouse_id,
        }),
      ]);

      setWorkpoints(workpointRows);

      setOrders(orderRows);

      setShelves(shelfRows);
    } catch (error) {
      setMessage({
        kind: "error",

        text:
          error instanceof Error
            ? error.message
            : "Không thể tải dữ liệu Inbound / Outbound.",
      });
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    void load();

    const timer =
      window.setInterval(
        () => void load(),
        5000,
      );

    const unsubscribe =
      onScheduleUpdated(
        () => void load(),
      );

    return () => {
      window.clearInterval(timer);

      unsubscribe();
    };
  }, []);


  const inboundDocks =
    useMemo(
      () =>
        workpoints.filter(
          (point) =>
            point.enabled &&
            flowKind(point) ===
              "INBOUND",
        ),

      [workpoints],
    );

  const outboundDocks =
    useMemo(
      () =>
        workpoints.filter(
          (point) =>
            point.enabled &&
            flowKind(point) ===
              "OUTBOUND",
        ),

      [workpoints],
    );

  const shelfByRack =
    useMemo(
      () =>
        new Map(
          shelves.map((shelf) => [
            shelf.layout_rack_id ||
              shelf.code,

            shelf,
          ]),
        ),

      [shelves],
    );

  const flowOrders =
    useMemo(
      () =>
        orders.filter(
          (order) =>
            order.type === mode,
        ),

      [orders, mode],
    );

  const filtered =
    useMemo(() => {
      return flowOrders.filter(
        (order) => {
          if (
            status === "ACTIVE" &&
            !ACTIVE.has(order.status)
          ) {
            return false;
          }

          if (
            status === "DONE" &&
            !DONE.has(order.status)
          ) {
            return false;
          }

          if (
            status !== "ALL" &&
            status !== "ACTIVE" &&
            status !== "DONE" &&
            order.status !== status
          ) {
            return false;
          }

          const itemCode =
            String(
              order.metadata
                ?.item_code || "",
            );

          const haystack = [
            order.order_no,
            order.external_ref,
            itemCode,
            order.source.code,
            order.destination.code,
            order.status,
          ]
            .join(" ")
            .toLowerCase();

          const needle =
            search.trim().toLowerCase();

          return (
            !needle ||
            haystack.includes(needle)
          );
        },
      );
    }, [
      flowOrders,
      status,
      search,
    ]);

  const counts =
    useMemo(() => {
      const count = (
        type: FlowMode,
        predicate: (
          order: WarehouseOrder,
        ) => boolean,
      ) =>
        orders.filter(
          (order) =>
            order.type === type &&
            predicate(order),
        ).length;

      return {
        inboundActive: count(
          "INBOUND",
          (order) =>
            ACTIVE.has(order.status),
        ),

        inboundRunning: count(
          "INBOUND",
          (order) =>
            order.status ===
            "RUNNING",
        ),

        outboundActive: count(
          "OUTBOUND",
          (order) =>
            ACTIVE.has(order.status),
        ),

        outboundRunning: count(
          "OUTBOUND",
          (order) =>
            order.status ===
            "RUNNING",
        ),
      };
    }, [orders]);

  const currentFile =
    mode === "INBOUND"
      ? inboundFile
      : outboundFile;

  const selectFile = (
    flow: FlowMode,
    file: File | null,
  ) => {
    if (file) {
      const extension =
        file.name
          .toLowerCase()
          .split(".")
          .pop();

      if (
        extension !== "xlsx" &&
        extension !== "json"
      ) {
        setMessage({
          kind: "error",

          text:
            "Chỉ hỗ trợ file .xlsx hoặc .json.",
        });

        return;
      }
    }

    if (flow === "INBOUND") {
      setInboundFile(file);

      if (file) {
        setOutboundFile(null);
      }
    } else {
      setOutboundFile(file);

      if (file) {
        setInboundFile(null);
      }
    }

    setImportResult(null);

    setMessage(null);
  };

  const clearCurrentFile = () => {
    if (mode === "INBOUND") {
      setInboundFile(null);
    } else {
      setOutboundFile(null);
    }

    setImportResult(null);
  };

  const importAndSchedule =
    async () => {
      if (
        !inboundFile &&
        !outboundFile
      ) {
        setMessage({
          kind: "error",

          text:
            "Hãy chọn file INBOUND hoặc OUTBOUND.",
        });

        return;
      }

      if (
        inboundFile &&
        outboundFile
      ) {
        setMessage({
          kind: "error",

          text:
            "Hãy chạy INBOUND trước. Khi hàng đã được đưa lên shelf mới chạy OUTBOUND.",
        });

        return;
      }

      const body =
        new FormData();

      if (inboundFile) {
        body.append(
          "inbound_file",
          inboundFile,
        );
      }

      if (outboundFile) {
        body.append(
          "outbound_file",
          outboundFile,
        );
      }

      setBusy(true);

      setMessage(null);

      setImportResult(null);

      try {
        const result =
          await schedulerApi.importOrders(
            body,
          );

        setImportResult(result);

        const summary =
          result.summary;

        setMessage({
          kind:
            summary.failed ||
            summary.unscheduled
              ? "error"
              : "ok",

          text:
            `Batch ${result.batch_id}: ` +
            `${summary.scheduled}/${summary.rows} đơn đã được schedule, ` +
            `${summary.failed} lỗi, ` +
            `${summary.unscheduled} đang chờ robot, ` +
            `${summary.duplicates} trùng.`,
        });

        setInboundFile(null);

        setOutboundFile(null);

        await load();
      } catch (error) {
        setMessage({
          kind: "error",

          text:
            error instanceof Error
              ? error.message
              : "Import tháº¥t báº¡i.",
        });
      } finally {
        setBusy(false);
      }
    };

  const cancel =
    async (
      order: WarehouseOrder,
    ) => {
      const confirmed =
        window.confirm(
          `Hủy đơn ${order.order_no}?`,
        );

      if (!confirmed) {
        return;
      }

      setBusy(true);

      try {
        await schedulerApi.cancelOrder(
          order.id,
        );

        setMessage({
          kind: "ok",

          text:
            `${order.order_no} đã được hủy.`,
        });

        await load();
      } catch (error) {
        setMessage({
          kind: "error",

          text:
            error instanceof Error
              ? error.message
              : "Không thể hủy đơn.",
        });
      } finally {
        setBusy(false);
      }
    };

  return (
      <div className="flow-wrap">
          {/* ================= HEADER ================= */}

          <header className="flow-head">
            <div className="flow-title">
              <small>
                WAREHOUSE ORDER MANAGEMENT
              </small>

              <h2>
                Inbound / Outbound
              </h2>

              <p>
                Import đơn hàng và tự động
                phân lịch cho robot.
              </p>
            </div>

            <div className="flow-head-actions">
              <button
                type="button"
                className="flow-btn"
                disabled={busy}
                onClick={() =>
                  void load()
                }
              >
                ↻ Refresh
              </button>

              <button
                type="button"
                className="flow-btn"
                onClick={() => openWindow({ id: "scheduler", kind: "scheduler", title: "Robot Scheduler" })}
              >
                Robot Scheduler →
              </button>
            </div>
          </header>

          {/* ================= KPI ================= */}

          <section className="flow-kpis">
            <div className="flow-kpi">
              <small>
                INBOUND ACTIVE
              </small>

              <b>
                {counts.inboundActive}
              </b>

              <span>
                {
                  counts.inboundRunning
                }{" "}
                đang chạy
              </span>
            </div>

            <div className="flow-kpi">
              <small>
                OUTBOUND ACTIVE
              </small>

              <b>
                {
                  counts.outboundActive
                }
              </b>

              <span>
                {
                  counts.outboundRunning
                }{" "}
                đang chạy
              </span>
            </div>

            <div className="flow-kpi">
              <small>
                INBOUND DOCK
              </small>

              <b>
                {inboundDocks.length}
              </b>

              <span>
                điểm nhận hàng
              </span>
            </div>

            <div className="flow-kpi">
              <small>
                OUTBOUND DOCK
              </small>

              <b>
                {outboundDocks.length}
              </b>

              <span>
                {shelves.length} shelf
              </span>
            </div>
          </section>

          {/* ================= MODE ================= */}

          <div className="flow-tabs">
            <button
              type="button"
              className={
                mode === "INBOUND"
                  ? "flow-tab active"
                  : "flow-tab"
              }
              onClick={() => {
                setMode("INBOUND");
                setMessage(null);
              }}
            >
              <span className="flow-tab-icon">
                ↓
              </span>

              <span className="flow-tab-text">
                <b>INBOUND</b>

                <span>
                  Dock → Shelf
                </span>
              </span>
            </button>

            <button
              type="button"
              className={
                mode === "OUTBOUND"
                  ? "flow-tab active"
                  : "flow-tab"
              }
              onClick={() => {
                setMode("OUTBOUND");
                setMessage(null);
              }}
            >
              <span className="flow-tab-icon">
                ↑
              </span>

              <span className="flow-tab-text">
                <b>OUTBOUND</b>

                <span>
                  Shelf → Dock
                </span>
              </span>
            </button>
          </div>

          {/* ================= MESSAGE ================= */}

          {message && (
            <div
              className={
                `flow-message ` +
                message.kind
              }
            >
              {message.text}
            </div>
          )}

          {/* ================= CONTENT ================= */}

          <main className="flow-body">
            {/* ===== LEFT IMPORT PANEL ===== */}

            <aside className="flow-import-panel">
              <div className="flow-section-title">
                1. CHỌN FILE ĐƠN HÀNG
              </div>

              <section className="flow-upload">
                <div className="flow-upload-head">
                  <b>
                    {mode ===
                    "INBOUND"
                      ? "Nhập hàng vào kho"
                      : "Xuất hàng khỏi kho"}
                  </b>

                  <span>
                    {mode ===
                    "INBOUND"
                      ? "Robot nhận hàng tại Inbound Dock và đưa lên shelf."
                      : "Robot lấy hàng từ shelf và đưa tới Outbound Dock."}
                  </span>
                </div>

                <label className="flow-dropzone">
                  <input
                    type="file"
                    accept=".xlsx,.json,application/json,application/vnd.openxmlformats-officedocument.spreadsheetml.sheet"
                    onChange={(
                      event,
                    ) =>
                      selectFile(
                        mode,
                        event
                          .target
                          .files?.[0] ||
                          null,
                      )
                    }
                  />

                  <span className="flow-drop-icon">
                    ⇧
                  </span>

                  <b>
                    Chọn Excel hoặc JSON
                  </b>

                  <span>
                    .xlsx Â· .json
                  </span>
                </label>

                <div className="flow-file">
                  <span className="flow-file-name">
                    {fileLabel(
                      currentFile,
                    )}
                  </span>

                  {currentFile && (
                    <button
                      type="button"
                      className="flow-file-remove"
                      title="Remove file"
                      onClick={
                        clearCurrentFile
                      }
                    >
                      ×
                    </button>
                  )}
                </div>

                <div className="flow-mini-actions">
                  <button
                    type="button"
                    className="flow-btn"
                    onClick={() =>
                      downloadJsonExample(
                        mode,
                      )
                    }
                  >
                    Táº£i JSON máº«u
                  </button>
                </div>
              </section>

              {/* ===== HELP COLLAPSED ===== */}

              <details className="flow-help">
                <summary>
                  Format file cần những cột nào?
                </summary>

                <div className="flow-help-content">
                  <p>
                    <code>
                      order_no
                    </code>
                    ,{" "}
                    <code>
                      item_code
                    </code>
                    ,{" "}
                    <code>
                      quantity
                    </code>
                    ,{" "}
                    <code>
                      priority
                    </code>
                    ,{" "}
                    <code>
                      shelf_code
                    </code>
                    ,{" "}
                    <code>
                      dock_code
                    </code>
                    ,{" "}
                    <code>
                      payload_weight_kg
                    </code>
                    ,{" "}
                    <code>
                      due_at
                    </code>
                  </p>

                  <p>
                    <strong>
                      INBOUND:
                    </strong>{" "}
                    shelf_code có thể
                    để trống. Backend
                    tự chọn shelf còn
                    chỗ.
                  </p>

                  <p>
                    <strong>
                      OUTBOUND:
                    </strong>{" "}
                    shelf_code là vị
                    trí hiện tại của
                    hàng.
                  </p>

                  <p>
                    1 dòng file =
                    1 đơn hàng =
                    1 slot shelf =
                    12.5%.
                  </p>

                  <p>
                    Nên chạy INBOUND
                    hoàn thành trước,
                    sau đó mới chạy
                    OUTBOUND.
                  </p>
                </div>
              </details>

              {/* ===== IMPORT ===== */}

              <button
                type="button"
                className="flow-btn primary flow-run"
                disabled={
                  busy ||
                  loading ||
                  !currentFile
                }
                onClick={() =>
                  void importAndSchedule()
                }
              >
                {busy
                  ? "Đang import & schedule..."
                  : mode ===
                      "INBOUND"
                    ? "Import INBOUND & Auto Schedule"
                    : "Import OUTBOUND & Auto Schedule"}
              </button>

              {/* ===== RESULT ===== */}

              {importResult && (
                <section className="flow-result">
                  <div className="flow-result-grid">
                    <div>
                      <small>
                        TOTAL
                      </small>

                      <b>
                        {
                          importResult
                            .summary
                            .rows
                        }
                      </b>
                    </div>

                    <div>
                      <small>
                        SCHEDULED
                      </small>

                      <b>
                        {
                          importResult
                            .summary
                            .scheduled
                        }
                      </b>
                    </div>

                    <div>
                      <small>
                        WAITING
                      </small>

                      <b>
                        {
                          importResult
                            .summary
                            .unscheduled
                        }
                      </b>
                    </div>

                    <div>
                      <small>
                        FAILED
                      </small>

                      <b>
                        {
                          importResult
                            .summary
                            .failed
                        }
                      </b>
                    </div>
                  </div>

                  <div className="flow-result-errors">
                    {importResult.results
                      .filter(
                        (result) =>
                          result.status ===
                            "FAILED" ||
                          result.status ===
                            "UNSCHEDULED",
                      )
                      .slice(0, 10)
                      .map(
                        (
                          result,
                          index,
                        ) => (
                          <div
                            className="flow-result-error"
                            key={`${result.filename}-${result.row}-${index}`}
                          >
                            <b>
                              {
                                result.flow
                              }{" "}
                              Â· row{" "}
                              {
                                result.row
                              }
                            </b>

                            <span>
                              {result.order_no ||
                                result.item_code ||
                                result.filename}
                            </span>

                            <em>
                              {result.message ||
                                "Not scheduled"}
                            </em>
                          </div>
                        ),
                      )}
                  </div>
                </section>
              )}
            </aside>

            {/* ===== RIGHT ORDER LIST ===== */}

            <section className="flow-list">
              <header className="flow-list-head">
                <div className="flow-list-title">
                  <small>
                    {mode} ORDERS
                  </small>

                  <b>
                    {filtered.length}{" "}
                    hiển thị ·{" "}
                    {
                      flowOrders.length
                    }{" "}
                    tổng
                  </b>
                </div>

                <div className="flow-filters">
                  <input
                    className="flow-input"
                    value={search}
                    placeholder="Tìm order, SKU, shelf..."
                    onChange={(
                      event,
                    ) =>
                      setSearch(
                        event.target
                          .value,
                      )
                    }
                  />

                  <select
                    className="flow-select"
                    value={status}
                    onChange={(
                      event,
                    ) =>
                      setStatus(
                        event.target
                          .value,
                      )
                    }
                  >
                    <option value="ACTIVE">
                      Active
                    </option>

                    <option value="ALL">
                      All
                    </option>

                    <option value="NEW">
                      New
                    </option>

                    <option value="PLANNED">
                      Planned
                    </option>

                    <option value="QUEUED">
                      Queued
                    </option>

                    <option value="RUNNING">
                      Running
                    </option>

                    <option value="DONE">
                      Finished
                    </option>

                    <option value="COMPLETED">
                      Completed
                    </option>

                    <option value="FAILED">
                      Failed
                    </option>

                    <option value="CANCELLED">
                      Cancelled
                    </option>
                  </select>
                </div>
              </header>

              <div className="flow-table-wrap">
                <table className="flow-table">
                  <thead>
                    <tr>
                      <th style={{
                        width:
                          "18%",
                      }}>
                        Order / Item
                      </th>

                      <th style={{
                        width:
                          "11%",
                      }}>
                        Status
                      </th>

                      <th style={{
                        width:
                          "21%",
                      }}>
                        Route
                      </th>

                      <th style={{
                        width:
                          "18%",
                      }}>
                        Shelf
                      </th>

                      <th style={{
                        width:
                          "10%",
                      }}>
                        Priority
                      </th>

                      <th style={{
                        width:
                          "14%",
                      }}>
                        Due
                      </th>

                      <th style={{
                        width:
                          "8%",
                      }}>
                        Action
                      </th>
                    </tr>
                  </thead>

                  <tbody>
                    {filtered.map(
                      (order) => {
                        const shelfPoint =
                          mode ===
                          "INBOUND"
                            ? order.destination
                            : order.source;

                        const rack =
                          shelfRackId(
                            shelfPoint,
                          );

                        const shelf =
                          shelfByRack.get(
                            rack,
                          );

                        const itemCode =
                          String(
                            order
                              .metadata
                              ?.item_code ||
                              "",
                          );

                        return (
                          <tr
                            key={
                              order.id
                            }
                          >
                            <td>
                              <b>
                                {
                                  order.order_no
                                }
                              </b>

                              <small>
                                {itemCode ||
                                  order.external_ref ||
                                  "No item code"}
                              </small>
                            </td>

                            <td>
                              <span
                                className={
                                  `flow-status ` +
                                  statusClass(
                                    order.status,
                                  )
                                }
                              >
                                {
                                  order.status
                                }
                              </span>
                            </td>

                            <td>
                              <div className="flow-route">
                                <b>
                                  {
                                    order
                                      .source
                                      .code
                                  }
                                </b>

                                <span>
                                  →
                                </span>

                                <b>
                                  {
                                    order
                                      .destination
                                      .code
                                  }
                                </b>
                              </div>
                            </td>

                            <td>
                              <b>
                                {rack}
                              </b>

                              <small>
                                {shelf
                                  ? `${shelf.current_load}/8 Â· ${shelf.current_load * 12.5}%`
                                  : "Không có dữ liệu"}
                              </small>
                            </td>

                            <td>
                              <span
                                className={
                                  `flow-priority ` +
                                  String(
                                    order.priority ||
                                      "",
                                  ).toLowerCase()
                                }
                              >
                                {
                                  order.priority
                                }
                              </span>
                            </td>

                            <td>
                              {
                                fmtDate(
                                  order.due_at,
                                )
                              }

                              <small>
                                {
                                  fmtDate(
                                    order.created_at,
                                  )
                                }{" "}
                                created
                              </small>
                            </td>

                            <td>
                              {ACTIVE.has(order.status) ? (
                                <button
                                  type="button"
                                  className="flow-btn danger"
                                  disabled={
                                    busy
                                  }
                                  onClick={() =>
                                    void cancel(
                                      order,
                                    )
                                  }
                                >
                                Hủy
                                </button>
                              ) : (
                                "—"
                              )}
                            </td>
                          </tr>
                        );
                      },
                    )}
                  </tbody>
                </table>

                {!loading &&
                  filtered.length ===
                    0 && (
                    <div className="flow-empty">
                      Không có đơn{" "}
                      {mode} phù hợp
                      với bộ lọc.
                    </div>
                  )}

                {loading && (
                  <div className="flow-empty">
                    Đang tải dữ
                    liệu...
                  </div>
                )}
              </div>
            </section>
          </main>
      </div>
  );
}
