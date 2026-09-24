import {
  createContext,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import { createPortal } from "react-dom";

import {
  STATUS_COLOR,
  layout,
  tickToClock,
  useStore,
  type ModalKind,
  type WindowInstance,
} from "../../state/store";

import { simControl } from "../../simulation/runner";
import { apiFetch } from "../../services/api";

import type {
  TwinEvent,
  TaskPriority,
  TaskType,
} from "../../schema/twin_state";

import { Dot } from "../ui/primitives";
import { useFocusTrap } from "../ui/useFocusTrap";
import { percText } from "../panels/RightPanels";
import { taskError } from "../../simulation/rules";

import { SchedulerModal } from "./SchedulerModal";
import { InboundOutboundModal } from "./InboundOutboundModal";
import { ShelfInventoryWindow } from "./ShelfInventoryWindow";
import { ConveyorWindow } from "./ConveyorWindow";
import { RobotQuickDetailModal } from "../robot/RobotQuickDetailModal";

import {
  schedulerApi,
  type RobotSchedule,
  type WorkPoint,
} from "../../services/scheduler";

/* ============================================================
 * WINDOW MANAGER CONFIG
 * ============================================================ */

type WindowSize = {
  width: number;
  height: number;
};

const WINDOW_SIZE: Record<ModalKind, WindowSize> = {
  audit: {
    width: 1180,
    height: 680,
  },

  tasks: {
    width: 1320,
    height: 720,
  },

  robot: {
    width: 820,
    height: 650,
  },

  fleet: {
    width: 1200,
    height: 700,
  },

  scheduler: {
    width: 1380,
    height: 800,
  },

  flows: {
    width: 1450,
    height: 820,
  },

  shelf: {
    width: 620,
    height: 720,
  },

  conveyor: {
    width: 560,
    height: 620,
  },
};

const WindowKindContext =
  createContext<WindowInstance | null>(null);

function useCurrentWindow() {
  return useContext(WindowKindContext);
}

function viewportWidth() {
  if (typeof window === "undefined") {
    return 1600;
  }

  return window.innerWidth;
}

function viewportHeight() {
  if (typeof window === "undefined") {
    return 900;
  }

  return window.innerHeight;
}

/* ============================================================
 * WINDOW MANAGER STYLE
 * ============================================================ */

function WindowManagerStyle() {
  return (
    <style>{`
      /*
       * =======================================================
       * ROOT
       * =======================================================
       */

      .wm-root {
        position: fixed;
        inset: 0;

        z-index: 2147000000;

        pointer-events: none;

        font-family:
          Inter,
          ui-sans-serif,
          system-ui,
          -apple-system,
          BlinkMacSystemFont,
          "Segoe UI",
          sans-serif;
      }

      .wm-desktop {
        position: fixed;

        inset:
          0
          0
          58px
          0;

        overflow: hidden;

        pointer-events: none;
      }

      /*
       * =======================================================
       * WINDOW
       * =======================================================
       */

      .wm-window {
        position: fixed;

        display: flex;
        flex-direction: column;

        min-width: 460px;
        min-height: 280px;

        max-width:
          calc(100vw - 20px);

        max-height:
          calc(100vh - 74px);

        overflow: hidden;

        resize: both;

        pointer-events: auto;

        isolation: isolate;

        color: #dce5f2;

        background: #09111c;

        border:
          1px solid #28374b;

        border-radius: 10px;

        box-shadow:
          0 18px 50px
          rgba(0, 0, 0, 0.5);
      }

      .wm-window.wm-active {
        border-color:
          #4d6d99;

        box-shadow:
          0 24px 80px
          rgba(0, 0, 0, 0.72),
          0 0 0 1px
          rgba(59, 130, 246, 0.13);
      }

      /*
       * Không cho component con tự biến thành fullscreen.
       */

      .wm-window .scheduler-wrap,
      .wm-window .flow-wrap {
        width: 100%;
        height: 100%;

        min-width: 0;
        min-height: 0;

        flex: 1;

        display: flex;
        flex-direction: column;

        overflow: hidden;
      }

      .wm-window .flow-body {
        flex: 1;

        min-width: 0;
        min-height: 0;

        overflow: hidden;
      }

      .wm-window .shelf-inventory-card {
        position: relative;
        inset: auto;
        width: 100%;
        height: 100%;
        max-width: none;
        box-sizing: border-box;
        flex: 1 1 auto;
      }

      /*
       * =======================================================
       * WINDOW SYSTEM BUTTONS
       * =======================================================
       */

      .wm-controls {
        position: absolute;

        top: 8px;
        right: 8px;

        z-index: 500;

        display: flex;
        align-items: center;

        gap: 5px;

        pointer-events: auto;
      }

      .wm-control {
        width: 31px;
        height: 31px;

        display: grid;
        place-items: center;

        padding: 0;

        border:
          1px solid #33445c;

        border-radius: 6px;

        color: #a4b2c5;

        background: #111c2b;

        font-size: 14px;
        font-weight: 700;

        cursor: pointer;
      }

      .wm-control:hover {
        color: #ffffff;

        background: #1a2a40;

        border-color:
          #536b8d;
      }

      .wm-control.wm-close:hover {
        color: white;

        background: #991b1b;

        border-color:
          #dc2626;
      }

      /*
       * =======================================================
       * TITLE BAR
       * =======================================================
       */

      .wm-window .modal-h {
        flex: 0 0 auto;

        min-height: 48px;

        display: flex;
        align-items: center;

        gap: 8px;

        margin: 0;

        padding:
          0
          90px
          0
          14px;

        color: #e8eef7;

        background: #0d1724;

        border: 0;

        border-bottom:
          1px solid #263247;

        cursor: move;

        user-select: none;
      }

      .wm-window.wm-active .modal-h {
        background:
          linear-gradient(
            90deg,
            #102035,
            #0d1724
          );
      }

      .wm-window .modal-h .spacer {
        flex: 1;
      }

      /*
       * =======================================================
       * WINDOW BODY
       * =======================================================
       */

      .wm-window .modal-b {
        flex: 1;

        min-width: 0;
        min-height: 0;

        overflow: auto;
      }

/* ================= WINDOWS TASKBAR ================= */

.window-taskbar {
  position: fixed !important;

  left: 50% !important;
  right: auto !important;
  top: auto !important;
  bottom: 12px !important;

  transform: translateX(-50%) !important;

  width: auto !important;
  max-width: calc(100vw - 32px) !important;

  height: 46px !important;
  min-height: 46px !important;
  max-height: 46px !important;

  display: flex !important;
  flex-direction: row !important;
  align-items: center !important;
  justify-content: center !important;

  gap: 6px !important;
  padding: 6px !important;

  overflow-x: auto !important;
  overflow-y: hidden !important;

  white-space: nowrap;

  pointer-events: auto;

  z-index: 2147480000;

  background: rgba(8, 15, 24, 0.96);

  border: 1px solid #334155;
  border-radius: 10px;

  box-shadow:
    0 12px 35px rgba(0, 0, 0, 0.55);

  backdrop-filter: blur(10px);
}

.window-task {
  position: relative !important;

  flex: 0 0 auto !important;

  width: auto !important;
  height: 32px !important;

  display: flex !important;
  flex-direction: row !important;
  align-items: center !important;

  overflow: hidden;

  border: 1px solid #334155;
  border-radius: 6px;

  background: #111c2b;
}

.window-task.active {
  border-color: #3b82f6;

  background: rgba(59, 130, 246, 0.18);
}

.window-task.minimized {
  opacity: 0.65;
}

.window-task-main {
  width: auto !important;

  min-width: 110px;
  max-width: 200px;

  height: 30px;

  padding: 0 10px;

  border: 0;

  overflow: hidden;

  color: #cbd5e1;
  background: transparent;

  font-size: 11px;

  text-align: left;

  text-overflow: ellipsis;
  white-space: nowrap;

  cursor: pointer;
}

.window-task-main:hover {
  color: white;

  background: #1e293b;
}

.window-task-close {
  flex: 0 0 30px !important;

  width: 30px !important;
  height: 30px !important;

  padding: 0;

  border: 0;

  color: #94a3b8;
  background: transparent;

  font-size: 15px;

  cursor: pointer;
}

.window-task-close:hover {
  color: white;
  background: #991b1b;
}
      /*
       * =======================================================
       * SCROLLBAR
       * =======================================================
       */

      .wm-window ::-webkit-scrollbar,
      .window-taskbar::-webkit-scrollbar {
        width: 7px;
        height: 7px;
      }

      .wm-window ::-webkit-scrollbar-track,
      .window-taskbar::-webkit-scrollbar-track {
        background: #08101a;
      }

      .wm-window ::-webkit-scrollbar-thumb,
      .window-taskbar::-webkit-scrollbar-thumb {
        background: #304057;

        border-radius: 10px;
      }

      .wm-window ::-webkit-scrollbar-thumb:hover,
      .window-taskbar::-webkit-scrollbar-thumb:hover {
        background: #435670;
      }

      /*
       * =======================================================
       * SMALL SCREEN
       * =======================================================
       */

      @media (max-width: 800px) {
        .wm-window {
          left: 5px !important;
          top: 5px !important;

          width:
            calc(
              100vw - 10px
            ) !important;

          height:
            calc(
              100vh - 70px
            ) !important;

          max-width: none;
          max-height: none;

          resize: none;
        }

        .window-taskbar {
          bottom: 5px;

          max-width:
            calc(
              100vw - 10px
            );
        }
      }
    `}</style>
  );
}

/* ============================================================
 * TASKBAR
 * ============================================================ */

function WindowTaskbar() {
  const windows =
    useStore((s) => s.windows);

  const minimized =
    useStore(
      (s) =>
        s.minimizedWindows,
    );

  const activeWindowId = useStore((s) => s.activeWindowId);

  const restore =
    useStore(
      (s) =>
        s.restoreWindow,
    );

  const focus =
    useStore(
      (s) =>
        s.focusWindow,
    );

  const close =
    useStore(
      (s) =>
        s.closeWindow,
    );

  /*
   * Không có window nào:
   * taskbar biến mất hoàn toàn.
   */
  if (windows.length === 0) {
    return null;
  }

  return (
    <div
      className="window-taskbar"
      aria-label="Open windows"
    >
      {windows.map((win) => {
        const isMinimized = minimized.includes(win.id);

        const isActive = activeWindowId === win.id && !isMinimized;

        return (
          <div
            key={win.id}
            className={[
              "window-task",

              isActive
                ? "active"
                : "",

              isMinimized
                ? "minimized"
                : "",
            ]
              .filter(Boolean)
              .join(" ")}
          >
            <button
              type="button"
              className="window-task-main"
              title={win.title}
              onClick={() => {
                if (isMinimized) restore(win.id);
                else focus(win.id);
              }}
            >
              {win.title}
            </button>

            <button
              type="button"
              className="window-task-close"
              title="Close"
              aria-label={`Close ${win.title}`}
              onClick={() => close(win.id)}
            >
              ×
            </button>
          </div>
        );
      })}
    </div>
  );
}

/* ============================================================
 * WINDOW FRAME
 * ============================================================ */

function WindowFrame({
  win,
  index,
  children,
}: {
  win: WindowInstance;
  index: number;
  children: React.ReactNode;
}) {
  const focus =
    useStore(
      (s) =>
        s.focusWindow,
    );

  const minimize =
    useStore(
      (s) =>
        s.minimizeWindow,
    );

  const close =
    useStore(
      (s) =>
        s.closeWindow,
    );

  const activeWindowId = useStore((s) => s.activeWindowId);

  const order =
    useStore(
      (s) =>
        s.windowOrder,
    );

  /*
   * Chỉ window active mới giữ keyboard focus.
   */
  const trap =
    useFocusTrap<HTMLDivElement>(
      activeWindowId === win.id,
    );

  const defaultSize =
    WINDOW_SIZE[win.kind];

  const [pos, setPos] =
    useState(() => {
      const vw =
        viewportWidth();

      const vh =
        viewportHeight();

      const width =
        Math.min(
          defaultSize.width,
          vw - 30,
        );

      const height =
        Math.min(
          defaultSize.height,
          vh - 80,
        );

      /*
       * Cascade window.
       */
      const offset =
        index % 8;

      const wantedX =
        40 + offset * 34;

      const wantedY =
        42 + offset * 28;

      return {
        x: Math.max(
          8,
          Math.min(
            wantedX,
            vw -
              width -
              12,
          ),
        ),

        y: Math.max(
          8,
          Math.min(
            wantedY,
            vh -
              height -
              64,
          ),
        ),
      };
    });

  const drag =
    useRef<{
      x: number;
      y: number;
      px: number;
      py: number;
    } | null>(null);

  const isActive = activeWindowId === win.id;

  /*
   * windowOrder cuối = nằm cao nhất.
   */
  const zIndex =
    1000 +
    Math.max(
      0,
      order.indexOf(win.id),
    );

  const handlePointerDown = (
    event:
      React.PointerEvent<HTMLDivElement>,
  ) => {
    focus(win.id);

    const target =
      event.target as HTMLElement;

    /*
     * Chỉ drag từ header.
     */
    if (
      !target.closest(
        ".modal-h",
      )
    ) {
      return;
    }

    /*
     * Không drag khi người dùng click button/input/select.
     */
    if (
      target.closest(
        "button,input,select,textarea,a,label",
      )
    ) {
      return;
    }

    drag.current = {
      x: pos.x,
      y: pos.y,

      px: event.clientX,
      py: event.clientY,
    };

    event.currentTarget.setPointerCapture(
      event.pointerId,
    );
  };

  const handlePointerMove = (
    event:
      React.PointerEvent<HTMLDivElement>,
  ) => {
    if (!drag.current) {
      return;
    }

    const nextX =
      drag.current.x +
      event.clientX -
      drag.current.px;

    const nextY =
      drag.current.y +
      event.clientY -
      drag.current.py;

    setPos({
      /*
       * Cho phép một phần window ra ngoài,
       * nhưng luôn chừa ít nhất 150px để kéo lại.
       */
      x: Math.max(
        -defaultSize.width +
          150,

        Math.min(
          nextX,
          viewportWidth() -
            150,
        ),
      ),

      y: Math.max(
        4,

        Math.min(
          nextY,
          viewportHeight() -
            90,
        ),
      ),
    });
  };

  const stopDrag = () => {
    drag.current = null;
  };

  const width =
    Math.min(
      defaultSize.width,
      viewportWidth() - 30,
    );

  const height =
    Math.min(
      defaultSize.height,
      viewportHeight() - 76,
    );

  return (
    <div
      ref={trap}
      role="dialog"
      aria-label={
        win.title
      }
      tabIndex={-1}
      className={[
        "wm-window",

        isActive
          ? "wm-active"
          : "",

        win.kind === "scheduler"
          ? "scheduler-modal"
          : "",

        win.kind === "flows"
          ? "flow-modal"
          : "",
      ]
        .filter(Boolean)
        .join(" ")}
      style={{
        left: pos.x,
        top: pos.y,

        width,
        height,

        zIndex,
      }}
      onPointerDown={
        handlePointerDown
      }
      onPointerMove={
        handlePointerMove
      }
      onPointerUp={
        stopDrag
      }
      onPointerCancel={
        stopDrag
      }
      onMouseDown={() =>
        focus(win.id)
      }
    >
      {/* WINDOW SYSTEM CONTROLS */}

      <div
        className="wm-controls"
        onPointerDown={(e) =>
          e.stopPropagation()
        }
      >
        <button
          type="button"
          className="wm-control"
          title="Minimize"
          aria-label="Minimize"
          onClick={() =>
            minimize(win.id)
          }
        >
          —
        </button>

        <button
          type="button"
          className="wm-control wm-close"
          title="Close"
          aria-label="Close"
          onClick={() =>
            close(win.id)
          }
        >
          ×
        </button>
      </div>

      <WindowKindContext.Provider
        value={win}
      >
        {children}
      </WindowKindContext.Provider>
    </div>
  );
}

/* ============================================================
 * CONTENT ROUTER
 * ============================================================ */

function WindowContent({
  win,
}: {
  win: WindowInstance;
}) {
  switch (win.kind) {
    case "audit":
      return <AuditLog />;

    case "tasks":
      return <TaskTable />;

    case "robot":
      return win.entityId ? <RobotDetail robotId={win.entityId} /> : null;

    case "fleet":
      return <FleetList />;

    case "scheduler":
      return (
        <SchedulerModal />
      );

    case "flows":
      return (
        <InboundOutboundModal />
      );

    case "shelf":
      return win.entityId ? <ShelfInventoryWindow shelfId={win.entityId} /> : null;

    case "conveyor":
      return win.entityId ? <ConveyorWindow conveyorId={win.entityId} /> : null;

    default:
      return null;
  }
}

/* ============================================================
 * ROOT
 * ============================================================ */

export function Modals() {
  const windows =
    useStore(
      (s) => s.windows,
    );

  const minimized =
    useStore(
      (s) =>
        s.minimizedWindows,
    );

  const activeWindowId = useStore((s) => s.activeWindowId);

  const close =
    useStore(
      (s) =>
        s.closeWindow,
    );

  useEffect(() => {
    const handleKeyDown = (
      event: KeyboardEvent,
    ) => {
      /*
       * ESC chỉ đóng window đang active.
       */
      if (
        event.key ===
          "Escape" &&
        activeWindowId
      ) {
        close(activeWindowId);
      }
    };

    window.addEventListener(
      "keydown",
      handleKeyDown,
    );

    return () => {
      window.removeEventListener(
        "keydown",
        handleKeyDown,
      );
    };
  }, [activeWindowId, close]);

  if (
    typeof document ===
    "undefined"
  ) {
    return null;
  }

  /*
   * PORTAL toàn bộ Window Manager ra body.
   *
   * Label Three.js sẽ không thể đè lên window.
   */
  return createPortal(
    <div className="wm-root">
      <WindowManagerStyle />

      <div className="wm-desktop">
        {windows
          .filter((win) => !minimized.includes(win.id))
          .map((win, index) => (
              <WindowFrame
                key={win.id}
                win={win}
                index={index}
              >
                <WindowContent win={win} />
              </WindowFrame>
          ))}
      </div>

      <WindowTaskbar />
      <RobotQuickDetailModal />
    </div>,

    document.body,
  );
}

/* ============================================================
 * COMMON HEADER
 * ============================================================ */

function Head({
  title,
  children,
}: {
  title: string;
  children?: React.ReactNode;
}) {
  const kind =
    useCurrentWindow();

  return (
    <header className="modal-h">
      <strong>
        {title}
      </strong>

      <span className="spacer" />

      {children}

      {/*
       * Chừa chỗ cho buttons — × của WindowFrame.
       */}
      {kind && (
        <span
          style={{
            width: 60,

            flex:
              "0 0 60px",
          }}
        />
      )}
    </header>
  );
}

/* ============================================================
 * AUDIT LOG
 * ============================================================ */

function AuditLog() {
  const source =
    useStore(
      (s) => s.source,
    );

  const local =
    useStore(
      (s) =>
        s.twin
          .recent_events,
    );

  const select =
    useStore(
      (s) => s.select,
    );

  const openWindow =
    useStore(
      (s) => s.openWindow,
    );

  const [
    remote,
    setRemote,
  ] =
    useState<
      TwinEvent[] | null
    >(null);

  const [sev, setSev] =
    useState("");

  const [src, setSrc] =
    useState("");

  const [q, setQ] =
    useState("");

  useEffect(() => {
    if (
      source !== "online"
    ) {
      setRemote(null);

      return;
    }

    let alive = true;

    apiFetch(
      "/api/events?limit=500",
    )
      .then((response) =>
        response.json(),
      )
      .then(
        (
          data: TwinEvent[],
        ) => {
          if (alive) {
            setRemote(data);
          }
        },
      )
      .catch(() => {
        if (alive) {
          setRemote(null);
        }
      });

    return () => {
      alive = false;
    };
  }, [source]);

  const events =
    remote ?? local;

  const rows =
    useMemo(() => {
      return events.filter(
        (event) => {
          if (
            sev &&
            event.severity !==
              sev
          ) {
            return false;
          }

          if (
            src &&
            event.source !==
              src
          ) {
            return false;
          }

          if (q) {
            const haystack =
              `${event.message} ${event.robot_id ?? ""} ${event.zone_id ?? ""} ${event.type}`.toLowerCase();

            if (
              !haystack.includes(
                q.toLowerCase(),
              )
            ) {
              return false;
            }
          }

          return true;
        },
      );
    }, [
      events,
      sev,
      src,
      q,
    ]);

  const exportFile = (
    kind:
      | "json"
      | "csv",
  ) => {
    const body =
      kind === "json"
        ? JSON.stringify(
            rows,
            null,
            2,
          )
        : [
            "id,tick,time,type,source,severity,robot,task,zone,message",

            ...rows.map(
              (event) =>
                [
                  event.id,
                  event.tick,

                  tickToClock(
                    event.tick,
                    100,
                    true,
                  ),

                  event.type,
                  event.source,
                  event.severity,

                  event.robot_id ??
                    "",

                  event.task_id ??
                    "",

                  event.zone_id ??
                    "",

                  `"${event.message.replace(
                    /"/g,
                    '""',
                  )}"`,
                ].join(","),
            ),
          ].join("\n");

    const url =
      URL.createObjectURL(
        new Blob([body], {
          type:
            kind === "json"
              ? "application/json"
              : "text/csv",
        }),
      );

    const anchor =
      document.createElement(
        "a",
      );

    anchor.href = url;

    anchor.download =
      `audit_log.${kind}`;

    anchor.click();

    URL.revokeObjectURL(
      url,
    );
  };

  return (
    <>
      <Head title="Audit / Event Log">
        <span
          className="hint"
          style={{
            margin: 0,
          }}
        >
          {remote
            ? "backend SQLite"
            : "local ring buffer"}
        </span>

        <button
          className="btn"
          onClick={() =>
            exportFile("csv")
          }
        >
          Export CSV
        </button>

        <button
          className="btn"
          onClick={() =>
            exportFile("json")
          }
        >
          Export JSON
        </button>
      </Head>

      <div className="modal-b">
        <div className="filters">
          <select
            value={sev}
            onChange={(event) =>
              setSev(
                event.target
                  .value,
              )
            }
          >
            <option value="">
              All severities
            </option>

            {[
              "CRITICAL",
              "HIGH",
              "MEDIUM",
              "LOW",
              "INFO",
            ].map((value) => (
              <option
                key={value}
              >
                {value}
              </option>
            ))}
          </select>

          <select
            value={src}
            onChange={(event) =>
              setSrc(
                event.target
                  .value,
              )
            }
          >
            <option value="">
              All sources
            </option>

            {[
              "ROBOT",
              "FLEET_MANAGER",
              "PLANNER",
              "SIMULATION",
              "CONVEYOR",
              "CAMERA",
              "VLM",
              "LIFT",
              "USER",
              "AI_AGENT",
            ].map((value) => (
              <option
                key={value}
              >
                {value}
              </option>
            ))}
          </select>

          <input
            placeholder="search message / robot / zone…"
            value={q}
            onChange={(event) =>
              setQ(
                event.target
                  .value,
              )
            }
            style={{
              flex: 1,
            }}
          />

          <span className="count">
            {rows.length} /{" "}
            {events.length}
          </span>
        </div>

        <table className="dt full">
          <thead>
            <tr>
              <th>Time</th>
              <th>Tick</th>
              <th>Severity</th>
              <th>Source</th>
              <th>Type</th>
              <th>Message</th>
              <th>Robot</th>
              <th>Zone</th>
            </tr>
          </thead>

          <tbody>
            {rows.map(
              (event) => (
                <tr
                  key={event.id}
                  style={{
                    cursor:
                      event.robot_id
                        ? "pointer"
                        : "default",
                  }}
                  onClick={() => {
                    if (
                      event.robot_id
                    ) {
                      select(
                        event.robot_id,
                      );

                      /*
                       * Mở Robot Detail,
                       * không đóng Audit.
                       */
                      openWindow({
                        id: `robot:${event.robot_id}`,
                        kind: "robot",
                        entityId: event.robot_id,
                        title: `Robot ${event.robot_id}`,
                      });
                    }
                  }}
                >
                  <td>
                    {tickToClock(
                      event.tick,
                      100,
                      true,
                    )}
                  </td>

                  <td>
                    {event.tick}
                  </td>

                  <td
                    className={
                      "sev-" +
                      event.severity
                    }
                  >
                    {
                      event.severity
                    }
                  </td>

                  <td>
                    {event.source}
                  </td>

                  <td>
                    {event.type}
                  </td>

                  <td>
                    {
                      event.message
                    }
                  </td>

                  <td>
                    {event.robot_id ??
                      ""}
                  </td>

                  <td>
                    {event.zone_id ??
                      ""}
                  </td>
                </tr>
              ),
            )}
          </tbody>
        </table>
      </div>
    </>
  );
}

/* ============================================================
 * TASK TABLE
 * ============================================================ */

function TaskTable() {
  const tasks =
    useStore(
      (s) =>
        s.twin.tasks,
    );

  const robots =
    useStore(
      (s) =>
        s.twin.robots,
    );

  const locs =
    useStore(
      (s) =>
        s.locations,
    );

  const select =
    useStore(
      (s) => s.select,
    );

  const openWindow =
    useStore(
      (s) => s.openWindow,
    );

  const [type, setType] =
    useState<TaskType>(
      "PICK",
    );

  const [
    priority,
    setPriority,
  ] =
    useState<TaskPriority>(
      "HIGH",
    );

  const [source, setSource] =
    useState("SHELF-A12");

  const [dest, setDest] =
    useState("PACK-01");

  const [
    robotId,
    setRobotId,
  ] = useState("");

  const [
    status,
    setStatus,
  ] = useState("");

  const [
    assigning,
    setAssigning,
  ] =
    useState<string | null>(
      null,
    );

  const statusOrder: Record<
    string,
    number
  > = {
    IN_PROGRESS: 0,
    ASSIGNED: 1,
    WAITING: 2,
    TRANSFERRED: 3,
    COMPLETED: 4,
    FAILED: 5,
    CANCELLED: 6,
  };

  const rows =
    Object.values(tasks)
      .filter(
        (task) =>
          !status ||
          task.status ===
            status,
      )
      .sort(
        (a, b) =>
          statusOrder[
            a.status
          ] -
            statusOrder[
              b.status
            ] ||
          b.created_tick -
            a.created_tick,
      );

  const pretty = (
    id: string,
  ) => {
    const location =
      locs[id];

    if (!location) {
      return id;
    }

    if (
      location.kind ===
      "SHELF"
    ) {
      return `Shelf ${id.replace(
        "SHELF-",
        "",
      )}`;
    }

    return id.replace(
      "-",
      " ",
    );
  };

  const opts =
    layout.locations.filter(
      (location) =>
        location.kind !==
        "CHARGING",
    );

  const availableRobots =
    Object.values(robots)
      .filter(
        (robot) =>
          robot.status !==
            "OFFLINE" &&
          robot.status !==
            "ERROR" &&
          robot.fsm ===
            "IDLE" &&
          robot.battery > 20 &&
          robot.load.current ===
            0,
      )
      .sort((a, b) =>
        a.id.localeCompare(
          b.id,
        ),
      );

  const error =
    taskError(
      locs,
      type,
      source,
      dest,
    );

  const create = (
    event:
      React.FormEvent,
  ) => {
    event.preventDefault();

    if (error) {
      return;
    }

    simControl.createTask({
      type,
      priority,
      source,
      destination: dest,

      robot_id:
        robotId || null,
    });
  };

  const manualAssign = (
    taskId: string,
    rid: string,
  ) => {
    if (!rid) {
      return;
    }

    setAssigning(taskId);

    simControl.assignTask(
      taskId,
      rid,
    );

    window.setTimeout(
      () =>
        setAssigning(null),
      300,
    );
  };

  return (
    <>
      <Head
        title={`Tasks · ${rows.length}`}
      >
        <span
          className="hint"
          style={{
            margin: 0,
          }}
        >
          Create task and
          assign robot
        </span>
      </Head>

      <div className="modal-b">
        <form
          className="form"
          onSubmit={create}
        >
          <label>
            Type

            <select
              value={type}
              onChange={(event) =>
                setType(
                  event.target
                    .value as TaskType,
                )
              }
            >
              {[
                "PICK",
                "TRANSPORT",
                "REPLENISH",
                "RETURN",
              ].map((value) => (
                <option
                  key={value}
                >
                  {value}
                </option>
              ))}
            </select>
          </label>

          <label>
            Priority

            <select
              value={
                priority
              }
              onChange={(event) =>
                setPriority(
                  event.target
                    .value as TaskPriority,
                )
              }
            >
              {[
                "LOW",
                "NORMAL",
                "HIGH",
                "CRITICAL",
              ].map((value) => (
                <option
                  key={value}
                >
                  {value}
                </option>
              ))}
            </select>
          </label>

          <label>
            Source

            <select
              value={source}
              onChange={(event) =>
                setSource(
                  event.target
                    .value,
                )
              }
            >
              {opts.map(
                (location) => (
                  <option
                    key={
                      location.id
                    }
                    value={
                      location.id
                    }
                  >
                    {pretty(
                      location.id,
                    )}
                  </option>
                ),
              )}
            </select>
          </label>

          <label>
            Destination

            <select
              value={dest}
              onChange={(event) =>
                setDest(
                  event.target
                    .value,
                )
              }
            >
              {opts.map(
                (location) => (
                  <option
                    key={
                      location.id
                    }
                    value={
                      location.id
                    }
                  >
                    {pretty(
                      location.id,
                    )}
                  </option>
                ),
              )}
            </select>
          </label>

          <label>
            Assign to

            <select
              value={robotId}
              onChange={(event) =>
                setRobotId(
                  event.target
                    .value,
                )
              }
            >
              <option value="">
                Auto dispatch
              </option>

              {availableRobots.map(
                (robot) => (
                  <option
                    key={robot.id}
                    value={robot.id}
                  >
                    {robot.id} ·{" "}
                    {robot.battery.toFixed(
                      0,
                    )}
                    % ·{" "}
                    {robot.zone ??
                      "—"}
                  </option>
                ),
              )}
            </select>
          </label>

          <label>
            Filter

            <select
              value={status}
              onChange={(event) =>
                setStatus(
                  event.target
                    .value,
                )
              }
            >
              <option value="">
                All
              </option>

              {Object.keys(
                statusOrder,
              ).map((value) => (
                <option
                  key={value}
                >
                  {value}
                </option>
              ))}
            </select>
          </label>

          <button
            className="btn primary"
            type="submit"
            disabled={!!error}
            title={
              error ??
              "Create task"
            }
          >
            + Create task
          </button>

          {error && (
            <span className="form-err">
              {error}
            </span>
          )}
        </form>

        <table className="dt full">
          <thead>
            <tr>
              <th>Task</th>
              <th>Type</th>
              <th>Priority</th>
              <th>Status</th>
              <th>From</th>
              <th>To</th>
              <th>
                Assigned Robot
              </th>
              <th>Reassign</th>
              <th>Created</th>
              <th>Assigned</th>
              <th>Completed</th>
              <th>Duration</th>
              <th>Parent</th>
            </tr>
          </thead>

          <tbody>
            {rows.map((task) => {
              const canAssign =
                task.status ===
                  "WAITING" ||
                (task.status ===
                  "ASSIGNED" &&
                  task.started_tick ===
                    null);

              return (
                <tr
                  key={task.id}
                  style={{
                    cursor:
                      task.assigned_robot
                        ? "pointer"
                        : "default",
                  }}
                  onClick={() => {
                    if (
                      task.assigned_robot
                    ) {
                      select(
                        task.assigned_robot,
                      );

                      /*
                       * Mở Robot Detail,
                       * không đóng Tasks.
                       */
                      openWindow({
                        id: `robot:${task.assigned_robot}`,
                        kind: "robot",
                        entityId: task.assigned_robot,
                        title: `Robot ${task.assigned_robot}`,
                      });
                    }
                  }}
                >
                  <td>
                    #{task.id}
                  </td>

                  <td>
                    {task.type}
                  </td>

                  <td>
                    {task.priority}
                  </td>

                  <td>
                    {task.status}
                  </td>

                  <td>
                    {pretty(
                      task.source,
                    )}
                  </td>

                  <td>
                    {pretty(
                      task.destination,
                    )}
                  </td>

                  <td>
                    {task.assigned_robot ??
                      "AUTO"}
                  </td>

                  <td
                    onClick={(event) =>
                      event.stopPropagation()
                    }
                  >
                    {canAssign ? (
                      <select
                        defaultValue={
                          task.assigned_robot ??
                          ""
                        }
                        disabled={
                          assigning ===
                          task.id
                        }
                        onChange={(
                          event,
                        ) =>
                          manualAssign(
                            task.id,
                            event.target
                              .value,
                          )
                        }
                      >
                        <option value="">
                          Select…
                        </option>

                        {Object.values(
                          robots,
                        )
                          .filter(
                            (
                              robot,
                            ) =>
                              robot.id ===
                                task.assigned_robot ||
                              (robot.status !==
                                "OFFLINE" &&
                                robot.status !==
                                  "ERROR" &&
                                robot.fsm ===
                                  "IDLE" &&
                                robot.battery >
                                  20 &&
                                robot.load
                                  .current ===
                                  0),
                          )
                          .sort(
                            (
                              a,
                              b,
                            ) =>
                              a.id.localeCompare(
                                b.id,
                              ),
                          )
                          .map(
                            (
                              robot,
                            ) => (
                              <option
                                key={
                                  robot.id
                                }
                                value={
                                  robot.id
                                }
                              >
                                {
                                  robot.id
                                }{" "}
                                ·{" "}
                                {robot.battery.toFixed(
                                  0,
                                )}
                                %
                              </option>
                            ),
                          )}
                      </select>
                    ) : (
                      <span className="hint">
                        locked after
                        start
                      </span>
                    )}
                  </td>

                  <td>
                    {tickToClock(
                      task.created_tick,
                      100,
                      true,
                    )}
                  </td>

                  <td>
                    {task.assigned_tick !==
                    null
                      ? tickToClock(
                          task.assigned_tick,
                          100,
                          true,
                        )
                      : "—"}
                  </td>

                  <td>
                    {task.completed_tick !==
                    null
                      ? tickToClock(
                          task.completed_tick,
                          100,
                          true,
                        )
                      : "—"}
                  </td>

                  <td>
                    {task.completed_tick !==
                    null
                      ? `${(
                          (task.completed_tick -
                            task.created_tick) /
                          10
                        ).toFixed(
                          0,
                        )}s`
                      : "—"}
                  </td>

                  <td>
                    {task.parent_task_id ??
                      ""}
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </>
  );
}

/* ============================================================
 * FLEET
 * ============================================================ */

function FleetList() {
  const robots =
    useStore(
      (s) =>
        s.twin.robots,
    );

  const tasks =
    useStore(
      (s) =>
        s.twin.tasks,
    );

  const openRobotQuickDetail = useStore(
    (s) => s.openRobotQuickDetail,
  );

  const [
    status,
    setStatus,
  ] = useState("");

  const [sort, setSort] =
    useState<
      | "id"
      | "battery"
      | "status"
      | "tasks"
      | "distance"
    >("id");

  const rows =
    useMemo(() => {
      const list =
        Object.values(
          robots,
        ).filter(
          (robot) =>
            !status ||
            robot.status ===
              status,
        );

      const key = (
        robot:
          (typeof list)[number],
      ) => {
        switch (sort) {
          case "battery":
            return robot.battery;

          case "tasks":
            return -robot.stats
              .tasks_completed;

          case "distance":
            return -robot.stats
              .distance_m;

          case "status":
            return robot.status;

          default:
            return robot.id;
        }
      };

      return list.sort(
        (a, b) => {
          const ka = key(a);
          const kb = key(b);

          if (ka < kb) {
            return -1;
          }

          if (ka > kb) {
            return 1;
          }

          return a.id.localeCompare(
            b.id,
          );
        },
      );
    }, [
      robots,
      status,
      sort,
    ]);

  const counts =
    useMemo(() => {
      return Object.values(
        robots,
      ).reduce<
        Record<
          string,
          number
        >
      >((result, robot) => {
        result[robot.status] =
          (result[
            robot.status
          ] ?? 0) + 1;

        return result;
      }, {});
    }, [robots]);

  const Th = ({
    k,
    children,
  }: {
    k: typeof sort;
    children: React.ReactNode;
  }) => (
    <th
      onClick={() =>
        setSort(k)
      }
      style={{
        cursor: "pointer",

        color:
          sort === k
            ? "var(--accent)"
            : undefined,
      }}
    >
      {children}

      {sort === k
        ? " ↓"
        : ""}
    </th>
  );

  return (
    <>
      <Head
        title={`Robot Fleet · ${Object.keys(robots).length} AMRs`}
      >
        <span
          className="hint"
          style={{
            margin: 0,
          }}
        >
          Click robot to open
          detail
        </span>
      </Head>

      <div className="modal-b">
        <div className="filters">
          {[
            "",
            "ACTIVE",
            "IDLE",
            "CHARGING",
            "WARNING",
            "ERROR",
            "OFFLINE",
          ].map((value) => (
            <button
              key={value}
              className={
                "chip" +
                (status ===
                value
                  ? " on"
                  : "")
              }
              onClick={() =>
                setStatus(value)
              }
              style={
                value
                  ? {
                      color:
                        STATUS_COLOR[
                          value as keyof typeof STATUS_COLOR
                        ],
                    }
                  : undefined
              }
            >
              {value || "All"}

              {value
                ? ` ${counts[value] ?? 0}`
                : ` ${Object.keys(robots).length}`}
            </button>
          ))}
        </div>

        <table className="dt full">
          <thead>
            <tr>
              <Th k="id">
                Robot
              </Th>

              <th>Floor</th>

              <Th k="status">
                Status
              </Th>

              <th>State</th>

              <Th k="battery">
                Battery
              </Th>

              <th>Task</th>

              <th>Zone</th>

              <th>Speed</th>

              <th>
                Perception
              </th>

              <Th k="tasks">
                Done
              </Th>

              <Th k="distance">
                Distance
              </Th>
            </tr>
          </thead>

          <tbody>
            {rows.map(
              (robot) => {
                const task =
                  robot.current_task_id
                    ? tasks[
                        robot
                          .current_task_id
                      ]
                    : null;

                const color =
                  STATUS_COLOR[
                    robot.status
                  ];

                return (
                  <tr
                    key={robot.id}
                    style={{
                      cursor:
                        "pointer",
                    }}
                    onClick={() => {
                      openRobotQuickDetail(robot.id);
                    }}
                  >
                    <td
                      style={{
                        fontWeight:
                          700,
                      }}
                    >
                      {robot.id}
                    </td>

                    <td>
                      {robot.lift_id
                        ? `🛗 ${robot.lift_id}`
                        : `F${robot.floor}`}
                    </td>

                    <td
                      style={{
                        color,
                      }}
                    >
                      <Dot
                        color={color}
                      />

                      {robot.status[0] +
                        robot.status
                          .slice(1)
                          .toLowerCase()}
                    </td>

                    <td>
                      {robot.fsm}
                    </td>

                    <td>
                      {robot.battery.toFixed(
                        0,
                      )}
                      %
                    </td>

                    <td>
                      {task
                        ? `#${task.id} ${task.type}`
                        : "—"}
                    </td>

                    <td>
                      {robot.zone
                        ? `Zone ${robot.zone}`
                        : "—"}
                    </td>

                    <td>
                      {robot.velocity >
                      0.05
                        ? `${robot.velocity.toFixed(
                            2,
                          )} m/s`
                        : "—"}
                    </td>

                    <td>
                      {percText(
                        robot,
                      )}
                    </td>

                    <td>
                      {
                        robot.stats
                          .tasks_completed
                      }
                    </td>

                    <td>
                      {robot.stats
                        .distance_m >=
                      1000
                        ? `${(
                            robot.stats
                              .distance_m /
                            1000
                          ).toFixed(
                            2,
                          )} km`
                        : `${robot.stats.distance_m.toFixed(
                            0,
                          )} m`}
                    </td>
                  </tr>
                );
              },
            )}
          </tbody>
        </table>
      </div>
    </>
  );
}

/* ============================================================
 * ROBOT DETAIL
 * ============================================================ */

function RobotDetail({ robotId }: { robotId: string }) {
  const [
    taskToAssign,
    setTaskToAssign,
  ] = useState("");

  const [
    workpoints,
    setWorkpoints,
  ] =
    useState<WorkPoint[]>([]);

  const [
    robotSchedules,
    setRobotSchedules,
  ] =
    useState<
      RobotSchedule[]
    >([]);

  const [
    sourceId,
    setSourceId,
  ] = useState("");

  const [
    destinationId,
    setDestinationId,
  ] = useState("");

  const [
    scheduleType,
    setScheduleType,
  ] = useState("MOVE");

  const [
    schedulePriority,
    setSchedulePriority,
  ] = useState("NORMAL");

  const [
    plannedStart,
    setPlannedStart,
  ] = useState(() => {
    const now =
      new Date();

    const date =
      new Date(
        Date.now() +
          10000 -
          now.getTimezoneOffset() *
            60000,
      );

    return date
      .toISOString()
      .slice(0, 16);
  });

  const [
    scheduleBusy,
    setScheduleBusy,
  ] = useState(false);

  const [
    scheduleMessage,
    setScheduleMessage,
  ] = useState("");

  const [
    scheduleError,
    setScheduleError,
  ] = useState("");

  const robot =
    useStore((s) => s.twin.robots[robotId]);

  const task =
    useStore((s) =>
      robot?.current_task_id
        ? s.twin.tasks[
            robot
              .current_task_id
          ]
        : undefined,
    );

  const events =
    useStore(
      (s) =>
        s.twin
          .recent_events,
    );

  const locs =
    useStore(
      (s) =>
        s.locations,
    );

  const allTasks =
    useStore(
      (s) =>
        s.twin.tasks,
    );

  const sourceMode =
    useStore(
      (s) => s.source,
    );

  useEffect(() => {
    /*
     * Clear stale scheduler data khi chuyển local/offline.
     */
    if (
      !robot ||
      sourceMode !== "online"
    ) {
      setWorkpoints([]);

      setRobotSchedules(
        [],
      );

      return;
    }

    let alive = true;

    Promise.all([
      schedulerApi.workpoints(),
      schedulerApi.schedules(),
    ])
      .then(
        ([
          points,
          schedules,
        ]) => {
          if (!alive) {
            return;
          }

          setWorkpoints(
            points.filter(
              (point) =>
                point.enabled &&
                point.kind !==
                  "CHARGING",
            ),
          );

          setRobotSchedules(
            schedules.filter(
              (schedule) =>
                schedule.robot_id ===
                  robot.id &&
                [
                  "PLANNED",
                  "QUEUED",
                  "RUNNING",
                ].includes(
                  schedule.status,
                ),
            ),
          );
        },
      )
      .catch((error) => {
        if (alive) {
          setScheduleError(
            error instanceof Error
              ? error.message
              : String(error),
          );
        }
      });

    return () => {
      alive = false;
    };
  }, [
    robot?.id,
    sourceMode,
  ]);

  if (!robot) {
    return (
      <>
        <Head title="Robot" />

        <div className="modal-b hint">
          No robot selected.
        </div>
      </>
    );
  }

  const assignableTasks =
    Object.values(allTasks)
      .filter(
        (task) =>
          task.status ===
            "WAITING" ||
          (task.status ===
            "ASSIGNED" &&
            task.started_tick ===
              null),
      )
      .sort(
        (a, b) =>
          b.created_tick -
          a.created_tick,
      );

  const color =
    STATUS_COLOR[
      robot.status
    ];

  const pretty = (
    value:
      | string
      | null,
  ) => {
    if (!value) {
      return "—";
    }

    const location =
      locs[value];

    if (!location) {
      return value;
    }

    if (
      location.kind ===
      "SHELF"
    ) {
      return `Shelf ${value.replace(
        "SHELF-",
        "",
      )}`;
    }

    return value.replace(
      "-",
      " ",
    );
  };

  const mine =
    events
      .filter(
        (event) =>
          event.robot_id ===
          robot.id,
      )
      .slice(0, 40);

  const assignedTasks =
    Object.values(
      allTasks,
    ).filter(
      (task) =>
        task.assigned_robot ===
          robot.id &&
        [
          "ASSIGNED",
          "IN_PROGRESS",
        ].includes(
          task.status,
        ),
    );

  const pointName = (
    point:
      | WorkPoint
      | undefined,
  ) => {
    if (!point) {
      return "—";
    }

    return `[${point.kind}] ${point.code}`;
  };

  const locationOptions =
    sourceMode === "online"
      ? workpoints.map(
          (point) => ({
            value:
              String(
                point.id,
              ),

            label:
              pointName(
                point,
              ),
          }),
        )
      : layout.locations
          .filter(
            (point) =>
              point.kind !==
              "CHARGING",
          )
          .map((point) => ({
            value:
              point.id,

            label:
              point.kind ===
              "SHELF"
                ? `Shelf ${point.id.replace(
                    "SHELF-",
                    "",
                  )}`
                : `${point.kind} · ${point.id}`,
          }));

  const createRobotSchedule =
    async () => {
      setScheduleBusy(true);

      setScheduleError("");

      setScheduleMessage("");

      try {
        if (
          !sourceId ||
          !destinationId ||
          sourceId ===
            destinationId
        ) {
          throw new Error(
            "Chọn Input và Output khác nhau.",
          );
        }

        if (
          sourceMode !==
          "online"
        ) {
          /*
           * Mapping local đúng type.
           */
          const localType: TaskType =
            scheduleType ===
            "INBOUND"
              ? "REPLENISH"

              : scheduleType ===
                  "OUTBOUND"
                ? "PICK"

              : scheduleType ===
                  "PICK"
                ? "PICK"

              : scheduleType ===
                  "REPLENISH"
                ? "REPLENISH"

              : "TRANSPORT";

          simControl.createTask({
            type:
              localType,

            priority:
              schedulePriority as TaskPriority,

            source:
              sourceId,

            destination:
              destinationId,

            robot_id:
              robot.id,
          });

          setScheduleMessage(
            `Đã thêm task ${localType} cho ${robot.id}.`,
          );

          return;
        }

        const order =
          await schedulerApi.createOrder(
            {
              source_id:
                Number(
                  sourceId,
                ),

              destination_id:
                Number(
                  destinationId,
                ),

              type:
                scheduleType,

              priority:
                schedulePriority,

              load_units: 1,

              quantity: 1,

              notes:
                `Scheduled from robot ${robot.id}`,
            },
          );

        const created =
          await schedulerApi.createSchedule(
            {
              order_id:
                order.id,

              planned_start:
                new Date(
                  plannedStart,
                ).toISOString(),

              mode:
                "MANUAL",

              robot_id:
                robot.id,
            },
          );

        setRobotSchedules(
          (current) =>
            [
              ...current,
              created,
            ].sort(
              (a, b) =>
                +new Date(
                  a.planned_start,
                ) -
                +new Date(
                  b.planned_start,
                ),
            ),
        );

        setScheduleMessage(
          `${created.order_no} đã được xếp cho ${robot.id}.`,
        );

        const nextStart =
          created.planned_end
            ? new Date(
                new Date(
                  created.planned_end,
                ).getTime() +
                  1000,
              )
            : new Date(
                Date.now() +
                  60000,
              );

        setPlannedStart(
          new Date(
            nextStart.getTime() -
              nextStart.getTimezoneOffset() *
                60000,
          )
            .toISOString()
            .slice(0, 16),
        );
      } catch (error) {
        setScheduleError(
          error instanceof Error
            ? error.message
            : String(error),
        );
      } finally {
        setScheduleBusy(false);
      }
    };

  const KV = ({
    k,
    v,
  }: {
    k: string;
    v: React.ReactNode;
  }) => (
    <div className="kv">
      <span className="k">
        {k}
      </span>

      <span className="v">
        {v}
      </span>
    </div>
  );

  return (
    <>
      <Head
        title={`Robot ${robot.id}`}
      >
        <span
          className="status-text"
          style={{
            color,
          }}
        >
          <Dot color={color} />

          {robot.status}
        </span>

        {robot.status ===
        "OFFLINE" ? (
          <button
            className="btn"
            onClick={() =>
              simControl.clearInjection(
                "ROBOT_FAILURE",
                robot.id,
              )
            }
          >
            Restore
          </button>
        ) : (
          <button
            className="btn danger"
            onClick={() =>
              simControl.inject({
                kind:
                  "ROBOT_FAILURE",

                robot_id:
                  robot.id,
              })
            }
          >
            Fail robot
          </button>
        )}
      </Head>

      <div className="modal-b">
        <div className="robot-detail-hero">
          <div className="robot-detail-model">
            <span>
              MODEL
            </span>

            <strong>
              {robot.model}
            </strong>

            <small>
              {robot.id} ·
              Floor{" "}
              {robot.floor} ·{" "}
              {robot.zone ??
                "No zone"}
            </small>
          </div>

          <div className="robot-route">
            <span>
              LIVE ROUTE
            </span>

            <strong>
              {Math.max(
                0,
                robot.path.length -
                  robot.path_index,
              )}{" "}
              cells remaining
            </strong>

            <small>
              {pretty(
                robot.destination ??
                  task?.destination ??
                  null,
              )}{" "}
              ·{" "}
              {robot.eta_s !==
              null
                ? `ETA ${robot.eta_s}s`
                : "ETA —"}
            </small>
          </div>
        </div>

        <section className="robot-schedule-card">
          <div className="robot-schedule-title">
            <div>
              <b>
                Schedule task
                for{" "}
                {robot.id}
              </b>

              <small>
                Input → Output
              </small>
            </div>

            <span>
              {sourceMode ===
              "online"
                ? "BACKEND SCHEDULER"
                : "LOCAL SIMULATION"}
            </span>
          </div>

          <div className="robot-schedule-grid">
            <label>
              Input

              <select
                value={sourceId}
                onChange={(event) =>
                  setSourceId(
                    event.target
                      .value,
                  )
                }
              >
                <option value="">
                  Chọn điểm vào…
                </option>

                {locationOptions.map(
                  (point) => (
                    <option
                      key={
                        point.value
                      }
                      value={
                        point.value
                      }
                    >
                      {
                        point.label
                      }
                    </option>
                  ),
                )}
              </select>
            </label>

            <label>
              Output

              <select
                value={
                  destinationId
                }
                onChange={(event) =>
                  setDestinationId(
                    event.target
                      .value,
                  )
                }
              >
                <option value="">
                  Chọn điểm ra…
                </option>

                {locationOptions.map(
                  (point) => (
                    <option
                      key={
                        point.value
                      }
                      value={
                        point.value
                      }
                    >
                      {
                        point.label
                      }
                    </option>
                  ),
                )}
              </select>
            </label>

            <label>
              Type

              <select
                value={
                  scheduleType
                }
                onChange={(event) =>
                  setScheduleType(
                    event.target
                      .value,
                  )
                }
              >
                {[
                  "MOVE",
                  "PICK",
                  "REPLENISH",
                  "INBOUND",
                  "OUTBOUND",
                  "TRANSFER",
                ].map((value) => (
                  <option
                    key={value}
                  >
                    {value}
                  </option>
                ))}
              </select>
            </label>

            <label>
              Priority

              <select
                value={
                  schedulePriority
                }
                onChange={(event) =>
                  setSchedulePriority(
                    event.target
                      .value,
                  )
                }
              >
                {[
                  "LOW",
                  "NORMAL",
                  "HIGH",
                  "CRITICAL",
                ].map((value) => (
                  <option
                    key={value}
                  >
                    {value}
                  </option>
                ))}
              </select>
            </label>

            <label>
              Planned start

              <input
                type="datetime-local"
                value={
                  plannedStart
                }
                onChange={(event) =>
                  setPlannedStart(
                    event.target
                      .value,
                  )
                }
              />
            </label>
          </div>

          <button
            className="btn primary"
            disabled={
              scheduleBusy ||
              !sourceId ||
              !destinationId
            }
            onClick={() =>
              void createRobotSchedule()
            }
          >
            {scheduleBusy
              ? "Scheduling…"
              : "+ Add schedule"}
          </button>

          {scheduleError && (
            <div className="form-err">
              {scheduleError}
            </div>
          )}

          {scheduleMessage && (
            <div className="scheduler-alert ok">
              {scheduleMessage}
            </div>
          )}
        </section>

        <section className="robot-queue-card">
          <div className="robot-schedule-title">
            <div>
              <b>
                Robot queue
              </b>

              <small>
                {
                  assignedTasks.length
                }{" "}
                task ·{" "}
                {
                  robotSchedules.length
                }{" "}
                schedule
              </small>
            </div>
          </div>

          {robotSchedules.map(
            (schedule) => (
              <div
                className="robot-queue-row"
                key={
                  schedule.id
                }
              >
                <strong>
                  {
                    schedule.order_no
                  }
                </strong>

                <span>
                  {
                    schedule.status
                  }{" "}
                  ·{" "}
                  {
                    schedule.priority
                  }
                </span>

                <small>
                  {new Date(
                    schedule.planned_start,
                  ).toLocaleString()}{" "}
                  ·{" "}
                  {schedule.estimated_distance_m.toFixed(
                    1,
                  )}
                  m
                </small>
              </div>
            ),
          )}

          {assignedTasks.map(
            (assigned) => (
              <div
                className="robot-queue-row"
                key={`task-${assigned.id}`}
              >
                <strong>
                  #{assigned.id}
                </strong>

                <span>
                  {
                    assigned.status
                  }{" "}
                  ·{" "}
                  {
                    assigned.priority
                  }
                </span>

                <small>
                  {pretty(
                    assigned.source,
                  )}{" "}
                  →{" "}
                  {pretty(
                    assigned.destination,
                  )}
                </small>
              </div>
            ),
          )}

          {!robotSchedules.length &&
            !assignedTasks.length && (
              <div className="hint">
                Chưa có task
                hoặc schedule.
              </div>
            )}
        </section>

        {robot.fsm ===
          "IDLE" &&
          robot.status !==
            "OFFLINE" &&
          robot.status !==
            "ERROR" &&
          robot.battery >
            20 &&
          robot.load.current ===
            0 && (
            <div
              style={{
                display: "flex",
                gap: 8,

                alignItems:
                  "end",

                marginBottom:
                  14,

                padding:
                  "10px 12px",

                border:
                  "1px solid var(--line)",

                borderRadius:
                  8,
              }}
            >
              <div
                style={{
                  flex: 1,
                }}
              >
                <div className="drawer-sub">
                  Assign work to{" "}
                  {robot.id}
                </div>

                <select
                  value={
                    taskToAssign
                  }
                  onChange={(event) =>
                    setTaskToAssign(
                      event.target
                        .value,
                    )
                  }
                  style={{
                    width:
                      "100%",
                  }}
                >
                  <option value="">
                    Select waiting
                    task…
                  </option>

                  {assignableTasks.map(
                    (available) => (
                      <option
                        key={
                          available.id
                        }
                        value={
                          available.id
                        }
                      >
                        #
                        {
                          available.id
                        }{" "}
                        ·{" "}
                        {
                          available.type
                        }{" "}
                        ·{" "}
                        {
                          available.priority
                        }
                      </option>
                    ),
                  )}
                </select>
              </div>

              <button
                className="btn primary"
                disabled={
                  !taskToAssign
                }
                onClick={() => {
                  if (
                    taskToAssign
                  ) {
                    simControl.assignTask(
                      taskToAssign,
                      robot.id,
                    );

                    setTaskToAssign(
                      "",
                    );
                  }
                }}
              >
                Assign
              </button>
            </div>
          )}

        <div className="kv-grid">
          <div>
            <KV
              k="Status"
              v={robot.status}
            />

            <KV
              k="FSM"
              v={robot.fsm}
            />

            <KV
              k="Battery"
              v={`${robot.battery.toFixed(
                1,
              )}%`}
            />

            <KV
              k="Speed"
              v={`${robot.velocity.toFixed(
                2,
              )} m/s`}
            />

            <KV
              k="Health"
              v={`${robot.health}%`}
            />

            <KV
              k="Zone"
              v={
                robot.zone ??
                "—"
              }
            />
          </div>

          <div>
            <KV
              k="Current Task"
              v={
                task
                  ? `#${task.id} ${task.type}`
                  : "—"
              }
            />

            <KV
              k="From"
              v={pretty(
                task?.source ??
                  null,
              )}
            />

            <KV
              k="Destination"
              v={pretty(
                task?.destination ??
                  robot.destination,
              )}
            />

            <KV
              k="Load"
              v={`${robot.load.current} / ${robot.load.capacity}`}
            />

            <KV
              k="ETA"
              v={
                robot.eta_s !==
                null
                  ? `${robot.eta_s} s`
                  : "—"
              }
            />

            <KV
              k="Path"
              v={`${Math.max(
                0,
                robot.path.length -
                  robot.path_index,
              )} cells`}
            />
          </div>

          <div>
            <KV
              k="Tasks completed"
              v={
                robot.stats
                  .tasks_completed
              }
            />

            <KV
              k="Distance"
              v={`${robot.stats.distance_m.toFixed(
                0,
              )} m`}
            />

            <KV
              k="Energy"
              v={`${robot.stats.energy_wh.toFixed(
                0,
              )} Wh`}
            />

            <KV
              k="Busy"
              v={`${(
                robot.stats
                  .busy_ticks /
                10
              ).toFixed(
                0,
              )} s`}
            />

            <KV
              k="Waiting"
              v={`${(
                robot.stats
                  .wait_ticks /
                10
              ).toFixed(
                0,
              )} s`}
            />

            <KV
              k="Position"
              v={`${robot.position[0].toFixed(
                1,
              )}, ${robot.position[2].toFixed(
                1,
              )}`}
            />
          </div>
        </div>

        <h4 className="drawer-sub">
          Recent events
        </h4>

        <table className="dt full">
          <tbody>
            {mine.map(
              (event) => (
                <tr
                  key={event.id}
                >
                  <td>
                    {tickToClock(
                      event.tick,
                      100,
                      true,
                    )}
                  </td>

                  <td
                    className={
                      "sev-" +
                      event.severity
                    }
                  >
                    {
                      event.severity
                    }
                  </td>

                  <td>
                    {
                      event.message
                    }
                  </td>
                </tr>
              ),
            )}
          </tbody>
        </table>
      </div>
    </>
  );
}
