import React, { useEffect, useRef, useState } from "react";
import { useStore, tickToClock } from "../../state/store";
import { simControl } from "../../simulation/runner";
import { Icon } from "../ui/primitives";
import { logout } from "../../services/auth";
import { DEMO_MODE } from "../../config";

function go(path: string) {
  window.history.pushState({}, "", path);
  window.dispatchEvent(new PopStateEvent("popstate"));
}



type SvgProps = { className?: string };

type Accent = "sky" | "amber" | "rose" | "emerald" | "violet" | "slate";

function SvgIcon({ children, className = "h-4 w-4" }: React.PropsWithChildren<SvgProps>) {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" className={className} aria-hidden="true">
      {children}
    </svg>
  );
}

function UserIcon({ className }: SvgProps) {
  return <SvgIcon className={className}><path d="M19 21v-1.5a4.5 4.5 0 0 0-4.5-4.5h-5A4.5 4.5 0 0 0 5 19.5V21" /><circle cx="12" cy="7" r="4" /></SvgIcon>;
}
function BellIcon({ className }: SvgProps) {
  return <SvgIcon className={className}><path d="M18 8a6 6 0 0 0-12 0c0 7-3 7-3 9h18c0-2-3-2-3-9" /><path d="M10 21h4" /></SvgIcon>;
}
function FleetIcon({ className }: SvgProps) {
  return <SvgIcon className={className}><rect x="3" y="6" width="18" height="11" rx="2" /><path d="M7 6V4h10v2M7 17v3M17 17v3M7 11h.01M17 11h.01" /></SvgIcon>;
}
function ClipboardIcon({ className }: SvgProps) {
  return <SvgIcon className={className}><rect x="5" y="4" width="14" height="17" rx="2" /><path d="M9 4V3h6v1M8 9h8M8 13h8M8 17h5" /></SvgIcon>;
}
function ActivityIcon({ className }: SvgProps) {
  return <SvgIcon className={className}><path d="M3 12h4l2-7 4 14 2-7h6" /></SvgIcon>;
}
function ShieldIcon({ className }: SvgProps) {
  return <SvgIcon className={className}><path d="M12 3l7 3v5c0 4.5-2.7 7.7-7 10-4.3-2.3-7-5.5-7-10V6l7-3Z" /><path d="m9.5 12 1.7 1.7 3.6-3.8" /></SvgIcon>;
}
function UsersIcon({ className }: SvgProps) {
  return <SvgIcon className={className}><circle cx="9" cy="8" r="3" /><path d="M3 20a6 6 0 0 1 12 0M17 11a3 3 0 1 0 0-6M17 14a4.5 4.5 0 0 1 4 6" /></SvgIcon>;
}
function WarehouseIcon({ className }: SvgProps) {
  return <SvgIcon className={className}><path d="M3 10 12 4l9 6v10H3V10Z" /><path d="M7 20v-6h4v6M11 14h6M14 10h.01" /></SvgIcon>;
}
function FlaskIcon({ className }: SvgProps) {
  return <SvgIcon className={className}><path d="M9 3h6M10 3v6l-5.5 9.5A2 2 0 0 0 6.2 21h11.6a2 2 0 0 0 1.7-2.5L14 9V3" /><path d="M8 15h8" /></SvgIcon>;
}
function SettingsIcon({ className }: SvgProps) {
  return <SvgIcon className={className}><path d="M12 8.5a3.5 3.5 0 1 0 0 7 3.5 3.5 0 0 0 0-7Z" /><path d="m19.4 15 .1.1a2 2 0 0 1-2.8 2.8l-.1-.1a2 2 0 0 0-3.4 1.4V19a2 2 0 0 1-4 0v-.2a2 2 0 0 0-3.4-1.4l-.1.1a2 2 0 1 1-2.8-2.8l.1-.1A2 2 0 0 0 1.6 11H2a2 2 0 0 1 0-4h-.4a2 2 0 0 1 1.4 3.4l-.1.1A2 2 0 1 1 5.7 7.7l.1.1A2 2 0 0 0 9.2 6.4V6a2 2 0 0 1 4 0v.4a2 2 0 0 0 3.4 1.4l.1-.1A2 2 0 1 1 19.5 10l-.1.1a2 2 0 0 0 1.4 3.4h.2a2 2 0 0 1 0 4h-.2a2 2 0 0 0-1.4-3.4Z" /></SvgIcon>;
}
function LogoutIcon({ className }: SvgProps) {
  return <SvgIcon className={className}><path d="M10 17l5-5-5-5M15 12H3M14 4h5a2 2 0 0 1 2 2v12a2 2 0 0 1-2 2h-5" /></SvgIcon>;
}
function ChevronRightIcon({ className }: SvgProps) {
  return <SvgIcon className={className}><path d="m9 5 7 7-7 7" /></SvgIcon>;
}
function ChevronDownIcon({ className }: SvgProps) {
  return <SvgIcon className={className}><path d="m6 9 6 6 6-6" /></SvgIcon>;
}

function MenuSectionLabel({ children }: React.PropsWithChildren) {
  return <div className="px-2 pb-1.5 pt-1 text-[9px] font-bold tracking-[0.18em] text-slate-600">{children}</div>;
}

function MenuIconBox({ accent, children }: { accent: Accent; children: React.ReactNode }) {
  const styles: Record<Accent, string> = {
    sky: "border-sky-400/10 bg-sky-400/[0.07] text-sky-300",
    amber: "border-amber-400/10 bg-amber-400/[0.07] text-amber-300",
    rose: "border-rose-400/10 bg-rose-400/[0.07] text-rose-300",
    emerald: "border-emerald-400/10 bg-emerald-400/[0.07] text-emerald-300",
    violet: "border-violet-400/10 bg-violet-400/[0.07] text-violet-300",
    slate: "border-white/[0.07] bg-white/[0.03] text-slate-300",
  };
  return <span className={["grid h-9 w-9 shrink-0 place-items-center rounded-lg border", styles[accent]].join(" ")}>{children}</span>;
}

function MenuItem({
  icon,
  accent,
  title,
  description,
  trailing,
  onClick,
}: {
  icon: React.ReactNode;
  accent: Accent;
  title: string;
  description: string;
  trailing?: React.ReactNode;
  onClick: () => void;
}) {
  return (
    <button type="button" role="menuitem" className="group flex w-full items-center gap-3 rounded-xl border border-transparent px-2.5 py-2.5 text-left transition-all duration-150 hover:border-white/[0.05] hover:bg-white/[0.045]" onClick={onClick}>
      <MenuIconBox accent={accent}>{icon}</MenuIconBox>
      <span className="min-w-0 flex-1">
        <span className="block truncate text-[12px] font-semibold text-slate-200 transition-colors group-hover:text-white">{title}</span>
        <span className="mt-0.5 block truncate text-[10px] text-slate-500">{description}</span>
      </span>
      {trailing ?? <ChevronRightIcon className="h-3.5 w-3.5 shrink-0 text-slate-700 transition-all group-hover:translate-x-0.5 group-hover:text-slate-400" />}
    </button>
  );
}

function AdminMenuItem({ icon, title, description, onClick }: { icon: React.ReactNode; title: string; description: string; onClick: () => void }) {
  return (
    <button type="button" role="menuitem" className="group flex w-full items-center gap-3 rounded-xl px-2.5 py-2.5 text-left transition-colors hover:bg-violet-400/[0.06]" onClick={onClick}>
      <span className="grid h-8 w-8 shrink-0 place-items-center rounded-lg border border-violet-400/10 bg-violet-400/[0.06] text-violet-300">{icon}</span>
      <span className="min-w-0 flex-1">
        <span className="block truncate text-[11px] font-semibold text-slate-300 transition-colors group-hover:text-white">{title}</span>
        <span className="mt-0.5 block truncate text-[9px] text-slate-600 group-hover:text-slate-500">{description}</span>
      </span>
      <ChevronRightIcon className="h-3 w-3 text-slate-700 transition-all group-hover:translate-x-0.5 group-hover:text-violet-300" />
    </button>
  );
}

function goAndClose(path: string, setUserMenuOpen: (value: boolean) => void, setAdminMenuOpen: (value: boolean) => void) {
  setUserMenuOpen(false);
  setAdminMenuOpen(false);
  go(path);
}

export function TopBar() {
  const mode = useStore((s) => s.twin.sim.mode);
  const tick = useStore((s) => s.twin.sim.tick);
  const speed = useStore((s) => s.speed);
  const paused = useStore((s) => s.paused);
  const source = useStore((s) => s.source);
  const runtimeMode = useStore((s) => s.runtimeMode);
  const rosConnected = useStore((s) => s.rosConnected);
  const nav2State = useStore((s) => s.nav2State);
  const drawer = useStore((s) => s.drawer);
  const setDrawer = useStore((s) => s.setDrawer);
  const setModal = useStore((s) => s.setModal);
  const alerts = useStore((s) => s.twin.alerts);
  const quality = useStore((s) => s.quality);
  const setQuality = useStore((s) => s.setQuality);
  const authUser = useStore((s) => s.authUser);

  const [userMenuOpen, setUserMenuOpen] = useState(false);
  const [adminMenuOpen, setAdminMenuOpen] = useState(false);

  const userMenuRef = useRef<HTMLDivElement>(null);

  const unack = Object.values(alerts).filter((a) => !a.acknowledged).length;
  const simDate = "2026/05/20";

  useEffect(() => {
    function handleOutsideClick(event: MouseEvent) {
      if (
        userMenuRef.current &&
        !userMenuRef.current.contains(event.target as Node)
      ) {
        setUserMenuOpen(false);
        setAdminMenuOpen(false);
      }
    }

    document.addEventListener("mousedown", handleOutsideClick);
    return () => document.removeEventListener("mousedown", handleOutsideClick);
  }, []);

  async function handleLogout() {
    setUserMenuOpen(false);
    setAdminMenuOpen(false);
    if (DEMO_MODE) return;
    await logout();
    go("/login");
  }

  return (
    <header className="topbar">
      <div className="brand">
        <span className="ai">Ware</span>
        <span>Twin</span>
        <span className="brand-sub">Warehouse Digital Twin</span>
      </div>

      <div className="topbar-statuses" aria-label="System status">
        <span
          className={
            "badge-live " +
            (paused ? "paused" : mode === "WHATIF" ? "whatif" : "live")
          }
        >
          <span
            className="dot"
            style={{
              background: "currentColor",
              width: 6,
              height: 6,
            }}
          />
          {paused ? "PAUSED" : mode === "WHATIF" ? "SIMULATION" : "LIVE"}
        </span>

        <span className="badge-src online" title={`ROS bridge ${rosConnected ? "connected" : "offline"}; Nav2 ${nav2State}`}>
          <span className="dot" style={{ background: rosConnected || runtimeMode === "LOCAL_SIM" ? "currentColor" : "#f87171", width: 6, height: 6 }} />
          {runtimeMode.replace("_", " ")}
        </span>

        <span
          className={"badge-src " + source}
          title={
            source === "online"
              ? "Connected to Django backend (Channels WebSocket)"
              : source === "local"
                ? "Frontend demo mode — local simulation engine"
                : source === "offline"
                  ? "Django backend unreachable — state is frozen at the last confirmed value"
                  : source === "unauthorized"
                    ? "Backend authentication failed"
                    : "Connecting to Django backend…"
          }
        >
          <span
            className="dot"
            style={{
              background: "currentColor",
              width: 6,
              height: 6,
            }}
          />
          {source === "online"
            ? "BACKEND"
            : source === "local"
              ? "LOCAL"
              : source === "offline"
                ? "OFFLINE"
                : source === "unauthorized"
                  ? "AUTH"
                  : "CONNECTING"}
        </span>
      </div>

      <div className="topbar-right">
        <div className="sim-controls" title="Simulation controls">
          <button
            className={!paused ? "on" : ""}
            title="Play"
            onClick={() => simControl.play()}
          >
            {Icon.play}
          </button>

          <button
            className={paused ? "on" : ""}
            title="Pause"
            onClick={() => simControl.pause()}
          >
            {Icon.pause}
          </button>

          <button
            title="Reset (same seed)"
            onClick={() => simControl.reset()}
          >
            {Icon.reset}
          </button>

          {([1, 2, 5, 10] as const).map((x) => (
            <button
              key={x}
              className={speed === x ? "on" : ""}
              onClick={() => simControl.play(x)}
            >
              {x}×
            </button>
          ))}
        </div>

        <div className="clock" title="Simulation time">
          <div className="t">{tickToClock(tick, 100, true)}</div>
          <div className="d">
            {simDate} · T{tick}
          </div>
        </div>

        <div className="vsep" />

        <button
          className={
            "tb-btn" + (drawer === "scenarios" ? " on" : "")
          }
          onClick={() => setDrawer("scenarios")}
          title="Failure injection"
        >
          {Icon.bolt}
          <span>Scenarios</span>
        </button>

        <button
          className={"tb-btn" + (drawer === "ops" ? " on" : "")}
          onClick={() => setDrawer("ops")}
          title="AI Operations: KPI + explainable decisions"
        >
          {Icon.brain}
          <span>AI Ops</span>
        </button>

        <button
          className={"tb-btn" + (drawer === "whatif" ? " on" : "")}
          onClick={() => setDrawer("whatif")}
          title="What-if simulation: clone the twin, inject, compare KPI"
        >
          {Icon.fork}
          <span>What-if</span>
        </button>

        <div className="vsep" />

        <button
          className="icon-btn"
          title="Audit log"
          onClick={() => setModal("audit")}
        >
          {Icon.bell}
          {unack > 0 && <span className="dot">{unack}</span>}
        </button>

        <button
          className="icon-btn"
          title={`Render quality: ${quality} (click to cycle)`}
          onClick={() =>
            setQuality(
              quality === "low"
                ? "medium"
                : quality === "medium"
                  ? "high"
                  : "low",
            )
          }
        >
          {Icon.gear}
        </button>

        <div className="vsep" />

        {authUser?.role === "admin" && (
          <button
            className="tb-btn"
            onClick={() => go("/admin")}
            title="Admin Panel"
          >
            {Icon.gear}
            <span>Admin Panel</span>
          </button>
        )}
        {/* USER ACCOUNT */}
        <div
          ref={userMenuRef}
          className="topbar-user-anchor relative"
          onMouseEnter={() => setUserMenuOpen(true)}
          onMouseLeave={() => {
            setUserMenuOpen(false);
            setAdminMenuOpen(false);
          }}
        >
          <button
            type="button"
            className={[
              "group flex h-10 items-center gap-2.5 rounded-xl px-2.5",
              "border border-transparent text-slate-300",
              "transition-all duration-200",
              "hover:border-white/[0.08] hover:bg-white/[0.045] hover:text-white",
              "focus:outline-none focus-visible:ring-2 focus-visible:ring-sky-400/30",
              userMenuOpen ? "border-white/[0.10] bg-white/[0.06] text-white" : "",
            ].join(" ")}
            onClick={() => setUserMenuOpen((v) => !v)}
            title="Account menu"
            aria-expanded={userMenuOpen}
            aria-haspopup="menu"
          >
            <span className="relative grid h-8 w-8 shrink-0 place-items-center rounded-full border border-white/10 bg-slate-900/70 text-slate-300 transition-all group-hover:border-sky-400/30 group-hover:text-white">
              <UserIcon className="h-4 w-4" />
              <span className="absolute -bottom-0.5 -right-0.5 h-2.5 w-2.5 rounded-full border-2 border-[#0b1020] bg-emerald-400 shadow-[0_0_8px_rgba(52,211,153,.55)]" />
            </span>

            <span className="hidden max-w-[150px] truncate text-[12px] font-medium xl:inline">
              {authUser?.username ?? "Guest"}
              <span className="mx-1 text-slate-600">·</span>
              <span className="text-slate-500">{authUser?.role?.toUpperCase() ?? "GUEST"}</span>
            </span>

            <ChevronDownIcon
              className={[
                "h-3.5 w-3.5 text-slate-500 transition-transform duration-200",
                userMenuOpen ? "rotate-180 text-slate-300" : "",
              ].join(" ")}
            />
          </button>

          {userMenuOpen && (
            <div
              className={[
                "fixed right-4 top-[58px] z-[99999]",
                "w-[360px] max-w-[calc(100vw-24px)]",
                "overflow-visible rounded-2xl border border-white/[0.10]",
                "bg-[#0b1220]/[0.985] text-slate-200",
                "shadow-[0_28px_90px_rgba(0,0,0,0.52)]",
                "backdrop-blur-2xl",
                "ring-1 ring-black/20",
              ].join(" ")}
              role="menu"
              onMouseEnter={() => setUserMenuOpen(true)}
              onMouseLeave={() => {
                setUserMenuOpen(false);
                setAdminMenuOpen(false);
              }}
            >
              {/* Account identity */}
              <div className="p-3.5">
                <div className="relative overflow-hidden rounded-xl border border-white/[0.07] bg-gradient-to-br from-white/[0.05] to-white/[0.015] p-3.5">
                  <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-sky-400/35 to-transparent" />

                  <div className="flex items-center gap-3">
                    <div className="relative grid h-11 w-11 shrink-0 place-items-center rounded-xl border border-white/[0.09] bg-slate-900 text-slate-200 shadow-inner">
                      <UserIcon className="h-[19px] w-[19px]" />
                      <span className="absolute -bottom-0.5 -right-0.5 h-2.5 w-2.5 rounded-full border-2 border-[#142033] bg-emerald-400 shadow-[0_0_8px_rgba(52,211,153,.55)]" />
                    </div>

                    <div className="min-w-0 flex-1">
                      <div className="truncate text-[13px] font-semibold text-white">
                        {authUser?.username ?? "Guest"}
                      </div>
                      <div className="mt-0.5 text-[11px] text-slate-400">
                        {authUser?.role === "admin" ? "System Administrator" : "Operations Operator"}
                      </div>
                      <div className="mt-1.5 flex items-center gap-1.5 text-[10px] font-medium uppercase tracking-[0.12em] text-emerald-300/80">
                        <span className="h-1.5 w-1.5 rounded-full bg-emerald-400 shadow-[0_0_7px_rgba(52,211,153,.7)]" />
                        Session active
                      </div>
                    </div>

                    <span className={[
                      "shrink-0 rounded-md border px-2 py-1",
                      "text-[9px] font-bold tracking-[0.16em]",
                      authUser?.role === "admin"
                        ? "border-sky-400/20 bg-sky-400/[0.08] text-sky-300"
                        : "border-slate-400/15 bg-white/[0.04] text-slate-300",
                    ].join(" ")}>
                      {authUser?.role?.toUpperCase() ?? "GUEST"}
                    </span>
                  </div>
                </div>
              </div>

              <div className="mx-3 h-px bg-white/[0.07]" />

              {/* Operations */}
              <div
                className="p-2.5"
                onMouseEnter={() => setAdminMenuOpen(false)}
              >
                <MenuSectionLabel>OPERATIONS</MenuSectionLabel>

                <MenuItem
                  icon={<FleetIcon />}
                  accent="sky"
                  title="Fleet Management"
                  description="Robots, health, battery & assignments"
                  onClick={() => {
                    setUserMenuOpen(false);
                    setAdminMenuOpen(false);
                    go("/admin/robots");
                  }}
                />

                <MenuItem
                  icon={<ClipboardIcon />}
                  accent="amber"
                  title="Task Operations"
                  description="Dispatch, assignment & task history"
                  onClick={() => {
                    setUserMenuOpen(false);
                    setAdminMenuOpen(false);
                    go("/admin/tasks");
                  }}
                />

                <MenuItem
                  icon={
                    <div className="relative">
                      <BellIcon />
                      {unack > 0 && (
                        <span className="absolute -right-2.5 -top-2 grid min-w-[15px] h-[15px] place-items-center rounded-full bg-rose-500 px-1 text-[8px] font-bold text-white ring-2 ring-[#0b1220]">
                          {unack > 99 ? "99+" : unack}
                        </span>
                      )}
                    </div>
                  }
                  accent="rose"
                  title="Notifications"
                  description={
                    unack > 0
                      ? `${unack} active alert${unack > 1 ? "s" : ""} require attention`
                      : "No active alerts require attention"
                  }
                  trailing={
                    unack > 0 ? (
                      <span className="rounded-md bg-rose-500/10 px-1.5 py-1 text-[9px] font-bold text-rose-300">
                        {unack}
                      </span>
                    ) : undefined
                  }
                  onClick={() => {
                    setUserMenuOpen(false);
                    setAdminMenuOpen(false);
                    setModal("audit");
                  }}
                />

                <MenuItem
                  icon={<ActivityIcon />}
                  accent="emerald"
                  title="System Status"
                  description={
                    source === "online"
                      ? "Backend, WebSocket & simulation connected"
                      : source === "local"
                        ? "Backend unavailable · local engine active"
                        : "Connecting to backend…"
                  }
                  trailing={
                    <span
                      className={[
                        "h-2 w-2 rounded-full",
                        source === "online"
                          ? "bg-emerald-400 shadow-[0_0_10px_rgba(52,211,153,.8)]"
                          : "bg-amber-400 shadow-[0_0_10px_rgba(251,191,36,.7)]",
                      ].join(" ")}
                    />
                  }
                  onClick={() => {
                    setUserMenuOpen(false);
                    setAdminMenuOpen(false);
                    go("/admin/system");
                  }}
                />
              </div>

              {/* Administration */}
              {authUser?.role === "admin" && (
                <>
                  <div className="mx-3 h-px bg-white/[0.07]" />

                  <div className="p-2.5">
                    <MenuSectionLabel>ADMINISTRATION</MenuSectionLabel>

                    <div
                      className="relative"
                      onMouseEnter={() => setAdminMenuOpen(true)}
                      onMouseLeave={() => setAdminMenuOpen(false)}
                    >
                      <button
                        type="button"
                        role="menuitem"
                        aria-haspopup="menu"
                        aria-expanded={adminMenuOpen}
                        className={[
                          "group flex w-full items-center gap-3 rounded-xl px-2.5 py-2.5 text-left",
                          "border transition-all duration-150",
                          adminMenuOpen
                            ? "border-violet-400/15 bg-violet-400/[0.07]"
                            : "border-transparent hover:border-white/[0.05] hover:bg-white/[0.045]",
                        ].join(" ")}
                        onClick={() => setAdminMenuOpen((v) => !v)}
                      >
                        <MenuIconBox accent="violet">
                          <ShieldIcon />
                        </MenuIconBox>

                        <span className="min-w-0 flex-1">
                          <span className="block text-[12px] font-semibold text-slate-200 group-hover:text-white">
                            Administration
                          </span>
                          <span className="mt-0.5 block text-[10px] text-slate-500">
                            Users, robots, warehouse & system
                          </span>
                        </span>

                        <span className={[
                          "grid h-6 w-6 place-items-center rounded-md",
                          "border border-white/[0.05] bg-white/[0.025]",
                          "transition-all duration-200",
                          adminMenuOpen
                            ? "border-violet-400/20 bg-violet-400/10 text-violet-300"
                            : "text-slate-600 group-hover:text-slate-400",
                        ].join(" ")}>
                          <ChevronRightIcon className="h-3.5 w-3.5" />
                        </span>
                      </button>

                      {/* Professional flyout */}
                      {adminMenuOpen && (
                        <div
                          className={[
                            "absolute right-full top-1/2 z-[100000] mr-0 w-[250px] -translate-y-1/2",
                            "rounded-2xl border border-white/[0.10]",
                            "bg-[#0b1220]/[0.985] p-2",
                            "shadow-[0_24px_70px_rgba(0,0,0,.5)] backdrop-blur-2xl",
                            "ring-1 ring-black/20",
                          ].join(" ")}
                          onMouseEnter={() => setAdminMenuOpen(true)}
                          onMouseLeave={() => setAdminMenuOpen(false)}
                          role="menu"
                        >
                          <div className="mb-1 px-2 py-1.5">
                            <div className="text-[9px] font-bold uppercase tracking-[0.16em] text-violet-300/80">
                              Administration
                            </div>
                            <div className="mt-1 text-[10px] text-slate-600">
                              Platform configuration
                            </div>
                          </div>

                          <AdminMenuItem
                            icon={<UsersIcon />}
                            title="Users & Access"
                            description="Accounts and permissions"
                            onClick={() => goAndClose("/admin/users", setUserMenuOpen, setAdminMenuOpen)}
                          />
                          <AdminMenuItem
                            icon={<FleetIcon />}
                            title="Fleet & Robots"
                            description="Robot operations"
                            onClick={() => goAndClose("/admin/robots", setUserMenuOpen, setAdminMenuOpen)}
                          />
                          <AdminMenuItem
                            icon={<ClipboardIcon />}
                            title="Task Dispatch"
                            description="Assignment and queue"
                            onClick={() => goAndClose("/admin/tasks", setUserMenuOpen, setAdminMenuOpen)}
                          />
                          <AdminMenuItem
                            icon={<WarehouseIcon />}
                            title="Warehouse Data"
                            description="Warehouse, zones & shelves"
                            onClick={() => goAndClose("/admin/warehouse", setUserMenuOpen, setAdminMenuOpen)}
                          />
                          <AdminMenuItem
                            icon={<WarehouseIcon />}
                            title="Warehouse Editor"
                            description="Layout and navigation"
                            onClick={() => goAndClose("/admin/warehouse-editor", setUserMenuOpen, setAdminMenuOpen)}
                          />
                          <AdminMenuItem
                            icon={<FlaskIcon />}
                            title="Scenarios"
                            description="Simulation & what-if"
                            onClick={() => goAndClose("/admin/scenarios", setUserMenuOpen, setAdminMenuOpen)}
                          />
                          <AdminMenuItem
                            icon={<SettingsIcon />}
                            title="System"
                            description="Backend and configuration"
                            onClick={() => goAndClose("/admin/system", setUserMenuOpen, setAdminMenuOpen)}
                          />
                        </div>
                      )}
                    </div>
                  </div>
                </>
              )}

              <div className="mx-3 h-px bg-white/[0.07]" />

              {/* Account */}
              <div
                className="p-2.5"
                onMouseEnter={() => setAdminMenuOpen(false)}
              >
                <MenuSectionLabel>ACCOUNT</MenuSectionLabel>

                <MenuItem
                  icon={<UserIcon />}
                  accent="slate"
                  title="My Profile"
                  description="Account information & role"
                  onClick={() => {
                    setUserMenuOpen(false);
                    setAdminMenuOpen(false);
                    go("/profile");
                  }}
                />

                <MenuItem
                  icon={<SettingsIcon />}
                  accent="slate"
                  title="Preferences"
                  description="Display & workspace settings"
                  onClick={() => {
                    setUserMenuOpen(false);
                    setAdminMenuOpen(false);
                    go("/preferences");
                  }}
                />
              </div>

              <div className="mx-3 h-px bg-white/[0.07]" />

              {/* Logout */}
              <div className="p-2.5">
                <button
                  type="button"
                  role="menuitem"
                  className="group flex w-full items-center gap-3 rounded-xl border border-transparent px-2.5 py-2.5 text-left transition-all hover:border-rose-400/10 hover:bg-rose-500/[0.07]"
                  onClick={() => void handleLogout()}
                >
                  <span className="grid h-9 w-9 shrink-0 place-items-center rounded-lg border border-rose-400/10 bg-rose-400/[0.07] text-rose-300">
                    <LogoutIcon className="h-4 w-4" />
                  </span>
                  <span className="min-w-0 flex-1">
                    <span className="block text-[12px] font-semibold text-rose-300">
                      {DEMO_MODE ? "Local Demo" : "Sign out"}
                    </span>
                    <span className="mt-0.5 block text-[10px] text-slate-600 group-hover:text-slate-500">
                      End current session
                    </span>
                  </span>
                </button>
              </div>
            </div>
          )}
        </div>
      </div>
    </header>
  );
}
