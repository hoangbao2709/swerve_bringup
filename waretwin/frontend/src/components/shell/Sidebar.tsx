import { useStore, type ModalKind } from "../../state/store";

type SidebarProps = {
  collapsed: boolean;
  onToggle: () => void;
};

type MenuItem = {
  label: string;
  caption?: string;
  icon: string;
  path?: string;
  modal?: ModalKind;
  drawer?: "scenarios" | "ops" | "whatif";
};

function navigate(path: string) {
  window.history.pushState({}, "", path);
  window.dispatchEvent(new PopStateEvent("popstate"));
}

function isActive(item: MenuItem, pathname: string, activeWindowId: string | null) {
  if (item.path) return pathname === item.path;
  if (item.modal) return activeWindowId === item.modal;
  return false;
}

export function Sidebar({ collapsed, onToggle }: SidebarProps) {
  const source = useStore((s) => s.source);
  const authUser = useStore((s) => s.authUser);
  const activeWindowId = useStore((s) => s.activeWindowId);
  const setModal = useStore((s) => s.setModal);
  const setDrawer = useStore((s) => s.setDrawer);
  const pathname = window.location.pathname;

  const workspace: MenuItem[] = [
    { label: "Overview", caption: "Operations overview", icon: "⌂", path: "/" },
    { label: "Operations", caption: "Live warehouse console", icon: "◫", path: "/operations" },
    { label: "Robot Fleet", caption: "Fleet status", icon: "▣", modal: "fleet" },
    { label: "Robot Scheduler", caption: "Plans & assignments", icon: "◌", modal: "scheduler" },
    { label: "Inbound / Outbound", caption: "Order flows", icon: "⇄", modal: "flows" },
    { label: "Tasks", caption: "Task queue", icon: "✓", modal: "tasks" },
    { label: "Audit / Events", caption: "Activity log", icon: "≡", modal: "audit" },
  ];

  const operations: MenuItem[] = [
    { label: "Scenarios", caption: "Run a scenario", icon: "▷", drawer: "scenarios" },
    { label: "AI Ops", caption: "Decisions & insights", icon: "✦", drawer: "ops" },
    { label: "What-if", caption: "Test an alternative", icon: "◇", drawer: "whatif" },
  ];

  const activate = (item: MenuItem) => {
    if (item.path) {
      if (pathname !== item.path) navigate(item.path);
      return;
    }
    if (item.modal) {
      setModal(item.modal);
      return;
    }
    if (item.drawer) setDrawer(item.drawer);
  };

  const statusLabel = source === "online" ? "Backend online" : source === "local" ? "Local simulator" : source;
  const statusClass = source === "online" || source === "local" ? "online" : "degraded";

  return (
    <aside className={`wt-sidebar${collapsed ? " collapsed" : ""}`} aria-label="WareTwin navigation">
      <div className="wt-sidebar-head">
        <div className="wt-sidebar-brand" title="WareTwin">
          <span className="wt-sidebar-mark">W</span>
          <span className="wt-sidebar-brand-copy"><b>WareTwin</b><small>Operations Console</small></span>
        </div>
        <button className="wt-sidebar-toggle" onClick={onToggle} title={collapsed ? "Expand sidebar" : "Collapse sidebar"} aria-label={collapsed ? "Expand sidebar" : "Collapse sidebar"}>
          {collapsed ? "›" : "‹"}
        </button>
      </div>

      <div className="wt-sidebar-status">
        <span className={`wt-status-dot ${statusClass}`} />
        <span className="wt-sidebar-status-copy"><b>{statusLabel.toUpperCase()}</b><small>Map workspace ready</small></span>
      </div>

      <nav className="wt-sidebar-nav">
        <div className="wt-sidebar-section-label">Workspace</div>
        {workspace.map((item) => (
          <button key={item.label} className={`wt-sidebar-item${isActive(item, pathname, activeWindowId) ? " active" : ""}`} onClick={() => activate(item)} title={collapsed ? item.label : undefined}>
            <span className="wt-sidebar-icon">{item.icon}</span>
            <span className="wt-sidebar-item-copy"><b>{item.label}</b><small>{item.caption}</small></span>
            {isActive(item, pathname, activeWindowId) && <span className="wt-sidebar-active-mark" />}
          </button>
        ))}

        <div className="wt-sidebar-section-label">Operations</div>
        {operations.map((item) => (
          <button key={item.label} className="wt-sidebar-item" onClick={() => activate(item)} title={collapsed ? item.label : undefined}>
            <span className="wt-sidebar-icon">{item.icon}</span>
            <span className="wt-sidebar-item-copy"><b>{item.label}</b><small>{item.caption}</small></span>
          </button>
        ))}

        {authUser?.role === "admin" && <>
          <div className="wt-sidebar-section-label">Administration</div>
          <button className="wt-sidebar-item" onClick={() => navigate("/admin/warehouse")} title={collapsed ? "Warehouse Data" : undefined}>
            <span className="wt-sidebar-icon">▤</span><span className="wt-sidebar-item-copy"><b>Warehouse Data</b><small>Layout & records</small></span>
          </button>
          <button className="wt-sidebar-item" onClick={() => navigate("/admin/warehouse-editor")} title={collapsed ? "Map Editor" : undefined}>
            <span className="wt-sidebar-icon">⌗</span><span className="wt-sidebar-item-copy"><b>Map Editor</b><small>Configure workspace</small></span>
          </button>
        </>}
      </nav>

      <div className="wt-sidebar-footer">
        <div className="wt-sidebar-user">
          <span className="wt-sidebar-avatar">{(authUser?.username ?? "OP").slice(0, 2).toUpperCase()}</span>
          <span className="wt-sidebar-item-copy"><b>{authUser?.username ?? "operator"}</b><small>{authUser?.role ?? "operator"}</small></span>
        </div>
        {!collapsed && <div className="wt-sidebar-hint">Use the taskbar to open and focus windows.</div>}
      </div>
    </aside>
  );
}
