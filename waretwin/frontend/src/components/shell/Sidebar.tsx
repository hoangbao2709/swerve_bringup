import { useStore } from "../../state/store";

type SidebarProps = {
  collapsed: boolean;
  onToggle: () => void;
};

type MenuItem = {
  label: string;
  caption?: string;
  icon: string;
  path?: string;
};

function navigate(path: string) {
  window.history.pushState({}, "", path);
  window.dispatchEvent(new PopStateEvent("popstate"));
}

function isActive(item: MenuItem, pathname: string) {
  return Boolean(item.path && pathname === item.path);
}

export function Sidebar({ collapsed, onToggle }: SidebarProps) {
  const source = useStore((s) => s.source);
  const pathname = window.location.pathname;

  const workspace: MenuItem[] = [
    { label: "Robot Control", caption: "Manual + point navigation", icon: "◎", path: "/control" },
  ];

  const activate = (item: MenuItem) => {
    if (item.path && pathname !== item.path) navigate(item.path);
  };

  const statusLabel = source === "online" ? "Backend online" : source;
  const statusClass = source === "online" ? "online" : "degraded";

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
          <button key={item.label} className={`wt-sidebar-item${isActive(item, pathname) ? " active" : ""}`} onClick={() => activate(item)} title={collapsed ? item.label : undefined}>
            <span className="wt-sidebar-icon">{item.icon}</span>
            <span className="wt-sidebar-item-copy"><b>{item.label}</b><small>{item.caption}</small></span>
            {isActive(item, pathname) && <span className="wt-sidebar-active-mark" />}
          </button>
        ))}
      </nav>

      <div className="wt-sidebar-footer">
        {!collapsed && <div className="wt-sidebar-hint">Use the taskbar to open and focus windows.</div>}
      </div>
    </aside>
  );
}
