export type ControlSection = "CONTROL" | "MAPPING" | "LOCALIZATION" | "SYSTEM" | "VDA5050";

type SidebarProps = {
  activeSection?: ControlSection;
  onSectionChange?: (section: ControlSection) => void;
  /** Legacy props accepted for the dormant Overview component, never shown in the operator route. */
  collapsed?: boolean;
  onToggle?: () => void;
};

const sections: Array<{ id: ControlSection; label: string; icon: string }> = [
  { id: "CONTROL", label: "CONTROL", icon: "⌖" },
  { id: "MAPPING", label: "MAPPING", icon: "▦" },
  { id: "LOCALIZATION", label: "LOCALIZATION", icon: "◎" },
  { id: "SYSTEM", label: "SYSTEM", icon: "⚙" },
  { id: "VDA5050", label: "VDA5050", icon: "⇄" },
];

export function Sidebar({ activeSection = "CONTROL", onSectionChange = () => undefined }: SidebarProps) {
  return (
    <aside className="wt-sidebar industrial-tool-rail" aria-label="Robot control sections">
      <div className="wt-sidebar-head">
        <div className="wt-sidebar-brand" aria-label="WareTwin">
          <span className="wt-sidebar-mark">W</span>
        </div>
      </div>
      <nav className="wt-sidebar-nav" role="tablist" aria-label="Control views" aria-orientation="vertical">
        {sections.map((section) => (
          <button key={section.id} type="button" role="tab"
            aria-label={section.label}
            aria-selected={activeSection === section.id}
            className={`wt-sidebar-item${activeSection === section.id ? " active" : ""}`}
            onClick={() => onSectionChange(section.id)}>
            <span className="wt-sidebar-icon" aria-hidden="true">{section.icon}</span>
            <span className="wt-sidebar-item-copy"><b>{section.label}</b></span>
          </button>
        ))}
      </nav>
    </aside>
  );
}
