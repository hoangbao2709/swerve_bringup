
import type { ReactNode } from "react";
import {
  handleLocalLinkClick,
  isSettingsPathname,
  sectionForPathname,
  useLocalPathname,
  LOCAL_APP_PATHS,
} from "../routing/localRoutes";

type SidebarProps = {
  collapsed?: boolean;
  onToggle?: () => void;
};

type IconName =
  | "system"
  | "control"
  | "settings"
  | "diagnosis"
  | "mapping"
  | "localization"
  | "vda";

const iconPaths: Record<IconName, readonly string[]> = {
  system: [
    "M3 4h18v14H3z",
    "M8 22h8",
    "M12 18v4",
    "M7 9h4",
    "M7 13h7",
  ],
  control: [
    "M12 2v3",
    "M12 19v3",
    "M2 12h3",
    "M19 12h3",
    "M12 7a5 5 0 1 0 0 10a5 5 0 1 0 0-10",
    "M12 10v4",
    "M10 12h4",
  ],
  settings: [
    "M12 2.5 14 4l2.4-.3.7 2.3 2.2 1.1-1 2.2 1 2.7-1 2.7 1 2.2-2.2 1.1-.7 2.3L14 20l-2 1.5-2-1.5-2.4.3-.7-2.3-2.2-1.1 1-2.2-1-2.7 1-2.7-1-2.2 2.2-1.1.7-2.3L10 4z",
    "M12 9a3 3 0 1 0 0 6a3 3 0 1 0 0-6",
  ],
  diagnosis: [
    "M3 12h4l3-6 4 12 3-6h4",
  ],
  mapping: [
    "M3 3h7v7H3z",
    "M14 3h7v7h-7z",
    "M3 14h7v7H3z",
    "M14 14h7v7h-7z",
  ],
  localization: [
    "M12 2v3",
    "M12 19v3",
    "M2 12h3",
    "M19 12h3",
    "M12 7a5 5 0 1 0 0 10a5 5 0 1 0 0-10",
    "M12 10a2 2 0 1 0 0 4a2 2 0 1 0 0-4",
  ],
  vda: [
    "M5 7h14",
    "m16 4 3 3-3 3",
    "M19 17H5",
    "m8 14-3 3 3 3",
  ],
};

function NavIcon({ name }: { name: IconName }) {
  return (
    <svg
      aria-hidden="true"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="1.7"
      strokeLinecap="round"
      strokeLinejoin="round"
      className="h-[17px] w-[17px] shrink-0"
    >
      {iconPaths[name].map((path, index) => (
        <path key={`${name}-${index}`} d={path} />
      ))}
    </svg>
  );
}

type NavigationLinkProps = {
  path: string;
  label: string;
  active: boolean;
  icon: IconName;
};

function NavigationLink({
  path,
  label,
  active,
  icon,
}: NavigationLinkProps) {
  return (
    <a
      href={path}
      aria-label={label.toUpperCase()}
      aria-current={active ? "page" : undefined}
      onClick={(event) => handleLocalLinkClick(event, path)}
      className={[
        "wt-sidebar-item group relative",
        "flex min-h-10 min-w-0 items-center gap-2.5",
        "overflow-hidden rounded-[4px]",
        "px-2.5 py-2 no-underline",
        "text-[0.8125rem]",
        "transition-colors duration-150",
        "focus-visible:outline-2",
        "focus-visible:outline-offset-[-2px]",
        "focus-visible:outline-ptagv-blue",
        active
          ? "active bg-blue-50 font-semibold text-ptagv-navy"
          : "font-medium text-slate-700 hover:bg-slate-100 hover:text-ptagv-navy",
      ].join(" ")}
    >
      {/* Bottom line: grow LEFT to RIGHT on hover. */}
      <span
        aria-hidden="true"
        className={[
          "pointer-events-none",
          "absolute bottom-0 left-0",
          "h-[3px] w-full",
          "origin-left bg-red-500",
          "shadow-[0_0_10px_rgba(239,68,68,0.45)]",
          "transition-transform",
          "duration-400 ease-linear",
          "motion-reduce:transition-none",
          active
            ? "scale-x-100"
            : "scale-x-0 group-hover:scale-x-100 group-focus-visible:scale-x-100",
        ].join(" ")}
      />

      <span
        className={[
          "relative z-10 flex shrink-0",
          active
            ? "text-ptagv-blue"
            : "text-slate-500 group-hover:text-ptagv-blue",
        ].join(" ")}
      >
        <NavIcon name={icon} />
      </span>

      <span className="relative z-10 min-w-0 flex-1 leading-snug [overflow-wrap:anywhere]">
        {label}
      </span>
    </a>
  );
}

function NavigationSection({
  title,
  children,
}: {
  title: string;
  children: ReactNode;
}) {
  return (
    <section
      aria-label={`${title} navigation`}
      className="grid gap-1"
    >
      <h2 className="m-0 px-3 pb-1 pt-1 text-[0.68rem] font-bold leading-4 tracking-[0.1em] text-slate-500 max-[900px]:py-0">
        {title}
      </h2>

      <div className="grid gap-0.5">
        {children}
      </div>
    </section>
  );
}

export function Sidebar(_props: SidebarProps = {}) {
  const pathname = useLocalPathname();
  const activeSection = sectionForPathname(pathname);
  const inSettings = isSettingsPathname(pathname);

  return (
    <aside
      aria-label="PTAGV navigation"
      className={[
        "flex h-screen w-[216px] shrink-0 flex-col",
        "overflow-hidden border-r border-ptagv-border bg-white",
        "max-[900px]:h-auto",
        "max-[900px]:max-h-[45vh]",
        "max-[900px]:w-full",
        "max-[900px]:border-r-0",
        "max-[900px]:border-b",
      ].join(" ")}
    >
      <div
        role="group"
        aria-label="PTAGV"
        className="wt-sidebar-brand app-shell-header-height flex shrink-0 items-center gap-3 border-b border-white/10 bg-ptagv-navy px-4 py-2 text-white"
      >
        <div
          aria-hidden="true"
          className="wt-sidebar-mark grid h-9 w-9 shrink-0 place-items-center rounded-[3px] border border-white/25 bg-white/10 text-lg font-black tracking-tight"
        >
          P
        </div>

        <div className="min-w-0">
          <div className="text-[0.86rem] font-extrabold tracking-[0.11em]">
            PTAGV
          </div>

          <div className="mt-0.5 text-[0.61rem] font-semibold tracking-[0.16em] text-blue-100/80">
            WEB LOCAL
          </div>
        </div>
      </div>

      <nav
        aria-label="Web Local pages"
        className="wt-sidebar-nav flex min-h-0 flex-1 flex-col gap-2 overflow-y-auto overflow-x-hidden px-2.5 py-3 [scrollbar-width:thin] max-[900px]:gap-1 max-[900px]:py-2"
      >
        <NavigationSection title="ROBOT OPERATION">
          <NavigationLink
            path={LOCAL_APP_PATHS.CONTROL}
            label="CONTROL"
            active={activeSection === "CONTROL"}
            icon="control"
          />

          <NavigationLink
            path={LOCAL_APP_PATHS.MAPPING}
            label="MAPPING"
            active={activeSection === "MAPPING"}
            icon="mapping"
          />

          <NavigationLink
            path={LOCAL_APP_PATHS.LOCALIZATION}
            label="LOCALIZATION"
            active={activeSection === "LOCALIZATION"}
            icon="localization"
          />
        </NavigationSection>

        <NavigationSection title="MONITORING">
          <NavigationLink
            path={LOCAL_APP_PATHS.SYSTEM}
            label="SYSTEM OVERVIEW"
            active={activeSection === "SYSTEM"}
            icon="system"
          />

          <NavigationLink
            path={LOCAL_APP_PATHS.DIAGNOSIS}
            label="DIAGNOSIS"
            active={activeSection === "DIAGNOSIS"}
            icon="diagnosis"
          />
        </NavigationSection>

        <NavigationSection title="INTEGRATION">
          <NavigationLink
            path={LOCAL_APP_PATHS.VDA5050}
            label="VDA5050"
            active={activeSection === "VDA5050"}
            icon="vda"
          />
        </NavigationSection>

        <div className="mx-2.5 my-1 border-t border-ptagv-border-soft" />

        <section
          aria-label="Settings navigation"
          className="grid gap-0.5"
        >
          <NavigationLink
            path={LOCAL_APP_PATHS.SETTINGS}
            label="SETTINGS"
            active={inSettings}
            icon="settings"
          />
        </section>
      </nav>
    </aside>
  );
}
