import { useEffect, useRef } from "react";
import { handleLocalLinkClick, navigateLocalPath, useLocalPathname, LOCAL_APP_PATHS } from "../../routing/localRoutes";
import { FactorySettingsPage } from "./FactorySettingsPage";
import { NtpSettingsPage } from "./NtpSettingsPage";
import { OperationModeSettingsPage } from "./OperationModeSettingsPage";
import { VolumeSettingsPage } from "./VolumeSettingsPage";
import { WifiSettingsPage } from "./WifiSettingsPage";

const SETTINGS_SECTIONS = [
  { id: "wifi", label: "Wi-Fi", path: LOCAL_APP_PATHS.SETTINGS_WIFI },
  { id: "volume", label: "Volume", path: LOCAL_APP_PATHS.SETTINGS_VOLUME },
  { id: "ntp", label: "NTP", path: LOCAL_APP_PATHS.SETTINGS_NTP },
  { id: "operation-mode", label: "Operation Mode", path: LOCAL_APP_PATHS.SETTINGS_OPERATION_MODE },
  { id: "factory", label: "Factory Setting", path: LOCAL_APP_PATHS.SETTINGS_FACTORY },
] as const;

function sectionForPath(pathname: string) {
  const canonicalPath = pathname.replace(/\/+$/, "") || "/";
  return SETTINGS_SECTIONS.find(({ path }) => path === canonicalPath)?.id ?? "wifi";
}

export function SettingsWorkspacePage() {
  const pathname = useLocalPathname();
  const activeSection = sectionForPath(pathname);
  const initialRender = useRef(true);

  useEffect(() => {
    const initial = initialRender.current;
    initialRender.current = false;
    if (initial && pathname === LOCAL_APP_PATHS.SETTINGS) return;
    document.getElementById(`settings-${activeSection}`)?.scrollIntoView?.({ behavior: initial ? "auto" : "smooth", block: "start" });
  }, [activeSection, pathname]);

  return <main aria-label="Settings workspace" className="settings-workspace flex min-h-0 min-w-0 flex-1 flex-col overflow-auto bg-ptagv-page px-hmi-3 pb-hmi-4">
    <nav aria-label="Settings sections" className="settings-section-nav sticky top-0 z-20 -mx-hmi-3 mb-hmi-3 border-b border-ptagv-border bg-ptagv-page px-hmi-3 py-hmi-2 shadow-sm">
      <div className="flex min-w-0 flex-wrap items-center justify-between gap-2">
        <div className="flex min-w-0 flex-wrap gap-1.5" role="group" aria-label="Jump to settings section">
          {SETTINGS_SECTIONS.map(({ id, label, path }) => {
            const active = activeSection === id;
            return <button key={id} type="button" aria-label={label} aria-controls={`settings-${id}`}
              aria-current={active ? "location" : undefined} onClick={() => navigateLocalPath(path)}
              className={`min-h-9 border px-3 text-xs font-bold tracking-wide transition-colors focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-ptagv-blue ${active
                ? "border-ptagv-blue bg-ptagv-navy text-white"
                : "border-ptagv-border bg-white text-ptagv-text hover:border-ptagv-blue hover:bg-blue-50 hover:text-ptagv-navy"}`}>
              {label}
            </button>;
          })}
        </div>
        <a className="settings-map-link inline-flex min-h-9 shrink-0 items-center border border-ptagv-border bg-white px-3 text-xs font-bold tracking-wide text-ptagv-navy no-underline transition-colors hover:border-ptagv-blue hover:bg-blue-50 focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-ptagv-blue"
          href={LOCAL_APP_PATHS.SETTINGS_MAP} aria-label="Open map settings"
          onClick={(event) => handleLocalLinkClick(event, LOCAL_APP_PATHS.SETTINGS_MAP)}>
          MAP SETTINGS
        </a>
      </div>
    </nav>

    <div className="grid min-w-0 gap-hmi-4">
      <WifiSettingsPage embedded active={activeSection === "wifi"} />
      <VolumeSettingsPage embedded active={activeSection === "volume"} />
      <NtpSettingsPage embedded active={activeSection === "ntp"} />
      <OperationModeSettingsPage embedded active={activeSection === "operation-mode"} />
      <FactorySettingsPage embedded active={activeSection === "factory"} />
    </div>
  </main>;
}
