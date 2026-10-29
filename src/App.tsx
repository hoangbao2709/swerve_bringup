import { lazy, Suspense, useEffect, useState, type ComponentType, type ReactNode } from "react";
import { useBackendRealtime } from "./services/backendRuntime";
import { IS_SHOWCASE, IS_STATIC_UI } from "./config";
import { AppLayout } from "./app/AppLayout";
import { ControlPage } from "./pages/control/ControlPage";
import { MappingPage } from "./pages/mapping/MappingPage";
import { LocalizationPage } from "./pages/localization/LocalizationPage";
import { Vda5050Page } from "./pages/vda5050/Vda5050Page";
import { DiagnosisPage } from "./pages/diagnosis/DiagnosisPage";
import { SystemOverviewPage } from "./pages/system/SystemOverviewPage";
import { SettingsWorkspacePage } from "./pages/settings/SettingsWorkspacePage";
import { handleLocalLinkClick, navigateLocalPath, resolveLocalRoute, useLocalPathname, type LocalAppSection } from "./routing/localRoutes";

const StaticUiApp = IS_STATIC_UI || IS_SHOWCASE ? lazy(() => import("./mock/StaticUiApp")) : null;
const PAGE_COMPONENTS: Record<LocalAppSection, ComponentType> = {
  SYSTEM: SystemOverviewPage,
  CONTROL: ControlPage,
  MAPPING: MappingPage,
  LOCALIZATION: LocalizationPage,
  VDA5050: Vda5050Page,
  DIAGNOSIS: DiagnosisPage,
  SETTINGS: SettingsWorkspacePage,
  SETTINGS_WIFI: SettingsWorkspacePage,
  SETTINGS_VOLUME: SettingsWorkspacePage,
  SETTINGS_NTP: SettingsWorkspacePage,
  SETTINGS_OPERATION_MODE: SettingsWorkspacePage,
  SETTINGS_FACTORY: SettingsWorkspacePage,
  SETTINGS_MAP: MappingPage,
};

const PAGE_TITLES: Record<LocalAppSection, string> = {
  SYSTEM: "SYSTEM",
  CONTROL: "CONTROL",
  MAPPING: "MAPPING",
  LOCALIZATION: "LOCALIZATION",
  VDA5050: "VDA5050",
  DIAGNOSIS: "DIAGNOSIS",
  SETTINGS: "SETTINGS",
  SETTINGS_WIFI: "SETTINGS",
  SETTINGS_VOLUME: "SETTINGS",
  SETTINGS_NTP: "SETTINGS",
  SETTINGS_OPERATION_MODE: "SETTINGS",
  SETTINGS_FACTORY: "SETTINGS",
  SETTINGS_MAP: "SETTING · MAP",
};

/** The retained local HMI is designed for desktop, with its compact fallback gate. */
const GATE_W = 1024;
function NarrowScreenGate({ children }: { children: ReactNode }) {
  const [dismissed, setDismissed] = useState(false);
  const [narrow, setNarrow] = useState(() => window.innerWidth < GATE_W);
  useEffect(() => {
    const onResize = () => setNarrow(window.innerWidth < GATE_W);
    window.addEventListener("resize", onResize);
    return () => window.removeEventListener("resize", onResize);
  }, []);
  if (narrow && !dismissed) {
    return (
      <div className="grid min-h-screen place-items-center bg-ptagv-page p-hmi-4 text-ptagv-text">
        <section className="grid w-full max-w-xl gap-hmi-3 border border-ptagv-border bg-white p-hmi-5 shadow-sm">
          <span className="text-hmi-kicker font-extrabold tracking-widest text-ptagv-blue">PTAGV WEB LOCAL</span>
          <h2 className="m-0 text-xl font-bold">Designed for desktop</h2>
          <p className="m-0 text-sm leading-relaxed">Robot Control works best on screens ≥ 1280 px wide (it still runs, scaled down, from 1024 px). On a phone the controls and map would become difficult to read.</p>
          <p className="m-0 text-sm leading-relaxed">Open the control console on a laptop or desktop browser for the full experience.</p>
          <button className="min-h-10 w-fit border border-ptagv-blue bg-ptagv-blue px-hmi-3 text-hmi-kicker font-bold text-white hover:bg-blue-700 focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-ptagv-blue" onClick={() => setDismissed(true)}>Continue anyway</button>
        </section>
      </div>
    );
  }
  return <>{children}</>;
}

export default function App() {
  return <LocalAppRouter />;
}

function LocalAppRouter() {
  const pathname = useLocalPathname();
  const route = resolveLocalRoute(pathname);
  const redirectTo = route.kind === "redirect" ? route.to : null;

  useEffect(() => {
    if (redirectTo) navigateLocalPath(redirectTo, true);
  }, [redirectTo]);

  if (route.kind === "redirect") {
    return <main className="grid min-h-screen place-items-center bg-ptagv-page p-hmi-4 text-hmi-body font-semibold text-ptagv-muted" role="status">Opening CONTROL…</main>;
  }
  if (route.kind === "not-found") {
    return <main className="grid min-h-screen place-items-center bg-ptagv-page p-hmi-4 text-ptagv-text">
      <div className="grid max-w-lg gap-hmi-3 border border-ptagv-border bg-white p-hmi-5" role="alert">
        <span className="text-hmi-kicker font-bold tracking-widest text-ptagv-blue">PTAGV WEB LOCAL</span>
        <h1 className="m-0 text-xl font-bold">PAGE NOT FOUND</h1>
        <p className="m-0 text-sm leading-relaxed">This local application does not have a page at this address.</p>
        <a className="w-fit border border-ptagv-blue bg-ptagv-blue px-hmi-3 py-2 text-hmi-kicker font-bold text-white no-underline hover:bg-blue-700 focus-visible:outline-2 focus-visible:outline-ptagv-blue" href="/control" onClick={(event) => handleLocalLinkClick(event, "/control")}>GO TO CONTROL</a>
      </div>
    </main>;
  }

  const Page = PAGE_COMPONENTS[route.section];
  const sharedLayout = <AppLayout pageTitle={PAGE_TITLES[route.section]} isControlPage={route.section === "CONTROL"}>
    <Page />
  </AppLayout>;

  return <NarrowScreenGate>
    {IS_STATIC_UI || IS_SHOWCASE
      ? StaticUiApp
        ? <Suspense fallback={<main className="grid min-h-screen place-items-center bg-ptagv-page p-hmi-4 text-hmi-body font-semibold text-ptagv-muted" role="status">Opening PTAGV Web Local…</main>}>
          <StaticUiApp showcase={IS_SHOWCASE}>{sharedLayout}</StaticUiApp>
        </Suspense>
        : <div className="m-hmi-4 border border-red-300 bg-red-50 p-hmi-3 text-sm text-ptagv-error" role="alert">The standalone view could not be loaded.</div>
      : <BackendConnectedConsole>{sharedLayout}</BackendConnectedConsole>}
  </NarrowScreenGate>;
}

function BackendConnectedConsole({ children }: { children: ReactNode }) {
  useBackendRealtime();
  return <>{children}</>;
}
