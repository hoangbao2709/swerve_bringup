import { useSyncExternalStore, type MouseEvent as ReactMouseEvent } from "react";

export const LOCAL_APP_PATHS = {
  SYSTEM: "/system",
  CONTROL: "/control",
  MAPPING: "/mapping",
  LOCALIZATION: "/localization",
  VDA5050: "/vda5050",
  DIAGNOSIS: "/diagnosis",
  SETTINGS: "/settings",
  SETTINGS_WIFI: "/settings/wifi",
  SETTINGS_VOLUME: "/settings/volume",
  SETTINGS_NTP: "/settings/ntp",
  SETTINGS_OPERATION_MODE: "/settings/operation-mode",
  SETTINGS_FACTORY: "/settings/factory",
  SETTINGS_MAP: "/settings/map",
} as const;

export type LocalAppSection = keyof typeof LOCAL_APP_PATHS;

export type LocalRoute =
  | { kind: "page"; section: LocalAppSection; pathname: string }
  | { kind: "redirect"; to: string }
  | { kind: "not-found"; pathname: string };

const SECTION_BY_PATH = Object.fromEntries(
  Object.entries(LOCAL_APP_PATHS).map(([section, path]) => [path, section]),
) as Record<string, LocalAppSection>;

const LOCAL_NAVIGATION_EVENT = "ptagv:local-navigation";

function canonicalPathname(pathname: string): string {
  if (pathname === "/") return pathname;
  return pathname.replace(/\/+$/, "") || "/";
}

export function resolveLocalRoute(pathname: string): LocalRoute {
  const canonical = canonicalPathname(pathname);
  if (canonical === "/") return { kind: "redirect", to: LOCAL_APP_PATHS.CONTROL };
  const section = SECTION_BY_PATH[canonical];
  if (section) return { kind: "page", section, pathname: canonical };
  return { kind: "not-found", pathname: canonical };
}

export function sectionForPathname(pathname: string): LocalAppSection | null {
  const route = resolveLocalRoute(pathname);
  return route.kind === "page" ? route.section : null;
}

export function isSystemPathname(pathname: string): boolean {
  const canonical = canonicalPathname(pathname);
  return canonical === LOCAL_APP_PATHS.SYSTEM || canonical === LOCAL_APP_PATHS.DIAGNOSIS;
}

export function isSettingsPathname(pathname: string): boolean {
  const canonical = canonicalPathname(pathname);
  return canonical === LOCAL_APP_PATHS.SETTINGS || canonical.startsWith(`${LOCAL_APP_PATHS.SETTINGS}/`);
}

function currentPathname(): string {
  return window.location.pathname;
}

function subscribeToPathname(onChange: () => void): () => void {
  window.addEventListener("popstate", onChange);
  window.addEventListener(LOCAL_NAVIGATION_EVENT, onChange);
  return () => {
    window.removeEventListener("popstate", onChange);
    window.removeEventListener(LOCAL_NAVIGATION_EVENT, onChange);
  };
}

/** The browser URL is the only source of truth for the active local page. */
export function useLocalPathname(): string {
  return useSyncExternalStore(subscribeToPathname, currentPathname, () => "/control");
}

export function navigateLocalPath(path: string, replace = false): boolean {
  const target = new URL(path, window.location.href);
  if (target.origin !== window.location.origin) return false;

  const nextUrl = `${target.pathname}${target.search}${target.hash}`;
  const currentUrl = `${window.location.pathname}${window.location.search}${window.location.hash}`;
  if (nextUrl === currentUrl) return true;

  if (replace) window.history.replaceState(window.history.state, "", nextUrl);
  else window.history.pushState(window.history.state, "", nextUrl);
  window.dispatchEvent(new Event(LOCAL_NAVIGATION_EVENT));
  return true;
}

export function navigateToSection(section: LocalAppSection): boolean {
  return navigateLocalPath(LOCAL_APP_PATHS[section]);
}

export function handleLocalLinkClick(
  event: ReactMouseEvent<HTMLAnchorElement>,
  path: string,
): void {
  if (event.defaultPrevented || event.button !== 0
      || event.metaKey || event.ctrlKey || event.shiftKey || event.altKey
      || (event.currentTarget.target && event.currentTarget.target !== "_self")
      || event.currentTarget.hasAttribute("download")) return;

  event.preventDefault();
  navigateLocalPath(path);
}
