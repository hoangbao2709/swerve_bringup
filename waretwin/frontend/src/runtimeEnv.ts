// Loaded before the application module by the installed static server.
declare global {
  interface Window { WARETWIN_CONFIG?: Record<string, string | undefined> }
}
export const runtimeEnv = {
  ...((import.meta as unknown as { env?: Record<string, string | undefined> }).env ?? {}),
  ...(typeof window === "undefined" ? {} : window.WARETWIN_CONFIG ?? {}),
};
