/** Runtime flags for the frontend. */
import { runtimeEnv as env } from "./runtimeEnv";
export type RuntimeMode = "GAZEBO_ROS" | "REAL_ROBOT";
export type UiMode = "BACKEND_CONNECTED" | "STATIC_UI" | "SHOWCASE";

export function resolveUiMode(value: string | undefined, isDevelopment: boolean, isShowcaseBuild = false): UiMode {
  const requested = value?.trim() || "BACKEND_CONNECTED";
  if (requested === "STATIC_UI") {
    if (!isDevelopment) throw new Error("STATIC_UI is available only in the Vite development server.");
    return requested;
  }
  if (requested === "SHOWCASE") {
    if (!isShowcaseBuild) throw new Error("SHOWCASE is available only in the dedicated Vite showcase build.");
    return requested;
  }
  if (requested === "BACKEND_CONNECTED") return requested;
  throw new Error(`Unsupported VITE_UI_MODE: ${requested}`);
}

export const UI_MODE = resolveUiMode(env.VITE_UI_MODE, import.meta.env.DEV, import.meta.env.MODE === "showcase");
export const IS_STATIC_UI = UI_MODE === "STATIC_UI";
export const IS_SHOWCASE = UI_MODE === "SHOWCASE";
export const IS_STANDALONE_UI = IS_STATIC_UI || IS_SHOWCASE;
export const RUNTIME_MODE: RuntimeMode = env.VITE_RUNTIME_MODE === "REAL_ROBOT" ? "REAL_ROBOT" : "GAZEBO_ROS";

// The browser is never a robot simulator. Retained pages require the backend
// runtime even if an old deployment still sets VITE_DEMO_MODE.
export const DEMO_MODE = false;

/** In backend mode the Django server is authoritative; never fall back to local simulation. */
export const BACKEND_REQUIRED = !DEMO_MODE;
