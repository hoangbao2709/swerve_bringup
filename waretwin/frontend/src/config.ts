/** Runtime flags for the frontend. */
import { runtimeEnv as env } from "./runtimeEnv";
export type RuntimeMode = "GAZEBO_ROS" | "REAL_ROBOT";
export const RUNTIME_MODE: RuntimeMode = env.VITE_RUNTIME_MODE === "REAL_ROBOT" ? "REAL_ROBOT" : "GAZEBO_ROS";

// The browser is never a robot simulator. Retained pages require the backend
// runtime even if an old deployment still sets VITE_DEMO_MODE.
export const DEMO_MODE = false;

/** In backend mode the Django server is authoritative; never fall back to local simulation. */
export const BACKEND_REQUIRED = !DEMO_MODE;
