/** Runtime flags for the frontend. */
const env = (import.meta as unknown as { env?: Record<string, string | undefined> }).env ?? {};
export type RuntimeMode = "LOCAL_SIM" | "GAZEBO_ROS" | "REAL_ROBOT";
export const RUNTIME_MODE: RuntimeMode = env.VITE_RUNTIME_MODE === "GAZEBO_ROS" || env.VITE_RUNTIME_MODE === "REAL_ROBOT" ? env.VITE_RUNTIME_MODE : "LOCAL_SIM";

/**
 * Frontend-only demo mode is the default so WareTwin can be demonstrated
 * without a running FastAPI backend and without a login screen.
 * Set VITE_DEMO_MODE=false for the authenticated backend deployment.
 */
export const DEMO_MODE = env.VITE_DEMO_MODE !== "false" && RUNTIME_MODE === "LOCAL_SIM";

/** In backend mode the Django server is authoritative; never fall back to local simulation. */
export const BACKEND_REQUIRED = !DEMO_MODE;

export const DEMO_USER = {
  id: 0,
  username: "demo",
  email: "demo@localhost",
  role: "admin" as const,
  is_active: true,
};
