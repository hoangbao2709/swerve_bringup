import { defineConfig, loadEnv } from "vite";
import react from "@vitejs/plugin-react";
import tailwindcss from "@tailwindcss/vite";

export default defineConfig(({ mode }) => {
  // Read the same host/port contract used by setup_full_stack/start_stack.
  // process.env wins so an operator can override a checked-in .env per run.
  const env = loadEnv(mode, process.cwd(), "");
  const rawPort = process.env.FRONTEND_PORT || env.FRONTEND_PORT || env.VITE_FRONTEND_PORT || "5173";
  const parsedPort = Number(rawPort);
  const port = Number.isInteger(parsedPort) && parsedPort > 0 && parsedPort < 65536 ? parsedPort : 5173;
  return {
    plugins: [
      react(),
      tailwindcss(),
    ],
    test: {
      // Control workflow tests render map and LiDAR views and need extra time.
      testTimeout: 30_000,
    },
    server: {
      // Allow other machines on the LAN to open the dev UI.
      host: process.env.FRONTEND_HOST || env.FRONTEND_HOST || "0.0.0.0",
      port,
    },
  };
});
