import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import tailwindcss from "@tailwindcss/vite";

export default defineConfig({
  plugins: [
    react(),
    tailwindcss(),
  ],
  test: {
    // The deterministic simulation/perception stress tests intentionally run
    // thousands of ticks and exceed Vitest's 5 s default on this machine.
    testTimeout: 30_000,
  },
  server: {
    // Allow other machines on the LAN to open the dev UI.
    host: "0.0.0.0",
    port: 5173,
  },
});
