import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import tailwindcss from "@tailwindcss/vite";

export default defineConfig({
  plugins: [
    react(),
    tailwindcss(),
  ],
  server: {
    // Allow other machines on the LAN to open the dev UI.
    host: "0.0.0.0",
    port: 5173,
  },
});
