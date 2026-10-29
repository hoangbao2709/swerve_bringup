# PTAGV Web Local — Standalone Showcase

This repository contains the standalone Vite frontend for the PTAGV Web Local product showcase.

## Local production build

Run: npm ci
Then: npm run build:showcase
Preview: npm run preview:showcase

The Showcase profile uses deterministic browser-local sample state. It does not require Django, ROS 2, MQTT, a robot, or private network services. It does not send robot commands or change host system settings. Local previews and saved preferences are not physical robot operations.

## Tests

Run: npm run test

## Vercel

- Root Directory: .
- Framework: Vite
- Install Command: npm ci
- Build Command: npm run build:showcase
- Output Directory: dist
- Production Branch: web-local-showcase-vercel

vercel.json configures the build and single-page application rewrite. No public environment variables or credentials are required.
