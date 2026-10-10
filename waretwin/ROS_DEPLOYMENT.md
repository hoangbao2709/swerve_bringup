# WareTwin ROS 2 Humble deployment

This package uses the existing project workspace and installs the Django ASGI application and React production build into the `waretwin_web` ament package. Runtime database, secrets, logs, layout edits, and default map artifacts live under `${XDG_STATE_HOME:-~/.local/state}/waretwin`. Launches serialize through one ownership lock, which is also honored by the source-tree start/stop scripts. The installed tree is read-only application code; the runtime database and map artifacts are separate writable data.

## Install and build

On Ubuntu 22.04, install ROS 2 Humble and the project prerequisites. Node 22 and npm 10 or newer are needed only while building the frontend; runtime serving uses Python. The existing setup script validates and installs the project ROS stack dependencies and Python/web dependencies:

```bash
nvm install 22
nvm use 22
source /opt/ros/humble/setup.bash
cd /path/to/swerve_bringup
./scripts/setup_full_stack.sh
```

The setup script builds all three nested packages. To build directly:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/swerve_bringup
./scripts/build_ros.sh
source install/setup.bash
ros2 pkg list | rg '^(swerve_bringup|swerve_bridge|waretwin_web)$'
ros2 pkg prefix waretwin_web
```

`build_ros.sh` discovers the root and both nested packages explicitly. Its default build is a regular install, so installed files are copied rather than symlinked. Rebuilding `waretwin_web` requires npm and network access to the package registry; the resulting ROS install does not include `node_modules` and runtime serving does not require Node.

## Prepare and start Web Local

```bash
source /opt/ros/humble/setup.bash
source /path/to/swerve_bringup/install/setup.bash
SHARE="$(ros2 pkg prefix waretwin_web)/share/waretwin_web"
RUNTIME_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/waretwin"
mkdir -p "$RUNTIME_DIR"
# One-time dependency provisioning with the target Python 3.10. Normal launch
# never installs packages, invokes npm, or rebuilds the frontend.
/usr/bin/python3 "$SHARE/runtime/service.py" --share "$SHARE" \
  --runtime-dir "$RUNTIME_DIR" --setup
ros2 launch waretwin_web web.launch.py
```

The defaults bind to `127.0.0.1:8000` for Django and `127.0.0.1:5173` for the React static server. Run `ros2 launch waretwin_web web.launch.py --show-args` to see all arguments. For repeatable operation, pass the same writable runtime directory used during setup:

```bash
ros2 launch waretwin_web web.launch.py runtime_dir:="$RUNTIME_DIR"
```

For repeatable configuration, copy the installed `runtime/config.env.example` to the writable runtime directory and edit it:

```bash
RUNTIME_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/waretwin"
mkdir -p "$RUNTIME_DIR"
install -m 600 "$(ros2 pkg prefix waretwin_web)/share/waretwin_web/runtime/config.env.example" "$RUNTIME_DIR/config.env"
${EDITOR:-vi} "$RUNTIME_DIR/config.env"
```

Then launch with `config_file:="$RUNTIME_DIR/config.env"`. Supported launch arguments include `backend_host`, `backend_port`, `frontend_host`, `frontend_port`, `ros_domain_id`, `config_file`, `runtime_dir`, and `backend_python`. Environment variables override config-file values; launch arguments override both. The backend and bridge share `ROS_DOMAIN_ID`, `ROS_WS_URL`, `WARETWIN_ROS_BRIDGE_TOKEN`, `WARETWIN_RUNTIME_MODE`, and `WARETWIN_ARTIFACT_ROOT` from that resolved runtime configuration. Secret values are stored once in `secrets.json` with mode 0600 when they are not supplied.

### Operator access modes

`LOCAL_LOOPBACK` is the default and accepts control requests only from loopback. Bind both services to loopback. It is intended for a trusted single-machine HMI; it does not protect against another local process or OS user. Do not bind it to a LAN address.

`PROTECTED_LAN` is not a standalone login feature. It requires a same-host TLS reverse proxy/authentication gateway that authenticates the operator, strips all client-supplied `X-WareTwin-*` headers, then signs short-lived assertions for each REST request and WebSocket handshake. The gateway must enforce secure cookies/session expiry and close existing WebSockets on logout/expiry. Configure exact HTTPS `WARETWIN_PUBLIC_ORIGINS`, a random `WARETWIN_OPERATOR_GATEWAY_SECRET` (at least 32 bytes), and the gateway's exact loopback source in `WARETWIN_TRUSTED_PROXY_CIDRS`; both Django and the static frontend are required to bind only to loopback so clients cannot bypass the proxy. Route same-origin `/api/` and `/ws/` to Django, and other paths to the static frontend. Store the configuration in an owner-only file (`chmod 600`). The gateway secret never belongs in `runtime-config.js` or browser code. Enabled VDA5050 broker connections in this mode must use TLS. Do not enable LAN operation until this gateway path has been deployed and its authentication/authorization boundary independently tested.

The browser-to-Django authorization boundary does not secure ROS 2 DDS topics. ROS domain participants can currently publish directly to velocity/control topics unless the deployment separately restricts DDS participants. Until ROS 2 Security/SROS2 enclaves and topic ACLs are configured and acceptance-tested, keep the ROS domain on a physically/logically trusted network and do not treat HMI authorization as protection against a hostile ROS participant. The software E-Stop is not a certified safety function; production requires an independent hardware safety relay/PLC/controller assessment.

To preserve existing source-tree records and canonical maps when moving to the installed launch, point `WARETWIN_DATABASE_PATH` and `WARETWIN_ARTIFACT_ROOT` in the config file at those existing paths. The package never copies over or replaces a database. Otherwise it initializes a new persistent database in the runtime directory; publish or synchronize the intended canonical map before starting navigation.

The browser gets API and WebSocket addresses from `/runtime-config.js`; the protected mode uses same-origin API/WebSocket paths so the gateway session is carried without embedding a privileged bearer token. Configure exact Django hosts/origins and gateway routing. TLS termination belongs at the trusted gateway; neither the built-in static server nor the Uvicorn backend should be exposed directly to an untrusted network.

## Start the complete robot stack

Publish and verify a canonical map bundle before navigation. The full launch defaults to Gazebo simulation, `unified` mode, and the existing robot readiness gate. It uses the same revision/hash validation, map synchronization request files, lifecycle owner, and navigation readiness probe as the existing stack supervisor.

```bash
source /opt/ros/humble/setup.bash
source /path/to/swerve_bringup/install/setup.bash
ros2 launch swerve_bringup full_stack.launch.py
```

Supported modes are `mode:=unified`, `mode:=mapping`, and `mode:=navigation`. Use `use_sim:=false` plus a valid `real_sensor_launch:=...` for the real robot driver path; this does not start Gazebo. The explicit development map fallback remains opt-in with `allow_dev_world:=true`. GUI and RViz default to off; enable with `gui:=true` or `start_rviz:=true`.

Stop a ROS launch with Ctrl+C. Installed launches can also be inspected and stopped with `scripts/status_stack.sh` and `scripts/stop_stack.sh`; they resolve the active runtime directory through the shared ownership registry. These scripts continue to support legacy start modes and reject attempts to start a duplicate stack.

```bash
./scripts/status_stack.sh
./scripts/stop_stack.sh
```

Logs and `web-status.json` are under the runtime directory. The status command checks the backend and frontend endpoints and reports `STARTING`, `READY`, `DEGRADED`, `ERROR`, `STOPPED`, or `BRIDGE_DISCONNECTED`. Readiness and health are not inferred from process existence alone; an essential-child failure leaves an `ERROR` state and diagnostic reason after owned-process cleanup.

## Automated deployment checks

After building and sourcing the workspace, run the installed lifecycle checks from outside the checkout:

```bash
source /opt/ros/humble/setup.bash
source /path/to/swerve_bringup/install/setup.bash
cd /tmp
WARETWIN_DEPLOYMENT_TEST=1 /usr/bin/python3 -m pytest /path/to/swerve_bringup/tests/test_web_deployment.py -q
```

The suite starts the installed launch, tests the REST and Channels endpoints, built assets, SPA refresh, persistence, restart and process cleanup, invalid settings, occupied ports, and fail-closed canonical map selection. The regular repository tests run with Humble sourced and the Django virtual environment's pytest. Simulation and physical robot acceptance require their corresponding Gazebo world and robot hardware/driver configuration. Test the final copied install from `/tmp`; a source or symlink install alone is not install evidence.

When the source database has a valid published revision 23 fixture and Gazebo/Nav2 are installed, run the isolated simulation acceptance separately:

```bash
cd /tmp
WARETWIN_SIMULATION_TEST=1 /path/to/swerve_bringup/waretwin/backend/.venv/bin/python -m pytest /path/to/swerve_bringup/tests/test_full_stack_deployment.py -q -s
```

It creates a fresh temporary SQLite database, applies migrations, seeds only the versioned `WH-TEST-01` canonical-map fixture, and publishes that fixture beneath the temporary artifact root. It does not open, copy, or modify the source checkout database or robot-local maps. It uses ROS domain 213, waits for the existing unified-mode navigation readiness probe, executes the acceptance sequence, and stops the owned launch with SIGINT.

### Latest final-install result (2026-10-09)

The final copied, non-symlink `install5` build was launched from `/tmp` and completed package discovery, launch argument resolution, Web Local deployment checks (11/11), Django tests (198), frontend tests/build (161 tests), and focused ROS/runtime tests (109). The overall Gazebo acceptance is **FAIL**, not PASS. Both navigation actions returned `SUCCEEDED`, but the direct mission's map-frame and projected Gazebo pose disagreed beyond the release criteria; the Web mission's terminal map transform was stale. Observed Gazebo RTF was approximately `0.36`. The existing `0.05 m` / `0.05 rad` limits were not relaxed, and five repeatable accepted missions were not obtained.

The same real installed-stack run exercised Mapping → Pause → Save and the supervised map load with actual SLAM Toolbox, Nav2 map server, bridge, and supervisor (no ROS/Gazebo mocks). Save produced and validated PGM/YAML plus SLAM `.data`/`.posegraph` files for an isolated temporary robot map. The loaded `/navigation_map` occupancy grid matched the saved image exactly (699 × 598, 0.05 m resolution; 418,002 cells) and was reported as local-only, not canonical. The workflow then **failed** to establish fresh, consistent post-load map TF for the initial-pose confirmation and executed zero navigation goals after loading. Therefore Save → Load → Initial Pose → Navigation remains unaccepted. The machine-readable sequence report and test logs are linked in [`../docs/RELEASE_EVIDENCE.md`](../docs/RELEASE_EVIDENCE.md).

The bridge now rejects stale/future dynamic localization TF, requires the initial-pose confirmation transform to be newer than the request, and clears its TF buffer and invalidates in-flight pose/map operations when Gazebo time rewinds. These guards and clock-reset tests passed; they did not resolve the final end-to-end TF/pose acceptance failure. Do not bypass these guards or report a map loaded solely because an HTTP request or supervisor transition succeeded.

Hosted GitHub Actions was not rerun on this uncommitted tree. Local `actionlint` passed, but the release gate needs a successful hosted run after the changes are pushed by an authorized maintainer. Protected-LAN gateway, DDS/SROS2 access control, live MQTT broker, 24–72-hour soak, target-computer performance, and physical robot acceptance are also unverified. This evidence supports only controlled development/demo use; it is not production LAN or hardware approval.

## Backup, restore, rollback, and soak evidence

Backups must go to an owner-only directory with enough free space. The runtime database command takes the shared instance lock, creates a consistent SQLite snapshot, validates it, and refuses to overwrite a prior backup:

```bash
SHARE="$(ros2 pkg prefix waretwin_web)/share/waretwin_web"
RUNTIME_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/waretwin"
PYTHON="$RUNTIME_DIR/venv/bin/python"
"$PYTHON" "$SHARE/runtime/database.py" backup \
  --database "$RUNTIME_DIR/db.sqlite3" --backup-dir "$RUNTIME_DIR/backups"
```

Restore only while the stack is stopped. The command validates the backup and creates a pre-restore snapshot of the current database before atomic replacement:

```bash
"$PYTHON" "$SHARE/runtime/database.py" restore \
  --database "$RUNTIME_DIR/db.sqlite3" \
  --backup "$RUNTIME_DIR/backups/waretwin-db-<timestamp>.sqlite3" \
  --backup-dir "$RUNTIME_DIR/backups"
```

Back up `WARETWIN_ARTIFACT_ROOT` separately with its canonical manifest and map bundle; do not hand-copy selected `.pgm`/`.yaml` files or change revisions. After restoring maps, verify manifest/hash/revision and robot registration before enabling navigation. Migration startup makes a private database backup and attempts rollback on migration failure. For release rollback, stop the runtime, restore the known-good database snapshot if the new schema is incompatible, reinstall the prior package build, restore its compatible configuration, then validate the canonical map bundle before allowing navigation. Do not run an older binary against a newer schema without a tested compatibility procedure.

Run a short soak first; 24-hour and 72-hour runs are supported by the read-only sampler but have not been run as part of this release audit. The output path must not already exist. For local loopback mode:

```bash
mkdir -m 700 -p "$RUNTIME_DIR/evidence"
"$PYTHON" /path/to/swerve_bringup/scripts/waretwin_soak_test.py \
  --health-url http://127.0.0.1:8000/api/health/ \
  --frontend-url http://127.0.0.1:5173/ \
  --origin http://127.0.0.1:5173 \
  --duration 86400 --interval 30 --require-websocket \
  --output "$RUNTIME_DIR/evidence/soak-$(date -u +%Y%m%dT%H%M%SZ).csv"
```

The CSV records HTTP/SPA/WebSocket latency and readiness, host CPU/memory, and the runtime's owned process-tree CPU/RSS when `/proc` is available. It does not drive the robot. LiDAR/render/controller performance and manual stop latency require separate browser/ROS/hardware instrumentation and are not inferred from this sampler. Preserve the CSV with `logs/` and the matching runtime status evidence.

## Troubleshooting

- Missing ROS package: source both `/opt/ros/humble/setup.bash` and the workspace `install/setup.bash`.
- Missing or incompatible Python dependencies: run `scripts/setup_full_stack.sh`; the installed runtime expects its isolated Python 3.10 venv under the runtime directory.
- Build fails before npm: install Node 22 and npm 10 or newer, then rerun `scripts/build_ros.sh`.
- Port conflict: select free `backend_port` or `frontend_port`. The launch reports a conflict and never terminates an unrelated listener.
- Navigation refuses startup: inspect `web-status.json`, `logs/ros.log`, and the readiness output; verify the selected canonical map bundle and Nav2 lifecycle state rather than enabling the development fallback implicitly.
- ROS bridge disconnected: verify matching `ROS_DOMAIN_ID` and `WARETWIN_ROS_BRIDGE_TOKEN`, the configured `ROS_WS_URL`, and bridge logs.
- Protected LAN request rejected: verify gateway TLS/session, same-origin routing, proxy source CIDR, exact HTTPS origin, signed per-request assertion and permission/robot scope. Never fix this by widening the trusted CIDR or exposing Django.
- Manual motion did not stop: remove the operator's hold, check lease/arbiter diagnostics and verify hardware stop behavior at reduced speed in the approved safety test area. Do not infer a measured physical stop distance/latency from software tests.
