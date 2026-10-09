# WareTwin ROS 2 Humble deployment

This package uses the existing project workspace and installs the Django ASGI application and React production build into the `waretwin_web` ament package. Runtime database, secrets, logs, layout edits, and default map artifacts live under `${XDG_STATE_HOME:-~/.local/state}/waretwin`. Launches serialize through one ownership lock, which is also honored by the source-tree start/stop scripts.

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

## Start Web Local

```bash
source /opt/ros/humble/setup.bash
source /path/to/swerve_bringup/install/setup.bash
ros2 launch waretwin_web web.launch.py
```

The defaults bind to `127.0.0.1:8000` for Django and `127.0.0.1:5173` for the React static server. Run `ros2 launch waretwin_web web.launch.py --show-args` to see all arguments. For example:

```bash
ros2 launch waretwin_web web.launch.py backend_host:=0.0.0.0 frontend_host:=0.0.0.0 backend_port:=8000 frontend_port:=5173
```

For repeatable configuration, copy the installed `runtime/config.env.example` to the writable runtime directory and edit it:

```bash
RUNTIME_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/waretwin"
mkdir -p "$RUNTIME_DIR"
install -m 600 "$(ros2 pkg prefix waretwin_web)/share/waretwin_web/runtime/config.env.example" "$RUNTIME_DIR/config.env"
${EDITOR:-vi} "$RUNTIME_DIR/config.env"
```

Then launch with `config_file:="$RUNTIME_DIR/config.env"`. Supported launch arguments include `backend_host`, `backend_port`, `frontend_host`, `frontend_port`, `ros_domain_id`, `config_file`, `runtime_dir`, and `backend_python`. Environment variables override config-file values; launch arguments override both. The backend and bridge share `ROS_DOMAIN_ID`, `ROS_WS_URL`, `WARETWIN_ROS_BRIDGE_TOKEN`, `WARETWIN_RUNTIME_MODE`, and `WARETWIN_ARTIFACT_ROOT` from that resolved runtime configuration. Secret values are stored once in `secrets.json` with mode 0600 when they are not supplied.

To preserve existing source-tree records and canonical maps when moving to the installed launch, point `WARETWIN_DATABASE_PATH` and `WARETWIN_ARTIFACT_ROOT` in the config file at those existing paths. The package never copies over or replaces a database. Otherwise it initializes a new persistent database in the runtime directory; publish or synchronize the intended canonical map before starting navigation.

The browser gets API and WebSocket addresses from `/runtime-config.js`, derived from its current host and the selected backend port. For HTTPS termination or a reverse proxy, set `VITE_API_BASE_URL` and `VITE_WS_BASE_URL` in runtime configuration and configure Django hosts and CORS for the public frontend origin. Binding to a LAN interface makes the UI accessible to LAN clients; network firewall policy remains a host deployment setting.

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

Logs and `web-status.json` are under the runtime directory. The status command checks the backend and frontend endpoints and reports `STARTING`, `READY`, `ERROR`, or `BRIDGE_DISCONNECTED` for full-stack launches. Readiness and health are not inferred from process existence alone.

## Automated deployment checks

After building and sourcing the workspace, run the installed lifecycle checks from outside the checkout:

```bash
source /opt/ros/humble/setup.bash
source /path/to/swerve_bringup/install/setup.bash
cd /tmp
WARETWIN_DEPLOYMENT_TEST=1 /usr/bin/python3 -m pytest /path/to/swerve_bringup/tests/test_web_deployment.py -q
```

The suite starts the installed launch, tests the REST and Channels endpoints, built assets, SPA refresh, persistence, restart and process cleanup, invalid settings, occupied ports, and fail-closed canonical map selection. The regular repository tests run with Humble sourced and the Django virtual environment's pytest. Simulation and physical robot acceptance require their corresponding Gazebo world and robot hardware/driver configuration.

When the source database has a valid published revision 23 fixture and Gazebo/Nav2 are installed, run the isolated simulation acceptance separately:

```bash
cd /tmp
WARETWIN_SIMULATION_TEST=1 /path/to/swerve_bringup/waretwin/backend/.venv/bin/python -m pytest /path/to/swerve_bringup/tests/test_full_stack_deployment.py -q -s
```

It snapshots the source SQLite database into a temporary directory, copies the selected published bundle there, and uses ROS domain 213. It waits for the existing unified-mode navigation readiness probe and then stops the launch with SIGINT.

## Troubleshooting

- Missing ROS package: source both `/opt/ros/humble/setup.bash` and the workspace `install/setup.bash`.
- Missing or incompatible Python dependencies: run `scripts/setup_full_stack.sh`; the installed runtime expects its isolated Python 3.10 venv under the runtime directory.
- Build fails before npm: install Node 22 and npm 10 or newer, then rerun `scripts/build_ros.sh`.
- Port conflict: select free `backend_port` or `frontend_port`. The launch reports a conflict and never terminates an unrelated listener.
- Navigation refuses startup: inspect `web-status.json`, `logs/ros.log`, and the readiness output; verify the selected canonical map bundle and Nav2 lifecycle state rather than enabling the development fallback implicitly.
- ROS bridge disconnected: verify matching `ROS_DOMAIN_ID` and `WARETWIN_ROS_BRIDGE_TOKEN`, the configured `ROS_WS_URL`, and bridge logs.
