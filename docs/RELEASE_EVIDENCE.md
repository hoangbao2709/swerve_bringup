# Release hardening evidence (NO-GO)

Evidence captured 2026-10-09 11:23 UTC on the uncommitted working tree. This is a release audit record, not a certification. Physical hardware was not connected or operated.

## Build identity

| Field | Value |
|---|---|
| Branch | `robot-real-sim-v5-demo-visual` |
| Base commit | `73f301eadc0f303c81442a8f4494c00f639cd66f` |
| Working tree | Modified and uncommitted; no commit or push performed |
| OS | Ubuntu 22.04.5 LTS |
| ROS | Humble |
| Python | 3.10.12 |
| Node / npm | 22.23.2 / 10.9.8 |
| Python lock SHA-256 | `3498e8eb5e7f31aa36d330fbbcfce76dfb7303e5fb45ff5e1aca34f04a101aef` |
| npm lock SHA-256 | `8bb61e7867505203a1f110aa615decb0acecd481443e855fb75c1b571e96a079` |

The copied, non-symlink install was built under `/tmp/waretwin-release-final-20261009`. No build was performed into the source tree's `install/` directory.

## Test evidence

| Check | Result | Evidence |
|---|---|---|
| `./scripts/build_ros.sh --build-base /tmp/waretwin-release-final-20261009/build --install-base /tmp/waretwin-release-final-20261009/install --log-base /tmp/waretwin-release-final-20261009/log` | PASS: `swerve_bringup`, `swerve_bridge`, `waretwin_web` built and installed | Colcon summary: 3 packages finished. One existing Gazebo/CMake developer warning was emitted. |
| `colcon list --base-paths /home/yahboom/swerve_bringup /home/yahboom/swerve_bringup/swerve_bridge /home/yahboom/swerve_bringup/waretwin` | PASS: all three packages discovered | `ros.ament_cmake` / `ros.ament_python` package listing |
| `ros2 pkg prefix {swerve_bringup,swerve_bridge,waretwin_web}` from `/tmp` after sourcing copied install | PASS | All prefixes resolve under `/tmp/waretwin-release-final-20261009/install` |
| Both installed launch files with `--show-args` from `/tmp` | PASS | Web and full-stack arguments resolved by ROS launch |
| `scripts/check_waretwin_release_artifacts.py <installed waretwin_web share>` | PASS | No source DB, secrets, node_modules, venv, or local-map artifacts in installed package |
| Django `manage.py check`; `makemigrations --check --dry-run` | PASS | No system-check issues; no model/migration drift |
| Django `manage.py test twin.tests --verbosity 1` | PASS: 196 tests | Final source; isolated DB at `/tmp/waretwin-release-final-20261009/backend-test.sqlite3` |
| Frontend `npm run build` | PASS | `tsc --noEmit` and Vite production build completed |
| Frontend `npm test` | PASS: 161 tests in 26 files | Final frontend source |
| `npm audit` | PASS: 0 vulnerabilities | npm audit output |
| `pip-audit -r waretwin/runtime/requirements.lock` | PASS: no known vulnerabilities | pip-audit output using temporary audit venv |
| `pytest tests/test_setup_scripts.py scripts/tests waretwin/runtime/tests` | PASS: 22 tests | Runtime/setup regression suite |
| `pytest tests/test_manual_transport.py tests/test_manual_refresh_process.py` | PASS: 9 tests | Manual command transport/refresh suite |
| `pytest swerve_bridge/test/test_navigation_target_handoff.py` | PASS: 23 tests | Bridge and navigation handoff suite |
| `pytest tests/test_swerve_kinematics.py scripts/test_command_ownership.py swerve_bridge/test/test_navigation_target_handoff.py` | PASS: 32 tests | Kinematics, command ownership, and bridge coverage |
| Installed Web-only lifecycle suite from `/tmp` | PASS: 11 tests | Non-symlink package; API/Channels, assets, SPA, persistence, restart, config/port failures, fail-closed map selection |
| `colcon test` for all three packages | NOT TESTED by Colcon: 0 tests registered | Use the direct pytest/Django commands above; the green Colcon package summary alone is not test evidence |
| Installed Gazebo/Nav2 mission, final source build | FAIL: mission goal exceeded the 5 cm translation limit | `/tmp/pytest-of-yahboom/pytest-37/test_installed_full_stack_reac0/runtime/end-to-end.json`; settled error `0.0524255097 m`, yaw error `0.0062331274 rad`, action status `SUCCEEDED`. All preceding mission checks passed. |
| Installed Gazebo/Nav2 mission, immediately preceding shutdown-only adjustment | PASS: 45/45 stages | `/tmp/pytest-of-yahboom/pytest-35/test_installed_full_stack_reac0/runtime/end-to-end.json`; settled error `0.0483479853 m`, yaw error `0.0445555335 rad`. This is not counted as final-source simulation acceptance because the controller's final-zero shutdown race was subsequently fixed. |
| 24–72 hour soak | NOT TESTED | Harness exists at `scripts/waretwin_soak_test.py`; run on a representative deployment host |
| Protected-LAN gateway acceptance | NOT TESTED | No live TLS identity gateway was configured |
| Physical robot acceptance | NOT TESTED | No physical hardware was connected |

The latest simulation used an isolated ROS domain (`213`), a fresh temporary Django database, and a copied install launched from `/tmp`. Its Gazebo real-time factor was approximately `0.34`; Nav2 reported `SUCCEEDED`, but the final measured pose was outside the test's pre-existing 5 cm acceptance bound. The bound was not relaxed. A previous run on the immediately preceding build passed narrowly, demonstrating insufficient repeatability for a release gate.

The final-source shutdown log showed the owned Python controller, arbiter, odometry, LiDAR preprocessor, bridge, Gazebo and Nav2 processes exiting, and a process scan found no owned processes left behind. The external `ekf_node` was reported by ROS launch as signal-terminated (`exit code -2`) during Ctrl+C; no orphan remained. The failed mission assertion is the pytest failure, not a hidden shutdown pass.

## Protected data and limits

At initial inspection, the tracked source database and robot-local map registry already had user changes, and robot-local `.pgm` / `.yaml` and SLAM session files were already untracked. They were preserved. Integration tests used temporary data roots and did not copy or overwrite those files. The tracked development database remains in the repository and its contents have not been audited for operational records or secrets; it must be reviewed and separated before distributing a release artifact.

The frontend still has exactly five sidebar entries—`CONTROL`, `MAPPING`, `LOCALIZATION`, `VDA5050`, `DIAGNOSIS`—with map management inside `MAPPING`. No separate `MAPS` sidebar entry was introduced.

The browser-facing authorization boundary does not secure direct ROS 2 DDS publishers. SROS2 enclaves/topic ACLs, an independently validated Protected-LAN gateway, hardware E-stop feedback, a physical safety assessment, and long-duration soak evidence remain outstanding. See [ROS_DEPLOYMENT.md](../waretwin/ROS_DEPLOYMENT.md) for setup, launch, backup/restore, rollback, and troubleshooting procedures.
