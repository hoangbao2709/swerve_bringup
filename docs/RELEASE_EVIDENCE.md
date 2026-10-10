# Release hardening evidence — NO-GO

Evidence captured on 2026-10-09 from the final uncommitted working tree on Ubuntu 22.04.5 LTS / ROS 2 Humble. This is a software test record, not a certification. No physical robot was connected or operated. Machine-readable details are in [`release-evidence.json`](release-evidence.json); temporary logs and reports remain under `/tmp/waretwin-final-acceptance.9M9UZU` and `/tmp/pytest-of-yahboom/pytest-70` on the test host.

## Build identity

| Field | Value |
|---|---|
| Branch | `robot-real-sim-v5-demo-visual` |
| HEAD / starting commit | `d7491ea55df625b23cf74bdecac25184f6725e78` |
| Working tree | Modified and uncommitted; no commit or push |
| Build | `install5-nonsymlink-20261009` |
| OS / ROS | Ubuntu 22.04.5 LTS / Humble |
| Python | 3.10.12 |
| Node / npm | 22.23.2 / 10.9.8 |
| Python lock SHA-256 | `3498e8eb5e7f31aa36d330fbbcfce76dfb7303e5fb45ff5e1aca34f04a101aef` |
| npm lock SHA-256 | `8bb61e7867505203a1f110aa615decb0acecd481443e855fb75c1b571e96a079` |
| Copied install | `/tmp/waretwin-final-acceptance.9M9UZU/install5` |

## Final-source test results

| Test | Result | Evidence |
|---|---|---|
| `./scripts/build_ros.sh --build-base /tmp/waretwin-final-acceptance.9M9UZU/build5 --install-base /tmp/waretwin-final-acceptance.9M9UZU/install5 --log-base /tmp/waretwin-final-acceptance.9M9UZU/log5` | PASS — all 3 packages built and copied into a non-symlink install | Colcon summary: `swerve_bringup`, `swerve_bridge`, `waretwin_web` finished; build log under `log5/build_2026-10-09_22-49-22/` |
| `colcon list --base-paths . swerve_bridge waretwin`; `ros2 pkg prefix` for all 3 packages; both launch files with `--show-args` from `/tmp` | PASS | `/tmp/waretwin-final-acceptance.9M9UZU/simulation-evidence/final-{web,full}-launch-args.txt`; package prefixes resolved under `install5` |
| Installed package artifact scan | PASS | `scripts/check_waretwin_release_artifacts.py` found no bundled runtime database, secrets, virtualenv, `node_modules`, or robot-local map artifacts |
| Django `check`; `makemigrations --check --dry-run`; `manage.py test twin.tests --verbosity 1` | PASS — 198 tests | `/tmp/waretwin-final-acceptance.9M9UZU/simulation-evidence/django-final.log`; isolated test DB |
| Frontend `npm test && npm run build && npm audit --audit-level=low` | PASS — 161 tests / 26 files; typecheck and Vite build passed; audit reported 0 vulnerabilities | `/tmp/waretwin-final-acceptance.9M9UZU/simulation-evidence/frontend-final.log` (captured in test run; no generated source artifacts tracked) |
| Focused ROS, map, safety, readiness, deployment-contract and runtime tests | PASS — 109 tests | `simulation-evidence/ros-focused-final.xml` and `.log` |
| Manual transport/refresh, 8-direction swerve kinematics and command ownership | PASS — 18 tests with the copied `install5` overlay sourced | `/tmp/waretwin-final-acceptance.9M9UZU/simulation-evidence/manual-kinematics-final.log`; a first invocation without the generated action overlay had 4 import failures, then the correctly sourced rerun passed |
| Installed Web deployment from `/tmp`, non-symlink install | PASS — 11 tests, including HTTP/WS/assets/SPA/persistence/restart/child failure/ports/duplicate launch/fail-closed map | `simulation-evidence/web-deployment-final-install5.xml` and `.log` |
| `actionlint` 1.7.7 on `.github/workflows/quality.yml` | PASS locally | Workflow syntax/context lint passed; this does not establish hosted CI success |
| Hosted GitHub Actions quality run | NOT VERIFIED | The reported run [37925505444](https://github.com/hoangbao2709/swerve_bringup/actions/runs/37925505444) completed failed with zero jobs / zero-duration public metadata. The prior workflow used `runner.temp` at job-level; that context is unavailable there. The workflow now uses `github.workspace`, but no hosted run of this uncommitted tree was possible without publishing it. The remote failure's precise diagnostic is unavailable, so the expression defect is a confirmed workflow defect, not proven as the sole cause of that historical run. |
| Final installed Gazebo/Nav2 acceptance | FAIL | `simulation-evidence/full-stack-final-install5.xml`, `/tmp/pytest-of-yahboom/pytest-70/test_installed_full_stack_reac0/runtime/end-to-end.json`, and `.log`; one complete installed full-stack test, 361.863 s, exited 1 |
| Save → Load → initial-pose → navigation acceptance | FAIL / incomplete | `/tmp/pytest-of-yahboom/pytest-70/test_installed_full_stack_reac0/runtime/save-load-navigation.json`; save and map-server load/content checks passed, but a fresh post-load localization confirmation was not established; zero navigation runs followed |
| Protected-LAN gateway / TLS identity boundary | NOT TESTED | No deployed gateway or end-to-end live identity test |
| DDS/SROS2 authorization | NOT TESTED | No DDS ACL artifacts or negative participant-publish acceptance; `ROS_DOMAIN_ID` is not an access-control boundary |
| Live VDA5050 broker | NOT TESTED | Django/deterministic behavior is covered, no live broker used |
| 24–72 hour soak / target computer / physical robot | NOT TESTED | No long-duration or target hardware evidence; no physical robot was operated |

The final installed simulation produced two `SUCCEEDED` Nav2 actions but did not pass the release acceptance. In the direct mission, map-frame error after settling was `0.0493079451 m` and `0.0369712355 rad`, while projected Gazebo-ground-truth error was `0.0834782799 m` and `0.0174134580 rad`; map TF and projected ground truth disagreed by `0.0376594841 m` and `0.0543846935 rad`. In the Web mission, ground-truth error was `0.0131838481 m` / `0.0032819418 rad`, but the terminal map TF was stale (`1.405 s` in simulation time) and the acceptance rejected it. Gazebo RTF was about `0.366`. These measurements do not isolate a single underlying cause; the final build did not achieve five repeatable accepted missions. The 5 cm / 0.05 rad limits were not relaxed.

The real Gazebo mapping workflow saved these isolated test artifacts (not promoted to canonical): robot `R01`, map ID `ca5d93aa8a70452c9aa09a670493472a`, revision `ca5d93aa8a70`; source SLAM session `8c1edd105efc`, source map `SLAM-8c1edd105efc@session-8c1edd105efc`. The map was `699 × 598` at `0.05 m` resolution with 136,308 known cells. PGM, YAML, serialized `.data`, and `.posegraph` files were present and hashed in the JSON report. Nav2's fresh `/navigation_map` matched the saved occupancy grid exactly (418,002 cells). However, the initial-pose check encountered stale/inconsistent TF evidence after the runtime transition, so the sequence stopped before navigation. This is a partial Save/Load result, not end-to-end acceptance.

## Confirmed corrections and residual risks

- **Workflow:** replaced job-level `runner.temp` references with checkout-scoped `.ci-runtime` paths and documented that CI uploads no runtime artifacts. Local `actionlint` passes; hosted CI remains open.
- **Operator scope:** scoped operators can handshake with `operator:read` without wildcard robot scope. Robot-scoped state/broadcast filtering and command checks now have regression tests. The Django suite passes. This does not secure direct DDS publishers or establish a deployed Protected-LAN gateway.
- **Simulation clock / TF:** dynamic localization TF is freshness-checked; backward Gazebo clock jumps clear the TF buffer, invalidate pending pose/map operations and stop the manual command lease. Focused regression tests pass. The end-to-end TF and final-pose acceptance still fails under observed simulation load.
- **Map artifacts:** save validates the paused SLAM snapshot and serializes the session before registering artifacts; existing files are not replaced. Integration proved files and occupancy consistency, but not the entire load-to-navigation workflow.
- **User data:** the source database and R01 map registry had pre-existing modifications, and robot-local map/session files were already present as untracked data. They were preserved. Integration used fresh temporary data roots. Schema review identified operational-looking records and credential-bearing field names; contents were not disclosed in this report. Review and rotate/separate operational data before distributing the repository or artifacts.

## Release decision

**NO-GO — software release blockers remain.** Do not expose this build to an untrusted LAN or authorize physical-robot operation. Close the hosted CI gate, diagnose and stabilize map/ground-truth TF and final pose, complete uninterrupted Save → Load → Initial Pose → Navigation acceptance, validate a real gateway and DDS security boundary, and complete qualified hardware safety acceptance before production. Exact build/test commands and install/runtime procedures are in [`ROS_DEPLOYMENT.md`](../waretwin/ROS_DEPLOYMENT.md).
