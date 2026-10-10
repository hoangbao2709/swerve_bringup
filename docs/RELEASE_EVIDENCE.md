# Release hardening evidence — NO-GO

The first sections below preserve historical 2026-10-09 results. The next dated follow-up records a one-time V30E-independent Save/Load/Navigation pass on `b85add1eaaaf8e6c2efc210c30081b0d1aac0b1c`. The final dated follow-up is newer and authoritative for the current targeted release decision on `693fab43e0d9a8446d68dc97b3baac1aba8aff5e`; it records later contradictory Gazebo failures. This is software evidence, not certification. No physical robot was connected or operated. Machine-readable results are in [`release-evidence.json`](release-evidence.json), with temporary raw evidence under `/tmp/` on the test host.

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

## 2026-10-10 follow-up — V30E-independent localization

The active baseline remained branch `robot-real-sim-v5-demo-visual`, HEAD `b85add1eaaaf8e6c2efc210c30081b0d1aac0b1c`. The build/test tree was dirty (25 tracked files modified at test time); two release-evidence documentation files were updated afterward, so the final tree has 27 tracked modified files. No commit or push was made. The isolated non-symlink build at `/tmp/swerve-release-acceptance-20261010T002100Z/install-v30e-independent-final-handoff-2` completed all three packages in 57.7 s. Package-prefix lookup, both installed launch files' `--show-args`, and the installed Web-package operational-artifact scan passed. The latest focused ROS acceptance suites passed 67 tests in 1.22 s; the isolated full Django suite passed 199 tests in 10.267 s; and the changed Web Local workflow component passed its targeted frontend suite (51 tests in 6.67 s).

The verified localization owners and mode contracts are:

| Mode | `map → odom` owner | Initial Pose behavior |
|---|---|---|
| MAPPING | SLAM Toolbox | No static-map Initial Pose service; SLAM owns scan-matched localization. |
| UNIFIED | SLAM Toolbox | No second localizer; Nav2 consumes the registered map while SLAM publishes localization TF. |
| NAVIGATION | Nav2 AMCL | Discover AMCL's `nav2_msgs/srv/SetInitialPose` endpoint and owner; use its verified `/initialpose` topic only if the service binding is unavailable. |
| REAL_ROBOT (`use_sim:=false mode:=navigation`) | Nav2 AMCL | Same runtime-discovered AMCL interface; physical driver/hardware behavior was not tested. |

`enable_v30e_sim` defaults to `false`; the legacy simulated V30E/SetPose path is opt-in and is not part of normal Web Local. The default static-map Navigation run reported `LOCALIZATION_INTERFACE_READY=PASS backend=AMCL map_to_odom_owner=amcl interface=/set_initial_pose api=service type=nav2_msgs/srv/SetInitialPose`. MAPPING/UNIFIED intentionally do not offer that static-map action because SLAM Toolbox is already the authoritative `map → odom` publisher.

The real installed Gazebo test completed one full Mapping → Pause → Save → supervised Navigation switch → Load → Initial Pose → Nav2 goal sequence without a V30E node or measurements. Save wrote and validated YAML, PGM, SLAM `.data`, and a 39.7 MB `.posegraph`. The loaded Nav2 `/navigation_map` exactly matched all 416,208 cells from the selected PGM, including 285,667 unknown cells. The actual `/map_server/get_state` lifecycle response was `active` and selected-map ID/revision matched. AMCL accepted the post-load request on `/set_initial_pose`: fresh TF followed the request at simulation stamp 12.556 s (request 12.100 s), age 0.044 s, over three stable samples; requested-pose error was 0.00669 m / 0.000654 rad. One real post-load Nav2 goal then succeeded with map-TF error 0.04396 m / 0.000759 rad and independent Gazebo-projected error 0.04619 m / 0.00425 rad, within 0.05 m / 0.05 rad. The full machine report is `/tmp/swerve-release-acceptance-20261010T002100Z/tmp/full-stack-v30e-independent-final-handoff-2/test_installed_full_stack_reac0/runtime/save-load-navigation.json`.

This does not turn the full-suite result into PASS. The encompassing installed full-stack pytest exited 1 after 311.02 s: the separate direct-navigation ground-truth translation was 0.08495 m and failed the 5 cm gate (Nav2 action status itself was `SUCCEEDED`); the Web-originated mission passed in this run (0.02936 m / 0.00238 rad); and a manual-right physical-motion check also failed. Only one saved-map post-load navigation mission ran, not the five required for repeatability. The latest hosted Actions run `38008005683` is FAILED: backend and frontend jobs succeeded, while `ros-packages` failed its non-symlink nested-package build step with exit code 2. Its detailed console log was unavailable in the public page. Therefore the localization handoff is verified, but the simulation release remains NO-GO and hosted CI is not green.

No changes were made to the operational SQLite database or active map/session registry; all integration state was directed to the isolated `/tmp/swerve-release-acceptance-20261010T002100Z/tmp/...` runtime. The installed-package scan found no bundled database, secrets, or local maps. This does not remove the separate risk of operational-looking data remaining visible in the public Git tree/history.

## 2026-10-10 targeted remediation follow-up — NO-GO

This section supersedes earlier run results for the current targeted acceptance. The source baseline is branch `robot-real-sim-v5-demo-visual`, HEAD `693fab43e0d9a8446d68dc97b3baac1aba8aff5e`, with uncommitted implementation/test changes. The copied non-symlink build is `/tmp/swerve-final-targeted-20261010.zycCcf/install-v4`; the current code-tree diff fingerprint and complete machine-readable results are recorded in `/tmp/swerve-final-targeted-20261010.zycCcf/release-acceptance-20261010T0425Z/`. Gazebo runtimes, database, maps, ports, and ROS domain were isolated under `/tmp`; no physical robot was used.

The confirmed CI argument-order defect is corrected: `--log-base` now precedes the `build` verb in `.github/workflows/quality.yml`. A Humble non-symlink Colcon build produced `swerve_bringup`, `swerve_bridge`, and `waretwin_web`; the copied-install artifact scan and package-prefix checks passed. The hosted workflow for the source commit remains failed, however, and no hosted run can verify the uncommitted correction until it is published. Current public metadata for [run 38018160887](https://github.com/hoangbao2709/swerve_bringup/actions/runs/38018160887) reports `ros-packages` exit 2.

A second build in a clean environment (no inherited ROS or workspace overlays) also completed all three packages with the corrected top-level argument order in 57.7 seconds (Colcon-reported). The copied install was `/tmp/swerve-final-targeted-20261010.zycCcf/install-release-gate`; all three package prefixes resolved there, both installed launch files exposed arguments from `/tmp`, and the installed Web artifact scan passed. This verifies a clean local package build and launch discovery only; the Gazebo integrations below used the separately built `install-v4`, so no integration result is attributed to this supplemental build. Logs: `log-release-gate/build_2026-10-10_12-30-37/logger_all.log` and `release-acceptance-20261010T0425Z/clean-install-{discovery,artifact-scan}.log` under the test root.

Two software ordering/configuration defects were corrected and regression-tested. Initial-pose confirmation metadata (`localization_owner` and `initial_pose_interface`) is now validated before any local-map confirmation state is written; the full backend suite passed 199 tests. The bridge's local robot map root now derives from the resolved `artifact_root`; the real saved-map run wrote YAML/PGM/SLAM artifacts beneath the isolated override `/tmp/.../runtime/maps/local_robot_maps/R01/`, and the recorded default operational root was not used.

The measured Gazebo results do not meet release acceptance:

- Direct Nav2 returned `SUCCEEDED`, but the projected Gazebo goal error was `0.05710 m`; map TF versus Gazebo yaw disagreement was `0.05859 rad`. Timestamps were paired at the same simulation time and the terminal TF was fresh (`0.0069 s` age). During motion SLAM's `map → odom` yaw changed by `−0.05933 rad` at simulation time `21.495 s`; this is an observed localization/map-alignment correction, not a stale-sample comparison. The controller reported its effective `0.03 m` XY and `0.05 rad` yaw settings.
- Web Nav2 returned `SUCCEEDED`, but map-TF/Gazebo translation disagreement was `0.05285 m` in the v5 run. A previous v4 Web mission passed (`0.02180 m` / `0.00515 rad` TF/Gazebo disagreement); that earlier PASS does not cancel the contradictory v5 failure.
- The physical Manual RIGHT check passed in both full Gazebo runs: body-right displacement was approximately `0.600 m`, with chassis yaw change below `0.0017 rad`, valid manual ownership/acknowledgment, and observed zero command after release. Seven other translation actions passed in v5. BACKWARD_RIGHT moved in the expected body direction (`0.4073 m`), with valid bridge/controller acknowledgments and zero command, but mechanical stop acceptance timed out after `45` simulation seconds / `100.76` wall seconds: encoder drift was `0.00610 rad/s` against the existing `0.005` settling bound, with measured body-position drift `0.000403 m/s`. Both rotation directions were not executed after that stop could not be proven. This is not a full manual-motion PASS.
- Save produced valid YAML/PGM and nonempty serialized SLAM artifacts; the saved occupancy matched Nav2's loaded `699 × 598`, `0.05 m` map exactly (`418,002` cells; zero mismatches). The selected map ID/revision matched. But after the supervised transition AMCL did not meet the existing post-request stable-TF confirmation guard. The request pose errors were within tolerance (`0.00581 m`, `0.00653 rad`), yet confirmation returned HTTP 502. AMCL's log reports 142 LaserScan message-filter drops from stamp `6.306` to `21.863 s` as “earlier than all the data in the transform cache”; `map → odom` is therefore not demonstrated as a stable post-request AMCL localization. The requested five post-load missions did not start (`0/5`).

The enclosing full-stack v5 test exited 1 after approximately `388.35 s`. A v4 real Gazebo run also failed, though it achieved one saved-map navigation PASS; neither run yields five independent passing missions. Acceptance thresholds remain `0.05 m / 0.05 rad`. Backend (199), frontend (161 tests, typecheck/build, npm audit zero vulnerabilities), broad ROS/Python regression (177 passed, 12 skipped), focused ROS (65 passed), copied-install Web deployment (11 tests), package build, and install artifact scan passed locally. `actionlint` was unavailable in this follow-up shell; no new lint PASS is claimed.

Operational source data remain intact. The tracked SQLite DB, R01 registry, and tracked `dg`/`warehouse` map/session artifacts were not modified or included in the copied ROS package. They remain visible in repository history and require explicit owner review/separation before public distribution; no deletion or history rewrite was performed. Protected-LAN authentication gateway, DDS access control, and physical hardware/E-stop acceptance remain separate unverified gates.

**Decision: NO-GO.** Direct pose consistency, post-load AMCL scan/TF handoff, BACKWARD_RIGHT stop verification, five-run repeatability, and hosted CI remain open. Nav2 `SUCCEEDED` does not waive the independent pose, TF-freshness, map-identity, or stopping gates.
