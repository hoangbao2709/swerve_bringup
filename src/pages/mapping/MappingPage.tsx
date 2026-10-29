import { useCallback, useEffect, useRef, useState } from "react";
import { getLocalRobotMaps, getLocalRuntimeMode, loadLocalRobotMap, resumeLocalRobotSlamSession, saveLocalRobotMap, setMappingState, type LocalRobotMap, type LocalMapLoadResult, type LocalSlamResumeResult } from "../../services/api";
import type { RobotDetailMapSnapshot, RobotDetailScan, RobotRuntimeCapabilities, RobotSystemDiagnostics, RobotState, RobotWorldPoint } from "../../schema/twin_state";
import { displayedFramePose } from "../../layout/robotPoseFrame";
import { useStore } from "../../state/store";
import { IS_SHOWCASE, IS_STATIC_UI as STATIC_UI_SETTING } from "../../config";
import { displayRuntimeValue, STATIC_UI_EMPTY_VALUE } from "../../mock/staticPresentation";
import { useLocalAppRuntime, type LocalAppRuntime } from "../../app/AppLayout";
import { MetricRow } from "../../components/shared/ui/HmiPrimitives";
import { valueNumber } from "../../components/shared/ui/RobotPagePrimitives";
import { MappingMapWorkspace } from "./components/MappingMapWorkspace";
import { MappingSessionCard } from "./components/MappingSessionCard";
import { SaveMapCard } from "./components/SaveMapCard";
import { SavedMapsPanel } from "./components/SavedMapsPanel";
import { SelectedMapDetails } from "./components/SelectedMapDetails";
import { MappingAdvancedDetails } from "./components/MappingAdvancedDetails";
import type { MappingLayers } from "./mappingTypes";
import { MapImportStager } from "./MapImportStager";
import "./mapping.css";

const IS_STATIC_UI = import.meta.env.DEV && STATIC_UI_SETTING;

type MappingPanelProps = {
  robotId: string; robot?: RobotState; slam2dMap: RobotDetailMapSnapshot | null;
  runtimeMapSnapshot: RobotDetailMapSnapshot | null; scan: RobotDetailScan | null;
  diagnostics: RobotSystemDiagnostics | null; controlOnline: boolean;
  controlMode: "MANUAL" | "AUTONOMOUS"; runtimeState: string;
  runtimeCapabilities: RobotRuntimeCapabilities | null; mappingSessionId?: string | null;
  activeLocalMapRevision: string | null; localMapSyncStatus: string | null;
  mapRevision: number | null; ensureManualMode?: () => void;
};

function loadOperationLabel(phase?: string, status?: string): string {
  const labels: Record<string, string> = {
    PREPARING: "PREPARING...", PAUSING_MAPPING: "PAUSING MAPPING...",
    SWITCHING_TO_NAVIGATION: "SWITCHING TO NAVIGATION...", STARTING_NAV2: "STARTING NAV2...",
    WAITING_FOR_MANUAL: "RESTORING MANUAL MODE...", LOADING_MAP: "LOADING MAP...",
    VERIFYING_MAP: "VERIFYING MAP...", ROLLING_BACK: "ROLLING BACK...",
  };
  if (phase && labels[phase]) return labels[phase];
  if (status === "REQUESTED") return labels.PREPARING;
  if (status === "RESTARTING" || status === "STARTING") return labels.SWITCHING_TO_NAVIGATION;
  if (status === "ROLLING_BACK") return labels.ROLLING_BACK;
  return labels.STARTING_NAV2;
}

const waitForLoadPoll = () => new Promise<void>((resolve) => window.setTimeout(resolve, 1000));

export function MappingPage() {
  const runtime = useLocalAppRuntime();
  const robotId = runtime.robotId;
  if (!robotId) return null;
  return <MappingPageContent robotId={robotId} runtime={runtime} />;
}

function MappingPageContent({ robotId, runtime }: { robotId: string; runtime: LocalAppRuntime }) {
  const robot = useStore((state) => state.twin?.robots?.[robotId]);
  const detail = useStore((state) => state.robotDetail[robotId]);
  const slam2dMap = detail?.slam2dMap ?? null;
  const runtimeMapSnapshot = detail?.runtimeMapSnapshot ?? null;
  const diagnostics = (detail?.diagnostics ?? runtime.diagnostics) as RobotSystemDiagnostics | null;
  const scan = detail?.scan ?? null;
  const runtimeState = useStore((state) => state.runtimeState);
  const runtimeCapabilities = useStore((state) => state.robotCapabilities[robotId] ?? null);
  const mappingSessionId = detail?.mappingSessionId ?? null;
  const activeLocalMapRevision = detail?.activeLocalMapRevision ?? null;
  const localMapSyncStatus = detail?.localMapSyncStatus ?? null;
  const mapRevision = useStore((state) => state.mapSync.publishedRevision);
  return <main className="flex min-h-0 min-w-0 flex-1 flex-col gap-hmi-2 overflow-hidden bg-ptagv-page p-hmi-2">
    <MappingPanel robotId={robotId} robot={robot} slam2dMap={slam2dMap} runtimeMapSnapshot={runtimeMapSnapshot}
      scan={scan} diagnostics={diagnostics} controlOnline={runtime.controlOnline} controlMode={runtime.controlMode}
      runtimeState={runtimeState} runtimeCapabilities={runtimeCapabilities} mappingSessionId={mappingSessionId}
      activeLocalMapRevision={activeLocalMapRevision} localMapSyncStatus={localMapSyncStatus} mapRevision={mapRevision}
      ensureManualMode={() => runtime.setControlMode("MANUAL", true)} />
  </main>;
}

function MappingPanel({ robotId, robot: rawRobot, slam2dMap, runtimeMapSnapshot, scan, diagnostics, controlOnline, controlMode, runtimeState, runtimeCapabilities, mappingSessionId, activeLocalMapRevision: selectedLocalMapRevision, localMapSyncStatus, mapRevision, ensureManualMode }: MappingPanelProps) {
  const setRobotDetail = useStore((state) => state.setRobotDetail);
  const [maps, setMaps] = useState<LocalRobotMap[]>([]);
  const [selected, setSelected] = useState("");
  const [name, setName] = useState("");
  const [mappingState, setMappingStateValue] = useState(IS_STATIC_UI || IS_SHOWCASE ? "UNAVAILABLE" : "UNKNOWN");
  const [mappingDuration, setMappingDuration] = useState(0);
  const [activeLocalMapId, setActiveLocalMapId] = useState<string | null>(() => IS_STATIC_UI || IS_SHOWCASE
    ? useStore.getState().robotDetail[robotId]?.activeLocalMapId ?? null : null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const [notice, setNotice] = useState("");
  const [loadError, setLoadError] = useState("");
  const [loadNotice, setLoadNotice] = useState("");
  const [resumeError, setResumeError] = useState("");
  const [resumeNotice, setResumeNotice] = useState("");
  const [resumeState, setResumeState] = useState("");
  const [operationState, setOperationState] = useState(IS_STATIC_UI || IS_SHOWCASE ? "UNAVAILABLE" : "READY");
  const [loadBlockReason, setLoadBlockReason] = useState("SELECT A SAVED MAP");
  const [trajectory, setTrajectory] = useState<RobotWorldPoint[]>([]);
  const [layers, setLayers] = useState<MappingLayers>({ robot: true, scan: true, trajectory: true, grid: false });
  const [previewOpen, setPreviewOpen] = useState(false);
  const trajectoryRef = useRef<{ mapId: string; points: RobotWorldPoint[]; lastAt: number }>({ mapId: "", points: [], lastAt: 0 });
  const manualRestoreRequest = useRef<string | null>(null);

  const slamRuntimeActive = runtimeState === "MAPPING" || runtimeState === "UNIFIED";
  const slamMap = slamRuntimeActive && !activeLocalMapId && slam2dMap?.robot_id === robotId
    && slam2dMap?.map_source === "SLAM_TOOLBOX"
    && (!mappingSessionId || slam2dMap.mapping_session_id === mappingSessionId) ? slam2dMap : null;
  const slamPose = slamMap && rawRobot ? displayedFramePose(rawRobot, slamMap) : undefined;
  const expectedLocalMapRevision = selectedLocalMapRevision
    ?? maps.find((map) => map.id === activeLocalMapId)?.revision ?? null;
  const selectedMap = maps.find((map) => map.id === selected) ?? null;
  const loadedLocalMap = activeLocalMapId && runtimeMapSnapshot?.map_source === "LOCAL_MAP"
    && runtimeMapSnapshot.active_map_id === activeLocalMapId
    && String(runtimeMapSnapshot.active_map_revision ?? "") === String(expectedLocalMapRevision ?? "")
    ? runtimeMapSnapshot : null;
  const selectedMapIsActive = !IS_STATIC_UI && !IS_SHOWCASE && Boolean(selectedMap && selectedMap.id === activeLocalMapId);
  const selectedMapPreview = selectedMapIsActive ? loadedLocalMap : null;
  const selectedMapExtent = selectedMap?.width && selectedMap?.height
    ? `${valueNumber(selectedMap.width * selectedMap.resolution, 2)} × ${valueNumber(selectedMap.height * selectedMap.resolution, 2)} m`
    : "N/A";
  const displayMap = IS_STATIC_UI || IS_SHOWCASE ? runtimeMapSnapshot : loadedLocalMap ?? slamMap;
  const displayPose = displayMap && rawRobot ? displayedFramePose(rawRobot, displayMap) : undefined;
  const robotPoseMatchesDisplayMap = Boolean(displayMap && displayPose);
  const fixtureRobotPose = (IS_STATIC_UI || IS_SHOWCASE) && displayMap?.map_source === "LOCAL_MAP"
      && rawRobot?.id === displayMap.robot_id
      && rawRobot.active_map_pose?.pose_source === (IS_SHOWCASE ? "LOCAL_SAMPLE" : "STATIC_UI_FIXTURE")
      && rawRobot.active_map_pose.map_source === (IS_SHOWCASE ? "LOCAL_MAP" : "STATIC_UI_FIXTURE")
      && rawRobot.active_map_pose.frame_id === displayMap.frame_id
      && rawRobot.active_map_pose.map_id === displayMap.active_map_id
      && String(rawRobot.active_map_pose.map_revision) === String(displayMap.active_map_revision ?? "")
    ? rawRobot.active_map_pose : undefined;
  const mapping = diagnostics?.mapping;

  const refresh = useCallback(async () => {
    const result = await getLocalRobotMaps(robotId);
    setMaps(result.maps);
    setMappingStateValue(result.mapping_state);
    setMappingDuration(Number(result.mapping_duration_s) || 0);
    setActiveLocalMapId(result.active_local_map_id);
    const activeLocalMap = result.maps.find((map) => map.id === result.active_local_map_id);
    setRobotDetail(robotId, {
      activeLocalMapId: result.active_local_map_id,
      activeLocalMapRevision: result.local_active_map_revision ?? activeLocalMap?.revision ?? null,
      localMapSyncStatus: result.map_sync_status,
    });
    if (!selected && result.maps.length) setSelected(result.maps[0].id);
  }, [robotId, selected, setRobotDetail]);

  useEffect(() => {
    if (IS_STATIC_UI) {
      let active = true;
      void import("../../mock/fixtures").then(({ STATIC_LOCAL_MAP }) => {
        if (active) {
          setMaps([STATIC_LOCAL_MAP]);
          setSelected(STATIC_LOCAL_MAP.id);
          setActiveLocalMapId(useStore.getState().robotDetail[robotId]?.activeLocalMapId ?? STATIC_LOCAL_MAP.id);
        }
      });
      return () => { active = false; };
    }
    if (IS_SHOWCASE) {
      let active = true;
      void refresh().catch((caught: unknown) => {
        if (active) setError(caught instanceof Error ? caught.message : "Sample map library is unavailable.");
      });
      return () => { active = false; };
    }
    if (!controlOnline) return;
    let active = true;
    const poll = () => refresh().catch((caught: unknown) => {
      if (active) setError(caught instanceof Error ? caught.message : "Map registry is unavailable");
    });
    void poll();
    const timer = window.setInterval(() => { void poll(); }, 3000);
    return () => { active = false; window.clearInterval(timer); };
  }, [controlOnline, refresh]);

  const run = async (action: () => Promise<unknown>, success: (result: unknown) => string, pendingState: string) => {
    setOperationState(pendingState);
    setBusy(true); setError(""); setNotice("");
    try { const result = await action(); setNotice(success(result)); await refresh(); setOperationState("READY"); }
    catch (caught) { setOperationState("ERROR"); setError(caught instanceof Error ? caught.message : "Robot map operation failed"); }
    finally { setBusy(false); }
  };

  const save = () => void run(() => saveLocalRobotMap(robotId, name), (raw) => {
    const result = raw as { map: LocalRobotMap };
    setName(""); setSelected(result.map.id);
    return IS_SHOWCASE
      ? `BROWSER COPY SAVED · ${result.map.name} · revision ${result.map.revision}`
      : `SAVE SUCCESS · ${result.map.name} · revision ${result.map.revision}`;
  }, "SAVING");
  const load = () => {
    if (loadBlockReason) { setLoadError(loadBlockReason); return; }
    const mapId = selected;
    void (async () => {
      setBusy(true); setLoadError(""); setLoadNotice(""); setOperationState("PREPARING...");
      try {
        let result: LocalMapLoadResult = await loadLocalRobotMap(robotId, mapId);
        if (IS_SHOWCASE) {
          if (result.status !== "PREVIEW_SELECTED" || !result.active_map || result.active_map.id !== mapId) {
            throw new Error(result.message || "Map preview could not be selected.");
          }
          setLoadNotice(`LOCAL PREVIEW · ${result.active_map.name} · r${result.active_map.revision} · robot map unchanged`);
          setOperationState("LOCAL PREVIEW");
          await refresh();
          return;
        }
        const deadline = Date.now() + 10 * 60 * 1000;
        while (result.status === "TRANSITIONING") {
          if (Date.now() >= deadline) throw new Error("Saved-map load timed out while the supervised runtime was transitioning");
          setOperationState(loadOperationLabel(result.phase, result.transition?.status));
          await waitForLoadPoll();
          const runtimeResult = await getLocalRuntimeMode(robotId);
          const transition = runtimeResult.transition;
          if (["ERROR", "ROLLED_BACK"].includes(String(transition.status).toUpperCase())) {
            throw new Error(transition.message || result.message || "The supervised navigation transition failed");
          }
          if (result.phase === "ROLLING_BACK"
              && transition.status === "READY"
              && String(transition.mode || "").toUpperCase() !== "NAVIGATION") {
            throw new Error(result.message || "Saved-map load failed; previous runtime was restored");
          }
          if (result.phase === "WAITING_FOR_MANUAL") {
            setOperationState("RESTORING MANUAL MODE...");
            const requestKey = result.request_id || `${robotId}:${mapId}`;
            if (manualRestoreRequest.current !== requestKey) {
              manualRestoreRequest.current = requestKey;
              ensureManualMode?.();
            }
          }
          if (transition.status === "READY" && String(transition.mode || "").toUpperCase() === "NAVIGATION") {
            setOperationState(loadOperationLabel(result.phase, transition.status));
            result = await loadLocalRobotMap(robotId, mapId);
          } else if (!result.request_id || transition.request_id === result.request_id) {
            result = await loadLocalRobotMap(robotId, mapId);
          }
        }
        if (result.status !== "LOADED" || !result.active_map
            || result.active_map.id !== mapId || result.active_map_id !== mapId
            || result.map_sync_status !== "LOCAL_ONLY" || result.map_source !== "LOCAL_MAP") {
          throw new Error(result.message || "ROS did not confirm the selected saved map as the active /map");
        }
        setActiveLocalMapId(result.active_map.id);
        setRobotDetail(robotId, {
          activeLocalMapId: result.active_map.id,
          activeLocalMapRevision: result.active_map.revision,
          localMapSyncStatus: "LOCAL_ONLY",
        });
        setLoadNotice(`MAP LOADED · ${result.active_map.name} · r${result.active_map.revision} · ${result.active_map.resolution.toFixed(3)} m/cell`);
        setOperationState("MAP LOADED");
        manualRestoreRequest.current = null;
        await refresh();
      } catch (caught) {
        setOperationState("LOAD FAILED");
        setLoadError(caught instanceof Error ? caught.message : "Saved-map load failed");
      } finally { setBusy(false); }
    })();
  };
  const changeMapping = (action: "start" | "stop") => void run(
    () => setMappingState(robotId, action),
    (raw) => {
      const result = raw as { mapping_state: string };
      setMappingStateValue(result.mapping_state);
      return `MAPPING ${result.mapping_state}`;
    }, action === "start" ? "STARTING" : "STOPPING",
  );

  const isUnified = runtimeState === "UNIFIED";
  const isMapping = runtimeState === "MAPPING"
    || (isUnified && (runtimeCapabilities?.mapping_available ?? Boolean(diagnostics?.slam)));
  const paused = mappingState === "PAUSED";
  useEffect(() => {
    if (IS_SHOWCASE) {
      setLoadBlockReason(selected ? "" : "SELECT A SAVED MAP");
      return;
    }
    if (busy) { setLoadBlockReason("MAP TRANSITION IN PROGRESS"); return; }
    if (!controlOnline) { setLoadBlockReason("ROBOT OFFLINE"); return; }
    if (!selected) { setLoadBlockReason("SELECT A SAVED MAP"); return; }
    if (controlMode !== "MANUAL" || (rawRobot?.control_mode && rawRobot.control_mode !== "MANUAL")) {
      setLoadBlockReason("SWITCH TO MANUAL"); return;
    }
    if (diagnostics?.command_ownership?.estop_active === true
        || String(rawRobot?.navigation_state || "").toUpperCase() === "EMERGENCY_STOPPED") {
      setLoadBlockReason("CLEAR E-STOP BEFORE LOADING MAP"); return;
    }
    const navigationState = String(rawRobot?.navigation_state || "").toUpperCase();
    if (!navigationState) { setLoadBlockReason("WAIT FOR ROBOT SAFETY STATUS"); return; }
    if (!["IDLE", "MANUAL", "SUCCEEDED", "COMPLETED", "FAILED", "ERROR", "CANCELED", "CANCELLED", "REJECTED"].includes(navigationState)) {
      setLoadBlockReason("NAVIGATION GOAL MUST BE CANCELLED"); return;
    }
    const velocities = [rawRobot?.vx, rawRobot?.vy, rawRobot?.wz].map(Number);
    if (!velocities.every((velocity) => Number.isFinite(velocity) && Math.abs(velocity) <= 0.05)) {
      setLoadBlockReason("STOP ROBOT BEFORE LOADING MAP"); return;
    }
    setLoadBlockReason("");
  }, [busy, controlMode, controlOnline, diagnostics?.command_ownership?.estop_active,
    isMapping, mappingState, rawRobot?.control_mode, rawRobot?.navigation_state,
    rawRobot?.vx, rawRobot?.vy, rawRobot?.wz, selected]);
  const startMapping = async () => {
    if (isMapping) { changeMapping("start"); return; }
    setError(isUnified ? "SLAM is not currently available in this Unified runtime. Wait for SLAM readiness before mapping." : "SLAM mapping requires the Unified runtime. Start one stack with ./scripts/start_stack.sh unified --gui --rviz.");
  };
  useEffect(() => {
    const mapId = `${robotId}:${slamMap?.active_map_id ?? "waiting"}`;
    if (trajectoryRef.current.mapId === mapId) return;
    trajectoryRef.current = { mapId, points: [], lastAt: 0 };
    setTrajectory([]);
  }, [robotId, slamMap?.active_map_id]);
  useEffect(() => {
    if (!slamMap || !slamPose || !isMapping || paused || !mapping?.tf_valid) return;
    const x = Number(slamPose.x);
    const y = Number(slamPose.y);
    if (!Number.isFinite(x) || !Number.isFinite(y)) return;
    const current = trajectoryRef.current;
    const previous = current.points[current.points.length - 1];
    const now = Date.now();
    if (previous && Math.hypot(x - previous[0], y - previous[1]) < 0.10 && now - current.lastAt < 2000) return;
    const points = [...current.points, [x, y] as RobotWorldPoint].slice(-500);
    trajectoryRef.current = { ...current, points, lastAt: now };
    setTrajectory(points);
  }, [isMapping, mapping?.tf_valid, paused, slamPose?.x, slamPose?.y, slamPose?.timestamp, slamMap]);
  const staticPreviewScan = IS_STATIC_UI && scan?.robot_id === robotId && scan.frame_id === displayMap?.frame_id ? scan : null;
  const mapScan = slamMap && scan?.robot_id === robotId && scan.mapping_session_id === slamMap.mapping_session_id ? scan : staticPreviewScan;
  const previewMap = IS_STATIC_UI ? runtimeMapSnapshot
    : IS_SHOWCASE && runtimeMapSnapshot && selectedMap ? { ...runtimeMapSnapshot,
        active_map_id: selectedMap.id, active_map_revision: selectedMap.revision }
      : IS_SHOWCASE ? runtimeMapSnapshot : selectedMapPreview;
  const previewStatus = IS_STATIC_UI ? STATIC_UI_EMPTY_VALUE : IS_SHOWCASE ? "NOT CONNECTED" : error || loadError || operationState === "ERROR" || operationState === "LOAD FAILED"
    || ["ERROR", "FAILED", "FAULT"].includes(mappingState.toUpperCase()) ? "ERROR"
      : !controlOnline ? "WAITING" : isMapping && paused ? "PAUSED"
        : isMapping && mappingState === "MAPPING" ? "MAPPING" : isUnified && !isMapping ? "WAITING" : "IDLE";
  const previewTone = IS_STATIC_UI ? "warning" : previewStatus === "ERROR" ? "error"
    : previewStatus === "PAUSED" || previewStatus === "WAITING" ? "warning" : previewStatus === "MAPPING" ? "ready" : "warning";
  const startDisabled = IS_SHOWCASE || !controlOnline || controlMode !== "MANUAL" || busy || !isMapping || mappingState === "MAPPING";
  const pauseDisabled = IS_SHOWCASE || !controlOnline || !isMapping || busy || paused;
  const startDisabledReason = IS_STATIC_UI ? "" : IS_SHOWCASE ? "Requires a connected robot and SLAM runtime."
    : !controlOnline ? "Robot is offline."
      : controlMode !== "MANUAL" ? "Switch to Manual mode to map."
        : busy ? "A map operation is in progress."
          : !isMapping ? "Waiting for SLAM mapping readiness."
            : mappingState === "MAPPING" ? "Mapping is already active." : "";
  const pauseDisabledReason = IS_STATIC_UI ? "" : IS_SHOWCASE ? "Requires an active SLAM session."
    : !controlOnline ? "Robot is offline."
      : !isMapping ? "SLAM mapping is not ready."
        : busy ? "A map operation is in progress."
          : paused ? "Mapping is already paused." : "";
  const saveDisabled = IS_SHOWCASE ? busy || !name.trim()
    : !controlOnline || !isMapping || !paused || !slamMap || busy || !name.trim();
  const saveDisabledReason = IS_STATIC_UI || IS_SHOWCASE ? ""
    : !controlOnline ? "Robot is offline."
      : !isMapping ? "SLAM mapping is not ready."
        : !paused ? "Pause Mapping before saving."
          : !slamMap ? "Accumulated SLAM map is not available."
            : busy ? "A map operation is in progress."
              : !name.trim() ? "Enter a map name." : "";
  const loadDisabled = !selectedMap || Boolean(loadBlockReason) || busy;
  const selectedSessionAvailable = selectedMap?.slam_session_state?.status?.toUpperCase() === "AVAILABLE"
    && selectedMap.slam_session_state.engine?.toUpperCase() === "SLAM_TOOLBOX";
  const resumeBlockReason = IS_STATIC_UI ? "PREVIEW ONLY" : IS_SHOWCASE ? "Requires a live SLAM runtime."
    : !selectedMap ? "SELECT A SAVED MAP"
      : !selectedSessionAvailable ? "SAVED SLAM POSE GRAPH UNAVAILABLE"
        : busy ? "MAP OPERATION IN PROGRESS"
          : !controlOnline ? "ROBOT OFFLINE"
            : isUnified ? "POSE-GRAPH RESTORE UNSUPPORTED IN UNIFIED MODE"
              : !["MAPPING", "NAVIGATION"].includes(runtimeState) ? "START A MAPPING OR NAVIGATION RUNTIME"
                : controlMode !== "MANUAL" || (rawRobot?.control_mode && rawRobot.control_mode !== "MANUAL") ? "SWITCH TO MANUAL"
                  : diagnostics?.command_ownership?.estop_active === true
                      || String(rawRobot?.navigation_state || "").toUpperCase() === "EMERGENCY_STOPPED" ? "CLEAR E-STOP BEFORE RESTORING SESSION"
                    : !rawRobot?.navigation_state ? "WAIT FOR ROBOT SAFETY STATUS"
                      : !["IDLE", "MANUAL", "SUCCEEDED", "COMPLETED", "FAILED", "ERROR", "CANCELED", "CANCELLED", "REJECTED"].includes(String(rawRobot.navigation_state).toUpperCase()) ? "NAVIGATION GOAL MUST BE CANCELLED"
                        : ![rawRobot.vx, rawRobot.vy, rawRobot.wz].map(Number).every((velocity) => Number.isFinite(velocity) && Math.abs(velocity) <= 0.05) ? "STOP ROBOT BEFORE RESTORING SESSION"
                          : "";
  const resumeSavedSession = () => {
    if (resumeBlockReason || !selectedMap) {
      setResumeError(resumeBlockReason || "Select a saved SLAM session");
      return;
    }
    const mapId = selectedMap.id;
    const mapName = selectedMap.name;
    void (async () => {
      setBusy(true);
      setResumeError("");
      setResumeNotice("");
      setResumeState("REQUESTING SLAM RESTORE...");
      setOperationState("RESTORING SLAM SESSION...");
      try {
        let result: LocalSlamResumeResult = await resumeLocalRobotSlamSession(robotId, mapId);
        const deadline = Date.now() + 190_000;
        while (result.status === "TRANSITIONING") {
          if (Date.now() >= deadline) throw new Error("Saved SLAM session restore timed out during the supervised restart");
          setResumeState("RESTARTING MAPPING RUNTIME...");
          await waitForLoadPoll();
          const runtimeResult = await getLocalRuntimeMode(robotId);
          const transitionStatus = String(runtimeResult.transition?.status || "").toUpperCase();
          if (["ERROR", "ROLLED_BACK"].includes(transitionStatus)) {
            throw new Error(runtimeResult.transition?.message || result.message || "The Mapping runtime restart failed");
          }
          result = await resumeLocalRobotSlamSession(robotId, mapId);
        }
        if (result.status !== "RESUMED" || result.map_id !== mapId || result.restore_evidence?.passed !== true) {
          throw new Error(result.message || "The backend did not confirm restoration of this saved SLAM map");
        }
        setMappingStateValue("MAPPING");
        setActiveLocalMapId(null);
        setRobotDetail(robotId, { activeLocalMapId: null, activeLocalMapRevision: null, localMapSyncStatus: null });
        setResumeNotice(`SLAM SESSION RESTORED · ${mapName}`);
        setResumeState("");
        setOperationState("SESSION RESTORED");
        await refresh();
      } catch (caught) {
        const message = caught instanceof Error ? caught.message : "Saved SLAM session restore failed";
        setResumeError(message);
        setResumeState("");
        setOperationState("SESSION RESTORE FAILED");
        void refresh().catch(() => undefined);
      } finally {
        setBusy(false);
      }
    })();
  };
  const mapSource = IS_STATIC_UI ? STATIC_UI_EMPTY_VALUE : IS_SHOWCASE ? "SAMPLE MAP · LOCAL ONLY"
    : loadedLocalMap ? "SAVED MAP · r" + (runtimeMapSnapshot?.active_map_revision ?? "—")
      : slamMap ? "SLAM TOOLBOX /map" : "WAITING FOR MAP";
  const mapSize = displayRuntimeValue(IS_STATIC_UI, displayMap ? displayMap.width + " × " + displayMap.height + " cells" : "Waiting for map");
  const sessionDuration = displayRuntimeValue(IS_STATIC_UI, valueNumber(mappingDuration, 1, " s") + (isMapping && !paused ? " · LIVE" : ""));
  const robotPoseSummary = robotPoseMatchesDisplayMap && displayPose
    ? displayRuntimeValue(IS_STATIC_UI, valueNumber(displayPose.x, 2) + ", " + valueNumber(displayPose.y, 2) + " m · " + valueNumber(displayPose.yaw, 2) + " rad")
    : displayRuntimeValue(IS_STATIC_UI, "WAITING FOR MATCHING MAP POSE");
  const mapOrigin = displayRuntimeValue(IS_STATIC_UI, slamMap ? valueNumber(slamMap.origin.x, 2) + ", " + valueNumber(slamMap.origin.y, 2) + " m" : "—");
  const activeMapValue = IS_STATIC_UI ? STATIC_UI_EMPTY_VALUE : IS_SHOWCASE ? `${displayMap?.active_map_id ?? "SAMPLE"} · SAMPLE ONLY`
    : activeLocalMapId ? activeLocalMapId + " · r" + (runtimeMapSnapshot?.active_map_revision ?? "—")
      : isMapping ? (slamMap?.active_map_id ?? "SLAM SESSION WAITING") + " · LOCAL ONLY" : "CANONICAL";
  const startLabel = IS_STATIC_UI ? "START / RESUME MAPPING" : IS_SHOWCASE ? "MAPPING UNAVAILABLE"
    : isMapping && mappingState === "MAPPING" ? "MAPPING ACTIVE"
      : paused ? "RESUME MAPPING" : isMapping ? "START MAPPING"
        : isUnified ? "SLAM UNAVAILABLE" : "UNIFIED RUNTIME REQUIRED";

  return <div className="mapping-workspace grid min-h-0 min-w-0 flex-1 grid-cols-1 gap-hmi-3 overflow-hidden lg:grid-cols-[minmax(0,2.12fr)_minmax(320px,1fr)]">
    <div className="grid min-h-0 min-w-0 grid-rows-[minmax(0,1fr)_auto_auto] gap-hmi-3">
      <MappingMapWorkspace map={displayMap} robot={!IS_STATIC_UI && robotPoseMatchesDisplayMap ? rawRobot : undefined}
        fixtureRobotPose={IS_STATIC_UI ? undefined : fixtureRobotPose} scan={IS_STATIC_UI ? null : mapScan}
        pose={displayPose} layers={layers} trajectory={IS_STATIC_UI ? [] : trajectory}
        isSlamMap={Boolean(slamMap)} isStatic={IS_STATIC_UI} isShowcase={IS_SHOWCASE} loadedLocalMap={Boolean(loadedLocalMap)}
        emptyMessage={activeLocalMapId ? "Waiting for the confirmed active saved /map snapshot." : "Waiting for a fresh accumulated SLAM Toolbox /map. Live LiDAR frames are sensor views and are not the warehouse map."}
        mapSource={mapSource} onToggleLayer={(layer) => setLayers((current) => ({ ...current, [layer]: !current[layer] }))} />
      <div className="grid min-w-0 grid-cols-2 border border-ptagv-border bg-white md:grid-cols-5" aria-label="Map and session summary">
        <MetricRow layout="stacked" label="MAP SIZE" value={mapSize} mono />
        <MetricRow layout="stacked" label="RESOLUTION" value={displayRuntimeValue(IS_STATIC_UI, valueNumber(displayMap?.resolution, 3, " m/cell"))} mono />
        <MetricRow layout="stacked" label="EXPLORED AREA" value={displayRuntimeValue(IS_STATIC_UI, valueNumber(displayMap?.explored_area_m2, 2, " m²"))} mono />
        <MetricRow layout="stacked" label="SESSION DURATION" value={sessionDuration} mono />
        <MetricRow layout="stacked" label="MAP SOURCE" value={IS_STATIC_UI ? STATIC_UI_EMPTY_VALUE : IS_SHOWCASE ? "SAMPLE MAP" : loadedLocalMap ? "SAVED LOCAL MAP" : slamMap ? "SLAM TOOLBOX" : "WAITING"} />
      </div>
      <MappingAdvancedDetails isStatic={IS_STATIC_UI} runtimeState={runtimeState} controlMode={controlMode}
        capabilities={runtimeCapabilities} diagnostics={diagnostics} slamMap={slamMap} runtimeMap={runtimeMapSnapshot}
        localMapSyncStatus={localMapSyncStatus}
        mappingState={mappingState} trajectory={trajectory} maps={maps} operationState={operationState}
        canonicalRevision={mapRevision} robotPoseSummary={robotPoseSummary} mapOrigin={mapOrigin} activeMapValue={activeMapValue} />
    </div>
    <aside className={`mapping-operation-column flex min-h-0 min-w-0 flex-col gap-hmi-3 overflow-x-hidden overflow-y-auto pr-0.5 [scrollbar-width:thin] ${IS_STATIC_UI ? "pb-10" : ""}`} aria-label="Mapping operations">
      <MappingSessionCard status={previewStatus} statusTone={previewTone} isStatic={IS_STATIC_UI}
        primaryLabel={startLabel} primaryDisabled={startDisabled} pauseDisabled={pauseDisabled}
        primaryReason={startDisabledReason} pauseReason={pauseDisabledReason} operationState={operationState}
        busy={busy} error="" onStartOrResume={() => void startMapping()} onPause={() => changeMapping("stop")} />
      <SaveMapCard name={name} onNameChange={setName} disabled={saveDisabled} disabledReason={saveDisabledReason}
        busy={busy} operationState={operationState} error={error} notice={notice} isShowcase={IS_SHOWCASE} onSave={save} />
      <SavedMapsPanel maps={maps} selected={selected} activeMapId={IS_SHOWCASE ? null : activeLocalMapId} isStatic={IS_STATIC_UI} isShowcase={IS_SHOWCASE}
        resumeDisabledReason={resumeBlockReason} resumeBusy={busy && Boolean(resumeState)} resumeState={resumeState}
        resumeError={resumeError} resumeNotice={resumeNotice}
        onSelect={(id) => { setSelected(id); setPreviewOpen(false); setResumeError(""); setResumeNotice(""); }}
        onResume={resumeSavedSession} />
      <SelectedMapDetails selectedMap={selectedMap} isActive={selectedMapIsActive} isStatic={IS_STATIC_UI} isShowcase={IS_SHOWCASE}
        previewOpen={previewOpen} previewMap={previewMap} robot={IS_STATIC_UI ? undefined : rawRobot} fixtureRobotPose={IS_STATIC_UI ? undefined : fixtureRobotPose}
        localMapSyncStatus={localMapSyncStatus}
        canonicalRevision={mapRevision} activeLocalMapId={activeLocalMapId} activeLocalMapRevision={expectedLocalMapRevision}
        extent={selectedMapExtent} loadDisabled={loadDisabled} loadBlockReason={loadBlockReason}
        loadButtonLabel={IS_SHOWCASE ? "SELECT LOCAL PREVIEW" : selectedMapIsActive ? "RELOAD ACTIVE MAP" : "LOAD SAVED MAP"}
        operationState={operationState} busy={busy} loadError={loadError} loadNotice={loadNotice}
        onTogglePreview={() => setPreviewOpen((open) => !open)} onLoad={load} />
      <MapImportStager />
    </aside>
  </div>;
}
