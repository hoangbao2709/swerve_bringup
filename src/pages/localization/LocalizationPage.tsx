import { useEffect, useMemo, useRef, useState } from "react";
import { initializeLocalRobotPose } from "../../services/api";
import { IS_SHOWCASE, IS_STATIC_UI as STATIC_UI_SETTING } from "../../config";
import { displayRuntimeValue, STATIC_UI_EMPTY_VALUE } from "../../mock/staticPresentation";
import { useLocalAppRuntime } from "../../app/AppLayout";
import type { RobotDetailMapSnapshot, RobotState } from "../../schema/twin_state";
import { useStore } from "../../state/store";
import { displayedFramePose, type MapPoseIdentity } from "../../layout/robotPoseFrame";
import { decodeOccupancyGrid } from "../../components/shared/maps/occupancyGrid";
import { CurrentPoseCard, type CurrentPoseData } from "./components/CurrentPoseCard";
import { InitialPoseConfirmation } from "./components/InitialPoseConfirmation";
import { InitialPoseEditor } from "./components/InitialPoseEditor";
import { LocalizationMapWorkspace } from "./components/LocalizationMapWorkspace";
import { LocalizationStatusCard, type LocalizationStateLabel } from "./components/LocalizationStatusCard";
import { TfDiagnosticsPanel } from "./components/TfDiagnosticsPanel";
import { useLocalizationEventHistory } from "./hooks/useLocalizationEventHistory";
import { localizationMapIdentityKey, normalizeYawRadians, radiansToDisplayYaw,
  validateInitialPose, type AngleUnit, type InitialPose } from "./localizationModel";
import "./localization.css";

const IS_STATIC_UI = import.meta.env.DEV && STATIC_UI_SETTING;
const EMPTY_POSE: InitialPose = { x: Number.NaN, y: Number.NaN, yaw: Number.NaN };
type PoseDraft = { pose: InitialPose; mapKey: string; stage: "AWAITING_HEADING" | "READY" };
type AcceptedRequest = {
  pose: InitialPose;
  mapKey: string;
  mapId: string;
  mapRevision: string;
  localizationOwner: string;
  baselinePoseTimestamp: string;
  positionToleranceM: number;
  yawToleranceRad: number;
};
type ConfirmationContext = { pose: InitialPose; mapKey: string; mapId: string; mapRevision: string };

function resolveLocalizationMap(detail: ReturnType<typeof useStore.getState>["robotDetail"][string] | undefined,
  runtimeState: string): RobotDetailMapSnapshot | null {
  if (!detail) return null;
  const map = detail.runtimeMapSnapshot ?? null;
  const activeMapId = detail.activeLocalMapId;
  const activeMapRevision = detail.activeLocalMapRevision;
  if (activeMapId) {
    return map?.map_source === "LOCAL_MAP" && map.active_map_id === activeMapId
      && String(map.active_map_revision ?? "") === String(activeMapRevision ?? "") ? map : null;
  }
  return runtimeState === "MAPPING" || runtimeState === "UNIFIED" ? detail.slam2dMap ?? null : map;
}

function displayedPoseForMap(robot: RobotState | undefined, map: RobotDetailMapSnapshot | null,
  staticUi: boolean, now: number, showcase = false): CurrentPoseData | null {
  if (!robot || !map || robot.id !== map.robot_id) return null;
  let pose: RobotState["active_map_pose"] | undefined;
  if (staticUi) {
    const fixture = robot.active_map_pose;
    if (fixture?.valid !== true || fixture.pose_source !== "STATIC_UI_FIXTURE"
      || fixture.map_source !== "STATIC_UI_FIXTURE" || fixture.frame_id !== map.frame_id
      || fixture.map_id !== map.active_map_id
      || String(fixture.map_revision) !== String(map.active_map_revision ?? "")
      || ![fixture.x, fixture.y, fixture.yaw].every(Number.isFinite)) return null;
    pose = fixture;
  } else if (showcase) {
    const sample = robot.active_map_pose;
    if (sample?.valid !== true || sample.frame_id !== map.frame_id || sample.map_id !== map.active_map_id
      || String(sample.map_revision) !== String(map.active_map_revision ?? "")
      || ![sample.x, sample.y, sample.yaw].every(Number.isFinite)) return null;
    pose = sample;
  } else {
    const identity: MapPoseIdentity = {
      frame_id: map.frame_id, active_map_id: map.active_map_id,
      active_map_revision: map.active_map_revision, map_source: map.map_source,
      mapping_session_id: map.mapping_session_id,
    };
    pose = displayedFramePose(robot, identity);
  }
  if (!pose) return null;
  const parsedTimestamp = Date.parse(pose.timestamp);
  return {
    x: pose.x, y: pose.y, yaw: pose.yaw, frameId: pose.frame_id,
    mapId: pose.map_id, mapRevision: String(pose.map_revision),
    source: staticUi || showcase ? "SAMPLE DATA" : pose.pose_source,
    timestamp: pose.timestamp,
    ageSeconds: staticUi || showcase ? 0 : Math.max(0, (now - parsedTimestamp) / 1000),
  };
}

function statusForLocalization(input: {
  staticUi: boolean; online: boolean; robotIdVerified: boolean; map: RobotDetailMapSnapshot | null;
  capabilities: { localization_ready?: boolean; nav2_available?: boolean } | null;
  currentPose: CurrentPoseData | null; runtimeState: string; requestAccepted: boolean; requestConfirmed: boolean;
}): { state: LocalizationStateLabel; detail: string } {
  if (input.staticUi) return { state: "UNAVAILABLE", detail: "Live localization service unavailable." };
  if (!input.online) return { state: "UNAVAILABLE", detail: "Robot telemetry or the ROS bridge is offline." };
  if (!input.robotIdVerified) return { state: "UNAVAILABLE", detail: "The single-robot identity contract is not verified." };
  if (!input.map) return { state: "UNKNOWN", detail: "An active, identity-matched map snapshot is not available." };
  if (input.requestAccepted && !input.requestConfirmed) {
    return { state: "LOCALIZATION DEGRADED", detail: "The initial pose request was accepted; waiting for fresh runtime localization telemetry to confirm this map-frame pose." };
  }
  if (input.capabilities?.localization_ready === true && input.currentPose) {
    return { state: "LOCALIZED", detail: "Backend readiness and fresh map-matched pose telemetry confirm localization on the active map." };
  }
  if (input.capabilities?.localization_ready === true && !input.currentPose) {
    return { state: "LOCALIZATION DEGRADED", detail: "Backend reports localization ready, but a fresh identity-matched pose is not available." };
  }
  if (input.capabilities?.localization_ready === false && input.runtimeState === "NAVIGATION"
    && ["LOCAL_MAP", "NAV2_MAP"].includes(input.map.map_source ?? "")) {
    if (input.capabilities.nav2_available === false) return { state: "UNAVAILABLE", detail: "Nav2 localization is not available in the current runtime." };
    return { state: "INITIAL POSE REQUIRED", detail: "The active navigation map has no confirmed localization pose. Set a stopped robot's initial pose to begin the backend confirmation workflow." };
  }
  if (input.capabilities?.localization_ready === false) return { state: "LOCALIZATION DEGRADED", detail: "Runtime telemetry reports localization is not ready for the current map and mode." };
  return { state: "UNKNOWN", detail: "The backend has not reported an authoritative localization readiness state." };
}

function readableMapRevision(map: RobotDetailMapSnapshot | null): string {
  if (!map) return "UNKNOWN";
  const revision = map.active_map_revision ?? map.map_content_revision ?? map.map_version;
  return revision === null || revision === undefined ? "UNKNOWN" : String(revision);
}

function blockers(input: {
  staticUi: boolean; map: RobotDetailMapSnapshot | null; mapReady: boolean | undefined; runtimeState: string;
  robotOnline: boolean; controlOnline: boolean; controlMode: string; robot?: RobotState;
  estopState: string; nav2Available: boolean | undefined; nav2Ready: boolean | undefined;
}): string {
  if (input.staticUi) return "APPLY UNAVAILABLE IN STATIC UI";
  if (!input.map) return "WAITING FOR THE ACTIVE MAP SNAPSHOT";
  if (!input.map.active_map_id || !input.map.active_map_revision) return "ACTIVE MAP IDENTITY OR REVISION IS UNVERIFIED";
  if (!["LOCAL_MAP", "NAV2_MAP"].includes(input.map.map_source ?? "")) {
    return input.runtimeState === "MAPPING" || input.runtimeState === "UNIFIED"
      ? "SLAM TOOLBOX OWNS MAP → ODOM; STATIC-MAP INITIAL POSE IS UNAVAILABLE IN THIS MODE"
      : "INITIAL POSE REQUIRES A SAVED OR STATIC NAVIGATION MAP";
  }
  if (input.runtimeState !== "NAVIGATION") return "SWITCH THE RUNTIME TO NAVIGATION MODE";
  if (!input.robotOnline) return "ROBOT OR ROS BRIDGE IS OFFLINE";
  if (!input.controlOnline) return "SINGLE-ROBOT IDENTITY OR CONTROL LINK IS NOT VERIFIED";
  if (input.estopState !== "CLEAR") return input.estopState === "ACTIVE" ? "E-STOP IS ACTIVE" : "E-STOP STATE IS UNKNOWN; WAIT FOR FRESH TELEMETRY";
  if (input.controlMode !== "MANUAL" || input.robot?.control_mode !== "MANUAL") return "SWITCH TO MANUAL AND WAIT FOR APPLIED MODE TELEMETRY";
  if (input.mapReady !== true) return "BACKEND HAS NOT CONFIRMED THE ACTIVE MAP IS READY";
  if (input.nav2Available !== true) return "NAV2 LOCALIZATION SERVICE IS UNAVAILABLE";
  if (input.nav2Ready !== true) return "WAIT FOR NAV2 LOCALIZATION READINESS";
  const navigationState = String(input.robot?.navigation_state ?? "").toUpperCase();
  if (["EMERGENCY_STOPPED", "NAVIGATING", "ACTIVE", "PENDING", "PAUSED"].includes(navigationState)) {
    return "STOP THE ACTIVE NAVIGATION GOAL FIRST";
  }
  const velocities = [input.robot?.vx, input.robot?.vy, input.robot?.wz];
  if (!velocities.every((value) => typeof value === "number" && Number.isFinite(value))) return "WAIT FOR FRESH ODOMETRY VELOCITY";
  if (velocities.some((value) => Math.abs(value as number) > 0.05)) return "ROBOT MUST BE STOPPED (≤ 0.05 m/s AND rad/s)";
  return "";
}

function radiansEquivalent(a: number, b: number): number {
  return Math.atan2(Math.sin(a - b), Math.cos(a - b));
}

export function LocalizationPage() {
  const runtime = useLocalAppRuntime();
  const robotId = runtime.robotId;
  if (!robotId) return null;
  return <LocalizationWorkstation robotId={robotId} />;
}

function LocalizationWorkstation({ robotId }: { robotId: string }) {
  const runtime = useLocalAppRuntime();
  const robot = useStore((state) => state.twin?.robots?.[robotId]);
  const detail = useStore((state) => state.robotDetail[robotId]);
  const runtimeState = useStore((state) => state.runtimeState);
  const capabilities = useStore((state) => state.robotCapabilities[robotId] ?? null);
  const localizationMap = useMemo(() => resolveLocalizationMap(detail, runtimeState), [detail, runtimeState]);
  const mapKey = localizationMapIdentityKey(robotId, localizationMap);
  const mapId = localizationMap?.active_map_id ?? "UNKNOWN";
  const mapRevision = readableMapRevision(localizationMap);
  const scan = detail?.scan?.robot_id === robotId ? detail.scan : null;
  const diagnostics = runtime.diagnostics;
  const mappingDiagnostics = diagnostics?.mapping;
  const { events, record, clear } = useLocalizationEventHistory();
  const [clockNow, setClockNow] = useState(() => Date.now());
  const [occupancy, setOccupancy] = useState<{ mapKey: string; cells: Int8Array | null } | null>(null);
  const [draft, setDraft] = useState<PoseDraft | null>(null);
  const [pickMode, setPickMode] = useState(false);
  const [selectionResetVersion, setSelectionResetVersion] = useState(0);
  const [pointModeRequestVersion, setPointModeRequestVersion] = useState(0);
  const [angleUnit, setAngleUnit] = useState<AngleUnit>("DEG");
  const [requestPending, setRequestPending] = useState(false);
  const [requestAttempt, setRequestAttempt] = useState<{ mapKey: string; state: "PENDING" | "FAILED" } | null>(null);
  const [acceptedRequest, setAcceptedRequest] = useState<AcceptedRequest | null>(null);
  const [requestError, setRequestError] = useState("");
  const [confirmation, setConfirmation] = useState<ConfirmationContext | null>(null);
  const requestLatch = useRef(false);
  const previousMapKey = useRef(mapKey);
  const previousState = useRef<LocalizationStateLabel | null>(null);
  const previousPoseFresh = useRef<boolean | null>(null);

  useEffect(() => {
    if (!localizationMap) { setOccupancy(null); return; }
    let current = true;
    const key = localizationMapIdentityKey(robotId, localizationMap);
    setOccupancy(null);
    void decodeOccupancyGrid(localizationMap).then((cells) => {
      if (current) setOccupancy({ mapKey: key, cells });
    });
    return () => { current = false; };
  }, [localizationMap, robotId]);

  useEffect(() => {
    if (IS_STATIC_UI || !robot?.active_map_pose?.timestamp) return;
    const timer = window.setInterval(() => setClockNow(Date.now()), 1000);
    return () => window.clearInterval(timer);
  }, [robot?.active_map_pose?.timestamp]);

  const currentPose = useMemo(() => displayedPoseForMap(robot, localizationMap, IS_STATIC_UI, clockNow, IS_SHOWCASE),
    [clockNow, localizationMap, robot]);
  const cells = occupancy?.mapKey === mapKey ? occupancy.cells : null;
  const draftForMap = draft?.mapKey === mapKey ? draft : null;
  const pose = draftForMap?.pose ?? EMPTY_POSE;
  const hasDraft = Boolean(draftForMap);
  const poseFinite = [pose.x, pose.y, pose.yaw].every(Number.isFinite);
  const poseValidation = !draftForMap
    ? { valid: false as const, reason: "SELECT A MAP POSITION OR ENTER X, Y AND YAW" }
    : draftForMap.stage === "AWAITING_HEADING"
      ? { valid: false as const, reason: "CLICK A SECOND TIME TO CONFIRM THE HEADING" }
      : validateInitialPose(pose, localizationMap, cells);
  const mapIdentityVerified = Boolean(localizationMap?.active_map_id && localizationMap.active_map_revision
    && (!detail?.activeLocalMapId || (detail.activeLocalMapId === localizationMap.active_map_id
      && String(detail.activeLocalMapRevision ?? "") === String(localizationMap.active_map_revision))));
  const nav2Ready = capabilities?.nav2_ready === true && diagnostics?.nav2_ready === true;
  const gate = blockers({ staticUi: IS_STATIC_UI, map: mapIdentityVerified ? localizationMap : null,
    mapReady: capabilities?.map_ready, runtimeState, robotOnline: runtime.robotOnline,
    controlOnline: runtime.controlOnline, controlMode: runtime.controlMode, robot,
    estopState: runtime.estopState, nav2Available: capabilities?.nav2_available, nav2Ready });
  const acceptedForMap = acceptedRequest?.mapKey === mapKey ? acceptedRequest : null;
  const baselinePoseTimestamp = acceptedForMap ? Date.parse(acceptedForMap.baselinePoseTimestamp) : Number.NaN;
  const currentPoseTimestamp = currentPose ? Date.parse(currentPose.timestamp) : Number.NaN;
  const hasNewPoseSample = Number.isFinite(currentPoseTimestamp)
    && (Number.isFinite(baselinePoseTimestamp) ? currentPoseTimestamp > baselinePoseTimestamp : Boolean(currentPose));
  const requestConfirmed = Boolean(acceptedForMap && capabilities?.localization_ready === true && currentPose
    && currentPose.mapId === acceptedForMap.mapId && currentPose.mapRevision === acceptedForMap.mapRevision
    && hasNewPoseSample
    && Math.hypot(currentPose.x - acceptedForMap.pose.x, currentPose.y - acceptedForMap.pose.y) <= acceptedForMap.positionToleranceM
    && Math.abs(radiansEquivalent(currentPose.yaw, acceptedForMap.pose.yaw)) <= acceptedForMap.yawToleranceRad);
  const localizationStatus = statusForLocalization({ staticUi: IS_STATIC_UI, online: runtime.robotOnline,
    robotIdVerified: runtime.controlOnline, map: localizationMap, capabilities,
    currentPose, runtimeState, requestAccepted: Boolean(acceptedForMap), requestConfirmed });
  const requestState = IS_STATIC_UI ? STATIC_UI_EMPTY_VALUE
    : IS_SHOWCASE ? requestAttempt?.mapKey === mapKey && requestAttempt.state === "FAILED" ? "LOCAL SELECTION ONLY · NOT SENT" : "NO ROBOT REQUEST SENT"
    : requestAttempt?.mapKey === mapKey && requestAttempt.state === "PENDING" ? "REQUEST PENDING"
      : acceptedForMap ? requestConfirmed ? "LOCALIZATION TELEMETRY CONFIRMED"
        : runtime.robotOnline ? "REQUEST ACCEPTED · WAITING FOR LOCALIZATION CONFIRMATION"
          : "REQUEST ACCEPTED · WAITING FOR ROBOT TELEMETRY"
        : requestAttempt?.mapKey === mapKey && requestAttempt.state === "FAILED" ? "REQUEST NOT CONFIRMED"
          : acceptedRequest ? "REQUEST BELONGS TO A PREVIOUS MAP REVISION" : "NO REQUEST FOR ACTIVE MAP";
  const validationMessage = poseValidation.valid ? "" : poseValidation.reason;
  const isPoseTelemetryFresh = !IS_SHOWCASE && Boolean(currentPose && (IS_STATIC_UI || currentPose.ageSeconds <= 3));
  const poseSource = IS_STATIC_UI ? STATIC_UI_EMPTY_VALUE : currentPose?.source ?? robot?.active_map_pose?.pose_source ?? "UNKNOWN";
  const poseAge = IS_STATIC_UI || IS_SHOWCASE ? null : currentPose?.ageSeconds ?? null;
  const localizationOwner = IS_STATIC_UI ? STATIC_UI_EMPTY_VALUE : IS_SHOWCASE ? "NOT CONNECTED"
    : acceptedForMap?.localizationOwner.toUpperCase() ?? mappingDiagnostics?.map_odom_owner ?? "NOT REPORTED";

  useEffect(() => {
    if (previousMapKey.current && mapKey && previousMapKey.current !== mapKey) {
      record("MAP_REVISION_CHANGED", `Active map identity changed to ${mapId} revision ${mapRevision}; unsent pose draft cleared.`);
      setDraft(null);
      setConfirmation(null);
      setRequestError("");
      setSelectionResetVersion((version) => version + 1);
    }
    previousMapKey.current = mapKey;
  }, [mapId, mapKey, mapRevision, record]);

  useEffect(() => {
    if (previousState.current !== null && previousState.current !== localizationStatus.state) {
      record("LOCALIZATION_STATE_CHANGED", `${previousState.current} → ${localizationStatus.state}`);
    }
    previousState.current = localizationStatus.state;
  }, [localizationStatus.state, record]);

  useEffect(() => {
    if (previousPoseFresh.current !== null && previousPoseFresh.current !== isPoseTelemetryFresh) {
      if (isPoseTelemetryFresh) record("POSE_TELEMETRY_RECOVERED", "Fresh map-matched pose telemetry is available again.");
      else if (!IS_STATIC_UI && robot?.active_map_pose?.valid) record("POSE_TELEMETRY_STALE", "Fresh map-matched pose telemetry is no longer available.");
    }
    previousPoseFresh.current = isPoseTelemetryFresh;
  }, [isPoseTelemetryFresh, record, robot?.active_map_pose?.valid]);

  const updatePose = (key: keyof InitialPose, value: number) => {
    setDraft((previous) => ({ pose: { ...(previous?.mapKey === mapKey ? previous.pose : EMPTY_POSE), [key]: value },
      mapKey, stage: "READY" }));
    setRequestError("");
  };
  const resetSelection = () => setSelectionResetVersion((version) => version + 1);
  const setMapInteractionMode = (mode: "PAN" | "POINT") => {
    if (mode === "POINT") {
      setPickMode(true);
      setDraft(null);
      setRequestError("");
      resetSelection();
    } else {
      setPickMode(false);
      if (draftForMap?.stage === "AWAITING_HEADING") setDraft(null);
    }
  };
  const pickAgain = () => {
    setDraft(null);
    setPickMode(true);
    setPointModeRequestVersion((version) => version + 1);
    resetSelection();
  };
  const cancelMapSelection = () => {
    if (draftForMap) record("POSE_SELECTED", "Map pose selection was cleared before submission.");
    setDraft(null);
    resetSelection();
  };
  const onPickPosition = (point: Pick<InitialPose, "x" | "y">, defaultYaw?: number) => {
    const yaw = Number.isFinite(defaultYaw) ? Number(defaultYaw) : currentPose?.yaw ?? 0;
    setDraft({ pose: { ...point, yaw }, mapKey, stage: "AWAITING_HEADING" });
    setRequestError("");
    record("POSE_SELECTED", `Map position selected at x=${point.x.toFixed(3)} m, y=${point.y.toFixed(3)} m; awaiting heading click.`);
  };
  const onConfirmHeading = (point: Pick<InitialPose, "x" | "y">, yaw: number) => {
    setDraft({ pose: { ...point, yaw: normalizeYawRadians(yaw) }, mapKey, stage: "READY" });
    record("POSE_DIRECTION_CONFIRMED", `Map heading confirmed at ${radiansToDisplayYaw(yaw, angleUnit).toFixed(1)} ${angleUnit === "DEG" ? "deg" : "rad"}.`);
  };

  const openConfirmation = () => {
    if (!poseValidation.valid || (gate && !IS_SHOWCASE) || !localizationMap?.active_map_id || !localizationMap.active_map_revision) return;
    setRequestError("");
    setConfirmation({ pose: { ...pose }, mapKey, mapId: localizationMap.active_map_id,
      mapRevision: String(localizationMap.active_map_revision) });
  };

  const confirmInitialPose = async () => {
    if (!confirmation || requestLatch.current || IS_STATIC_UI) return;
    const currentState = useStore.getState();
    const latestDetail = currentState.robotDetail[robotId];
    const latestMap = resolveLocalizationMap(latestDetail, currentState.runtimeState);
    const latestKey = localizationMapIdentityKey(robotId, latestMap);
    if (latestKey !== confirmation.mapKey || latestMap?.active_map_id !== confirmation.mapId
      || String(latestMap?.active_map_revision ?? "") !== confirmation.mapRevision) {
      setRequestError("ACTIVE MAP IDENTITY CHANGED. RESELECT THE POSE ON THE CURRENT MAP.");
      setRequestAttempt({ mapKey: confirmation.mapKey, state: "FAILED" });
      record("POSE_REQUEST_REJECTED", "Request blocked in the browser because active map identity changed before submission.");
      return;
    }
    if (IS_SHOWCASE) {
      setRequestAttempt({ mapKey: confirmation.mapKey, state: "FAILED" });
      setRequestError("No robot localization request was sent.");
      setConfirmation(null);
      record("POSE_REQUEST_NOT_SENT", `Local pose selection ${confirmation.pose.x.toFixed(3)}, ${confirmation.pose.y.toFixed(3)} on sample map ${confirmation.mapId} was not sent to a robot.`);
      return;
    }
    requestLatch.current = true;
    setRequestPending(true);
    setRequestAttempt({ mapKey: confirmation.mapKey, state: "PENDING" });
    setRequestError("");
    record("POSE_REQUEST_SENT", `Initial pose request sent for map ${confirmation.mapId} revision ${confirmation.mapRevision}.`);
    const baselinePoseTimestamp = currentPose?.timestamp ?? "";
    try {
      const result = await initializeLocalRobotPose(robotId, {
        ...confirmation.pose, frame_id: "map", active_map_id: confirmation.mapId,
        active_map_revision: confirmation.mapRevision,
      });
      if (!result.ok || result.robot_id !== robotId || result.active_map_id !== confirmation.mapId
        || String(result.active_map_revision) !== confirmation.mapRevision) {
        throw new Error("Backend response did not confirm the requested robot and active map identity.");
      }
      if (!Number.isFinite(result.position_tolerance_m) || result.position_tolerance_m <= 0
        || result.position_tolerance_m > 0.05
        || !Number.isFinite(result.yaw_tolerance_rad) || result.yaw_tolerance_rad <= 0
        || result.yaw_tolerance_rad > 0.05) {
        throw new Error("Backend response omitted valid localization confirmation tolerances.");
      }
      setAcceptedRequest({ pose: confirmation.pose, mapKey: confirmation.mapKey,
        mapId: confirmation.mapId, mapRevision: confirmation.mapRevision,
        localizationOwner: result.localization_owner, baselinePoseTimestamp,
        positionToleranceM: result.position_tolerance_m, yawToleranceRad: result.yaw_tolerance_rad });
      setRequestAttempt(null);
      setConfirmation(null);
      record("POSE_REQUEST_ACCEPTED", `Backend accepted the pose through ${result.localization_owner} on map ${confirmation.mapId} revision ${confirmation.mapRevision}; waiting for fresh UI telemetry.`);
    } catch (caught) {
      const message = caught instanceof Error ? caught.message : "Initial pose request failed or timed out.";
      setRequestError(message);
      setRequestAttempt({ mapKey: confirmation.mapKey, state: "FAILED" });
      record("POSE_REQUEST_REJECTED", `Initial pose request was rejected or not confirmed: ${message}`);
    } finally {
      requestLatch.current = false;
      setRequestPending(false);
    }
  };

  const selectedCurrentPose = currentPose ? { x: currentPose.x, y: currentPose.y, yaw: currentPose.yaw } : null;
  const blocker = gate || (poseFinite && !poseValidation.valid ? poseValidation.reason : "");
  return <main className="localization-workstation flex min-h-0 min-w-0 flex-1 overflow-hidden bg-ptagv-page p-hmi-2 max-[1279px]:overflow-y-auto">
    <div className="grid min-h-0 min-w-0 flex-1 grid-cols-1 gap-hmi-2 xl:grid-cols-[minmax(0,2.1fr)_minmax(20rem,1fr)]">
      <LocalizationMapWorkspace robot={IS_STATIC_UI ? undefined : robot} map={localizationMap} sample={IS_STATIC_UI} waitingForSavedMap={Boolean(detail?.activeLocalMapId)} scan={IS_STATIC_UI ? null : scan}
        pose={pose} showPose={poseFinite} pickMode={pickMode} selectionResetVersion={selectionResetVersion}
        pointModeRequestVersion={pointModeRequestVersion}
        onPickModeChange={setMapInteractionMode} onPickPosition={onPickPosition} onConfirmHeading={onConfirmHeading}
        onCancelSelection={cancelMapSelection} />
      <aside className="localization-control-panel min-h-0 min-w-0 space-y-hmi-2 overflow-y-auto pr-1 max-[1279px]:overflow-visible" aria-label="Localization control panel">
      <LocalizationStatusCard state={localizationStatus.state} detail={localizationStatus.detail}
          sampleData={IS_SHOWCASE}
          sample={IS_STATIC_UI} runtimeMode={runtimeState} mapId={mapId} mapRevision={mapRevision}
          requestState={requestState} estopState={runtime.estopState} />
        <CurrentPoseCard pose={IS_STATIC_UI ? null : currentPose} sample={IS_STATIC_UI} sampleData={IS_SHOWCASE} />
        <InitialPoseEditor pose={pose} hasDraft={hasDraft} mapId={displayRuntimeValue(IS_STATIC_UI, mapId)} mapRevision={displayRuntimeValue(IS_STATIC_UI, mapRevision)}
          angleUnit={angleUnit} validationMessage={validationMessage} blocker={IS_STATIC_UI || IS_SHOWCASE ? "" : blocker}
          requestState={requestState} requestPending={requestPending} applyDisabled={IS_STATIC_UI || (!IS_SHOWCASE && Boolean(gate))}
          canResetToCurrentPose={!IS_STATIC_UI && Boolean(selectedCurrentPose)}
          resetPoseLabel="RESET TO CURRENT POSE"
          onAngleUnitChange={setAngleUnit} onChange={updatePose}
          onRotate={(delta) => updatePose("yaw", normalizeYawRadians(pose.yaw + delta))}
          onResetToCurrentPose={() => selectedCurrentPose && setDraft({ pose: selectedCurrentPose, mapKey, stage: "READY" })}
          onClear={() => { setDraft(null); resetSelection(); setRequestError(""); }}
          onPickAgain={pickAgain} onOpenConfirmation={openConfirmation} />
        <TfDiagnosticsPanel diagnostics={diagnostics} mapping={mappingDiagnostics} staticUi={IS_STATIC_UI}
          nav2Ready={IS_STATIC_UI ? false : capabilities?.nav2_ready === undefined ? diagnostics?.nav2_ready : nav2Ready}
          localizationOwner={localizationOwner} poseSource={poseSource} poseAge={poseAge}
          events={events} onClearHistory={clear} />
      </aside>
    </div>
    <InitialPoseConfirmation open={Boolean(confirmation)} pose={confirmation?.pose ?? EMPTY_POSE}
      mapId={confirmation?.mapId ?? mapId} mapRevision={confirmation?.mapRevision ?? mapRevision}
      pending={requestPending} error={requestError} showcase={IS_SHOWCASE} onCancel={() => !requestPending && setConfirmation(null)}
      onConfirm={() => { void confirmInitialPose(); }} />
  </main>;
}
