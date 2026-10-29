import { MetricRow } from "../../../components/shared/ui/HmiPrimitives";
import { valueNumber, valueText } from "../../../components/shared/ui/RobotPagePrimitives";
import { Alert, Badge, Button, Card, CardBody, CardHeader, EmptyState } from "../../../components/shared/ui/HmiPrimitives";
import { PosePickerMap } from "../../../components/shared/maps/LocalMapViews";
import type { FramePose, RobotDetailMapSnapshot, RobotState } from "../../../schema/twin_state";
import type { LocalRobotMap } from "../../../services/api";
import { displayFixtureMapName, displayRuntimeValue, STATIC_UI_EMPTY_VALUE } from "../../../mock/staticPresentation";
import { useState } from "react";
import { useFocusTrap } from "../../../components/ui/useFocusTrap";

type Props = {
  selectedMap: LocalRobotMap | null;
  isActive: boolean;
  isStatic: boolean;
  isShowcase?: boolean;
  previewOpen: boolean;
  previewMap: RobotDetailMapSnapshot | null;
  robot?: RobotState;
  fixtureRobotPose?: FramePose;
  localMapSyncStatus: string | null;
  canonicalRevision: string | number | null;
  activeLocalMapId: string | null;
  activeLocalMapRevision: string | null;
  extent: string;
  loadDisabled: boolean;
  loadBlockReason: string;
  loadButtonLabel: string;
  operationState: string;
  busy: boolean;
  loadError: string;
  loadNotice: string;
  onTogglePreview: () => void;
  onLoad: () => void;
};

export function SelectedMapDetails({ selectedMap, isActive, isStatic, isShowcase = false, previewOpen, previewMap, robot, fixtureRobotPose, localMapSyncStatus, canonicalRevision, activeLocalMapId, activeLocalMapRevision, extent, loadDisabled, loadBlockReason, loadButtonLabel, operationState, busy, loadError, loadNotice, onTogglePreview, onLoad }: Props) {
  const [confirmingLoad, setConfirmingLoad] = useState(false);
  const dialogRef = useFocusTrap<HTMLDivElement>(confirmingLoad);
  const loadMessage = busy ? operationState : isStatic ? ""
    : loadError || loadNotice || loadBlockReason || (isShowcase ? "LOCAL PREVIEW ONLY · ROBOT MAP UNCHANGED" : isActive ? "SELECTED MAP IS ACTIVE" : "READY · MANUAL MODE AND STOP REQUIRED");
  return <Card className="mapping-selected-card shrink-0 overflow-hidden @container/mapping" aria-labelledby="mapping-selected-title">
    <CardHeader>
      <div>
        <h2 className="m-0 text-hmi-title font-bold text-ptagv-text" id="mapping-selected-title">Map Details / Load</h2>
      </div>
      {selectedMap && !isStatic && <Badge className={`mapping-map-status${isActive ? " is-active" : ""}`} tone={isActive ? "success" : "neutral"}>{isShowcase ? "SAMPLE" : isActive ? "ACTIVE" : "STORED"}</Badge>}
    </CardHeader>
    <CardBody className="mapping-selected-body gap-hmi-3 px-hmi-3 py-hmi-3">
      {selectedMap ? <>
        <div className="hmi-map-detail-state flex min-h-9 min-w-0 flex-wrap items-center justify-between gap-2 border-b border-ptagv-border-soft pb-hmi-2">
          <b className="min-w-0 text-sm leading-snug text-ptagv-text [overflow-wrap:anywhere]">{displayFixtureMapName(isStatic, selectedMap.name)}</b>
          {isActive && !isStatic && <Badge className="local-map-active-badge" tone="success">ACTIVE MAP</Badge>}
        </div>
        <div className="hmi-map-details-grid grid min-w-0 grid-cols-1 gap-x-hmi-4 @lg/mapping:grid-cols-2">
          <MetricRow label="MAP ID" value={isStatic ? STATIC_UI_EMPTY_VALUE : valueText(selectedMap.map_id ?? selectedMap.id)} mono />
          <MetricRow label="REVISION" value={isStatic ? STATIC_UI_EMPTY_VALUE : valueText(selectedMap.revision)} mono />
          <MetricRow label="RESOLUTION" value={valueNumber(selectedMap.resolution, 3, " m/cell")} mono />
          <MetricRow label="SIZE" value={selectedMap.width && selectedMap.height ? `${selectedMap.width} × ${selectedMap.height}` : "N/A"} mono />
          <MetricRow label="EXTENT" value={extent} mono />
          <MetricRow label={isShowcase ? "SCOPE" : "SYNC"} value={isShowcase ? "BROWSER ONLY" : displayRuntimeValue(isStatic, isActive ? localMapSyncStatus ?? "UNVERIFIED" : "NOT ACTIVE")} />
          <MetricRow label="EXPLORED AREA" value={valueNumber(selectedMap.explored_area_m2, 2, " m²")} mono />
          <MetricRow label="CANONICAL REVISION" value={isStatic ? STATIC_UI_EMPTY_VALUE : isShowcase ? "NOT CONNECTED" : valueText(canonicalRevision)} mono />
          <MetricRow label={isShowcase ? "ROBOT MAP" : "ACTIVE LOCAL MAP"} value={isStatic ? STATIC_UI_EMPTY_VALUE : isShowcase ? "UNCHANGED" : activeLocalMapId ?? "NONE"} mono />
          <MetricRow label="ACTIVE REVISION" value={isStatic ? STATIC_UI_EMPTY_VALUE : isShowcase ? "NOT CONNECTED" : valueText(activeLocalMapRevision)} mono />
        </div>
        <details className="min-w-0 border-t border-ptagv-border-soft">
          <summary className="cursor-pointer py-hmi-2 text-xs font-bold text-ptagv-muted">ADDITIONAL MAP METADATA</summary>
          <div className="hmi-map-details-grid grid min-w-0 grid-cols-1 gap-x-hmi-4 @lg/mapping:grid-cols-2">
            <MetricRow label="FRAME" value={selectedMap.frame_id} mono />
            <MetricRow label="ORIGIN" value={selectedMap.origin?.slice(0, 3).map((part) => valueNumber(part, 3)).join(", ") ?? "N/A"} mono />
            <MetricRow label="CREATED" value={selectedMap.created_at} />
            <MetricRow label="SLAM SESSION" value={displayRuntimeValue(isStatic, selectedMap.slam_session_state?.status ?? "NOT AVAILABLE")} />
            <MetricRow label="KNOWN / FREE / OCCUPIED" value={`${selectedMap.known_cells ?? "N/A"} / ${selectedMap.free_cells ?? "N/A"} / ${selectedMap.occupied_cells ?? "N/A"}`} mono />
          </div>
        </details>
      </> : <EmptyState className="min-h-16 p-hmi-3">Select a saved map to inspect its details.</EmptyState>}
      <div className="mapping-selected-actions grid min-w-0 grid-cols-1 gap-hmi-2 sm:grid-cols-2">
        <Button aria-expanded={previewOpen} aria-controls="mapping-map-preview"
          disabled={!selectedMap} onClick={onTogglePreview}>{previewOpen ? "HIDE PREVIEW" : "PREVIEW MAP"}</Button>
        <Button className="hmi-map-load-button" tone="primary" aria-describedby={isStatic ? undefined : "saved-map-load-state"}
          disabled={loadDisabled} onClick={() => setConfirmingLoad(true)}>{busy ? operationState : loadButtonLabel}</Button>
      </div>
      {!isStatic && <Alert id="saved-map-load-state" className="mapping-load-message" tone={loadError ? "error" : loadNotice ? "success" : loadBlockReason ? "warning" : "neutral"}
        role={loadError ? "alert" : "status"}>{loadMessage}</Alert>
      }
      {previewOpen && selectedMap && <div className="hmi-mapping-preview mapping-preview-drawer min-w-0 border-t border-ptagv-border-soft pt-hmi-3" id="mapping-map-preview">
        <div className="mapping-preview-heading mb-hmi-2 font-mono text-[0.65rem] font-bold tracking-wide text-ptagv-muted">{isStatic || isShowcase ? "MAP PREVIEW" : "CONFIRMED ACTIVE MAP PREVIEW"}</div>
        {previewMap ? <PosePickerMap map={previewMap} robot={robot} fixtureRobotPose={fixtureRobotPose} pose={{ x: 0, y: 0, yaw: 0 }} active={false} onPick={() => undefined}
          ariaLabel={isStatic ? "Saved map preview" : `Active saved map preview ${previewMap.active_map_id}`} />
          : <EmptyState className="min-h-20 p-hmi-3">{isShowcase ? "Sample occupancy data is not available for this map." : isActive ? "Waiting for the confirmed active saved /map snapshot." : "This stored map has no occupancy snapshot. Load it through the supervised transition to receive its confirmed preview."}</EmptyState>}
      </div>}
    </CardBody>
    {confirmingLoad && selectedMap && !isStatic && <div className="fixed inset-0 z-50 grid place-items-center bg-slate-950/45 p-hmi-3" role="presentation">
      <div ref={dialogRef} role="dialog" aria-modal="true" aria-labelledby="confirm-saved-map-title" tabIndex={-1}
        className="grid w-full max-w-lg gap-hmi-3 border border-ptagv-border bg-white p-hmi-4 shadow-xl">
        <h3 id="confirm-saved-map-title" className="m-0 text-base font-bold text-ptagv-text">{isShowcase ? "Select map preview?" : "Apply saved map?"}</h3>
        <p className="m-0 text-sm leading-relaxed text-ptagv-muted">{isShowcase
          ? <>Show <b className="text-ptagv-text">{selectedMap.name}</b> as a local preview. The robot map is unchanged.</>
          : <>Load <b className="text-ptagv-text">{selectedMap.name}</b> (revision {selectedMap.revision}) through the supervised robot map transition.</>}</p>
        {!isShowcase && <p className="m-0 text-sm leading-relaxed text-ptagv-muted">The active map will change only after the backend confirms its identity and revision. Robot motion will not resume automatically.</p>}
        {loadBlockReason && <Alert tone="warning" role="note">{loadBlockReason}</Alert>}
        <div className="flex flex-wrap justify-end gap-2">
          <Button onClick={() => setConfirmingLoad(false)}>CANCEL</Button>
          <Button tone="primary" disabled={loadDisabled || busy} onClick={() => { setConfirmingLoad(false); onLoad(); }}>{isShowcase ? "SHOW PREVIEW" : "CONFIRM LOAD MAP"}</Button>
        </div>
      </div>
    </div>}
  </Card>;
}
