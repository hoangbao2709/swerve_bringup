import { useState } from "react";
import { useStore, type ViewTab, type SceneTool, type LabelLayer, type LabelLayers } from "../../state/store";
import { Icon } from "../ui/primitives";
import { Scene3D } from "../scene/Scene3D";
import { MapView2D } from "./MapView2D";

const TABS: Array<[ViewTab, string]> = [["3D", "3D VIEW"], ["MAP", "MAP VIEW"], ["TRAFFIC", "TRAFFIC VIEW"], ["HEATMAP", "HEATMAP"]];

export function Viewport({ onFullscreenChange }: { onFullscreenChange?: (active: boolean) => void } = {}) {
  const [settingsOpen, setSettingsOpen] = useState(false);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [measurePoints, setMeasurePoints] = useState<Array<[number, number, number]>>([]);
  const tab = useStore((s) => s.viewTab);
  const setTab = useStore((s) => s.setViewTab);
  const tool = useStore((s) => s.tool);
  const setTool = useStore((s) => s.setTool);
  const showPaths = useStore((s) => s.showPaths);
  const showLabels = useStore((s) => s.showLabels);
  const togglePaths = useStore((s) => s.togglePaths);
  const toggleLabels = useStore((s) => s.toggleLabels);
  const showLights = useStore((s) => s.showLights);
  const toggleLights = useStore((s) => s.toggleLights);
  const showCameras = useStore((s) => s.showCameras);
  const toggleCameras = useStore((s) => s.toggleCameras);
  const labelLayers = useStore((s) => s.labelLayers);
  const setLabelLayerVisible = useStore((s) => s.setLabelLayerVisible);
  const setLabelLayerZIndex = useStore((s) => s.setLabelLayerZIndex);
  const resetLabelLayers = useStore((s) => s.resetLabelLayers);
  const focus = useStore((s) => s.focus);
  const activeFloor = useStore((s) => s.activeFloor);
  const setActiveFloor = useStore((s) => s.setActiveFloor);
  const chooseMeasurePoint = (point: [number, number, number]) => {
    setMeasurePoints((current) => current.length === 2 ? [point] : [...current, point]);
  };
  function toggleFullscreen() {
    setIsFullscreen((value) => {
      const next = !value;
      onFullscreenChange?.(next);
      return next;
    });
  }
  const toolBtn = (t: SceneTool, icon: JSX.Element, title: string, on = tool === t, onClick = () => setTool(t)) => (
    <button className={on ? "on" : ""} title={title} onClick={onClick}>{icon}</button>
  );
  return (
    <div className={"viewport" + (isFullscreen ? " viewport-fullscreen" : "")}>
      <div className="view-tabs">{TABS.map(([k, l]) => <button key={k} className={tab === k ? "on" : ""} onClick={() => setTab(k)}>{l}</button>)}</div>
      <div className="vp-toolbar">
        <select className="sel" title="Floor" value={String(activeFloor)} onChange={(e) => { const v = e.target.value === "all" || e.target.value === "exploded" ? e.target.value as "all" | "exploded" : Number(e.target.value); setActiveFloor(v); if (v === 2) focus([31, 8, 51]); else focus([50, 0, 31]); }}>
          <option value="all">All floors</option>
          <option value="1">Floor 1</option>
          <option value="2">Floor 2</option>
          <option value="exploded">Exploded</option>
        </select>
        <button className="icon-btn" title={isFullscreen ? "Exit full screen" : "View full screen"} aria-label={isFullscreen ? "Exit full screen" : "View full screen"} onClick={() => void toggleFullscreen()}>{Icon.expand}</button>
        <button className={"icon-btn" + (settingsOpen ? " on" : "")} title="Label / viewport settings" onClick={() => setSettingsOpen((v) => !v)}>{Icon.gear}</button>
      </div>
      {settingsOpen && tab === "3D" && (
        <LabelSettingsPanel
          showLabels={showLabels}
          onToggleAll={toggleLabels}
          layers={labelLayers}
          onVisible={setLabelLayerVisible}
          onZIndex={setLabelLayerZIndex}
          onReset={resetLabelLayers}
          showLights={showLights}
          onToggleLights={toggleLights}
          showCameras={showCameras}
          onToggleCameras={toggleCameras}
          onClose={() => setSettingsOpen(false)}
        />
      )}
      {tab === "3D" ? <Scene3D measurePoints={measurePoints} onMeasurePoint={chooseMeasurePoint} /> : <MapView2D mode={tab} />}
      {tab === "3D" && (
        <div className="scene-toolbar">
          {toolBtn("select", Icon.cursor, "Select robot / shelf")}
          {toolBtn("pan", Icon.hand, "Pan (left-drag)")}
          {toolBtn("paths", Icon.path, "Toggle paths", showPaths, togglePaths)}
          {toolBtn("labels", Icon.tag, "Toggle labels", showLabels, toggleLabels)}
          {toolBtn("measure", Icon.ruler, "Measure distance", tool === "measure", () => { setTool("measure"); setMeasurePoints([]); })}
          {tool === "measure" && <span className="measure-hint"></span>}
          {tool === "measure" && measurePoints.length > 0 && <button className="measure-clear" title="Clear measurement" onClick={() => setMeasurePoints([])}>×</button>}
        </div>
      )}
    </div>
  );
}

const LABEL_LAYER_ROWS: Array<[LabelLayer, string, string]> = [
  ["zones", "Zone", "ZONE A / B / C / D / M"],
  ["workpoints", "Dock / Station", "INBOUND / OUTBOUND / PACKING"],
  ["lifts", "Lift", "LIFT-1 / LIFT-2"],
  ["lift1", "LIFT-1", "Bật/tắt label riêng cho LIFT-1"],
  ["lift2", "LIFT-2", "Bật/tắt label riêng cho LIFT-2"],
  ["rackLoads", "Shelf load", "% trên từng shelf"],
  ["robots", "Robot", "Robot ID / status"],
  ["people", "Human", "Human warning"],
];

function LabelSettingsPanel({
  showLabels, onToggleAll, layers, onVisible, onZIndex, onReset, showLights, onToggleLights, showCameras, onToggleCameras, onClose,
}: {
  showLabels: boolean;
  onToggleAll: () => void;
  layers: LabelLayers;
  onVisible: (layer: LabelLayer, visible: boolean) => void;
  onZIndex: (layer: LabelLayer, zIndex: number) => void;
  onReset: () => void;
  showLights: boolean;
  onToggleLights: () => void;
  showCameras: boolean;
  onToggleCameras: () => void;
  onClose: () => void;
}) {
  return (
    <div className="label-settings-panel">
      <div className="label-settings-head">
        <div><b>3D LABEL LAYERS</b><small>Bật/tắt riêng và chỉnh thứ tự z-index</small></div>
        <button type="button" onClick={onClose} aria-label="Close label settings">×</button>
      </div>
      <div className="label-master-row">
        <label><input type="checkbox" checked={showLabels} onChange={onToggleAll} /> <span>Hiện tất cả label</span></label>
        <button type="button" onClick={onReset}>Reset</button>
      </div>
      <div className="label-master-row scene-visibility-row">
        <label><input type="checkbox" checked={showLights} onChange={onToggleLights} /> <span>Hiện đèn 3D</span></label>
        <label><input type="checkbox" checked={showCameras} onChange={onToggleCameras} /> <span>Hiện camera</span></label>
      </div>
      <div className={"label-layer-list" + (!showLabels ? " disabled" : "")}>
        {LABEL_LAYER_ROWS.map(([key, title, hint]) => {
          const layer = layers[key];
          return (
            <div className="label-layer-row" key={key}>
              <label className="label-layer-toggle" title={hint}>
                <input type="checkbox" checked={layer.visible} disabled={!showLabels} onChange={(e) => onVisible(key, e.target.checked)} />
                <span><b>{title}</b><small>{hint}</small></span>
              </label>
              <div className="label-z-control" title="1 = phía sau, 9 = phía trước. UI panel luôn nằm trên label.">
                <span>Z</span>
                <input type="range" min="1" max="9" step="1" value={layer.zIndex} disabled={!showLabels || !layer.visible} onChange={(e) => onZIndex(key, Number(e.target.value))} />
                <input type="number" min="1" max="9" step="1" value={layer.zIndex} disabled={!showLabels || !layer.visible} onChange={(e) => onZIndex(key, Number(e.target.value))} />
              </div>
            </div>
          );
        })}
      </div>
      <div className="label-settings-note">Z-index 1–9: số lớn nằm phía trước label khác. Panel Shelf Inventory và toolbar luôn được giữ trên scene label.</div>
    </div>
  );
}
