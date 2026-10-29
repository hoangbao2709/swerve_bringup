import { useEffect, useRef, useState, type ChangeEvent } from "react";
import { Alert, Button, Card, CardBody, CardHeader, SectionHeading } from "../../components/shared/ui/HmiPrimitives";
import { validateRosMapFiles, type ValidatedRosMapPair } from "./mapImportValidation";
import { IS_SHOWCASE } from "../../config";

function formatSize(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
}

function MapImagePreview({ map }: { map: ValidatedRosMapPair }) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  useEffect(() => {
    const canvas = canvasRef.current;
    const context = canvas?.getContext("2d");
    if (!canvas || !context) return;
    const scale = Math.min(1, 256 / Math.max(map.width, map.height));
    const width = Math.max(1, Math.round(map.width * scale));
    const height = Math.max(1, Math.round(map.height * scale));
    canvas.width = width;
    canvas.height = height;
    const image = context.createImageData(width, height);
    for (let y = 0; y < height; y += 1) {
      const sourceY = Math.min(map.height - 1, Math.floor((y + 0.5) * map.height / height));
      for (let x = 0; x < width; x += 1) {
        const sourceX = Math.min(map.width - 1, Math.floor((x + 0.5) * map.width / width));
        const gray = map.pixels[sourceY * map.width + sourceX];
        const offset = (y * width + x) * 4;
        image.data[offset] = gray;
        image.data[offset + 1] = gray;
        image.data[offset + 2] = gray;
        image.data[offset + 3] = 255;
      }
    }
    context.putImageData(image, 0, 0);
  }, [map]);

  return <canvas ref={canvasRef} role="img" aria-label={`Local PGM map preview, ${map.width} by ${map.height} cells`}
    className="max-h-56 max-w-full border border-ptagv-border bg-slate-200 [image-rendering:pixelated]" />;
}

export function MapImportStager() {
  const [candidate, setCandidate] = useState<ValidatedRosMapPair | null>(null);
  const [fileNames, setFileNames] = useState<string[]>([]);
  const [validationError, setValidationError] = useState("");
  const [validating, setValidating] = useState(false);
  const [staged, setStaged] = useState(false);
  const validationRun = useRef(0);

  useEffect(() => () => { validationRun.current += 1; }, []);

  const selectFiles = async (event: ChangeEvent<HTMLInputElement>) => {
    const files = Array.from(event.currentTarget.files ?? []);
    event.currentTarget.value = "";
    const run = ++validationRun.current;
    setCandidate(null);
    setStaged(false);
    setValidationError("");
    setFileNames(files.map((file) => file.name));
    if (!files.length) { setValidating(false); return; }
    setValidating(true);
    const result = await validateRosMapFiles(files);
    if (run !== validationRun.current) return;
    if (result.ok) setCandidate(result.map);
    else setValidationError(result.error);
    setValidating(false);
  };

  const clear = () => {
    validationRun.current += 1;
    setCandidate(null);
    setFileNames([]);
    setValidationError("");
    setStaged(false);
    setValidating(false);
  };

  return <Card className="min-w-0 overflow-hidden" aria-labelledby="map-import-title">
    <CardHeader><SectionHeading id="map-import-title" className="text-hmi-kicker uppercase tracking-wider">APPLY NEW MAP</SectionHeading></CardHeader>
    <CardBody className="gap-hmi-2">
      <p className="m-0 text-xs leading-relaxed text-ptagv-muted">Validate a YAML + PGM map pair for local review.</p>
      <label className="grid gap-1 text-xs font-bold text-ptagv-muted">SELECT YAML + PGM FILES
        <input type="file" multiple accept=".yaml,.pgm" aria-label="Select ROS map YAML and PGM files"
          onChange={(event) => { void selectFiles(event); }}
          className="min-h-10 w-full min-w-0 border border-ptagv-border bg-white px-2 py-1.5 text-xs file:mr-2 file:min-h-8 file:border-0 file:bg-slate-100 file:px-2 file:font-bold focus-visible:outline-2 focus-visible:outline-ptagv-blue" />
      </label>
      {fileNames.length > 0 && <div className="grid gap-1 border border-ptagv-border-soft bg-slate-50 p-2">
        <span className="text-[0.62rem] font-bold tracking-wide text-ptagv-muted">SELECTED FILES</span>
        {fileNames.map((name) => <span key={name} className="break-all font-mono text-xs text-ptagv-text">{name}</span>)}
      </div>}
      {validating && <p className="m-0 text-sm text-ptagv-muted" role="status">Validating map metadata and image…</p>}
      {validationError && <Alert tone="error" role="alert">{validationError}</Alert>}
      {candidate && <div className="grid min-w-0 gap-2 border border-ptagv-border-soft p-2">
        <div className="flex flex-wrap items-center justify-between gap-2">
          <b className="text-xs text-ptagv-success">MAP PAIR VALIDATED</b>
          <span className="text-xs text-ptagv-muted">{formatSize(candidate.totalBytes)} total</span>
        </div>
        <div className="grid min-w-0 grid-cols-2 gap-x-3 text-xs">
          <span className="text-ptagv-muted">Format</span><b>YAML + PGM</b>
          <span className="text-ptagv-muted">Dimensions</span><b>{candidate.width} × {candidate.height} cells</b>
          <span className="text-ptagv-muted">Resolution</span><b>{candidate.resolution} m/cell</b>
          <span className="text-ptagv-muted">Origin</span><b className="break-all">{candidate.origin.join(", ")}</b>
        </div>
        <div className="grid justify-items-start gap-1">
          <span className="text-[0.62rem] font-bold tracking-wide text-ptagv-muted">LOCAL IMAGE PREVIEW</span>
          <MapImagePreview map={candidate} />
        </div>
        {!staged
          ? <Button tone="primary" onClick={() => setStaged(true)}>STAGE FILES FOR REVIEW</Button>
          : <Alert tone="info" role="status">Files staged in this browser. Robot map unchanged.</Alert>}
      </div>}
      <div className="flex flex-wrap gap-2 border-t border-ptagv-border-soft pt-hmi-2">
        <Button disabled={!candidate && fileNames.length === 0 && !validationError && !validating} onClick={clear}>CLEAR STAGING</Button>
        <Button tone="primary" disabled title={IS_SHOWCASE ? "Requires a connected robot runtime." : "This installation has no map-import API"}>IMPORT TO ROBOT</Button>
      </div>
      <small className="text-xs leading-relaxed text-ptagv-muted">{IS_SHOWCASE ? "No robot import is sent." : "Robot import requires a supported backend endpoint; the saved-map loader remains the activation path."}</small>
    </CardBody>
  </Card>;
}
