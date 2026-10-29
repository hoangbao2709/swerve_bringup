import type { ReactNode } from "react";
import { Card, CardBody, CardHeader, StatusIndicator, type HmiTone } from "./HmiPrimitives";

export function valueText(value: unknown, fallback = "N/A"): string {
  return value === null || value === undefined || value === "" ? fallback : String(value);
}

export function valueNumber(value: unknown, digits = 2, suffix = ""): string {
  const number = Number(value);
  return Number.isFinite(number) ? `${number.toFixed(digits)}${suffix}` : "N/A";
}

export function classForStatus(value: unknown): string {
  const status = String(value ?? "").toUpperCase();
  if (["OK", "ONLINE", "CONNECTED", "ACTIVE", "READY", "MAPPING", "LOCALIZED"].includes(status)) return "ok";
  if (["WAITING", "PAUSED", "DEGRADED", "STALE", "CONNECTING", "SAVED"].includes(status)) return "warning";
  if (["ERROR", "OFFLINE", "DISCONNECTED", "FAILED", "NO_PATH"].includes(status)) return "error";
  return "neutral";
}

export function Status({ value }: { value: unknown }) {
  const toneByStatus: Record<string, HmiTone> = { ok: "success", warning: "warning", error: "error", neutral: "neutral" };
  const tone = classForStatus(value);
  return <StatusIndicator tone={toneByStatus[tone]}>{valueText(value)}</StatusIndicator>;
}

export function SectionFrame({ children, className = "" }: { children: ReactNode; className?: string }) {
  return <div className={`grid min-h-0 min-w-0 gap-hmi-3 ${className}`}>{children}</div>;
}

export function SectionPanel({ title, children, className = "" }: { title: string; children: ReactNode; className?: string }) {
  return <Card className={`min-h-0 overflow-hidden ${className}`}>
    <CardHeader className="shrink-0 text-hmi-kicker uppercase tracking-wider">{title}</CardHeader>
    <CardBody className="min-h-0 flex-1 overflow-auto">{children}</CardBody>
  </Card>;
}

export function Metric({ label, value, mono = false }: { label: string; value: unknown; mono?: boolean }) {
  return <div className="grid min-h-9 min-w-0 grid-cols-2 items-center gap-x-hmi-3 border-b border-ptagv-border-soft py-hmi-2 last:border-b-0">
    <span className="min-w-0 text-hmi-kicker leading-snug text-ptagv-muted [overflow-wrap:anywhere]">{label}</span>
    <b className={`min-w-0 text-right text-sm font-semibold leading-snug text-ptagv-text [overflow-wrap:anywhere] ${mono ? "font-mono text-hmi-kicker" : ""}`}>{valueText(value)}</b>
  </div>;
}
