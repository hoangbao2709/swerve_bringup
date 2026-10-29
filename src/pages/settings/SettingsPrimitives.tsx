import type { ReactNode } from "react";
import { Alert, Card, CardBody, CardHeader, SectionHeading } from "../../components/shared/ui/HmiPrimitives";

export function SettingsPageFrame({
  title,
  children,
  embedded = false,
  active = true,
  sectionId,
}: {
  title: string;
  children: ReactNode;
  embedded?: boolean;
  active?: boolean;
  sectionId?: string;
}) {
  if (embedded) {
    return <section id={sectionId} aria-label={title} data-settings-section={title} hidden={!active}
      className={`settings-workspace-section min-w-0 scroll-mt-20 ${active ? "grid" : "hidden"}`}>
      <div className="grid min-w-0 gap-hmi-3">{children}</div>
    </section>;
  }

  return <main aria-label={title} className="settings-page flex min-h-0 min-w-0 flex-1 flex-col gap-hmi-3 overflow-auto bg-ptagv-page p-hmi-3">
    {children}
  </main>;
}

export function SettingsCard({ title, children, className = "" }: { title: string; children: ReactNode; className?: string }) {
  return <Card className={className}>
    <CardHeader><SectionHeading className="text-hmi-kicker uppercase tracking-wider">{title}</SectionHeading></CardHeader>
    <CardBody>{children}</CardBody>
  </Card>;
}

export function SettingsScopeNotice({ children }: { children: ReactNode }) {
  return <Alert tone="info" role="note" className="px-2 py-1 text-xs">{children}</Alert>;
}

export function SettingsSwitch({
  checked,
  onChange,
  label,
  disabled = false,
}: {
  checked: boolean;
  onChange: (checked: boolean) => void;
  label: string;
  disabled?: boolean;
}) {
  return <button type="button" role="switch" aria-checked={checked} aria-label={label} disabled={disabled}
    className={`inline-flex min-h-10 min-w-20 items-center justify-between gap-2 rounded-hmi border px-2 text-sm font-bold focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-ptagv-blue disabled:cursor-not-allowed disabled:opacity-60 ${checked ? "border-green-700 bg-green-700 text-white" : "border-ptagv-border bg-white text-ptagv-text"}`}
    onClick={() => onChange(!checked)}>
    <span>{checked ? "ON" : "OFF"}</span>
    <span aria-hidden="true" className={`size-3 rounded-full ${checked ? "bg-white" : "bg-slate-400"}`} />
  </button>;
}
