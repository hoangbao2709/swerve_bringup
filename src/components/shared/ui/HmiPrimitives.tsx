import type {
  ButtonHTMLAttributes,
  HTMLAttributes,
  InputHTMLAttributes,
  ReactNode,
  SelectHTMLAttributes,
} from "react";

type ClassName = { className?: string };
export type HmiTone = "success" | "warning" | "error" | "neutral" | "info";

const statusTone: Record<HmiTone, { badge: string; text: string; dot: string }> = {
  success: { badge: "border-green-300 bg-green-50", text: "text-ptagv-success", dot: "bg-green-600" },
  warning: { badge: "border-amber-300 bg-amber-50", text: "text-ptagv-warning", dot: "bg-amber-600" },
  error: { badge: "border-red-300 bg-red-50", text: "text-ptagv-error", dot: "bg-red-600" },
  neutral: { badge: "border-ptagv-border bg-ptagv-neutral-bg", text: "text-ptagv-neutral", dot: "bg-slate-500" },
  info: { badge: "border-blue-200 bg-blue-50", text: "text-blue-800", dot: "bg-ptagv-blue" },
};

const buttonTone: Record<"primary" | "secondary" | "danger" | "quiet", string> = {
  primary: "border-ptagv-blue bg-ptagv-blue text-white hover:bg-blue-700",
  secondary: "border-ptagv-border bg-white text-ptagv-text hover:bg-slate-50",
  danger: "border-red-700 bg-red-700 text-white hover:bg-red-800",
  quiet: "border-transparent bg-transparent text-ptagv-muted hover:bg-slate-100",
};
const buttonSize: Record<"compact" | "standard", string> = {
  compact: "min-h-7 px-2 text-hmi-kicker",
  standard: "min-h-10 px-hmi-3 text-hmi-kicker",
};

const controlBase = "min-h-10 rounded-hmi border border-ptagv-border bg-white px-hmi-2 text-sm text-ptagv-text focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-ptagv-blue disabled:cursor-not-allowed disabled:bg-slate-100 disabled:text-slate-500";

export function Card({ className = "", ...props }: HTMLAttributes<HTMLElement> & ClassName) {
  return <section className={`flex min-w-0 flex-col border border-ptagv-border bg-ptagv-surface text-ptagv-text ${className}`.trim()} {...props} />;
}

export function CardHeader({ className = "", ...props }: HTMLAttributes<HTMLElement> & ClassName) {
  return <header className={`flex min-h-12 items-center justify-between gap-hmi-2 border-b border-ptagv-border-soft bg-ptagv-surface-muted px-hmi-3 py-hmi-2 text-hmi-title font-semibold text-ptagv-text ${className}`.trim()} {...props} />;
}

export function CardBody({ className = "", ...props }: HTMLAttributes<HTMLDivElement> & ClassName) {
  return <div className={`flex min-w-0 flex-col gap-hmi-2 p-hmi-3 ${className}`.trim()} {...props} />;
}

export function Button({ tone = "secondary", size = "standard", className = "", type = "button", ...props }: ButtonHTMLAttributes<HTMLButtonElement> & ClassName & { tone?: keyof typeof buttonTone; size?: keyof typeof buttonSize }) {
  return <button type={type} className={`inline-flex min-w-0 items-center justify-center gap-hmi-1 rounded-hmi border font-bold tracking-wide transition-colors focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-ptagv-blue disabled:cursor-not-allowed disabled:border-ptagv-border disabled:bg-slate-100 disabled:text-slate-500 ${buttonSize[size]} ${buttonTone[tone]} ${className}`.trim()} {...props} />;
}

export function IconButton({ className = "", ...props }: ButtonHTMLAttributes<HTMLButtonElement> & ClassName) {
  return <button type="button" className={`inline-grid size-10 shrink-0 place-items-center rounded-hmi border border-ptagv-border bg-white text-ptagv-muted hover:bg-slate-50 focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-ptagv-blue disabled:cursor-not-allowed disabled:opacity-50 ${className}`.trim()} {...props} />;
}

export function Badge({ tone = "neutral", className = "", children, ...props }: HTMLAttributes<HTMLSpanElement> & ClassName & { tone?: HmiTone; children: ReactNode }) {
  return <span className={`inline-flex min-w-0 items-center rounded-hmi border px-2 py-1 text-hmi-kicker font-bold leading-tight ${statusTone[tone].badge} ${className}`.trim()} {...props}>{children}</span>;
}

export function StatusIndicator({ tone = "neutral", className = "", children, ...props }: HTMLAttributes<HTMLSpanElement> & ClassName & { tone?: HmiTone; children: ReactNode }) {
  return <span className={`robot-detail-status inline-flex min-w-0 items-center gap-1.5 whitespace-nowrap text-hmi-kicker font-bold tracking-wide ${statusTone[tone].text} ${className}`.trim()} {...props}>
    <span aria-hidden="true" className={`size-2 shrink-0 rounded-full ${statusTone[tone].dot}`} />
    {children}
  </span>;
}

export function Input({ className = "", ...props }: InputHTMLAttributes<HTMLInputElement> & ClassName) {
  return <input className={`${controlBase} w-full min-w-0 ${className}`.trim()} {...props} />;
}

export function Select({ className = "", ...props }: SelectHTMLAttributes<HTMLSelectElement> & ClassName) {
  return <select className={`${controlBase} w-full min-w-0 ${className}`.trim()} {...props} />;
}

export function MetricRow({ label, value, mono = false, layout = "horizontal", density = "normal", className = "" }: { label: ReactNode; value: ReactNode; mono?: boolean; layout?: "horizontal" | "stacked"; density?: "normal" | "compact"; className?: string }) {
  const rowLayout = layout === "stacked"
    ? "grid min-h-14 grid-cols-1 content-center gap-1 border-b border-r border-ptagv-border-soft px-2 py-2 last:border-r-0"
    : density === "compact"
      ? "grid min-h-7 grid-cols-2 items-center gap-x-hmi-3 border-b border-ptagv-border-soft py-hmi-1 last:border-b-0"
      : "grid min-h-9 grid-cols-2 items-center gap-x-hmi-3 border-b border-ptagv-border-soft py-hmi-2 last:border-b-0";
  const labelClass = "min-w-0 text-hmi-kicker leading-snug text-ptagv-muted [overflow-wrap:anywhere]";
  const valueClass = layout === "stacked"
    ? "min-w-0 text-left text-sm font-semibold leading-snug text-ptagv-text [overflow-wrap:anywhere]"
    : "min-w-0 text-right text-sm font-semibold leading-snug text-ptagv-text [overflow-wrap:anywhere]";
  return <div className={["min-w-0", rowLayout, className].filter(Boolean).join(" ")}>
    <span className={labelClass}>{label}</span>
    <b className={[valueClass, mono ? "font-mono text-hmi-kicker" : ""].filter(Boolean).join(" ")}>{value}</b>
  </div>;
}

export function MetricGrid({ className = "", ...props }: HTMLAttributes<HTMLDivElement> & ClassName) {
  return <div className={`grid min-w-0 grid-cols-1 gap-x-hmi-4 ${className}`.trim()} {...props} />;
}

export function Alert({ tone = "neutral", className = "", ...props }: HTMLAttributes<HTMLDivElement> & ClassName & { tone?: HmiTone }) {
  const toneClass: Record<HmiTone, string> = {
    success: "border-green-300 bg-green-50 text-ptagv-success",
    warning: "border-amber-300 bg-amber-50 text-ptagv-warning",
    error: "border-red-300 bg-red-50 text-ptagv-error",
    neutral: "border-ptagv-border-soft bg-slate-50 text-ptagv-neutral",
    info: "border-blue-200 bg-blue-50 text-blue-800",
  };
  return <div className={`min-w-0 border px-hmi-2 py-hmi-2 text-hmi-body leading-relaxed [overflow-wrap:anywhere] ${toneClass[tone]} ${className}`.trim()} {...props} />;
}

export function SectionHeading({ className = "", ...props }: HTMLAttributes<HTMLHeadingElement> & ClassName) {
  return <h2 className={`m-0 text-hmi-title font-bold leading-snug text-ptagv-text ${className}`.trim()} {...props} />;
}

export function EmptyState({ className = "", children, ...props }: HTMLAttributes<HTMLDivElement> & ClassName & { children: ReactNode }) {
  return <div className={`grid min-h-24 min-w-0 place-items-center border border-dashed border-ptagv-border-soft bg-slate-50 p-hmi-4 text-center text-hmi-body text-ptagv-muted [overflow-wrap:anywhere] ${className}`.trim()} {...props}>{children}</div>;
}

export function LoadingState({ className = "", children = "Loading…", ...props }: HTMLAttributes<HTMLDivElement> & ClassName & { children?: ReactNode }) {
  return <div className={`flex min-h-10 min-w-0 items-center gap-hmi-2 text-hmi-body text-ptagv-muted ${className}`.trim()} role="status" {...props}>
    <span aria-hidden="true" className="size-2 shrink-0 rounded-full bg-ptagv-blue" />
    {children}
  </div>;
}
