import type { LocalizationEvent } from "../hooks/useLocalizationEventHistory";
import { Button, EmptyState } from "../../../components/shared/ui/HmiPrimitives";

export function LocalizationEventHistory({ events, onClear }: { events: LocalizationEvent[]; onClear: () => void }) {
  return <details className="border border-ptagv-border-soft bg-white" data-testid="localization-event-history">
    <summary className="cursor-pointer px-hmi-3 py-hmi-2 text-hmi-kicker font-bold tracking-wide text-ptagv-text">SESSION EVENT HISTORY · {events.length}</summary>
    <div className="border-t border-ptagv-border-soft p-hmi-2">
      <div className="mb-hmi-2 flex items-center justify-between gap-hmi-2">
        <span className="text-xs text-ptagv-muted">Local browser session · newest first · maximum 100</span>
        <Button size="compact" disabled={!events.length} onClick={onClear}>CLEAR</Button>
      </div>
      {events.length ? <ol className="m-0 max-h-56 list-none space-y-1 overflow-y-auto p-0">
        {events.map((event) => <li key={event.id} className="grid min-w-0 grid-cols-[auto_minmax(0,1fr)] gap-x-hmi-2 border-t border-ptagv-border-soft py-hmi-1 text-xs first:border-t-0" data-testid="localization-event">
          <time className="font-mono text-ptagv-muted" dateTime={event.timestamp}>{new Date(event.timestamp).toLocaleTimeString()}</time>
          <span className="min-w-0 break-words"><b className="mr-1 text-ptagv-text">{event.kind.replace(/_/g, " ")}</b>{event.description}</span>
        </li>)}
      </ol> : <EmptyState className="min-h-12">NO LOCALIZATION EVENTS IN THIS SESSION</EmptyState>}
    </div>
  </details>;
}
