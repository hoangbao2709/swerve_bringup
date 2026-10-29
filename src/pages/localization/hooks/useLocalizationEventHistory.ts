import { useCallback, useRef, useState } from "react";

export type LocalizationEventKind = "POSE_SELECTED" | "POSE_DIRECTION_CONFIRMED" | "POSE_REQUEST_SENT"
  | "POSE_REQUEST_ACCEPTED" | "POSE_REQUEST_REJECTED" | "POSE_REQUEST_NOT_SENT" | "LOCALIZATION_STATE_CHANGED"
  | "POSE_TELEMETRY_STALE" | "POSE_TELEMETRY_RECOVERED" | "MAP_REVISION_CHANGED";

export type LocalizationEvent = {
  id: number;
  timestamp: string;
  kind: LocalizationEventKind;
  description: string;
};

const MAX_LOCALIZATION_EVENTS = 100;

export function useLocalizationEventHistory() {
  const [events, setEvents] = useState<LocalizationEvent[]>([]);
  const nextId = useRef(1);
  const record = useCallback((kind: LocalizationEventKind, description: string) => {
    const event = { id: nextId.current++, timestamp: new Date().toISOString(), kind, description };
    setEvents((previous) => [event, ...previous].slice(0, MAX_LOCALIZATION_EVENTS));
  }, []);
  return { events, record, clear: useCallback(() => setEvents([]), []) };
}
