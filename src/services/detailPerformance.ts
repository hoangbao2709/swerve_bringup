/** Opt-in, bounded acceptance instrumentation; never participates in control. */
export function detailPerformance(stage: string, fields: Record<string, unknown> = {}) {
  if (typeof window === "undefined" || !(window as Window & { __ROBOT_DETAIL_PERFORMANCE__?: boolean }).__ROBOT_DETAIL_PERFORMANCE__) return;
  window.dispatchEvent(new CustomEvent("robot-detail-performance", {
    detail: { stage, at_ms: performance.timeOrigin + performance.now(), ...fields },
  }));
}
