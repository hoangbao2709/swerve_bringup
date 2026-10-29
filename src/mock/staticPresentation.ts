/** Presentation helpers for development-only fixtures.
 * Runtime enums and fixture contracts remain unchanged; callers opt into a
 * neutral display value only at the presentation boundary.
 */
export const STATIC_UI_EMPTY_VALUE = "—";

export function displayRuntimeValue(
  staticUi: boolean,
  value: string | number | null | undefined,
  fallback = "UNKNOWN",
): string {
  if (staticUi) return STATIC_UI_EMPTY_VALUE;
  if (value === null || value === undefined || value === "") return fallback;
  return String(value);
}

export function displayRuntimeRows(
  staticUi: boolean,
  rows: ReadonlyArray<readonly [string, unknown]>,
): Array<[string, string]> {
  return rows.map(([label, value]) => [label, displayRuntimeValue(staticUi,
    typeof value === "string" || typeof value === "number" || value == null ? value : String(value))]);
}

export function displayFixtureMapName(staticUi: boolean, name: string): string {
  if (!staticUi) return name;
  return /^sample map(?:\b|\s|·)/i.test(name) ? "MAP" : name;
}
