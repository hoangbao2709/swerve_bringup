import { IS_SHOWCASE, IS_STATIC_UI } from "../config";
import { backendApiFetch } from "./backendApiTransport";

/** Selects the isolated local fixture transport or the unchanged backend transport. */
export async function runtimeApiFetch(path: string, init: RequestInit = {}): Promise<Response> {
  if (IS_SHOWCASE) {
    const { showcaseApiFetch } = await import("../mock/showcaseApi");
    return showcaseApiFetch(path, init);
  }
  if (import.meta.env.DEV && IS_STATIC_UI) {
    const { staticApiFetch } = await import("../mock/staticApi");
    return staticApiFetch(path, init);
  }
  return backendApiFetch(path, init);
}
