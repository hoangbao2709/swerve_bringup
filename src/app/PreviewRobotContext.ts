import { createContext } from "react";

/** Preview-only identity supplied by the development-only STATIC_UI entrypoint. */
export const PreviewRobotContext = createContext<string | null>(null);
