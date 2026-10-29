import { useEffect, type ReactNode } from "react";
import { STATIC_ROBOT_ID } from "./fixtures";
import { initializeStaticRuntime } from "./staticRuntime";
import { initializeShowcaseRuntime } from "./showcaseRuntime";
import { PreviewRobotContext } from "../app/PreviewRobotContext";

export default function StaticUiApp({ children, showcase = false }: { children: ReactNode; showcase?: boolean }) {
  useEffect(() => { showcase ? initializeShowcaseRuntime() : initializeStaticRuntime(); }, [showcase]);
  return <PreviewRobotContext.Provider value={STATIC_ROBOT_ID}>{children}</PreviewRobotContext.Provider>;
}
