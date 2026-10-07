import { useEffect, useMemo, useState } from "react";
import { Sidebar } from "../shell/Sidebar";
import { useStore } from "../../state/store";
import { RobotControlDetailPage } from "./RobotControlDetailPage";

/** The operator workflow lives on one route and always uses the shared
 * active-map navigation and manual-control pipeline. */
export function RobotControlPage() {
  const selectedRobot = useStore((state) => state.selectedRobot);
  const robotIdsKey = useStore((state) => Object.keys(state.twin?.robots ?? {}).join("\u0000"));
  const select = useStore((state) => state.select);
  const robotIds = useMemo(() => robotIdsKey ? robotIdsKey.split("\u0000") : [], [robotIdsKey]);
  const robotId = selectedRobot && robotIds.includes(selectedRobot)
    ? selectedRobot : robotIds[0] ?? selectedRobot ?? "";
  const [collapsed, setCollapsed] = useState(false);

  useEffect(() => {
    if (robotId && selectedRobot !== robotId) select(robotId);
  }, [robotId, selectedRobot, select]);

  return <div className={`robot-control-workspace wt-has-sidebar${collapsed ? " wt-sidebar-collapsed" : ""}`}>
    <Sidebar collapsed={collapsed} onToggle={() => setCollapsed((value) => !value)} />
    {robotId
      ? <RobotControlDetailPage key={robotId} robotId={robotId} />
      : <div className="robot-control-waiting" role="status">WAITING FOR ROBOT TELEMETRY</div>}
  </div>;
}
