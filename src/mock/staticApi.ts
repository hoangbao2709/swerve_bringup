import {
  STATIC_LOCAL_MAP,
  STATIC_MAP_ID,
  STATIC_MAP_REVISION,
  STATIC_ROBOT_ID,
  STATIC_VDA5050_CONFIGURATION,
} from "./fixtures";

function jsonResponse(body: unknown, status = 200): Response {
  return new Response(JSON.stringify(body), {
    status,
    headers: { "content-type": "application/json" },
  });
}

function blockedResponse(): Response {
  return jsonResponse({
    ok: false,
    code: "STATIC_PREVIEW_ONLY",
    error: "UNAVAILABLE",
    error_code: "STATIC_PREVIEW_ONLY",
    message: "UNAVAILABLE",
  }, 409);
}

/** Local fixture transport. It never calls window.fetch or a remote service. */
export async function staticApiFetch(path: string, init: RequestInit = {}): Promise<Response> {
  const method = (init.method ?? "GET").toUpperCase();
  const route = path.split("?", 1)[0];

  if (route === "/api/local/identity" || route === "/api/system/status/") {
    return jsonResponse({
      ok: false,
      code: "STATIC_UI_ENDPOINT_BLOCKED",
      error: "UNAVAILABLE",
    }, 403);
  }

  if (!new Set(["GET", "HEAD"]).has(method)) return blockedResponse();

  if (route.endsWith(`/api/robots/${STATIC_ROBOT_ID}/local/maps`)) {
    return jsonResponse({
      robot_id: STATIC_ROBOT_ID,
      maps: [STATIC_LOCAL_MAP],
      runtime_mode: "UNKNOWN",
      mapping_state: "UNAVAILABLE",
      mapping_duration_s: 0,
      robot_control_mode: "AUTONOMOUS",
      robot_stopped: true,
      robot_navigation_state: "UNKNOWN",
      estop_active: null,
      active_local_map_id: STATIC_MAP_ID,
      local_active_map_id: STATIC_MAP_ID,
      local_active_map_revision: STATIC_MAP_REVISION,
      active_map_id: STATIC_MAP_ID,
      active_map_revision: STATIC_MAP_REVISION,
      canonical_map_revision: null,
      map_sync_status: "UNAVAILABLE",
    });
  }

  if (route.endsWith(`/api/robots/${STATIC_ROBOT_ID}/local/runtime-mode`)) {
    return jsonResponse({
      robot_id: STATIC_ROBOT_ID,
      current_mode: "UNKNOWN",
      transition: { robot_id: STATIC_ROBOT_ID, mode: "UNKNOWN", status: "UNAVAILABLE", message: "UNAVAILABLE" },
    });
  }

  if (route.endsWith(`/api/robots/${STATIC_ROBOT_ID}/local/vda5050`)) {
    return jsonResponse(STATIC_VDA5050_CONFIGURATION);
  }

  if (route === "/api/map/sync-status") {
    return jsonResponse({
      published_revision: null,
      published_version: 0,
      ros_revision: null,
      gazebo_revision: null,
      nav2_revision: null,
      tag_map_revision: null,
      tf_status: false,
      status: "UNAVAILABLE",
      error: null,
      robot_map_sync: {},
    });
  }

  return jsonResponse({
    ok: false,
    code: "STATIC_FIXTURE_NOT_AVAILABLE",
    error: "UNAVAILABLE",
  }, 404);
}
