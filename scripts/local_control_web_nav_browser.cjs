const fs = require('fs');
const path = require('path');
const { chromium } = require('../waretwin/frontend/node_modules/playwright');

const dir = process.env.NAV_ACCEPTANCE_DIR;
const backend = process.env.BACKEND_URL;
const frontend = process.env.FRONTEND_URL;
const authStorageKey = 'waretwin.auth';
const readJson = name => {
  try { return JSON.parse(fs.readFileSync(path.join(dir, name), 'utf8')); }
  catch { return null; }
};
const writeJson = (name, value) => fs.writeFileSync(path.join(dir, name), JSON.stringify(value, null, 2));
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));

(async () => {
  const login = await fetch(`${backend}/api/auth/login`, {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ username: process.env.TWIN_ADMIN_USERNAME,
      password: process.env.TWIN_ADMIN_PASSWORD }),
  });
  if (!login.ok) throw Error(`Django login failed with HTTP ${login.status}`);
  const auth = await login.json();
  const browser = await chromium.launch({ headless: true, args: ['--no-sandbox'] });
  try {
    const context = await browser.newContext({ viewport: { width: 1500, height: 1000 } });
    const origin = new URL(frontend).origin;
    await context.addInitScript(({ key, session, expectedOrigin }) => {
      if (location.origin === expectedOrigin) localStorage.setItem(key, JSON.stringify(session));
    }, { key: authStorageKey, expectedOrigin: origin,
      session: { token: auth.access_token, user: auth.user } });
    const page = await context.newPage();
    const browserErrors = [];
    const sent = [];
    const received = [];
    page.on('pageerror', error => browserErrors.push(error.message));
    page.on('websocket', socket => {
      const collect = (target, event) => {
        try { target.push({ at_ms: Date.now(), message: JSON.parse(event.payload) }); } catch {}
      };
      socket.on('framesent', event => collect(sent, event));
      socket.on('framereceived', event => collect(received, event));
    });

    await page.goto(`${frontend}/robots/R01/control`, { waitUntil: 'domcontentloaded', timeout: 60000 });
    const mode = page.locator('.robot-detail-mode');
    await mode.waitFor({ state: 'visible', timeout: 60000 });
    const modeText = (await mode.textContent()) || '';
    if (!modeText.includes('MANUAL APPLIED')) {
      await page.locator('.robot-detail-manual-mode').getByRole('button',
        { name: 'MANUAL', exact: true }).click({ timeout: 30000 });
      await page.waitForFunction(() => document.querySelector('.robot-detail-mode')?.textContent === 'MANUAL APPLIED',
        null, { timeout: 30000 });
    }

    const savedMapName = process.env.SAVED_MAP_NAME || 'slam_accumulated_20261001_01';
    await page.getByRole('tab', { name: 'MAPPING', exact: true }).click();
    const savedMapRow = page.locator('.local-map-row').filter({ hasText: savedMapName }).first();
    await savedMapRow.waitFor({ state: 'visible', timeout: 60000 });
    await savedMapRow.click();
    const loadMapButton = page.getByRole('button', { name: 'LOAD SAVED MAP FOR NAVIGATION', exact: true });
    await page.waitForFunction(() => {
      const button = [...document.querySelectorAll('button')]
        .find(item => item.textContent?.trim() === 'LOAD SAVED MAP FOR NAVIGATION');
      return button && !button.disabled;
    }, null, { timeout: 30000 });
    await loadMapButton.click();
    await page.waitForFunction(() => [...document.querySelectorAll('.local-feedback.ok')]
      .some(item => item.textContent?.includes('MAP LOADED')), null, { timeout: 60000 });
    const loadNotice = (await page.locator('.local-feedback.ok').last().textContent()).trim();

    const poseSeed = readJson('map-pose-seed.json');
    if (!poseSeed || ![poseSeed.x, poseSeed.y, poseSeed.yaw].every(Number.isFinite)) {
      throw Error('ROS-observed saved-map pose seed is missing or invalid');
    }
    await page.getByRole('tab', { name: 'LOCALIZATION', exact: true }).click();
    const poseInputs = page.locator('.local-pose-fields input');
    await poseInputs.nth(0).fill(Number(poseSeed.x).toFixed(6));
    await poseInputs.nth(1).fill(Number(poseSeed.y).toFixed(6));
    await poseInputs.nth(2).fill(Number(poseSeed.yaw).toFixed(6));
    page.once('dialog', dialog => dialog.accept());
    const poseResponsePromise = page.waitForResponse(response =>
      response.url().includes('/local/initial-pose') && response.request().method() === 'POST',
      { timeout: 30000 });
    await page.getByRole('button', { name: 'SET INITIAL POSE', exact: true }).click();
    const poseResponse = await poseResponsePromise;
    const poseResult = poseResponse.ok() ? await poseResponse.json() : null;
    if (!poseResult?.ok || poseResult.localization_owner !== 'ekf_v30e') {
      throw Error(`authoritative localization service did not accept initial pose (HTTP ${poseResponse.status()})`);
    }
    await page.getByText(/INITIAL POSE ACCEPTED BY ekf_v30e/).waitFor({ state: 'visible', timeout: 30000 });
    writeJson('pose-set.json', { pose: poseSeed, gazebo_pose: readJson('pose-seed.json'),
      load_notice: loadNotice,
      localization_owner: poseResult.localization_owner,
      result_notice: (await page.locator('.local-feedback.ok').last().textContent()).trim() });

    await page.getByRole('tab', { name: 'CONTROL', exact: true }).click();
    await page.locator('.robot-detail-manual-mode').getByRole('button',
      { name: 'AUTONOMOUS', exact: true }).click({ timeout: 30000 });
    await page.waitForFunction(() => document.querySelector('.robot-detail-mode')?.textContent === 'AUTONOMOUS APPLIED',
      null, { timeout: 30000 });

    await page.getByRole('button', { name: 'RECENTER', exact: true }).click();
    await page.waitForFunction(() => document.querySelector('.robot-detail-map-toolbar button:nth-of-type(5)')
      ?.classList.contains('is-active'), null, { timeout: 10000 });
    const mapState = page.locator('.map-sync-warning-ready').first();
    await mapState.waitFor({ state: 'visible', timeout: 60000 });
    const initialMap = (await mapState.textContent()).trim();
    if (!/CANONICAL|LOCAL_ONLY/.test(initialMap)) throw Error(`active robot map is not navigable: ${initialMap}`);
    const mapIdentityParts = initialMap.split(' · ');
    const activeMapId = mapIdentityParts[0];
    const activeMapRevision = (mapIdentityParts[1] || '').replace(/^r/, '');
    const mapSnapshotDeadline = Date.now() + 60000;
    let localMapSnapshot = null;
    while (Date.now() < mapSnapshotDeadline) {
      localMapSnapshot = received.findLast(row => row.message.type === 'MAP_SNAPSHOT'
        && row.message.map?.map_source === 'LOCAL_MAP'
        && row.message.map?.active_map_id === activeMapId
        && String(row.message.map?.active_map_revision) === activeMapRevision)?.message.map || null;
      if (localMapSnapshot) break;
      await sleep(100);
    }
    if (!localMapSnapshot || localMapSnapshot.known_cells <= 0
        || localMapSnapshot.width <= 0 || localMapSnapshot.height <= 0) {
      throw Error(`GLOBAL MAP did not receive a non-empty snapshot for saved local map ${activeMapId}`);
    }
    const canvas = page.locator('canvas.robot-detail-map-canvas');
    await canvas.waitFor({ state: 'visible', timeout: 30000 });
    await page.waitForFunction(({ mapId, revision }) => {
      const readout = document.querySelector('.robot-detail-map-readout')?.textContent || '';
      return readout.includes('FRAME map') && readout.includes(mapId)
        && readout.includes(`r${revision}`) && !readout.includes('MAP WAITING');
    }, { mapId: activeMapId, revision: activeMapRevision }, { timeout: 60000 });
    const loadedMapReadout = (await page.locator('.robot-detail-map-readout').textContent()).trim();
    const box = await canvas.boundingBox();
    if (!box || box.width < 100 || box.height < 100) throw Error('GLOBAL MAP canvas has no usable viewport');
    const mapResolution = Number(localMapSnapshot.resolution);
    const mapOriginYaw = Number(localMapSnapshot.origin?.yaw || 0);
    const mapWidthM = Number(localMapSnapshot.width) * mapResolution;
    const mapHeightM = Number(localMapSnapshot.height) * mapResolution;
    const mapSpanX = Math.abs(mapWidthM * Math.cos(mapOriginYaw))
      + Math.abs(mapHeightM * Math.sin(mapOriginYaw));
    const mapSpanY = Math.abs(mapWidthM * Math.sin(mapOriginYaw))
      + Math.abs(mapHeightM * Math.cos(mapOriginYaw));
    const pixelsPerMeter = Math.min((box.width - 56) / mapSpanX, (box.height - 56) / mapSpanY);
    if (!Number.isFinite(pixelsPerMeter) || pixelsPerMeter <= 0) {
      throw Error('saved-map metadata cannot determine the GLOBAL MAP canvas scale');
    }

    let preview = null;
    let targetClick = null;
    const previewAttempts = [];
    const startPose = readJson('map-pose-seed.json');
    if (!startPose || ![startPose.x, startPose.y, startPose.yaw].every(Number.isFinite)) {
      throw Error('ROS-observed saved-map pose seed is missing or invalid');
    }
    // Follow centers the map on the robot. Choose a point beyond the robot's
    // 0.60 m forward footprint plus costmap inflation; sub-footprint goals can
    // trigger Nav2 recovery spins even when the global path preview is valid.
    const goalHeading = Number(process.env.NAV_GOAL_HEADING);
    if (!Number.isFinite(goalHeading)) throw Error('ROS-observed goal heading is required');
    await page.screenshot({ path: path.join(dir, 'before-preview.png') });
    // Derive the screen offset from the same saved-map bounds and 28 px
    // padding used by the canvas transform. This keeps test goals near 1.2 m
    // across different fit scales and avoids stretching the route unnecessarily.
    const requestedDistances = [1.18, 1.24, 1.30];
    const targetRadii = requestedDistances.map(distance => Math.max(1, Math.round(distance * pixelsPerMeter)));
    for (const radius of targetRadii) {
      // Map selection intentionally exits follow mode. Restore the robot-centered
      // transform before each trial so a rejected click cannot shift later goals.
      for (let retry = 0; retry < 2; retry += 1) {
        await page.getByRole('button', { name: 'RECENTER', exact: true }).click();
        await page.waitForFunction(() => document.querySelector('.robot-detail-map-toolbar button:nth-of-type(5)')
          ?.classList.contains('is-active'), null, { timeout: 10000 });
        await page.waitForTimeout(100);
        targetClick = { x: box.x + box.width / 2 + Math.cos(goalHeading) * radius,
          y: box.y + box.height / 2 - Math.sin(goalHeading) * radius };
        const previousCount = sent.filter(row => row.message.type === 'PATH_PREVIEW_REQUEST').length;
        await page.mouse.click(targetClick.x, targetClick.y);
        const deadline = Date.now() + 18000;
        let candidateStatus = null;
        let resultReceived = false;
        while (Date.now() < deadline) {
          const toolbar = (await page.locator('.robot-detail-goal-toolbar span').textContent()) || '';
          const requests = sent.filter(row => row.message.type === 'PATH_PREVIEW_REQUEST');
          const request = requests[requests.length - 1];
          const result = request && received.findLast(row => row.message.type === 'PATH_PREVIEW_RESULT'
            && row.message.request_id === request.message.request_id);
          if (result) {
            resultReceived = true;
            candidateStatus = result.message.status;
            const candidateGoal = result.message.goal || {};
            const candidateDistance = Math.hypot(Number(candidateGoal.x) - Number(startPose.x),
              Number(candidateGoal.y) - Number(startPose.y));
            const candidatePathLength = Number(result.message.path_length_m);
            previewAttempts.push({ radius_px: radius, retry: retry + 1,
              requested: request.message, status: candidateStatus, reason: result.message.reason,
              goal: candidateGoal, distance_m: candidateDistance,
              path_length_m: candidatePathLength, toolbar });
            writeJson('preview-attempts.json', previewAttempts);
            if (candidateStatus === 'VALID' && Array.isArray(result.message.path)
                && result.message.path.length > 0 && toolbar.includes('VALID')
                && Number.isFinite(candidateDistance) && candidateDistance >= 1.1 && candidateDistance <= 1.35
                && Number.isFinite(candidatePathLength) && candidatePathLength <= 3.0) {
              preview = { request: request.message, result: result.message, request_at_ms: request.at_ms,
                result_at_ms: result.at_ms, toolbar };
            }
            break;
          }
          await sleep(100);
        }
        if (preview) break;
        if (!resultReceived && sent.filter(row => row.message.type === 'PATH_PREVIEW_REQUEST').length <= previousCount) {
          throw Error('GLOBAL MAP click did not send PATH_PREVIEW_REQUEST');
        }
        if (candidateStatus !== 'NO_PATH' || retry === 1) break;
      }
      if (preview) break;
    }
    if (!preview) throw Error('nearby GLOBAL MAP clicks did not produce a valid Nav2 preview');
    const previewGoal = preview.result.goal || {};
    const targetDistance = Math.hypot(Number(previewGoal.x) - Number(startPose.x),
      Number(previewGoal.y) - Number(startPose.y));
    const previewPathLength = Number(preview.result.path_length_m);
    if (!Number.isFinite(targetDistance) || targetDistance < 1.1 || targetDistance > 1.35
        || !Number.isFinite(previewPathLength) || previewPathLength > 3.0) {
      throw Error(`nearby-goal safety gate rejected distance=${targetDistance}m path=${previewPathLength}m`);
    }
    const beforeSendMap = (await mapState.textContent()).trim();
    if (beforeSendMap !== initialMap) throw Error(`active map changed from ${initialMap} to ${beforeSendMap} before Send Goal`);
    await page.screenshot({ path: path.join(dir, 'path-preview.png') });
    const button = page.getByRole('button', { name: 'SEND GOAL', exact: true });
    if (await button.isDisabled()) throw Error('SEND GOAL is disabled after a valid path preview');
    writeJson('path-preview.json', { initial_map: initialMap, before_preview_map: initialMap,
      loaded_map_readout: loadedMapReadout,
      map_snapshot: { map_source: localMapSnapshot.map_source,
        active_map_id: localMapSnapshot.active_map_id,
        active_map_revision: localMapSnapshot.active_map_revision,
        resolution: localMapSnapshot.resolution,
        origin: localMapSnapshot.origin,
        width: localMapSnapshot.width, height: localMapSnapshot.height,
        known_cells: localMapSnapshot.known_cells },
      preview, before_send_map: beforeSendMap, target_click: targetClick,
      canvas_pixels_per_meter: pixelsPerMeter, target_radii_px: targetRadii,
      target_distance_m: targetDistance, nearby_distance_range_m: [1.1, 1.35],
      nearby_path_limit_m: 3.0,
      screenshot: path.join(dir, 'path-preview.png') });

    const releaseDeadline = Date.now() + 120000;
    while (!readJson('send-goal-ready.json') && Date.now() < releaseDeadline) await sleep(100);
    if (!readJson('send-goal-ready.json')) throw Error('observer did not release valid preview for Send Goal');
    const sendAt = Date.now();
    await button.click({ timeout: 30000 });
    const goalDeadline = Date.now() + 15000;
    let goalFrame = null;
    while (Date.now() < goalDeadline) {
      goalFrame = sent.findLast(row => row.message.type === 'NAV_GOAL'
        && row.message.preview_request_id === preview.result.request_id);
      if (goalFrame) break;
      await sleep(50);
    }
    if (!goalFrame) throw Error('SEND GOAL click produced no real Django WebSocket NAV_GOAL frame');
    if (goalFrame.message.active_map_id !== preview.result.active_map_id
        || String(goalFrame.message.active_map_revision) !== String(preview.result.active_map_revision)) {
      throw Error('NAV_GOAL map identity differs from approved preview');
    }
    writeJson('goal-sent.json', { clicked_at_ms: sendAt, frame_at_ms: goalFrame.at_ms,
      frame: goalFrame.message, map_at_send: (await mapState.textContent()).trim() });
    const configuredNavTimeout = Number(process.env.NAV_WALL_TIMEOUT_S || 600);
    const finishDeadline = Date.now() + (Math.max(60, configuredNavTimeout) + 30) * 1000;
    while (!readJson('nav-finished.json') && Date.now() < finishDeadline) await sleep(100);
    await page.screenshot({ path: path.join(dir, 'navigation-result.png'), fullPage: true });
    writeJson('browser-result.json', { errors: browserErrors,
      navigation_status_text: await page.locator('.robot-detail-runtime').textContent().catch(() => null),
      screenshots: [path.join(dir, 'path-preview.png'), path.join(dir, 'navigation-result.png')] });
    if (browserErrors.length) throw Error(`browser page error: ${browserErrors.join('; ')}`);
  } finally { await browser.close(); }
})().catch(error => { console.error(error.stack || error.message); process.exitCode = 1; });
