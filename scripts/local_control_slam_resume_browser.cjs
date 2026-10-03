const fs = require('fs');
const path = require('path');
const { chromium } = require('../waretwin/frontend/node_modules/playwright');

const dir = process.env.SLAM_RESUME_ACCEPTANCE_DIR;
const backend = process.env.BACKEND_URL;
const frontend = process.env.FRONTEND_URL;
const robotId = process.env.ROBOT_ID || 'R01';
const mapName = process.env.SAVED_MAP_NAME || 'slam_accumulated_20261001_01';
const skipResumeRequest = process.env.SLAM_RESUME_SKIP_REQUEST === '1';
const restoreEvidencePath = process.env.SLAM_RESUME_RESTORE_EVIDENCE;
const authStorageKey = 'waretwin.auth';
const readinessGateNames = [
  'CLOCK_FRESH', 'GAZEBO_PHYSICS_ACTIVE', 'CONTROLLERS_ACTIVE',
  'COMMAND_ARBITER_READY', 'SWERVE_CONTROLLER_READY', 'JOINT_STATES_FRESH',
  'ODOM_FRESH', 'ODOM_TF_FRESH', 'MAP_ODOM_TF_FRESH', 'MAP_BASE_TF_FRESH',
  'TF_LIDAR_FRESH', 'SCAN_FRESH', 'SLAM_ACTIVE', 'SLAM_INPUT_FRESH', 'MAP_LIVE',
];
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
const readJson = name => {
  try { return JSON.parse(fs.readFileSync(path.join(dir, name), 'utf8')); }
  catch { return null; }
};
const writeJson = (name, value) => fs.writeFileSync(path.join(dir, name), JSON.stringify(value, null, 2));
const hasFreshReadinessLease = gate => {
  const gates = gate?.gates;
  const age = Date.now() - Number(gate?.updated_at_ms || 0);
  return gate?.state === 'APPROVED' && age >= 0 && age <= 500
    && gates && readinessGateNames.length === Object.keys(gates).length
    && readinessGateNames.every(name => gates[name] === true);
};
const waitForFreshReadinessLease = async timeoutMs => {
  const deadline = Date.now() + timeoutMs;
  let latest = null;
  while (Date.now() < deadline) {
    latest = readJson('teleop-gate.json');
    if (latest?.state === 'BLOCKED') return { gate: latest, blocked: true };
    if (hasFreshReadinessLease(latest)) return { gate: latest, blocked: false };
    await sleep(100);
  }
  return { gate: latest, blocked: false };
};

(async () => {
  fs.mkdirSync(dir, { recursive: true });
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
    const received = [];
    const sent = [];
    const resumeResponses = [];
    page.on('pageerror', error => browserErrors.push(error.message));
    page.on('websocket', socket => {
      socket.on('framereceived', event => {
        try { received.push({ at_ms: Date.now(), message: JSON.parse(event.payload) }); } catch {}
      });
      socket.on('framesent', event => {
        try {
          const payload = typeof event.payload === 'string' ? event.payload : event.payload.toString();
          const message = JSON.parse(payload);
          if (['ROBOT_MANUAL', 'ROBOT_MODE'].includes(message.type)) {
            sent.push({ at_ms: Date.now(), message });
          }
        } catch {}
      });
    });
    page.on('response', async response => {
      if (!response.url().includes(`/local/maps/resume-session`) || response.request().method() !== 'POST') return;
      try {
        const body = await response.json();
        resumeResponses.push({ at_ms: Date.now(), status_code: response.status(), body });
        writeJson('resume-responses.json', resumeResponses);
      } catch {}
    });

    await page.goto(`${frontend}/robots/${encodeURIComponent(robotId)}/control`,
      { waitUntil: 'domcontentloaded', timeout: 60000 });
    await page.locator('.robot-detail-mode').waitFor({ state: 'visible', timeout: 60000 });
    const mode = page.locator('.robot-detail-mode');
    if (!(await mode.textContent() || '').includes('MANUAL APPLIED')) {
      await page.locator('.robot-detail-manual-mode').getByRole('button',
        { name: 'MANUAL', exact: true }).click({ timeout: 30000 });
      await page.waitForFunction(() => document.querySelector('.robot-detail-mode')?.textContent === 'MANUAL APPLIED',
        null, { timeout: 30000 });
    }

    await page.getByRole('tab', { name: 'MAPPING', exact: true }).click();
    const mapRow = page.locator('.local-map-row').filter({ hasText: mapName }).first();
    await mapRow.waitFor({ state: 'visible', timeout: 60000 });
    if (!(await mapRow.textContent() || '').includes('SLAM AVAILABLE')) {
      throw Error(`selected local map ${mapName} has no available serialized SLAM session`);
    }
    await mapRow.click();
    const resumeButton = page.getByRole('button', { name: 'RESUME SAVED SLAM SESSION', exact: true });
    await page.waitForFunction(() => {
      const button = [...document.querySelectorAll('button')]
        .find(item => item.textContent?.trim() === 'RESUME SAVED SLAM SESSION');
      return button && !button.disabled;
    }, null, { timeout: 30000 });
    let resumed = null;
    let restoredNotice = 'SLAM session restoration verified in the preceding Web request';
    let restorationCarryover = null;
    if (!skipResumeRequest) {
      page.once('dialog', dialog => dialog.accept());
      await resumeButton.click();
      await page.waitForFunction(() => [...document.querySelectorAll('.local-feedback.ok')]
        .some(item => item.textContent?.includes('SLAM SESSION RESTORED')),
      null, { timeout: 20 * 60 * 1000 });
      restoredNotice = (await page.locator('.local-feedback.ok').last().textContent()).trim();
      resumed = resumeResponses.findLast(item => item.body?.status === 'RESUMED');
      if (!resumed?.body?.restore_evidence?.passed) {
        throw Error('Web resume did not receive a positive live-map restoration result');
      }
    } else if (restoreEvidencePath) {
      restorationCarryover = JSON.parse(fs.readFileSync(restoreEvidencePath, 'utf8')).resume_response || null;
      if (restorationCarryover?.body?.status !== 'RESUMED'
          || !restorationCarryover.body?.restore_evidence?.passed) {
        throw Error('prior Web resume response does not contain positive live-map restoration evidence');
      }
    }
    let continuedActiveMapping = false;
    if (skipResumeRequest) {
      const resumeMapping = page.getByRole('button', { name: 'RESUME MAPPING', exact: true });
      if (await resumeMapping.count()) {
        if (await resumeMapping.isDisabled()) {
          throw Error('the active restored SLAM session is paused but Web RESUME MAPPING is disabled');
        }
        await resumeMapping.click();
        await page.waitForFunction(() => [...document.querySelectorAll('button')]
          .some(button => button.textContent?.trim() === 'MAPPING ACTIVE' && button.disabled),
        null, { timeout: 30000 });
        continuedActiveMapping = true;
      }
    }
    const mapDeadline = Date.now() + 90000;
    let initialSnapshot = null;
    while (Date.now() < mapDeadline) {
      initialSnapshot = received.findLast(row => row.message.type === 'MAP_SNAPSHOT'
        && row.message.map?.map_source === 'SLAM_TOOLBOX'
        && row.message.map?.mapping_session_id)?.message.map || null;
      if (initialSnapshot?.known_cells > 0) break;
      await sleep(150);
    }
    if (!initialSnapshot || initialSnapshot.known_cells <= 0) {
      throw Error('Web channel did not receive a live accumulated SLAM map after restoration');
    }
    writeJson('session-restored.json', { restored_notice: restoredNotice,
      resume_response: resumed || restorationCarryover,
      continued_active_mapping: continuedActiveMapping,
      initial_map_snapshot: initialSnapshot,
      screenshot: path.join(dir, 'session-restored.png') });
    await page.screenshot({ path: path.join(dir, 'session-restored.png'), fullPage: true });

    await page.getByRole('tab', { name: 'CONTROL', exact: true }).click();
    const forward = page.getByRole('button', { name: 'Forward (W / ↑)', exact: true });
    await forward.waitFor({ state: 'visible', timeout: 30000 });
    const gateDeadline = Date.now() + 150000;
    let gate = null;
    while (Date.now() < gateDeadline) {
      gate = readJson('teleop-gate.json');
      if (gate?.state === 'BLOCKED') {
        writeJson('browser-result.json', {
          errors: browserErrors, motion_started: false,
          readiness_gate: gate, restored_notice: restoredNotice,
          resume_responses: resumeResponses, restoration_carryover: restorationCarryover,
          initial_map_snapshot: initialSnapshot, extended_map_snapshot: null,
          teleop_start: null, teleop_end: null, mapping_paused: false,
        });
        return;
      }
      if (gate?.state === 'APPROVED'
          && Date.now() - Number(gate.updated_at_ms || 0) <= 500) break;
      gate = null;
      await sleep(100);
    }
    if (!gate || gate.state !== 'APPROVED'
        || Date.now() - Number(gate.updated_at_ms || 0) > 500) {
      writeJson('browser-result.json', {
        errors: browserErrors, motion_started: false,
        readiness_gate: gate, readiness_gate_timeout: true,
        restored_notice: restoredNotice, resume_responses: resumeResponses,
        restoration_carryover: restorationCarryover,
        initial_map_snapshot: initialSnapshot, extended_map_snapshot: null,
        teleop_start: null, teleop_end: null, mapping_paused: false,
      });
      return;
    }
    // Control remains disabled while the supervisor reconnects the authenticated
    // R01 bridge after switching from Navigation to Mapping. Wait for the real
    // UI online state only after ROS readiness has passed; do not fail merely
    // because the page rendered before that handover completed.
    const manualModeButton = page.locator('.robot-detail-manual-mode')
      .getByRole('button', { name: 'MANUAL', exact: true });
    try {
      await manualModeButton.waitFor({ state: 'visible', timeout: 30000 });
      await page.waitForFunction(() => {
        const button = document.querySelector('.robot-detail-manual-mode button');
        return button && !button.disabled;
      }, null, { timeout: 30000 });
    } catch (error) {
      const mode = (await page.locator('.robot-detail-mode').textContent().catch(() => ''))?.trim();
      const runtime = (await page.locator('.robot-detail-runtime').textContent().catch(() => ''))?.trim();
      await page.screenshot({ path: path.join(dir, 'teleop-not-ready.png'), fullPage: true }).catch(() => {});
      throw Error(`Web control did not reconnect after readiness; mode=${mode}; runtime=${runtime}; ${error.message}`);
    }
    const modeText = (await page.locator('.robot-detail-mode').textContent() || '').trim();
    if (!modeText.startsWith('MANUAL APPLIED')) {
      await manualModeButton.click();
      await page.waitForFunction(() =>
        document.querySelector('.robot-detail-mode')?.textContent?.trim() === 'MANUAL APPLIED',
      null, { timeout: 20000 });
    }
    if (await forward.isDisabled()) {
      throw Error('Web Teleop Forward is still disabled after bridge reconnect and MANUAL application');
    }
    gate = readJson('teleop-gate.json');
    if (!hasFreshReadinessLease(gate)) {
      const renewed = await waitForFreshReadinessLease(60000);
      if (!hasFreshReadinessLease(renewed.gate)) {
        const failed = renewed.gate?.failed_gates ||
          readinessGateNames.filter(name => renewed.gate?.gates?.[name] !== true);
        throw Error(`post-resume readiness lease did not renew before Forward; blocked=${renewed.blocked}; failed=${failed.join(',') || 'lease_timeout'}`);
      }
      gate = renewed.gate;
    }
    // Read the lease again at the last possible moment. A stale or incomplete
    // gate always prevents the command, even after Web controls reconnect.
    gate = readJson('teleop-gate.json');
    if (!hasFreshReadinessLease(gate)) {
      throw Error('post-resume readiness lease expired immediately before Forward');
    }
    // Keep each dead-man press bounded while allowing a measured approach to a
    // verified unmapped frontier; key-up and the explicit Web STOP remain required.
    const holdS = Math.max(2, Math.min(20, Number(process.env.SLAM_RESUME_TELEOP_HOLD_S || 3.5)));
    const commandAtMs = Date.now();
    const leaseAgeMs = commandAtMs - Number(gate.updated_at_ms || 0);
    if (gate.state !== 'APPROVED' || leaseAgeMs < 0 || leaseAgeMs > 500
        || readinessGateNames.length !== Object.keys(gate.gates || {}).length
        || !readinessGateNames.every(name => gate.gates?.[name] === true)) {
      throw Error('post-resume readiness lease expired immediately before Forward');
    }
    writeJson('teleop-start.json', { at_ms: commandAtMs, action: 'FORWARD', hold_s: holdS,
      initial_map_known_cells: initialSnapshot.known_cells,
      mapping_session_id: initialSnapshot.mapping_session_id,
      readiness_gate_state: gate.state,
      readiness_gate_updated_at_ms: gate.updated_at_ms,
      readiness_lease_age_ms: leaseAgeMs,
      readiness_gates: gate.gates });
    await page.keyboard.down('ArrowUp');
    await sleep(holdS * 1000);
    await page.keyboard.up('ArrowUp');
    await page.getByRole('button', { name: 'Stop', exact: true }).click();
    writeJson('teleop-end.json', { at_ms: Date.now(), action: 'FORWARD',
      manual_commands_sent: sent.filter(row => row.message.action === 'FORWARD').length,
      manual_stop_commands_sent: sent.filter(row => row.message.action === 'STOP').length });
    writeJson('websocket-manual-frames.json', sent);

    const extensionDeadline = Date.now() + 90000;
    let extendedSnapshot = null;
    while (Date.now() < extensionDeadline) {
      const candidate = received.findLast(row => row.message.type === 'MAP_SNAPSHOT'
        && row.message.map?.map_source === 'SLAM_TOOLBOX'
        && row.message.map?.mapping_session_id === initialSnapshot.mapping_session_id)?.message.map || null;
      if (candidate && (Number(candidate.known_cells) > Number(initialSnapshot.known_cells)
          || candidate.width !== initialSnapshot.width || candidate.height !== initialSnapshot.height)) {
        extendedSnapshot = candidate;
        break;
      }
      await sleep(150);
    }
    if (!extendedSnapshot) {
      extendedSnapshot = received.findLast(row => row.message.type === 'MAP_SNAPSHOT'
        && row.message.map?.map_source === 'SLAM_TOOLBOX'
        && row.message.map?.mapping_session_id === initialSnapshot.mapping_session_id)?.message.map || null;
    }
    await sleep(2000);
    extendedSnapshot = received.findLast(row => row.message.type === 'MAP_SNAPSHOT'
      && row.message.map?.map_source === 'SLAM_TOOLBOX'
      && row.message.map?.mapping_session_id === initialSnapshot.mapping_session_id)?.message.map || extendedSnapshot;
    if (extendedSnapshot) {
      writeJson('extended-map.json', extendedSnapshot);
      await page.screenshot({ path: path.join(dir, 'resumed-map-extended.png'), fullPage: true });
    }
    writeJson('extended-map-ready.json', {
      snapshot_available: Boolean(extendedSnapshot),
      mapping_session_id: extendedSnapshot?.mapping_session_id || null,
    });
    const extensionDecisionDeadline = Date.now() + 30000;
    let extensionDecision = null;
    while (Date.now() < extensionDecisionDeadline) {
      extensionDecision = readJson('map-extension-decision.json');
      if (extensionDecision?.resolved === true) break;
      await sleep(100);
    }
    if (extensionDecision?.resolved !== true || extensionDecision.passed !== true
        || extensionDecision.evidence?.passed !== true) {
      writeJson('browser-result.json', {
        errors: browserErrors, motion_started: true, readiness_gate: gate,
        restored_notice: restoredNotice, resume_responses: resumeResponses,
        restoration_carryover: restorationCarryover,
        initial_map_snapshot: initialSnapshot, extended_map_snapshot: extendedSnapshot,
        map_extension_decision: extensionDecision || {
          resolved: false, passed: false,
          reason: 'world-coordinate extension decision timed out',
        },
        teleop_start: readJson('teleop-start.json'),
        teleop_end: readJson('teleop-end.json'),
        mapping_paused: false, resumed_map_save: null,
      });
      return;
    }

    await page.getByRole('tab', { name: 'MAPPING', exact: true }).click();
    const stopMapping = page.getByRole('button', { name: 'STOP MAPPING', exact: true });
    await stopMapping.waitFor({ state: 'visible', timeout: 30000 });
    if (!(await stopMapping.isDisabled())) await stopMapping.click();
    await page.getByText('MAPPING PAUSED', { exact: true }).waitFor({ state: 'visible', timeout: 30000 });
    const resumedMapName = `${mapName}_resumed_${Date.now()}`.slice(0, 64);
    const mapNameInput = page.locator('.local-form-row input').first();
    await mapNameInput.fill(resumedMapName);
    await page.getByRole('button', { name: 'SAVE MAP', exact: true }).click();
    await page.locator('.local-feedback.ok')
      .filter({ hasText: `SAVE SUCCESS · ${resumedMapName}` })
      .waitFor({ state: 'visible', timeout: 120000 });
    const resumedMapRow = page.locator('.local-map-row').filter({ hasText: resumedMapName }).first();
    await resumedMapRow.waitFor({ state: 'visible', timeout: 30000 });
    const resumedMapRowText = (await resumedMapRow.textContent() || '').trim();
    if (!resumedMapRowText.includes('SLAM AVAILABLE')) {
      throw Error(`new resumed map ${resumedMapName} did not register a serialized SLAM session`);
    }
    const resumedMapSave = {
      name: resumedMapName,
      registry_row_text: resumedMapRowText,
      initial_map_name: mapName,
      distinct_name: resumedMapName !== mapName,
    };
    writeJson('resumed-map-save.json', resumedMapSave);
    writeJson('browser-result.json', {
      errors: browserErrors, motion_started: true,
      readiness_gate: gate, restored_notice: restoredNotice,
      resume_responses: resumeResponses, restoration_carryover: restorationCarryover,
      initial_map_snapshot: initialSnapshot,
      extended_map_snapshot: extendedSnapshot,
      map_extension_decision: extensionDecision,
      map_extension_observed: Boolean(extendedSnapshot && (
        Number(extendedSnapshot.known_cells) > Number(initialSnapshot.known_cells)
        || extendedSnapshot.width !== initialSnapshot.width
        || extendedSnapshot.height !== initialSnapshot.height)),
      teleop_start: readJson('teleop-start.json'),
      teleop_end: readJson('teleop-end.json'), mapping_paused: true,
      resumed_map_save: resumedMapSave,
      screenshots: [path.join(dir, 'session-restored.png'),
        ...(extendedSnapshot ? [path.join(dir, 'resumed-map-extended.png')] : [])],
    });
    if (browserErrors.length) throw Error(`browser page error: ${browserErrors.join('; ')}`);
  } finally { await browser.close(); }
})().catch(error => {
  console.error(error.stack || error.message);
  const previous = readJson('browser-result.json') || {};
  const restored = readJson('session-restored.json') || {};
  writeJson('browser-result.json', {
    ...previous,
    errors: [...(previous.errors || []), error.message || String(error)],
    failure_stack: error.stack || null,
    restored_notice: previous.restored_notice || restored.restored_notice || null,
    resume_responses: previous.resume_responses || readJson('resume-responses.json') || [],
    restoration_carryover: previous.restoration_carryover || null,
    initial_map_snapshot: previous.initial_map_snapshot
      || restored.initial_map_snapshot || null,
    extended_map_snapshot: previous.extended_map_snapshot
      || readJson('extended-map.json') || null,
    teleop_start: previous.teleop_start || readJson('teleop-start.json'),
    teleop_end: previous.teleop_end || readJson('teleop-end.json'),
    mapping_paused: Boolean(previous.mapping_paused),
  });
  process.exitCode = 1;
});
