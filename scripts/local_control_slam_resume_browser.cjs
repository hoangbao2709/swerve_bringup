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
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
const readJson = name => {
  try { return JSON.parse(fs.readFileSync(path.join(dir, name), 'utf8')); }
  catch { return null; }
};
const writeJson = (name, value) => fs.writeFileSync(path.join(dir, name), JSON.stringify(value, null, 2));

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
    if (!skipResumeRequest) {
      writeJson('session-restored.json', { restored_notice: restoredNotice,
        resume_response: resumed, initial_map_snapshot: initialSnapshot,
        screenshot: path.join(dir, 'session-restored.png') });
      await page.screenshot({ path: path.join(dir, 'session-restored.png'), fullPage: true });
    }

    await page.getByRole('tab', { name: 'CONTROL', exact: true }).click();
    const forward = page.getByRole('button', { name: 'Forward (W / ↑)', exact: true });
    await forward.waitFor({ state: 'visible', timeout: 30000 });
    if (await forward.isDisabled()) throw Error('Web Teleop Forward is disabled after session restoration');
    const box = await forward.boundingBox();
    if (!box) throw Error('Web Teleop Forward has no pointer target');
    const holdS = Math.max(6, Math.min(20, Number(process.env.SLAM_RESUME_TELEOP_HOLD_S || 15)));
    writeJson('teleop-start.json', { at_ms: Date.now(), action: 'FORWARD', hold_s: holdS,
      initial_map_known_cells: initialSnapshot.known_cells,
      mapping_session_id: initialSnapshot.mapping_session_id });
    await sleep(1200);
    await page.keyboard.down('ArrowUp');
    await sleep(holdS * 1000);
    await page.keyboard.up('ArrowUp');
    await page.getByRole('button', { name: 'Stop', exact: true }).click();
    writeJson('teleop-end.json', { at_ms: Date.now(), action: 'FORWARD',
      manual_commands_sent: sent.filter(row => row.message.action === 'FORWARD').length,
      manual_stop_commands_sent: sent.filter(row => row.message.action === 'STOP').length });
    writeJson('websocket-manual-frames.json', sent);

    const extensionDeadline = Date.now() + 180000;
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
    if (!extendedSnapshot) throw Error('Web Teleop did not produce a later extended SLAM /map snapshot');
    await sleep(2000);
    extendedSnapshot = received.findLast(row => row.message.type === 'MAP_SNAPSHOT'
      && row.message.map?.map_source === 'SLAM_TOOLBOX'
      && row.message.map?.mapping_session_id === initialSnapshot.mapping_session_id)?.message.map || extendedSnapshot;
    writeJson('extended-map.json', extendedSnapshot);
    await page.screenshot({ path: path.join(dir, 'resumed-map-extended.png'), fullPage: true });

    await page.getByRole('tab', { name: 'MAPPING', exact: true }).click();
    const stopMapping = page.getByRole('button', { name: 'STOP MAPPING', exact: true });
    await stopMapping.waitFor({ state: 'visible', timeout: 30000 });
    if (!(await stopMapping.isDisabled())) await stopMapping.click();
    await page.getByText('MAPPING PAUSED', { exact: true }).waitFor({ state: 'visible', timeout: 30000 });
    writeJson('browser-result.json', {
      errors: browserErrors, restored_notice: restoredNotice,
      resume_responses: resumeResponses, restoration_carryover: restorationCarryover,
      initial_map_snapshot: initialSnapshot,
      extended_map_snapshot: extendedSnapshot, teleop_start: readJson('teleop-start.json'),
      teleop_end: readJson('teleop-end.json'), mapping_paused: true,
      screenshots: [path.join(dir, 'session-restored.png'), path.join(dir, 'resumed-map-extended.png')],
    });
    if (browserErrors.length) throw Error(`browser page error: ${browserErrors.join('; ')}`);
  } finally { await browser.close(); }
})().catch(error => { console.error(error.stack || error.message); process.exitCode = 1; });
