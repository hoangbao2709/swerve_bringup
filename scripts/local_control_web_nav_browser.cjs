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
    if (!modeText.includes('AUTONOMOUS APPLIED')) {
      await page.locator('.robot-detail-manual-mode').getByRole('button',
        { name: 'AUTONOMOUS', exact: true }).click({ timeout: 30000 });
      await page.waitForFunction(() => document.querySelector('.robot-detail-mode')?.textContent === 'AUTONOMOUS APPLIED',
        { timeout: 30000 });
    }
    const mapState = page.locator('.map-sync-warning-ready').first();
    await mapState.waitFor({ state: 'visible', timeout: 60000 });
    const initialMap = (await mapState.textContent()).trim();
    if (!/CANONICAL|LOCAL_ONLY/.test(initialMap)) throw Error(`active robot map is not navigable: ${initialMap}`);
    const canvas = page.locator('canvas.robot-detail-map-canvas');
    await canvas.waitFor({ state: 'visible', timeout: 30000 });
    const box = await canvas.boundingBox();
    if (!box || box.width < 100 || box.height < 100) throw Error('GLOBAL MAP canvas has no usable viewport');

    let preview = null;
    let targetClick = null;
    // Follow centers the map on the robot. Nearby upward clicks choose forward
    // map-space points at short, increasing distances, using the real canvas UI.
    const goalHeading = Number(process.env.NAV_GOAL_HEADING);
    if (!Number.isFinite(goalHeading)) throw Error('ROS-observed goal heading is required');
    for (const radius of [7, 10, 13, 16]) {
      targetClick = { x: box.x + box.width / 2 + Math.cos(goalHeading) * radius,
        y: box.y + box.height / 2 - Math.sin(goalHeading) * radius };
      const previousCount = sent.filter(row => row.message.type === 'PATH_PREVIEW_REQUEST').length;
      await page.mouse.click(targetClick.x, targetClick.y);
      const deadline = Date.now() + 18000;
      while (Date.now() < deadline) {
        const toolbar = (await page.locator('.robot-detail-goal-toolbar span').textContent()) || '';
        const requests = sent.filter(row => row.message.type === 'PATH_PREVIEW_REQUEST');
        const request = requests[requests.length - 1];
        const result = request && received.findLast(row => row.message.type === 'PATH_PREVIEW_RESULT'
          && row.message.request_id === request.message.request_id);
        if (result) {
          preview = { request: request.message, result: result.message, request_at_ms: request.at_ms,
            result_at_ms: result.at_ms, toolbar };
          if (result.message.status === 'VALID' && Array.isArray(result.message.path)
              && result.message.path.length > 0 && toolbar.includes('VALID')) break;
          preview = null;
          break;
        }
        await sleep(100);
      }
      if (preview) break;
      if (sent.filter(row => row.message.type === 'PATH_PREVIEW_REQUEST').length <= previousCount) {
        throw Error('GLOBAL MAP click did not send PATH_PREVIEW_REQUEST');
      }
    }
    if (!preview) throw Error('nearby GLOBAL MAP clicks did not produce a valid Nav2 preview');
    const beforeSendMap = (await mapState.textContent()).trim();
    if (beforeSendMap !== initialMap) throw Error(`active map changed from ${initialMap} to ${beforeSendMap} before Send Goal`);
    await page.screenshot({ path: path.join(dir, 'path-preview.png') });
    const button = page.getByRole('button', { name: 'SEND GOAL', exact: true });
    if (await button.isDisabled()) throw Error('SEND GOAL is disabled after a valid path preview');
    writeJson('path-preview.json', { initial_map: initialMap, before_preview_map: initialMap,
      preview, before_send_map: beforeSendMap, target_click: targetClick,
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
    const finishDeadline = Date.now() + 240000;
    while (!readJson('nav-finished.json') && Date.now() < finishDeadline) await sleep(100);
    await page.screenshot({ path: path.join(dir, 'navigation-result.png'), fullPage: true });
    writeJson('browser-result.json', { errors: browserErrors,
      navigation_status_text: await page.locator('.robot-detail-runtime').textContent().catch(() => null),
      screenshots: [path.join(dir, 'path-preview.png'), path.join(dir, 'navigation-result.png')] });
    if (browserErrors.length) throw Error(`browser page error: ${browserErrors.join('; ')}`);
  } finally { await browser.close(); }
})().catch(error => { console.error(error.stack || error.message); process.exitCode = 1; });
