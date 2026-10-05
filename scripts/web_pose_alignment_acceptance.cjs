/* Exercise real production Web renderers and pointer teleop against ROS truth.
 * Run after starting mapping --gui --rviz and pose_alignment_observer.py.
 * Requires POSE_ALIGNMENT_DIR, BACKEND_URL and FRONTEND_URL. */
const fs = require('fs');
const path = require('path');
const assert = require('assert/strict');
const { chromium } = require('../waretwin/frontend/node_modules/playwright');
const dir = process.env.POSE_ALIGNMENT_DIR;
const backend = process.env.BACKEND_URL;
const frontend = process.env.FRONTEND_URL;
const rid = process.env.ROBOT_ID || 'R01';
const positionMethod = process.env.POSITION_METHOD || 'gazebo';
const delay = ms => new Promise(r => setTimeout(r, ms));
const truth = () => JSON.parse(fs.readFileSync(path.join(dir, 'observer.json')));
const freshTruth = () => {
  const t = truth();
  assert(Date.now()-t.observer_at_ms < 2000 && Date.now()-t.world_received_at_ms < 2000,
    'independent observer/Gazebo sample is stale');
  assert(t.slam && Math.abs(t.sim_time-t.slam_stamp) <= 2, 'independent SLAM TF sample is stale');
  return t;
};
const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
function compare(actual, expected) {
  const distance = Math.hypot(actual.x-expected.x, actual.y-expected.y);
  const yaw = Math.abs(wrap(actual.yaw-expected.yaw));
  assert(distance <= .05 && yaw <= .05, `alignment error distance=${distance}, yaw=${yaw}`);
  return { actual, expected, distance_error_m: distance, yaw_error_rad: yaw, passed: true };
}
async function settled() {
  const deadline = Date.now()+45000;
  let since = null;
  while (Date.now()<deadline) {
    const t = truth();
    const ready = Date.now()-t.world_received_at_ms < 2000 && t.stopped && t.selected_zero
      && t.slam && Math.abs(t.sim_time-t.slam_stamp) <= 2;
    since = ready ? (since ?? Date.now()) : null;
    if (since && Date.now()-since >= 2500) return t;
    await delay(100);
  }
  throw Error('robot did not settle with fresh Gazebo/TF and zero selected command');
}
(async () => {
  if (!dir || !backend || !frontend) throw Error('missing runtime configuration');
  assert(['gazebo','web-manual'].includes(positionMethod),'invalid position method');
  const browser = await chromium.launch({headless:false, args:['--no-sandbox', '--ozone-platform=x11',
    '--ignore-gpu-blocklist', '--use-angle=gl']});
  const report = { limits:{distance_m:.05,yaw_rad:.05}, position_method:positionMethod,
    render_quality:'medium', render_lights:true, positions:[], browser_errors:[], console_errors:[], failed_requests:[] };
  let control, warehouse, held = false;
  const progress = setInterval(() => {
    fs.writeFileSync(path.join(dir,'progress.json'),JSON.stringify(report,null,2));
    console.log(`PROBE_STAGE=${report.stage ?? 'loading'}`);
  },20000);
  try {
    const context = await browser.newContext({viewport:{width:1200,height:800}});
    warehouse = await context.newPage();
    control = await context.newPage();
    async function screenshot(page, name) {
      const cdp = await context.newCDPSession(page);
      try {
        // Capture the visible browser view directly. Waiting for extra
        // animation frames can stall on software-rendered VMware WebGL.
        const result = await cdp.send('Page.captureScreenshot', {format:'png',fromSurface:false});
        fs.writeFileSync(path.join(dir,name),Buffer.from(result.data,'base64'));
      } finally { await cdp.detach(); }
    }
    for (const page of [warehouse,control]) {
      page.on('pageerror', e => report.browser_errors.push(e.stack ?? e.message));
      page.on('console', m => {
        if (m.type()==='error') {
          const text=m.text().replace(/([?&]token=)[^&\s]+/g,'$1[redacted]');
          report.console_errors.push(text);
          console.log(`BROWSER_ERROR=${text}`);
        }
      });
      page.on('requestfailed', r => report.failed_requests.push({url:r.url().split('?')[0],error:r.failure()?.errorText}));
    }
    await warehouse.goto(frontend);
    await warehouse.getByRole('button',{name:'MAP VIEW',exact:true}).click();
    await control.goto(`${frontend}/robots/${rid}/control`);
    report.stage = 'manual-mode';
    const state = await (await fetch(`${backend}/api/state`)).json();
    if (state.robots[rid].control_mode !== 'MANUAL') {
      await control.locator('.robot-detail-manual-mode').getByRole('button',{name:'MANUAL',exact:true}).click({timeout:60000});
    }
    await control.waitForFunction(() => document.querySelector('.robot-detail-mode')?.textContent==='MANUAL APPLIED'
      && !document.querySelector('button[aria-label="Forward (W / ↑)"]')?.disabled, null, {timeout:60000});
    console.log('MANUAL_APPLIED_AND_ENABLED');

    async function hold(name, distance, angle = false) {
      await warehouse.getByRole('button',{name:'MAP VIEW',exact:true}).click();
      await control.bringToFront();
      await control.getByRole('tab',{name:'CONTROL',exact:true}).click();
      const button = control.getByRole('button',{name,exact:true});
      const box = await button.boundingBox();
      assert(box, 'manual button missing');
      await settled();
      const initial = truth().world;
      await control.mouse.move(box.x+box.width/2,box.y+box.height/2);
      await control.mouse.down(); held = true;
      const deadline = Date.now()+90000;
      try {
        while (Date.now()<deadline) {
          const t = truth();
          assert(Date.now()-t.world_received_at_ms < 2000, 'Gazebo pose became stale during movement');
          const movement = angle ? Math.abs(wrap(t.world.yaw-initial.yaw)) : Math.hypot(t.world.x-initial.x,t.world.y-initial.y);
          if (movement>=distance) return;
          await delay(100);
        }
        throw Error('motion did not reach measured displacement');
      } finally { await control.mouse.up(); held=false; }
    }
    const initialWorld = (await settled()).world;
    async function place(index) {
      await settled();
      assert(truth().placement_enabled === true,'observer requires explicit --allow-placement');
      // Test targets only; production conversion never uses these deltas.
      const target = {id:`${Date.now()}-${index}`,requested_at_ms:Date.now(),x:initialWorld.x+(index===2?.3:0),
        y:initialWorld.y+index*.4,yaw:initialWorld.yaw-(index===2?.35:0)};
      fs.writeFileSync(path.join(dir,'placement-request.json'),JSON.stringify(target));
      const deadline=Date.now()+45000;
      while (Date.now()<deadline) {
        const t=truth();
        if (t.placement?.id===target.id) {
          assert(t.placement.success,'Gazebo rejected placement');
          if (Math.hypot(t.world.x-target.x,t.world.y-target.y)<.02
            && Math.abs(wrap(t.world.yaw-target.yaw))<.02) return;
        }
        await delay(100);
      }
      throw Error('Gazebo placement was not observed at the requested physical pose');
    }
    for (let index=0;index<3;index++) {
      report.stage = `position-${index+1}`;
      console.log(report.stage);
      if (index>0 && positionMethod==='gazebo') await place(index);
      if (index===1 && positionMethod==='web-manual') await hold('Forward (W / ↑)', .25);
      if (index===2 && positionMethod==='web-manual') {
        await hold('Rotate right (E)', .3, true);
        await settled();
        await hold('Forward (W / ↑)', .25);
      }
      await settled();
      await warehouse.bringToFront();
      await warehouse.getByRole('button',{name:'MAP VIEW',exact:true}).click();
      await warehouse.locator(`g[data-robot-id="${rid}"]`).waitFor({timeout:30000});
      const pose2d = await warehouse.locator(`g[data-robot-id="${rid}"]`).evaluate(el => {
        const m=el.transform.baseVal.consolidate().matrix, line=el.querySelector('line');
        return {x:m.e,y:m.f,yaw:Math.atan2(Number(line.getAttribute('y2')),Number(line.getAttribute('x2'))),source:el.dataset.poseSource};
      });
      const two = compare(pose2d, freshTruth().world);
      report.stage = `position-${index+1}-2d-passed`;
      report.current_measurements = {web_2d:two};
      await screenshot(warehouse,`p${index+1}-2d.png`);
      // Invoke the real tab handler without waiting for a navigation: this
      // switches a canvas in-place and does not navigate the page.
      await warehouse.getByRole('button',{name:'3D VIEW',exact:true}).evaluate(el => el.click());
      const sample3d = await warehouse.waitForFunction(id => {
        const el = document.querySelector(`.lbl[data-robot-id="${id}"]`);
        if (!el || el.dataset.renderX === undefined) return false;
        const sample = {x:Number(el.dataset.renderX),y:Number(el.dataset.renderY),
          yaw:Number(el.dataset.renderYaw),source:el.dataset.poseSource};
        return [sample.x,sample.y,sample.yaw].every(Number.isFinite) ? sample : false;
      }, rid,{timeout:45000});
      const pose3d = await sample3d.jsonValue();
      const three = compare(pose3d, freshTruth().world);
      assert.equal(pose2d.source,'GAZEBO_MODEL_STATES'); assert.equal(pose3d.source,pose2d.source);
      report.stage = `position-${index+1}-3d-passed`;
      report.current_measurements.web_3d = three;
      await screenshot(warehouse,`p${index+1}-3d.png`);
      await warehouse.getByRole('button',{name:'MAP VIEW',exact:true}).evaluate(el => el.click());
      await control.bringToFront();
      await control.getByRole('tab',{name:'MAPPING',exact:true}).click();
      await control.locator(`.local-pose-map canvas[data-robot-id="${rid}"]`).waitFor({timeout:30000});
      const slam = await control.locator('.local-pose-map canvas').evaluate(el => ({
        x:Number(el.dataset.renderX),y:Number(el.dataset.renderY),yaw:Number(el.dataset.renderYaw),source:el.dataset.poseSource }));
      assert.equal(slam.source,'TF');
      const mapping = compare(slam, freshTruth().slam);
      await screenshot(control,`p${index+1}-mapping.png`);
      report.positions.push({index:index+1, web_2d:two, web_3d:three, web_mapping:mapping, truth:truth()});
      for (const previous of report.positions.slice(0,-1)) {
        assert(Math.hypot(previous.web_2d.expected.x-two.expected.x,
          previous.web_2d.expected.y-two.expected.y) >= .2,'robot positions are not distinct');
      }
      delete report.current_measurements;
      console.log(JSON.stringify(report.positions.at(-1)));
    }
    assert.equal(report.browser_errors.length,0);
    report.stage='complete';
    report.passed=true;
  } catch (error) {
    report.passed=false;report.error=error.message;
    if (control) {
      report.control_mode_text = await control.locator('.robot-detail-mode').textContent().catch(()=>null);
      await control.screenshot({path:path.join(dir,'failure-control.png')}).catch(()=>{});
    }
    if (warehouse) await warehouse.screenshot({path:path.join(dir,'failure-warehouse.png')}).catch(()=>{});
    throw error;
  } finally {
    clearInterval(progress);
    if (held) await control.mouse.up();
    fs.writeFileSync(path.join(dir,'result.json'),JSON.stringify(report,null,2));
    await browser.close();
  }
})().catch(e=>{console.error(e.message);process.exitCode=1;});
