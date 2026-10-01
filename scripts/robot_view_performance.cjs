// Real browser/Web/ROS visualization timing. No sensor or robot mocks.
const fs = require('fs');
const { chromium } = require('../waretwin/frontend/node_modules/playwright');
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
const output = process.env.VIEW_PERFORMANCE_OUTPUT || '.runtime/view-performance.json';
const count = Number(process.env.VIEW_SWITCH_COUNT || 10);
const summary = values => {
  const sorted=[...values].sort((a,b)=>a-b), mid=Math.floor(sorted.length/2);
  return {median:sorted.length?(sorted.length%2?sorted[mid]:(sorted[mid-1]+sorted[mid])/2):null,
    max:sorted.length?Math.max(...sorted):null,count:sorted.length};
};
(async () => {
  let login;
  for(let attempt=0;attempt<30;attempt++) {
    try { login=await fetch(`${process.env.BACKEND_URL}/api/auth/login`, {
      method:'POST', headers:{'Content-Type':'application/json'},
      body:JSON.stringify({username:process.env.TWIN_ADMIN_USERNAME,password:process.env.TWIN_ADMIN_PASSWORD}),
    });break; }catch(error){if(attempt===29)throw error;await sleep(1000);}
  }
  if (!login.ok) throw Error(`login HTTP ${login.status}`);
  const auth = await login.json();
  const browser = await chromium.launch({headless:true,args:['--no-sandbox']});
  const result = { switches:[], errors:[] };
  let page;
  try {
    const context = await browser.newContext({viewport:{width:1500,height:1000}});
    await context.addInitScript(({auth,origin}) => {
      if (location.origin === origin) localStorage.setItem('waretwin.auth',JSON.stringify({token:auth.access_token,user:auth.user}));
      window.__ROBOT_DETAIL_PERFORMANCE__=true;
      window.__viewTrace=[];
      const add = row => {window.__viewTrace.push(row); if(window.__viewTrace.length>2000)window.__viewTrace.shift();};
      new PerformanceObserver(list => list.getEntries().forEach(e => add({stage:'longtask',at_ms:performance.timeOrigin+e.startTime,duration_ms:e.duration}))).observe({type:'longtask',buffered:true});
      window.addEventListener('robot-detail-performance',event=>add(event.detail));
      const Original=window.WebSocket;
      window.WebSocket=class extends Original {
        constructor(...args){super(...args);this.addEventListener('message',event=>{
          try { const m=JSON.parse(event.data); add({stage:'wire',at_ms:performance.timeOrigin+performance.now(),type:m.type,bytes:event.data.length}); if(['ERROR','ROBOT_DETAIL_VIEW_STATUS','LIDAR_MAP_2D','LIDAR_MAP_3D','MAP_SNAPSHOT','SYSTEM_DIAGNOSTICS'].includes(m.type))
            add({stage:'receive',at_ms:performance.timeOrigin+performance.now(),type:m.type,
              request_id:m.request_id,view:m.applied_view,view_epoch:m.view_epoch,state:m.state,
              code:m.code,message:m.message,
              view_timing:m.view_timing,point_count:m.point_count,
              gazebo_rtf:m.diagnostics?.gazebo_rtf,web_output_fps:m.web_output_fps,
              map_revision:m.map?.active_map_revision}); }catch{}
        });}
        send(raw){try{const m=JSON.parse(raw);if(m.type==='ROBOT_DETAIL_VIEW')add({stage:'send',at_ms:performance.timeOrigin+performance.now(),...m});}catch{}return super.send(raw);}
      };
    },{auth,origin:new URL(process.env.FRONTEND_URL).origin});
    page=await context.newPage();
    const network=await context.newCDPSession(page);
    await network.send('Network.enable');
    result.network=[];
    network.on('Network.webSocketFrameReceived', event => {
      try { const m=JSON.parse(event.response.payloadData); if(m.type==='ROBOT_DETAIL_VIEW_STATUS')result.network.push({request_id:m.request_id,observed_ms:Date.now(),network_seconds:event.timestamp}); }catch{}
    });
    page.on('pageerror',e=>result.errors.push(e.message));
    await page.goto(`${process.env.FRONTEND_URL}/robots/R01/control`,{waitUntil:'domcontentloaded',timeout:60000});
    await page.locator('.robot-map-source-bar').waitFor({timeout:60000});
    await page.waitForFunction(()=>[...document.querySelectorAll('.robot-detail-header-actions button')].some(b=>b.textContent==='MANUAL'&&!b.disabled),{timeout:60000});
    await page.waitForFunction(()=>document.querySelector('.robot-detail-mode')?.textContent?.includes('APPLIED'),{timeout:60000});
    async function click(view){
      return page.evaluate(view=>{
        const button=[...document.querySelectorAll('.robot-map-source-bar button')].find(b=>b.textContent===({'GLOBAL':'GLOBAL MAP','LIDAR_2D':'2D','LIDAR_3D':'3D'}[view]));
        const start=performance.timeOrigin+performance.now();
        if(!button)throw Error(`view button ${view} not found`);
        button.click();return start;
      },view);
    }
    async function switchView(from,to,record=true){
      let t0;
      if(from==='GLOBAL'){
        t0=await page.evaluate(()=>{const b=[...document.querySelectorAll('.robot-map-source-bar button')].find(b=>b.textContent==='LIDAR MAP');const t=performance.timeOrigin+performance.now();b.click();return t;});
        // The retained dimension is 2D at every GLOBAL -> 2D transition.
      } else t0=await click(to);
      const deadline=Date.now()+20000;
      let rows=[],send,ack,render,frame;
      while(Date.now()<deadline){
        rows=await page.evaluate(t=>window.__viewTrace.filter(r=>r.at_ms>=t),t0);
        send=rows.find(r=>r.stage==='send'&&r.view===to);
        ack=rows.find(r=>r.type==='ROBOT_DETAIL_VIEW_STATUS'&&r.view===to&&(!send?.request_id||r.request_id===send.request_id));
        render=rows.find(r=>r.stage==='view_render'&&r.view===to&&r.useful);
        frame=rows.find(r=>r.stage==='receive'&&r.type===({'GLOBAL':'MAP_SNAPSHOT','LIDAR_2D':'LIDAR_MAP_2D','LIDAR_3D':'LIDAR_MAP_3D'}[to])&&(!send?.request_id||r.view_timing?.request_id===send.request_id));
        if(render&&ack&&(to==='GLOBAL'||frame))break;
        await sleep(25);
      }
      const row={from,to,t0,send,ack,render,frame,
        cached_switch_ms:render?render.at_ms-t0:null,
        applied_ms:ack?ack.at_ms-t0:null,
        fresh_ms:to==='GLOBAL'?(ack&&render?Math.max(ack.at_ms,render.at_ms)-t0:null):(frame?frame.at_ms-t0:null),
        map_messages:rows.filter(r=>r.type==='MAP_SNAPSHOT').length,
        raster_builds:rows.filter(r=>r.stage==='occupancy_raster').length,
        canvas_mounts:rows.filter(r=>r.stage==='canvas_3d_created').length,
        occupancy_timings:rows.filter(r=>r.stage==='occupancy_decode'||r.stage==='occupancy_raster')};
      if(!ack)row.transition_trace=rows.filter(r=>r.stage==='send'||r.type==='ROBOT_DETAIL_VIEW_STATUS'||r.type==='ERROR');
      if(record){result.switches.push(row);fs.writeFileSync(output,JSON.stringify(result,null,2));}
      if(!ack||!render||(to!=='GLOBAL'&&!frame))throw Error(`incomplete ${from} -> ${to}: ${JSON.stringify(row)}`);
      await sleep(100);
    }
    await switchView('GLOBAL','LIDAR_2D',false);
    await switchView('LIDAR_2D','LIDAR_3D',false);
    await switchView('LIDAR_3D','LIDAR_2D',false);
    await switchView('LIDAR_2D','GLOBAL',false);
    const cdp=await context.newCDPSession(page);
    await cdp.send('HeapProfiler.collectGarbage');
    result.heap_before=(await cdp.send('Runtime.getHeapUsage')).usedSize;
    if(process.env.VIEW_BROWSER_PROFILE==='1') {
      await cdp.send('Profiler.enable');await cdp.send('Profiler.start');
    }
    for(let i=0;i<count;i++)for(const [a,b] of [['GLOBAL','LIDAR_2D'],['LIDAR_2D','LIDAR_3D'],['LIDAR_3D','LIDAR_2D'],['LIDAR_2D','GLOBAL']])await switchView(a,b);
    if(process.env.VIEW_BROWSER_PROFILE==='1') {
      fs.writeFileSync('.runtime/view-browser.profile.json',JSON.stringify((await cdp.send('Profiler.stop')).profile));
    }
    await cdp.send('HeapProfiler.collectGarbage');
    result.heap_after=(await cdp.send('Runtime.getHeapUsage')).usedSize;
    result.fps={};
    if(process.env.VIEW_SAMPLE_FPS==='1'){
      await switchView('GLOBAL','LIDAR_2D',false);
      for(const view of ['LIDAR_2D','LIDAR_3D']){
        if(view==='LIDAR_3D')await switchView('LIDAR_2D',view,false);
        const start=Date.now();await sleep(6000);
        const times=await page.evaluate(({start,view})=>window.__viewTrace.filter(r=>r.stage==='receive'&&r.type===`LIDAR_MAP_${view.slice(-2)}`&&r.at_ms>=start).map(r=>r.at_ms),{start,view});
        result.fps[view]=times.length>1?(times.length-1)*1000/(times.at(-1)-times[0]):null;
      }
      await switchView('LIDAR_3D','LIDAR_2D',false);await switchView('LIDAR_2D','GLOBAL',false);
    }
    const trace=await page.evaluate(()=>window.__viewTrace);
    result.trace=trace;
    result.gazebo_rtf=summary(trace.filter(r=>r.gazebo_rtf!=null).map(r=>r.gazebo_rtf));
    result.summary={};
    for(const key of ['GLOBAL->LIDAR_2D','LIDAR_2D->LIDAR_3D','LIDAR_3D->LIDAR_2D','LIDAR_2D->GLOBAL']){
      const rows=result.switches.filter(r=>`${r.from}->${r.to}`===key);
      result.summary[key]={cached_ms:summary(rows.map(r=>r.cached_switch_ms).filter(v=>v!=null)),applied_ms:summary(rows.map(r=>r.applied_ms).filter(v=>v!=null)),fresh_ms:summary(rows.map(r=>r.fresh_ms).filter(v=>v!=null))};
    }
    result.passed=result.errors.length===0&&result.switches.length===count*4
      &&result.switches.every(r=>r.cached_switch_ms<=100&&r.applied_ms<=250&&r.fresh_ms<=({'LIDAR_2D':500,'LIDAR_3D':1000,'GLOBAL':250}[r.to]));
    console.log(JSON.stringify({summary:result.summary,passed:result.passed,errors:result.errors},null,2));
  }catch(e){result.error=e.message;if(page)result.trace=await page.evaluate(()=>window.__viewTrace).catch(()=>[]);throw e;}
  finally{fs.writeFileSync(output,JSON.stringify(result,null,2));await browser.close();}
})().catch(e=>{console.error(e.message);process.exitCode=1;});
