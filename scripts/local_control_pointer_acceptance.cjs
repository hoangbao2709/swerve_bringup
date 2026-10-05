const fs = require('fs');
const path = require('path');
const evidenceDir = process.env.POINTER_ACCEPTANCE_DIR;
if (!evidenceDir) throw Error('POINTER_ACCEPTANCE_DIR must identify a fresh evidence directory');
const { chromium } = require('../waretwin/frontend/node_modules/playwright');
(async () => {
 const browser = await chromium.launch({headless:true,args:['--no-sandbox']});
 const context = await browser.newContext({viewport:{width:1500,height:1000}});
 const page=await context.newPage(); const frames=[]; const errors=[];
 page.on('pageerror',error=>errors.push(error.message));
 page.on('websocket',socket=>socket.on('framesent',event=>{
  try { const m=JSON.parse(event.payload); if (m.type==='ROBOT_MANUAL') frames.push({time_ms:Date.now(),action:m.action}); } catch {}
 }));
 let release_ms=null; let down_ms=null;
 try {
  await page.goto(process.env.FRONTEND_URL+'/robots/R01/control');
  await page.locator('.robot-detail-manual-mode').getByRole('button',{name:'MANUAL',exact:true}).click({timeout:25000});
  await page.waitForFunction(()=>document.querySelector('.robot-detail-mode')?.textContent==='MANUAL APPLIED',{timeout:15000});
  const button=page.getByRole('button',{name:'Forward (W / ↑)',exact:true});
  const box=await button.boundingBox(); if(!box) throw Error('no forward button');
  await page.mouse.move(box.x+box.width/2,box.y+box.height/2);
  // The retained Robot Detail pad is click-latched, not press-and-hold:
  // clicking Forward starts its refreshed command stream until STOP is clicked.
  down_ms=Date.now(); await button.click();
  fs.writeFileSync(path.join(evidenceDir,'pointer-held.json'),JSON.stringify({down_ms}));
  const deadline=Date.now()+18000;
  while(!fs.existsSync(path.join(evidenceDir,'pointer-release-ready.json')) && Date.now()<deadline) await page.waitForTimeout(50);
  if(!fs.existsSync(path.join(evidenceDir,'pointer-release-ready.json'))) throw Error('no observed physical pointer motion');
  release_ms=Date.now();
  await page.getByRole('button',{name:'Stop',exact:true}).click();
  await page.waitForTimeout(1800);
  const wire_stop_seen=frames.some(f=>f.time_ms>=release_ms && f.action==='STOP');
  fs.writeFileSync(path.join(evidenceDir,'pointer-result.json'),JSON.stringify({down_ms,release_ms,wire_stop_seen,frames,errors}));
  if (!wire_stop_seen || errors.length) throw Error('pointer STOP or browser error');
 } finally { await browser.close(); }
})().catch(error=>{console.error(error.message);process.exitCode=1;});
