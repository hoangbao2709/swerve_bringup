import { useEffect, useState } from "react";
import type { ReactNode, FormEvent } from "react";
import { apiFetch } from "../../services/api";
import { logout } from "../../services/auth";
import { DEMO_MODE } from "../../config";
import { useStore, tickToClock } from "../../state/store";
import type { RobotState, TaskState, TwinEvent } from "../../schema/twin_state";

type User = { id:number; username:string; email:string; role:"admin"|"user"; is_active:boolean; created_at?:string; updated_at?:string };
type Tab = "overview"|"robots"|"tasks"|"users"|"scenarios"|"system";

const TABS: Array<[Tab,string]> = [["overview","Overview"],["robots","Robots"],["tasks","Tasks"],["users","Users"],["scenarios","Scenarios"],["system","System"]];

function go(path:string){ window.history.pushState({}, "", path); window.dispatchEvent(new PopStateEvent("popstate")); }
function Badge({children, tone="muted"}:{children:ReactNode;tone?:string}){ return <span className={`admin-badge ${tone}`}>{children}</span>; }
function statusTone(s:string){ return s==="ERROR"||s==="OFFLINE"||s==="BLOCKED"?"bad":s==="WARNING"||s==="LOW_BATTERY"||s==="CONGESTED"?"warn":s==="CHARGING"||s==="ACTIVE"||s==="NORMAL"?"good":"muted"; }

export function AdminPage(){
 const me=useStore(s=>s.authUser), twin=useStore(s=>s.twin), source=useStore(s=>s.source);
 const [path,setPath]=useState(window.location.pathname);
 const [users,setUsers]=useState<User[]>([]);
 const [events,setEvents]=useState<TwinEvent[]>([]);
 const [health,setHealth]=useState<any>(null);
 const [ai,setAi]=useState<any>(null);
 const [err,setErr]=useState<string|null>(null);
 const [search,setSearch]=useState("");
 const [busy,setBusy]=useState(false);
 const [form,setForm]=useState({username:"",email:"",password:"",role:"user" as "admin"|"user"});

 useEffect(()=>{const f=()=>setPath(window.location.pathname); addEventListener("popstate",f); return()=>removeEventListener("popstate",f)},[]);
 // WebSocket lifecycle is owned globally by App/useBackendRealtime.
 // Admin pages subscribe to the same connection instead of opening/closing a second socket.
 useEffect(()=>{ if(DEMO_MODE) useStore.getState().setSource("local"); },[]);
 const tab:Tab = path.endsWith("/robots")?"robots":path.endsWith("/tasks")?"tasks":path.endsWith("/users")?"users":path.endsWith("/scenarios")?"scenarios":path.endsWith("/system")?"system":"overview";

 async function loadUsers(){ const r=await apiFetch("/api/admin/users"); if(!r.ok) throw Error(await r.text()); setUsers(await r.json()); }
 async function loadAux(){
   const [h,a,e]=await Promise.all([apiFetch("/api/health"),apiFetch("/api/ai/status"),apiFetch("/api/events?limit=100")]);
   if(h.ok)setHealth(await h.json()); if(a.ok)setAi(await a.json()); if(e.ok)setEvents(await e.json());
 }
 useEffect(()=>{if(tab==="users")void loadUsers().catch(e=>setErr(String(e)));},[tab]);
 useEffect(()=>{
   const report = (error: unknown) => console.warn("[admin] auxiliary data refresh failed", error);
   void loadAux().catch(report);
   const id=window.setInterval(()=>void loadAux().catch(report),5000);
   return()=>clearInterval(id)
 },[]);

 const robots=Object.values(twin.robots);
 const tasks=Object.values(twin.tasks);
 const filteredRobots=robots.filter(r=>!search||`${r.id} ${r.model} ${r.zone??""} ${r.status}`.toLowerCase().includes(search.toLowerCase()));
 const filteredTasks=tasks.filter(t=>!search||`${t.id} ${t.type} ${t.status} ${t.assigned_robot??""}`.toLowerCase().includes(search.toLowerCase()));
 const fleet=twin.kpi.fleet;
 const lowBattery=robots.filter(r=>r.battery<20).length;
 const criticalEvents=events.filter(e=>e.severity==="CRITICAL"||e.severity==="HIGH").slice(0,8);

 async function sim(action:"PLAY"|"PAUSE"|"RESET"){
   if(action==="RESET"&&!confirm("Reset the simulation? This changes the live simulation state."))return;
   setBusy(true); try{const r=await apiFetch("/api/sim",{method:"POST",body:JSON.stringify({action})});if(!r.ok)throw Error(await r.text())}catch(e){setErr(String(e))}finally{setBusy(false)}
 }
 async function scenario(kind:string){
   const payload:any={kind};
   if(kind==="HUMAN_INTRUSION")Object.assign(payload,{zone_id:"B",duration_ticks:600});
   if(kind==="CONVEYOR_FAILURE")payload.conveyor_id="CV03";
   if(kind==="ROBOT_FAILURE")payload.robot_id=robots[0]?.id;
   if(kind==="ROBOT_BATTERY_SET")Object.assign(payload,{robot_id:robots[0]?.id,battery:8});
   if(kind==="TRAFFIC_CONGESTION")Object.assign(payload,{zone_id:"B",level:0.9,duration_ticks:600});
   if(!confirm(`Inject ${kind.replace(/_/g, " ")}?`))return;
   setBusy(true);try{const r=await apiFetch("/api/inject",{method:"POST",body:JSON.stringify(payload)});if(!r.ok)throw Error(await r.text())}catch(e){setErr(String(e))}finally{setBusy(false)}
 }
 async function createUser(e:FormEvent){e.preventDefault();setBusy(true);setErr(null);try{const r=await apiFetch("/api/admin/users",{method:"POST",body:JSON.stringify({...form,is_active:true})});if(!r.ok)throw Error(await r.text());setForm({username:"",email:"",password:"",role:"user"});await loadUsers()}catch(e){setErr(String(e))}finally{setBusy(false)}}
 async function patchUser(u:User, body:any){
   if(u.id===me?.id&&(body.is_active===false||body.role==="user")){alert("You cannot disable or demote yourself.");return}
   if(!confirm(`Confirm account change for ${u.username}?`))return;
   const r=await apiFetch(`/api/admin/users/${u.id}`,{method:"PATCH",body:JSON.stringify(body)});if(!r.ok){setErr(await r.text());return}await loadUsers()
 }
 async function delUser(u:User){if(u.id===me?.id)return alert("You cannot delete yourself.");if(!confirm(`Delete ${u.username}? This cannot be undone.`))return;const r=await apiFetch(`/api/admin/users/${u.id}`,{method:"DELETE"});if(!r.ok){setErr(await r.text());return}await loadUsers()}
 async function resetPw(u:User){const p=prompt("New password (8+ chars, letters + digits):");if(!p)return;const r=await apiFetch(`/api/admin/users/${u.id}/reset-password`,{method:"POST",body:JSON.stringify({password:p})});if(!r.ok)setErr(await r.text());}

 const title=TABS.find(x=>x[0]===tab)?.[1]??"Overview";
 return <div className="admin-shell">
  <header className="admin-topbar">
   <div className="admin-brand"><span className="admin-mark">W</span><div><strong>WareTwin</strong><small>ADMIN OPERATIONS</small></div></div>
   <div className="admin-top-status"><span className={`conn-dot ${source==="online"?"online":""}`}/>{source==="online"?"LIVE":"OFFLINE"} <span className="admin-user">{me?.username} · ADMIN</span><button className="btn" onClick={()=>go("/admin/warehouse")}>Warehouse Data</button><button className="btn" onClick={()=>go("/admin/warehouse-editor")}>Warehouse Editor</button><button className="btn" onClick={()=>go("/")}>Console</button><button className={DEMO_MODE ? "btn" : "btn danger"} disabled={DEMO_MODE} onClick={()=>void logout().then(()=>go("/login"))}>{DEMO_MODE ? "LOCAL DEMO" : "Logout"}</button></div>
  </header>
  <div className="admin-layout">
   <aside className="admin-sidebar">
    <div className="admin-section-label">OPERATIONS</div>
    {TABS.slice(0,3).map(([id,label])=><button key={id} className={tab===id?"active":""} onClick={()=>go(id==="overview"?"/admin":`/admin/${id}`)}><span>{id==="overview"?"◈":id==="robots"?"◉":"▤"}</span>{label}</button>)}
    <div className="admin-section-label">ADMINISTRATION</div>
    {TABS.slice(3).map(([id,label])=><button key={id} className={tab===id?"active":""} onClick={()=>go(`/admin/${id}`)}><span>{id==="users"?"♙":id==="scenarios"?"⚡":"⌁"}</span>{label}</button>)}
    <div className="admin-sidebar-foot">SOURCE OF TRUTH<br/><b>TwinState</b><br/>WebSocket FULL / PATCH</div>
   </aside>
   <main className="admin-content">
    <div className="admin-heading"><div><div className="eyebrow">ADMIN CONSOLE / {title.toUpperCase()}</div><h1>{title}</h1></div><div className="sim-actions"><button className="btn primary" disabled={busy} onClick={()=>void sim("PLAY")}>▶ PLAY</button><button className="btn" disabled={busy} onClick={()=>void sim("PAUSE")}>Ⅱ PAUSE</button><button className="btn danger" disabled={busy} onClick={()=>void sim("RESET")}>↻ RESET</button><Badge tone={twin.sim.mode==="PAUSED"?"warn":"good"}>{twin.sim.mode} · {twin.sim.speed}x</Badge></div></div>
    {err&&<div className="admin-error" onClick={()=>setErr(null)}>{err}</div>}
    {tab==="overview"&&<Overview fleet={fleet} lowBattery={lowBattery} robots={robots} tasks={tasks} twin={twin} events={criticalEvents}/>}
    {tab==="robots"&&<RobotTable robots={filteredRobots} search={search} setSearch={setSearch}/>}
    {tab==="tasks"&&<TaskTable tasks={filteredTasks} search={search} setSearch={setSearch}/>}
    {tab==="users"&&<Users users={users} me={me?.id??-1} form={form} setForm={setForm} createUser={createUser} patchUser={patchUser} delUser={delUser} resetPw={resetPw}/>}
    {tab==="scenarios"&&<Scenarios robots={robots} busy={busy} run={scenario}/>}
    {tab==="system"&&<System health={health} ai={ai} source={source} tick={twin.sim.tick}/>}
   </main>
  </div>
 </div>
}

function Overview({fleet,lowBattery,robots,tasks,twin,events}:{fleet:any;lowBattery:number;robots:RobotState[];tasks:TaskState[];twin:any;events:TwinEvent[]}){
 const battery=[0,0,0,0]; robots.forEach(r=>battery[r.battery<20?0:r.battery<50?1:r.battery<80?2:3]++);
 return <div className="admin-stack">
  <div className="admin-kpis">{[["TOTAL",fleet.total,""],["ACTIVE",fleet.active,"good"],["IDLE",fleet.idle,""],["CHARGING",fleet.charging,"good"],["ERROR",fleet.error,"bad"],["OFFLINE",fleet.offline,"bad"],["LOW BATTERY",lowBattery,"warn"],["TASKS",tasks.length,""]].map(([k,v,c])=><div className="admin-kpi" key={String(k)}><small>{k}</small><strong className={String(c)}>{String(v)}</strong></div>)}</div>
  <div className="admin-grid-2">
   <section className="admin-panel"><div className="panel-title">Fleet / Battery Health</div>{[["< 20%",battery[0]],["20–50%",battery[1]],["50–80%",battery[2]],["> 80%",battery[3]]].map(([label,n]:any,i:number)=><div className="bar-row" key={i}><span>{label}</span><div><i style={{width:`${fleet.total?n/fleet.total*100:0}%`}}/></div><b>{n}</b></div>)}</section>
   <section className="admin-panel"><div className="panel-title">Operational KPI</div><div className="metric-list"><Metric k="Throughput" v={`${twin.kpi.operation.throughput_per_min} tasks/min`}/><Metric k="On-time rate" v={`${Math.round(twin.kpi.operation.on_time_rate*100)}%`}/><Metric k="Utilization" v={`${Math.round(twin.kpi.operation.avg_utilization*100)}%`}/><Metric k="Congestion" v={`${Math.round(twin.kpi.efficiency.congestion_index*100)}%`}/><Metric k="Energy" v={`${twin.kpi.efficiency.energy_kwh.toFixed(2)} kWh`}/></div></section>
  </div>
  <div className="admin-grid-2"><section className="admin-panel"><div className="panel-title">Recent High Severity Events</div>{events.length?events.map(e=><div className="event-row" key={e.id}><Badge tone={statusTone(e.severity)}>{e.severity}</Badge><div><b>{e.type}</b><small>{e.message}</small></div><time>{tickToClock(e.tick)}</time></div>):<div className="empty">No high-severity events.</div>}</section>
  <section className="admin-panel"><div className="panel-title">Warehouse Status</div><div className="status-grid">{Object.entries(twin.subsystems).map(([k,v])=><div key={k}><span>{k}</span><Badge tone={statusTone(String(v))}>{String(v)}</Badge></div>)}</div><div className="panel-title second">Simulation</div><div className="sim-readout"><b>Tick {twin.sim.tick}</b><span>{tickToClock(twin.sim.tick,twin.sim.tick_ms,true)}</span><span>{twin.sim.speed}x</span></div></section></div>
 </div>
}
function Metric({k,v}:{k:string;v:string}){return <div><span>{k}</span><b>{v}</b></div>}
function RobotTable({robots,search,setSearch}:{robots:RobotState[];search:string;setSearch:(s:string)=>void}){return <section className="admin-panel table-panel"><TableTools value={search} setValue={setSearch} placeholder="Search robot, zone, status..." count={robots.length}/><div className="scroll-table"><table className="dt full"><thead><tr><th>Robot</th><th>Status</th><th>Battery</th><th>Task</th><th>Position</th><th>Zone</th><th>Speed</th><th>Health</th></tr></thead><tbody>{robots.map(r=><tr key={r.id} onClick={()=>go(`/admin/robots/${r.id}`)}><td><b>{r.id}</b><small className="subcell">{r.model}</small></td><td><Badge tone={statusTone(r.status)}>{r.status}</Badge><small className="subcell">{r.fsm}</small></td><td><b className={r.battery<20?"bad":r.battery<50?"warn":""}>{Math.round(r.battery)}%</b></td><td>{r.current_task_id??"—"}</td><td>{r.position.map(x=>x.toFixed(1)).join(" / ")}</td><td>{r.zone??"—"}</td><td>{r.velocity.toFixed(2)} m/s</td><td>{Math.round(r.health)}%</td></tr>)}</tbody></table></div></section>}
function TaskTable({tasks,search,setSearch}:{tasks:TaskState[];search:string;setSearch:(s:string)=>void}){return <section className="admin-panel table-panel"><TableTools value={search} setValue={setSearch} placeholder="Search task, robot, status..." count={tasks.length}/><div className="scroll-table"><table className="dt full"><thead><tr><th>Task</th><th>Type</th><th>Status</th><th>Priority</th><th>Robot</th><th>Source</th><th>Destination</th><th>ETA</th></tr></thead><tbody>{tasks.map(t=><tr key={t.id}><td><b>{t.id}</b></td><td>{t.type}</td><td><Badge tone={statusTone(t.status)}>{t.status}</Badge></td><td>{t.priority}</td><td>{t.assigned_robot??"—"}</td><td>{t.source}</td><td>{t.destination}</td><td>{t.eta_s==null?"—":`${Math.round(t.eta_s)}s`}</td></tr>)}</tbody></table></div></section>}
function TableTools({value,setValue,placeholder,count}:{value:string;setValue:(v:string)=>void;placeholder:string;count:number}){return <div className="table-tools"><input value={value} onChange={e=>setValue(e.target.value)} placeholder={placeholder}/><span>{count} records</span></div>}
function Users({users,me,form,setForm,createUser,patchUser,delUser,resetPw}:{users:User[];me:number;form:any;setForm:any;createUser:any;patchUser:any;delUser:any;resetPw:any}){return <div className="admin-grid-2"><section className="admin-panel"><div className="panel-title">Create User</div><form className="admin-form" onSubmit={createUser}><input placeholder="Username" value={form.username} onChange={e=>setForm({...form,username:e.target.value})}/><input placeholder="Email" value={form.email} onChange={e=>setForm({...form,email:e.target.value})}/><input placeholder="Password · 8+ letters + digits" type="password" value={form.password} onChange={e=>setForm({...form,password:e.target.value})}/><select value={form.role} onChange={e=>setForm({...form,role:e.target.value})}><option value="user">user</option><option value="admin">admin</option></select><button className="btn primary">Create account</button></form></section><section className="admin-panel table-panel"><div className="panel-title">Accounts · {users.length}</div><div className="scroll-table"><table className="dt full"><thead><tr><th>User</th><th>Role</th><th>Status</th><th>Actions</th></tr></thead><tbody>{users.map(u=><tr key={u.id}><td><b>{u.username}</b><small className="subcell">{u.email}</small></td><td><Badge tone={u.role==="admin"?"good":"muted"}>{u.role}</Badge></td><td><Badge tone={u.is_active?"good":"bad"}>{u.is_active?"ACTIVE":"DISABLED"}</Badge></td><td className="admin-actions"><button className="btn" disabled={u.id===me} onClick={()=>void patchUser(u,{is_active:!u.is_active})}>{u.is_active?"Disable":"Enable"}</button><button className="btn" disabled={u.id===me} onClick={()=>void patchUser(u,{role:u.role==="admin"?"user":"admin"})}>Role</button><button className="btn" onClick={()=>void resetPw(u)}>Reset</button><button className="btn danger" disabled={u.id===me} onClick={()=>void delUser(u)}>Delete</button></td></tr>)}</tbody></table></div></section></div>}
function Scenarios({robots,busy,run}:{robots:RobotState[];busy:boolean;run:(k:string)=>void}){const items=[["HUMAN_INTRUSION","Human intrusion · Zone B"],["CONVEYOR_FAILURE","Conveyor failure · CV03"],["ROBOT_FAILURE",`Robot failure · ${robots[0]?.id??"first robot"}`],["ROBOT_BATTERY_SET",`Low battery · ${robots[0]?.id??"first robot"}`],["TRAFFIC_CONGESTION","Traffic congestion · Zone B"]];return <div className="admin-grid-2">{items.map(([k,l])=><section className="admin-panel scenario-card" key={k}><div className="scenario-icon">⚡</div><div><h3>{l}</h3><p>Inject through the live backend simulation. No frontend-generated state.</p><button className="btn danger" disabled={busy||!robots.length&&k==="ROBOT_FAILURE"} onClick={()=>run(k)}>RUN SCENARIO</button></div></section>)}</div>}
function System({health,ai,source,tick}:{health:any;ai:any;source:string;tick:number}){const rows=[["Backend",health?.ok?"HEALTHY":"DEGRADED"],["Simulation",health?.sim_task_alive?"RUNNING":"STOPPED"],["Database",health?.db_ok?"HEALTHY":"ERROR"],["WebSocket",source==="online"?"CONNECTED":"DISCONNECTED"],["LLM",ai?.llm?"ENABLED":"DISABLED"],["VLM",ai?.vision_model?"ENABLED":"DISABLED"]];return <div className="admin-grid-2"><section className="admin-panel"><div className="panel-title">System Health</div>{rows.map(([k,v])=><div className="health-row" key={k}><span>{k}</span><Badge tone={statusTone(v)}>{v}</Badge></div>)}</section><section className="admin-panel"><div className="panel-title">Runtime</div><Metric k="Simulation tick" v={String(tick)}/><Metric k="Tick rate" v={`${health?.tick_rate??"—"} ticks/s`}/><Metric k="Connected clients" v={String(health?.clients??"—")}/><Metric k="Loop errors" v={String(health?.loop_errors??0)}/><Metric k="AI model" v={ai?.model??"—"}/></section></div>}
