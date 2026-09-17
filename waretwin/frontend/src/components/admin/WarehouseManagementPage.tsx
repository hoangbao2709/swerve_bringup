import { FormEvent, useEffect, useMemo, useState } from "react";
import { warehouseApi, type WarehouseTreeRecord, type WarehouseRecord, type ZoneRecord, type ShelfRecord, type WarehouseMapRecord } from "../../services/warehouse";
import { logout } from "../../services/auth";
import { DEMO_MODE } from "../../config";
import { useStore } from "../../state/store";
import { onLayoutUpdated } from "../../services/ws";

function go(path:string){ window.history.pushState({}, "", path); window.dispatchEvent(new PopStateEvent("popstate")); }
function n(v:unknown, fallback=0){ const x=Number(v); return Number.isFinite(x)?x:fallback; }
function errorText(e:unknown){ return e instanceof Error ? e.message : String(e); }

const EMPTY_W = {code:"",name:"",description:"",status:"ACTIVE",width:100,depth:70,height:16,units:"m"};
const EMPTY_Z = {code:"",name:"",type:"STORAGE",status:"ACTIVE",floor:1,color:"#3b82f6",polygon:"[]",description:""};
const EMPTY_S = {code:"",name:"",type:"STORAGE",status:"AVAILABLE",floor:1,position_x:0,position_y:0,position_z:0,width:3,depth:1.2,height:6,rotation_deg:0,levels:4,capacity:8,current_load:0,access_x:0,access_y:0,access_yaw:0,description:""};

type Editor = {kind:"warehouse"|"zone"|"shelf"; id?:number} | null;

export function WarehouseManagementPage(){
  const me=useStore(s=>s.authUser);
  const [tree,setTree]=useState<WarehouseTreeRecord[]>([]);
  const [maps,setMaps]=useState<WarehouseMapRecord[]>([]);
  const [warehouseId,setWarehouseId]=useState<number|null>(null);
  const [zoneId,setZoneId]=useState<number|null>(null);
  const [shelves,setShelves]=useState<ShelfRecord[]>([]);
  const [search,setSearch]=useState("");
  const [status,setStatus]=useState("");
  const [loading,setLoading]=useState(true);
  const [busy,setBusy]=useState(false);
  const [err,setErr]=useState<string|null>(null);
  const [ok,setOk]=useState<string|null>(null);
  const [editor,setEditor]=useState<Editor>(null);
  const [wf,setWf]=useState<any>(EMPTY_W);
  const [zf,setZf]=useState<any>(EMPTY_Z);
  const [sf,setSf]=useState<any>(EMPTY_S);

  const wh=tree.find(x=>x.id===warehouseId) ?? null;
  const zones=wh?.zones ?? [];
  const zone=zones.find(x=>x.id===zoneId) ?? null;
  const activeMap=maps.find(m=>m.is_active) ?? null;
  const selectedMap=maps.find(m=>m.warehouse_id===warehouseId) ?? null;

  async function loadTree(preferWarehouse?:number|null, preferZone?:number|null){
    const data=await warehouseApi.tree();
    setTree(data);
    const nextWh = (preferWarehouse && data.some(x=>x.id===preferWarehouse)) ? preferWarehouse : (warehouseId && data.some(x=>x.id===warehouseId) ? warehouseId : data[0]?.id ?? null);
    setWarehouseId(nextWh);
    const nextZones=data.find(x=>x.id===nextWh)?.zones ?? [];
    const nextZone=(preferZone && nextZones.some(x=>x.id===preferZone)) ? preferZone : (zoneId && nextZones.some(x=>x.id===zoneId) ? zoneId : nextZones[0]?.id ?? null);
    setZoneId(nextZone);
    return {warehouseId:nextWh, zoneId:nextZone};
  }
  async function loadShelves(zid=zoneId, wid=warehouseId){
    if(!wid){setShelves([]);return;}
    setShelves(await warehouseApi.shelves({warehouse:wid,zone:zid??undefined,search:search||undefined,status:status||undefined}));
  }
  async function refresh(preferWarehouse?:number|null, preferZone?:number|null){
    setLoading(true);setErr(null);
    try{
      const [next, mapRows]=await Promise.all([loadTree(preferWarehouse,preferZone), warehouseApi.maps()]);
      setMaps(mapRows);
      await loadShelves(next.zoneId,next.warehouseId);
    }catch(e){setErr(errorText(e));}finally{setLoading(false);}
  }
  useEffect(()=>{if(DEMO_MODE){setLoading(false);setErr("Warehouse master data requires the Django backend. Set VITE_DEMO_MODE=false and connect the API.");return;}void refresh();},[]);
  useEffect(()=>{
    if(DEMO_MODE)return;
    return onLayoutUpdated((meta)=>{ void refresh(warehouseId===meta.warehouse_id?warehouseId:undefined, zoneId); });
  },[warehouseId,zoneId]);
  useEffect(()=>{const t=setTimeout(()=>void loadShelves().catch(e=>setErr(errorText(e))),180);return()=>clearTimeout(t)},[search,status,zoneId,warehouseId]);

  const summary=useMemo(()=>{
    const total=zone ? shelves.length : (wh?.shelf_count ?? shelves.length);
    return {total,available:shelves.filter(s=>s.status==="AVAILABLE").length,full:shelves.filter(s=>s.status==="FULL").length,disabled:shelves.filter(s=>s.status==="DISABLED"||s.status==="MAINTENANCE").length};
  },[shelves,zone,wh]);

  function editWarehouse(w?:WarehouseRecord){
    setEditor({kind:"warehouse",id:w?.id});
    setWf(w?{...w}:{...EMPTY_W});
  }
  function editZone(z?:ZoneRecord){
    if(!warehouseId&&!z)return setErr("Create or select a warehouse first.");
    setEditor({kind:"zone",id:z?.id});
    setZf(z?{...z,polygon:JSON.stringify(z.polygon??[],null,2)}:{...EMPTY_Z});
  }
  function editShelf(s?:ShelfRecord){
    if(!zoneId&&!s)return setErr("Create or select a zone first.");
    setEditor({kind:"shelf",id:s?.id});
    setSf(s?{
      code:s.code,name:s.name,type:s.type,status:s.status,floor:s.floor,
      position_x:s.position.x,position_y:s.position.y,position_z:s.position.z,
      width:s.size.width,depth:s.size.depth,height:s.size.height,rotation_deg:s.rotation_deg,levels:s.levels,
      capacity:8,current_load:Math.min(8,s.current_load),access_x:s.access_point.x,access_y:s.access_point.y,access_yaw:s.access_point.yaw,
      description:s.description,
    }:{...EMPTY_S,floor:zone?.floor??1});
  }

  async function submitWarehouse(e:FormEvent){e.preventDefault();setBusy(true);setErr(null);try{
    const body={...wf,width:n(wf.width),depth:n(wf.depth),height:n(wf.height)};
    const saved=editor?.id?await warehouseApi.updateWarehouse(editor.id,body):await warehouseApi.createWarehouse(body);
    setEditor(null);setOk(`Warehouse ${saved.code} saved.`);await refresh(saved.id,null);
  }catch(e){setErr(errorText(e));}finally{setBusy(false)}}
  async function submitZone(e:FormEvent){e.preventDefault();if(!warehouseId)return;setBusy(true);setErr(null);try{
    let polygon:any=[];try{polygon=zf.polygon?.trim()?JSON.parse(zf.polygon):[]}catch{throw new Error("Polygon must be valid JSON, e.g. [[0,0],[10,0],[10,10],[0,10]]")}
    const body={...zf,warehouse_id:warehouseId,floor:n(zf.floor,1),polygon};
    const saved=editor?.id?await warehouseApi.updateZone(editor.id,body):await warehouseApi.createZone(body);
    setEditor(null);setOk(`Zone ${saved.code} saved.`);await refresh(saved.warehouse_id,saved.id);
  }catch(e){setErr(errorText(e));}finally{setBusy(false)}}
  async function submitShelf(e:FormEvent){e.preventDefault();if(!zoneId)return;setBusy(true);setErr(null);try{
    const numeric=["floor","position_x","position_y","position_z","width","depth","height","rotation_deg","levels","capacity","current_load","access_x","access_y","access_yaw"];
    const body:any={...sf,zone_id:zoneId,capacity:8};numeric.forEach(k=>body[k]=n(body[k]));
    body.capacity=8; body.current_load=Math.max(0,Math.min(8,body.current_load));
    const saved=editor?.id?await warehouseApi.updateShelf(editor.id,body):await warehouseApi.createShelf(body);
    setEditor(null);setOk(`Shelf ${saved.code} saved.`);await refresh(saved.warehouse_id,saved.zone_id);
  }catch(e){setErr(errorText(e));}finally{setBusy(false)}}

  async function removeWarehouse(w:WarehouseRecord){if(!confirm(`Delete warehouse ${w.code}? It can only be deleted when it has no zones.`))return;try{await warehouseApi.deleteWarehouse(w.id);setOk(`Warehouse ${w.code} deleted.`);await refresh(null,null)}catch(e){setErr(errorText(e))}}
  async function removeZone(z:ZoneRecord){if(!confirm(`Delete zone ${z.code}? It can only be deleted when it has no shelves.`))return;try{await warehouseApi.deleteZone(z.id);setOk(`Zone ${z.code} deleted.`);await refresh(z.warehouse_id,null)}catch(e){setErr(errorText(e))}}
  async function removeShelf(s:ShelfRecord){if(!confirm(`Delete shelf ${s.code}?`))return;try{await warehouseApi.deleteShelf(s.id);setOk(`Shelf ${s.code} deleted.`);await refresh(s.warehouse_id,s.zone_id)}catch(e){setErr(errorText(e))}}
  async function sync(){if(!confirm("Sync Warehouse / Zone / Shelf master data from the currently published warehouse layout? Existing matching records will be updated; manually added unmatched records are kept."))return;setBusy(true);try{const r=await warehouseApi.syncFromLayout();setOk(`Synced ${r.warehouses} warehouse, ${r.zones} zones and ${r.shelves} shelves.`);await refresh()}catch(e){setErr(errorText(e))}finally{setBusy(false)}}
  async function activateSelected(){
    if(!warehouseId)return;
    setBusy(true);setErr(null);
    try{
      await warehouseApi.activateMap(warehouseId);
      setOk(`Warehouse ${wh?.code ?? warehouseId} is now the live map.`);
      await refresh(warehouseId,zoneId);
    }catch(e){setErr(errorText(e))}finally{setBusy(false)}
  }

  return <div className="wm-shell">
    <header className="wm-topbar"><div className="admin-brand"><span className="admin-mark">W</span><div><strong>WareTwin</strong><small>WAREHOUSE MASTER DATA</small></div></div><div className="wm-actions"><span>{me?.username} · ADMIN</span><button className="btn" onClick={()=>go("/admin")}>Admin</button><button className="btn" onClick={()=>go("/admin/warehouse-editor")}>Map Editor</button><button className="btn" onClick={()=>go("/")}>Live Map</button><button className="btn danger" disabled={DEMO_MODE} onClick={()=>void logout().then(()=>go("/login"))}>{DEMO_MODE?"LOCAL DEMO":"Logout"}</button></div></header>
    <div className="wm-heading"><div><div className="eyebrow">ADMIN / MASTER DATA</div><h1>Warehouse · Zone · Shelf</h1><p>One warehouse contains many zones. One zone contains many shelves. Database master data and the live/editor map are one synchronized source. Saved changes propagate to all three pages in realtime.</p></div><div className="wm-heading-actions">
      <span className="admin-badge good">{activeMap ? `LIVE ${activeMap.warehouse_code} · r${activeMap.revision}` : "NO ACTIVE MAP"}</span>
      <button className="btn" disabled={!warehouseId||busy||selectedMap?.is_active} onClick={()=>void activateSelected()}>{selectedMap?.is_active?"✓ Live map":"Use on live map"}</button>
      <button className="btn" disabled={busy} onClick={()=>void sync()}>↻ Reconcile DB ↔ map</button><button className="btn primary" onClick={()=>editWarehouse()}>+ Warehouse</button><button className="btn" disabled={!warehouseId} onClick={()=>editZone()}>+ Zone</button><button className="btn" disabled={!zoneId} onClick={()=>editShelf()}>+ Shelf</button></div></div>
    {err&&<div className="admin-error" onClick={()=>setErr(null)}>{err}</div>}{ok&&<div className="wm-success" onClick={()=>setOk(null)}>{ok}</div>}
    <div className="wm-kpis"><Kpi label="Warehouses" value={tree.length}/><Kpi label="Zones" value={tree.reduce((a,w)=>a+w.zones.length,0)}/><Kpi label="Shelves in view" value={summary.total}/><Kpi label="Available" value={summary.available} tone="good"/><Kpi label="Full" value={summary.full} tone="warn"/><Kpi label="Disabled / Maint." value={summary.disabled} tone="bad"/></div>
    <div className="wm-grid">
      <aside className="wm-tree admin-panel"><div className="panel-title">Warehouse structure</div>{loading?<div className="empty">Loading…</div>:tree.length===0?<div className="empty">No warehouse master data. Use “Sync published layout” or create a warehouse.</div>:tree.map(w=><div key={w.id} className="wm-tree-group"><div className={`wm-tree-row wh ${warehouseId===w.id?"active":""}`} onClick={()=>{setWarehouseId(w.id);setZoneId(w.zones[0]?.id??null)}}><span>▾</span><div><b>{w.code}</b><small>{w.name} · {w.zone_count??w.zones.length} zones · {w.shelf_count??0} shelves</small></div><div className="wm-row-actions"><button onClick={e=>{e.stopPropagation();editWarehouse(w)}}>Edit</button><button onClick={e=>{e.stopPropagation();void removeWarehouse(w)}}>×</button></div></div>{warehouseId===w.id&&w.zones.map(z=><div key={z.id} className={`wm-tree-row zone ${zoneId===z.id?"active":""}`} onClick={()=>setZoneId(z.id)}><span>└</span><div><b>{z.code}</b><small>F{z.floor} · {z.type} · {z.shelf_count??0} shelves</small></div><div className="wm-row-actions"><button onClick={e=>{e.stopPropagation();editZone(z)}}>Edit</button><button onClick={e=>{e.stopPropagation();void removeZone(z)}}>×</button></div></div>)}</div>)}</aside>
      <main className="wm-main admin-panel"><div className="wm-context"><div><div className="panel-title no-border">{zone?`${wh?.code} / ${zone.code}`:wh?.code??"No selection"}</div><small>{zone?`${zone.name} · ${zone.status} · floor ${zone.floor}`:wh?.name??"Select a warehouse"}</small></div>{zone&&<span className="wm-color" style={{background:zone.color}}/>}</div><div className="table-tools"><input value={search} onChange={e=>setSearch(e.target.value)} placeholder="Search shelf code, name, rack id…"/><select value={status} onChange={e=>setStatus(e.target.value)}><option value="">All statuses</option>{["AVAILABLE","OCCUPIED","FULL","RESERVED","DISABLED","MAINTENANCE"].map(x=><option key={x}>{x}</option>)}</select><span>{shelves.length} records</span></div><div className="scroll-table wm-table"><table className="dt full"><thead><tr><th>Shelf</th><th>Status</th><th>Floor</th><th>Load</th><th>Position X/Y</th><th>Access X/Y/Yaw</th><th>Size W×D×H</th><th>Actions</th></tr></thead><tbody>{shelves.map(s=><tr key={s.id}><td><b>{s.code}</b><small className="subcell">{s.name}</small></td><td><Status value={s.status}/></td><td>F{s.floor}</td><td>{s.current_load}/{s.capacity}</td><td>{s.position.x.toFixed(2)} / {s.position.y.toFixed(2)}</td><td>{s.access_point.x.toFixed(2)} / {s.access_point.y.toFixed(2)} / {s.access_point.yaw.toFixed(2)}</td><td>{s.size.width}×{s.size.depth}×{s.size.height}</td><td className="admin-actions"><button className="btn" onClick={()=>editShelf(s)}>Edit</button><button className="btn danger" onClick={()=>void removeShelf(s)}>Delete</button></td></tr>)}</tbody></table>{!loading&&shelves.length===0&&<div className="empty">No shelves match this selection.</div>}</div></main>
      <aside className="wm-detail admin-panel"><div className="panel-title">Selection</div>{zone?<><Detail k="Warehouse" v={`${wh?.code} · ${wh?.name}`}/><Detail k="Map sync" v={selectedMap?`${selectedMap.is_active?"LIVE":"STORED"} · r${selectedMap.revision} · v${selectedMap.published_version}`:"Not initialized"}/><Detail k="Zone" v={`${zone.code} · ${zone.name}`}/><Detail k="Zone type" v={zone.type}/><Detail k="Status" v={zone.status}/><Detail k="Floor" v={`F${zone.floor}`}/><Detail k="Shelves" v={String(zone.shelf_count??shelves.length)}/><div className="wm-mini-map"><svg viewBox={`0 0 ${wh?.width??100} ${wh?.depth??70}`}><rect width={wh?.width??100} height={wh?.depth??70} fill="#07101c"/><polygon points={(zone.polygon??[]).map(p=>p.join(",")).join(" ")} fill={zone.color} fillOpacity=".12" stroke={zone.color} strokeWidth=".5"/>{shelves.slice(0,500).map(s=><rect key={s.id} x={s.position.x} y={s.position.y} width={Math.max(.4,s.size.width)} height={Math.max(.4,s.size.depth)} fill={s.status==="DISABLED"||s.status==="MAINTENANCE"?"#ef4444":"#f59e0b"} opacity=".8"/>)}</svg><small>Zone / shelf position preview</small></div></>:<div className="empty">Select a zone to view shelves.</div>}</aside>
    </div>
    {editor&&<div className="wm-modal-backdrop" onMouseDown={e=>{if(e.target===e.currentTarget)setEditor(null)}}><div className="wm-modal"><div className="wm-modal-head"><div><div className="eyebrow">{editor.id?"EDIT":"CREATE"}</div><h2>{editor.kind[0].toUpperCase()+editor.kind.slice(1)}</h2></div><button className="btn" onClick={()=>setEditor(null)}>✕</button></div>{editor.kind==="warehouse"?<WarehouseForm f={wf} setF={setWf} onSubmit={submitWarehouse} busy={busy}/>:editor.kind==="zone"?<ZoneForm f={zf} setF={setZf} onSubmit={submitZone} busy={busy}/>:<ShelfForm f={sf} setF={setSf} onSubmit={submitShelf} busy={busy}/>}</div></div>}
  </div>
}

function Kpi({label,value,tone=""}:{label:string;value:number|string;tone?:string}){return <div className="admin-kpi"><small>{label}</small><strong className={tone}>{value}</strong></div>}
function Detail({k,v}:{k:string;v:string}){return <div className="wm-detail-row"><span>{k}</span><b>{v}</b></div>}
function Status({value}:{value:string}){const tone=value==="AVAILABLE"||value==="ACTIVE"?"good":value==="FULL"||value==="RESERVED"?"warn":value==="DISABLED"||value==="MAINTENANCE"||value==="BLOCKED"?"bad":"muted";return <span className={`admin-badge ${tone}`}>{value}</span>}
function Field({label,children}:{label:string;children:any}){return <label className="wm-field"><span>{label}</span>{children}</label>}
function Num({value,onChange,min,max,step="0.1",disabled=false}:{value:any;onChange:(v:string)=>void;min?:number;max?:number;step?:string;disabled?:boolean}){return <input type="number" min={min} max={max} step={step} value={value} disabled={disabled} onChange={e=>onChange(e.target.value)}/>}
function WarehouseForm({f,setF,onSubmit,busy}:{f:any;setF:any;onSubmit:any;busy:boolean}){return <form className="wm-form" onSubmit={onSubmit}><div className="wm-form-grid"><Field label="Code *"><input required value={f.code} onChange={e=>setF({...f,code:e.target.value})}/></Field><Field label="Name *"><input required value={f.name} onChange={e=>setF({...f,name:e.target.value})}/></Field><Field label="Status"><select value={f.status} onChange={e=>setF({...f,status:e.target.value})}><option>ACTIVE</option><option>INACTIVE</option></select></Field><Field label="Units"><input value={f.units} onChange={e=>setF({...f,units:e.target.value})}/></Field><Field label="Width"><Num value={f.width} min={0.01} onChange={v=>setF({...f,width:v})}/></Field><Field label="Depth"><Num value={f.depth} min={0.01} onChange={v=>setF({...f,depth:v})}/></Field><Field label="Height"><Num value={f.height} min={0.01} onChange={v=>setF({...f,height:v})}/></Field></div><Field label="Description"><textarea rows={3} value={f.description} onChange={e=>setF({...f,description:e.target.value})}/></Field><button className="btn primary" disabled={busy}>{busy?"Saving…":"Save Warehouse"}</button></form>}
function ZoneForm({f,setF,onSubmit,busy}:{f:any;setF:any;onSubmit:any;busy:boolean}){return <form className="wm-form" onSubmit={onSubmit}><div className="wm-form-grid"><Field label="Code *"><input required value={f.code} onChange={e=>setF({...f,code:e.target.value})}/></Field><Field label="Name *"><input required value={f.name} onChange={e=>setF({...f,name:e.target.value})}/></Field><Field label="Type"><select value={f.type} onChange={e=>setF({...f,type:e.target.value})}>{["STORAGE","PICKING","BUFFER","CHARGING","RESTRICTED","OTHER"].map(x=><option key={x}>{x}</option>)}</select></Field><Field label="Status"><select value={f.status} onChange={e=>setF({...f,status:e.target.value})}>{["ACTIVE","INACTIVE","BLOCKED"].map(x=><option key={x}>{x}</option>)}</select></Field><Field label="Floor"><Num value={f.floor} min={1} step="1" onChange={v=>setF({...f,floor:v})}/></Field><Field label="Color"><input type="color" value={f.color} onChange={e=>setF({...f,color:e.target.value})}/></Field></div><Field label="Polygon JSON"><textarea rows={4} value={f.polygon} onChange={e=>setF({...f,polygon:e.target.value})} placeholder='[[4,10],[46,10],[46,32],[4,32]]'/></Field><Field label="Description"><textarea rows={2} value={f.description} onChange={e=>setF({...f,description:e.target.value})}/></Field><button className="btn primary" disabled={busy}>{busy?"Saving…":"Save Zone"}</button></form>}
function ShelfForm({f,setF,onSubmit,busy}:{f:any;setF:any;onSubmit:any;busy:boolean}){return <form className="wm-form" onSubmit={onSubmit}><div className="wm-form-grid"><Field label="Code *"><input required value={f.code} onChange={e=>setF({...f,code:e.target.value})}/></Field><Field label="Name *"><input required value={f.name} onChange={e=>setF({...f,name:e.target.value})}/></Field><Field label="Type"><select value={f.type} onChange={e=>setF({...f,type:e.target.value})}>{["STORAGE","PICK_FACE","BUFFER","OTHER"].map(x=><option key={x}>{x}</option>)}</select></Field><Field label="Status"><select value={f.status} onChange={e=>setF({...f,status:e.target.value})}>{["AVAILABLE","OCCUPIED","FULL","RESERVED","DISABLED","MAINTENANCE"].map(x=><option key={x}>{x}</option>)}</select></Field><Field label="Floor"><Num value={f.floor} min={1} step="1" onChange={v=>setF({...f,floor:v})}/></Field><Field label="Levels"><Num value={f.levels} min={1} step="1" onChange={v=>setF({...f,levels:v})}/></Field></div><h3 className="wm-form-section">Physical position (map X/Y, elevation Z)</h3><div className="wm-form-grid"><Field label="X"><Num value={f.position_x} min={0} onChange={v=>setF({...f,position_x:v})}/></Field><Field label="Y"><Num value={f.position_y} min={0} onChange={v=>setF({...f,position_y:v})}/></Field><Field label="Z"><Num value={f.position_z} min={0} onChange={v=>setF({...f,position_z:v})}/></Field><Field label="Rotation °"><Num value={f.rotation_deg} onChange={v=>setF({...f,rotation_deg:v})}/></Field></div><h3 className="wm-form-section">Dimensions</h3><div className="wm-form-grid"><Field label="Width"><Num value={f.width} min={0.01} onChange={v=>setF({...f,width:v})}/></Field><Field label="Depth"><Num value={f.depth} min={0.01} onChange={v=>setF({...f,depth:v})}/></Field><Field label="Height"><Num value={f.height} min={0.01} onChange={v=>setF({...f,height:v})}/></Field></div><h3 className="wm-form-section">Capacity</h3><div className="wm-form-grid"><Field label="Capacity (fixed)"><Num value={8} min={8} max={8} step="1" disabled onChange={()=>{}}/></Field><Field label="Orders on shelf"><Num value={f.current_load} min={0} max={8} step="1" onChange={v=>setF({...f,current_load:v})}/></Field></div><h3 className="wm-form-section">Robot access point</h3><div className="wm-form-grid"><Field label="Access X"><Num value={f.access_x} min={0} onChange={v=>setF({...f,access_x:v})}/></Field><Field label="Access Y"><Num value={f.access_y} min={0} onChange={v=>setF({...f,access_y:v})}/></Field><Field label="Access yaw (rad)"><Num value={f.access_yaw} onChange={v=>setF({...f,access_yaw:v})}/></Field></div><Field label="Description"><textarea rows={2} value={f.description} onChange={e=>setF({...f,description:e.target.value})}/></Field><button className="btn primary" disabled={busy}>{busy?"Saving…":"Save Shelf"}</button></form>}
