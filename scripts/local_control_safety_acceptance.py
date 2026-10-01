import subprocess
import tempfile
from pathlib import Path
exec(compile(Path('scripts/local_control_teleop_acceptance.py').read_text().split('results={')[0],
             'scripts/local_control_teleop_acceptance.py', 'exec'))
from std_msgs.msg import Bool
n.create_subscription(Bool,'/emergency_stop',lambda m:record('estop',bool(m.data)),10)
results={}
def save():
 Path(os.environ.get('SAFETY_OUTPUT','.runtime/resume-safety.json')).write_text(json.dumps(results,indent=2))
def wait(predicate,timeout=12):
 deadline=time.monotonic()+timeout
 while time.monotonic()<deadline:
  if predicate():return True
  time.sleep(.01)
 return bool(predicate())
def mode(value):
 start=time.monotonic();ws.send(json.dumps({'type':'ROBOT_MODE','robot_id':'R01','mode':value}))
 assert wait(lambda:any(t>=start and m.get('mode_transition_state')=='APPLIED' and m.get('applied_mode')==value for t,m in list(statuses)) and data['arbiter'][-1][1]['active_control_mode']==value), 'mode not actually APPLIED'
 return start
def begin():
 mode('MANUAL');rest=settle();assert rest['passed'],json.dumps(rest)
 start=time.monotonic();next_send=0;last_send=0
 while time.monotonic()-start<25:
  if time.monotonic()>=next_send:
   command('FORWARD');last_send=time.monotonic();next_send=last_send+.1
  if (data['arbiter'][-1][1]['active_command_source']=='WEB_MANUAL'
   and max(map(abs,data['/cmd_vel_selected'][-1][1]))>.01
   and max(map(abs,data['gazebo'][-1][1]['velocity']))>.02
   and max(map(abs,data['odom'][-1][1]['velocity']))>.02):return last_send
  time.sleep(.01)
 command('STOP');raise AssertionError('no actual live Web manual body/odom motion')
def zero(start,window=.8,timeout=8):
 deadline=time.monotonic()+timeout
 while time.monotonic()<deadline:
  rows=[row for row in list(data['/cmd_vel_selected']) if row[0]>=start]
  nonzero=[row[0] for row in rows if max(map(abs,row[1]))>1e-6]
  last=max(nonzero,default=start-1e-9)
  zeros=[row for row in rows if row[0]>last and max(map(abs,row[1]))<=1e-6]
  if len(zeros)>=30 and zeros[-1][0]-zeros[0][0]>=window and time.monotonic()-zeros[-1][0]<.3:
   return {'passed':True,'zero_latency_ms':1000*(zeros[0][0]-start),'zero_samples':len(zeros),'stable_wall_seconds':zeros[-1][0]-zeros[0][0]}
  time.sleep(.01)
 return {'passed':False,'reason':'selected zero window missing'}
def finish(key,start,**extra):
 z=zero(start);rest=settle() if z['passed'] else {'passed':False}
 owner=data['arbiter'][-1][1]['active_command_source']
 expected='ESTOP' if key=='COMMAND_ARBITER_ESTOP' else 'NONE'
 result={**z,'settling':rest,'owner_after_stop':owner,**extra};result['passed']=bool(z['passed'] and rest['passed'] and owner==expected)
 results[key]=result;save();print(key+'='+json.dumps(result),flush=True)
 assert result['passed'],key
 return result
try:
 assert wait(lambda:sim[0]>0 and all(data[k] for k in ('arbiter','joints','gazebo','odom','/cmd_vel_selected')))
 begin();start=time.monotonic();command('STOP');finish('WEB_MANUAL_STOP',start)
 begin();sender.hold(None);last=next(row['send_completed'] for row in reversed(sender.events) if row['action']=='FORWARD');finish('WEB_MANUAL_TIMEOUT_STOP',last)
 begin();sender.hold(None);start=time.monotonic();closed=True;ws.close();finish('WEB_DISCONNECT_STOP',start)
 ws=websocket.create_connection(backend.replace('http://','ws://')+'/ws?token='+token,timeout=5);ws.settimeout(.1)
 closed=False;reader=threading.Thread(target=receive,daemon=True);reader.start()
 begin();sender.hold(None);start=mode('AUTONOMOUS');finish('MODE_CHANGE_STOP',start,applied_mode=data['arbiter'][-1][1]['active_control_mode'])
 begin();sender.hold(None);start=time.monotonic();response=a.http_json(backend+'/api/robots/R01/emergency-stop',{},token=token)
 assert response['ok']
 assert wait(lambda:any(row[0]>=start and row[1] for row in list(data['estop'])) and data['arbiter'][-1][1]['estop_active'])
 receipt=next(row[0] for row in list(data['estop']) if row[0]>=start and row[1])
 estop=finish('COMMAND_ARBITER_ESTOP',start)
 ros_zero=next(row[0] for row in list(data['/cmd_vel_selected']) if row[0]>=receipt and max(map(abs,row[1]))<=1e-6)
 estop['ros_receipt_zero_latency_ms']=1000*(ros_zero-receipt)
 assert estop['ros_receipt_zero_latency_ms']<=100
 start=time.monotonic();response=a.http_json(backend+'/api/robots/R01/clear-emergency-stop',{},token=token);assert response['ok']
 result=zero(start,window=2)
 result['passed']=bool(result['passed'] and not any(max(map(abs,row[1]))>1e-6 for row in list(data['/cmd_vel_selected']) if row[0]>=start) and not data['arbiter'][-1][1]['estop_active'])
 results['ESTOP_CLEAR_NO_RESUME']=result;save();print('ESTOP_CLEAR_NO_RESUME='+json.dumps(result),flush=True);assert result['passed']
 sender.close();mode('AUTONOMOUS');closed=True;ws.close()
 # Real browser pointer hold/release, with read-only ROS evidence as the release gate.
 epoch_offset=time.time()-time.monotonic()
 evidence_dir=Path(tempfile.mkdtemp(prefix='pointer-resume-',dir='.runtime'))
 process=subprocess.Popen(['node','scripts/local_control_pointer_acceptance.cjs'],env=dict(os.environ,POINTER_ACCEPTANCE_DIR=str(evidence_dir)))
 pointer_ready_timeout=float(os.environ.get('POINTER_READY_TIMEOUT_S','90'))
 ready=wait(lambda:(evidence_dir/'pointer-held.json').exists() or process.poll() is not None,
             timeout=pointer_ready_timeout)
 assert ready, f'browser did not reach pointer hold within {pointer_ready_timeout:g}s'
 assert process.poll() is None, 'browser failed before pointer hold'
 marker=json.loads((evidence_dir/'pointer-held.json').read_text());held_start=marker['down_ms']/1000-epoch_offset
 assert wait(lambda:any(row[0]>=held_start and max(map(abs,row[1]))>.01 for row in list(data['/cmd_vel_manual'])) and any(row[0]>=held_start and max(map(abs,row[1]))>.01 for row in list(data['/cmd_vel_selected'])) and max(map(abs,data['gazebo'][-1][1]['velocity']))>.02,timeout=15)
 (evidence_dir/'pointer-release-ready.json').write_text('{}')
 assert wait(lambda:process.poll() is not None,timeout=20)
 assert process.returncode==0
 pointer=json.loads((evidence_dir/'pointer-result.json').read_text());released=pointer['release_ms']/1000-epoch_offset
 finish('POINTER_RELEASE_STOP',released,wire_stop_seen=pointer['wire_stop_seen'],browser_errors=pointer['errors'])
 results['STAGE_B_SAFETY_GATE']={'passed':all(v.get('passed') for v in results.values())};save()
finally:
 try:
  if 'process' in locals() and process.poll() is None:
   process.terminate()
   try:process.wait(timeout=3)
   except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)
 except Exception:pass
 try:
  if not closed:command('STOP');mode('AUTONOMOUS');ws.close()
 except Exception:pass
 closed=True;rclpy.shutdown();thread.join(3);n.destroy_node()
