#!/usr/bin/env python3
"""Resume only Web teleop directions with independent refresh and ROS observers.

Run from the checkout after production READY, with ros_env.sh and stack.env
sourced. Set ACCEPTANCE_ROS_DOMAIN_ID from the managed runtime. JSON output
belongs in .runtime, never in source control. No direct ROS motion is published.
"""
import os, sys, time, json, threading, math
from pathlib import Path
from collections import defaultdict, deque
sys.path.insert(0,str(Path.cwd()/'scripts'))
import end_to_end_acceptance as a
from mechanical_settling import MechanicalSettling
from manual_refresh import ManualRefreshWorker
import rclpy, websocket
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray, String
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
assert os.environ.get('ACCEPTANCE_ROS_DOMAIN_ID') is not None, 'managed domain must be supplied'
assert os.environ.get('ROS_DOMAIN_ID') == os.environ['ACCEPTANCE_ROS_DOMAIN_ID'], 'managed ROS domain mismatch'
backend=os.environ['BACKEND_URL'];token=a.authenticate(backend)
ws=websocket.create_connection(backend.replace('http://','ws://')+'/ws?token='+token,timeout=5);ws.settimeout(.1)
statuses=deque(maxlen=200);closed=False
def receive():
 while not closed:
  try:
   m=json.loads(ws.recv())
   if m.get('type') in ('ROBOT_CONTROL_STATUS','ERROR'):statuses.append((time.monotonic(),m))
  except websocket.WebSocketTimeoutException:pass
  except Exception:return
reader=threading.Thread(target=receive,daemon=True);reader.start()
rclpy.init();n=Node('web_teleop_trace');data=defaultdict(lambda:deque(maxlen=12000));sim=[0]
def twist(m):return [m.linear.x,m.linear.y,m.angular.z]
def record(key,value):data[key].append([time.monotonic(),value,sim[0]])
for topic in ('/cmd_vel_manual','/cmd_vel_selected'):
 n.create_subscription(Twist,topic,lambda m,key=topic:record(key,twist(m)),100)
for topic in ('/steering_controller/commands','/drive_controller/commands'):
 n.create_subscription(Float64MultiArray,topic,lambda m,key=topic:record(key,list(m.data)),50)
n.create_subscription(JointState,'/joint_states',lambda m:record('joints',{'positions':dict(zip(m.name,m.position)),'velocities':dict(zip(m.name,m.velocity))}),100)
n.create_subscription(Odometry,'/odom',lambda m:record('odom',{'pose':a.pose_from_odom(m),'velocity':twist(m.twist.twist)}),100)
def model(m):
 if 'swerve_base' in m.name:
  i=m.name.index('swerve_base');record('gazebo',{'pose':a.pose_from_pose(m.pose[i]),'velocity':twist(m.twist[i])})
n.create_subscription(ModelStates,'/model_states',model,100)
n.create_subscription(String,'/command_arbiter/diagnostics',lambda m:record('arbiter',json.loads(m.data)),50)
n.create_subscription(Clock,'/clock',lambda m:sim.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
thread=threading.Thread(target=lambda:rclpy.spin(n),daemon=True);thread.start()
geometry=n.create_client(GetParameters,'/swerve_controller/get_parameters')
assert geometry.wait_for_service(timeout_sec=5),'wheel geometry unavailable'
query=GetParameters.Request();query.names=['wheel_radius'];future=geometry.call_async(query)
deadline=time.monotonic()+5
while not future.done() and time.monotonic()<deadline:time.sleep(.01)
assert future.done(),'wheel geometry timed out'
wheel_radius=future.result().values[0].double_value
assert math.isfinite(wheel_radius) and wheel_radius>0
print('MEASURED_WHEEL_RADIUS='+str(wheel_radius),flush=True)
sender=ManualRefreshWorker(lambda message:(record('client',message),ws.send(json.dumps(message))), 'R01').start()
def command(action):
 if action=='STOP':sender.stop()
 else:sender.hold(action)
def settle(timeout=45):
 monitor=MechanicalSettling(sim[0],time.monotonic(),timeout_sim=timeout,wheel_radius=wheel_radius)
 while True:
  sample=None
  try:
   selected=data['/cmd_vel_selected'][-1][1];joint=data['joints'][-1][1]
   odom=data['odom'][-1][1];body=data['gazebo'][-1][1]
   keys=('joints','odom','gazebo','/cmd_vel_selected','/drive_controller/commands','/steering_controller/commands')
   sample={'selected':selected,'drive':data['/drive_controller/commands'][-1][1],
    'steering_targets':data['/steering_controller/commands'][-1][1],
    'wheel_positions':[joint['positions'][k] for k in ('wheel_front_drive_joint','wheel_rear_drive_joint')],
    'wheel_velocities':[joint['velocities'][k] for k in ('wheel_front_drive_joint','wheel_rear_drive_joint')],
    'steering_positions':[joint['positions'][k] for k in ('steer_front_joint','steer_rear_joint')],
    'body_velocity':body['velocity'],'odom_velocity':odom['velocity'],'body_pose':body['pose'],
    'source_wall_times':[data[k][-1][0] for k in keys]}
  except (IndexError,KeyError):pass
  result=monitor.update(sim[0],time.monotonic(),sample)
  if result is not None:return result
  time.sleep(.02)
results={'directions':{}, 'source':'authenticated Django /ws -> R01 -> real ROS/Gazebo',
 'wheel_position_rate_limit':.005}
def gap(rows):
 stamps=sorted(set(rows))
 return 1000*max((b-a for a,b in zip(stamps,stamps[1:])),default=0)
def ingress(start,stop):
 records=[]
 for line in Path('logs/ros.log').read_text(errors='replace').splitlines():
  if 'MANUAL_RECEIVE_TIMING ' not in line:continue
  try:row=json.loads(line.split('MANUAL_RECEIVE_TIMING ',1)[1])
  except (ValueError,IndexError):continue
  if start<=row.get('T0',-1)<=stop:records.append(row)
 return {'MAX_DJANGO_RECEIVE_GAP_MS':gap([r['T1'] for r in records if 'T1' in r]),
         'MAX_BRIDGE_RECEIVE_GAP_MS':gap([r['T4'] for r in records if 'T4' in r]),
         'ingress_correlated_samples':len(records)}

def save():Path(os.environ.get('TRACE_OUTPUT','.runtime/resume-teleop.json')).write_text(json.dumps(results,indent=2))
try:
 deadline=time.monotonic()+15
 while time.monotonic()<deadline and (sim[0]<=0 or not all(data[k] for k in ('gazebo','odom','joints','arbiter'))):time.sleep(.05)
 assert sim[0]>0, 'no live /clock; no motion permitted'
 mode_start=time.monotonic();ws.send(json.dumps({'type':'ROBOT_MODE','robot_id':'R01','mode':'MANUAL'}))
 deadline=time.monotonic()+10
 while time.monotonic()<deadline and data['arbiter'][-1][1]['active_control_mode']!='MANUAL':time.sleep(.02)
 assert data['arbiter'][-1][1]['active_control_mode']=='MANUAL'
 deadline=time.monotonic()+10
 while time.monotonic()<deadline and not any(t>=mode_start and m.get('applied_mode')=='MANUAL' and m.get('mode_transition_state')=='APPLIED' for t,m in list(statuses)):time.sleep(.02)
 assert any(t>=mode_start and m.get('applied_mode')=='MANUAL' and m.get('mode_transition_state')=='APPLIED' for t,m in list(statuses)), 'no arbiter-confirmed backend mode application'
 for action in os.environ.get('TRACE_ACTIONS','FORWARD,BACKWARD,LEFT,RIGHT,ROTATE_LEFT,ROTATE_RIGHT').split(','):
  rest=settle();results['directions'][action]={'settling':rest};save();print('SETTLE_'+action+'='+json.dumps(rest),flush=True)
  if not rest['passed']:break
  assert not data['arbiter'][-1][1]['estop_active'], 'E-STOP must be known false before motion'
  assert data['arbiter'][-1][1]['active_command_source']=='NONE', 'idle owner must be NONE'
  p0=data['gazebo'][-1][1]['pose'];o0=data['odom'][-1][1]['pose'];start=time.monotonic();s0=sim[0]
  command(action)
  while sim[0]-s0<float(os.environ.get('TRACE_DURATION','2')) and time.monotonic()-start<60:
   assert sender.error is None, str(sender.error)
   time.sleep(.005)
  stop=time.monotonic();stop_sim=sim[0];command('STOP');end_rest=settle()
  observe=float(os.environ.get('OBSERVE_STOP_WALL_SECONDS','0'))
  while time.monotonic()-stop<observe:time.sleep(.02)
  end=time.monotonic();rows={k:[row for row in list(v) if start-1<=row[0]<=end] for k,v in list(data.items())}
  gp=data['gazebo'][-1][1]['pose'];op=data['odom'][-1][1]['pose']
  result={'settling':rest,'stop_settling':end_rest,'requested':action,'sim_duration':sim[0]-s0,'start_sim':s0,'stop_sim':stop_sim,'end_sim':sim[0],'start':start,'stop':stop,'end':end,'gazebo_start':p0,'gazebo_end':gp,'gazebo_delta':a.MotionProbe.body_displacement(p0,gp),'odom_start':o0,'odom_end':op,'odom_delta':a.MotionProbe.body_displacement(o0,op),'trace':rows,'statuses':[v for t,v in statuses if start<=t<=stop+3]}
  component='forward_m' if action in ('FORWARD','BACKWARD') else 'lateral_m' if action in ('LEFT','RIGHT') else 'dyaw_rad'
  sign=-1 if action in ('BACKWARD','RIGHT','ROTATE_RIGHT') else 1
  result['movement_passed']=(sign*result['gazebo_delta'][component]>.05 and sign*result['odom_delta'][component]>.025)
  owner_rows=[r for r in rows['arbiter'] if start<=r[0]<stop]
  acquisition=next((r[0] for r in owner_rows if r[1]['active_command_source']=='WEB_MANUAL'),None)
  result['ownership_continuous']=bool(acquisition is not None and all(r[1]['active_command_source']=='WEB_MANUAL' for r in owner_rows if acquisition<=r[0]<stop))
  result['timing']={**ingress(start,stop),
    'MAX_CLIENT_SEND_GAP_MS':gap([r['T0'] for r in list(sender.events) if r['action']==action and start<=r['T0']<stop]),
    'MAX_CMD_VEL_MANUAL_GAP_MS':gap([r[0] for r in rows['/cmd_vel_manual'] if start<=r[0]<stop]),
    'MAX_SELECTED_CMD_GAP_MS':gap([r[0] for r in rows['/cmd_vel_selected'] if start<=r[0]<stop])}
  result['hold_wall_seconds']=stop-start
  result['hold_sim_seconds']=stop_sim-s0
  result['hold_rtf']=(stop_sim-s0)/(stop-start)
  result['passed']=bool(rest['passed'] and end_rest['passed'] and result['movement_passed'] and result['ownership_continuous'])
  results['directions'][action]=result;save();print('MOTION_'+action+'='+json.dumps({k:v for k,v in result.items() if k not in ('trace','statuses')}),flush=True)
  if not result['passed']:break
finally:
 try:
  sender.close()
  ws.send(json.dumps({'type':'ROBOT_MODE','robot_id':'R01','mode':'AUTONOMOUS'}))
 finally:
  closed=True
  ws.close()
  rclpy.shutdown()
  thread.join(3)
  n.destroy_node()
if not all(result.get('passed') for result in results['directions'].values()):
 raise SystemExit(1)
