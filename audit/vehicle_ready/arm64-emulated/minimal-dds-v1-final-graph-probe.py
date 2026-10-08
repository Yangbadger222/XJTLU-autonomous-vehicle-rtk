#!/usr/bin/env python3
"""Finite DDS diagnosis, separate lab domains, no vehicle nodes or control topics."""
import argparse,json,os,pathlib,platform,subprocess,sys,time,uuid
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from rclpy.utilities import get_rmw_implementation_identifier
p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--domain',type=int,required=True);p.add_argument('--worker',choices=['publisher','subscriber']);p.add_argument('--token');a=p.parse_args()
out=pathlib.Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
assert not out.exists()
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
def write(d):out.write_text(json.dumps(d,indent=2)+'\n')
def ctx():c=Context();rclpy.init(context=c,domain_id=a.domain);return c
def run_pair(separate):
 c1=ctx();c2=ctx() if separate else c1
 pub=Node('minimal_pub',context=c1);sub=Node('minimal_sub',context=c2)
 ep=SingleThreadedExecutor(context=c1);es=SingleThreadedExecutor(context=c2);ep.add_node(pub);es.add_node(sub)
 got=[];token=str(uuid.uuid4());w=pub.create_publisher(String,'/isolated_dds_probe',10)
 sub.create_subscription(String,'/isolated_dds_probe',lambda m:got.append(m.data),10)
 end=time.monotonic()+12;sent=0;graph=[]
 try:
  while time.monotonic()<end:
   w.publish(String(data=token));sent+=1
   ep.spin_once(timeout_sec=.02);es.spin_once(timeout_sec=.02)
   graph=sorted(n for n,ns in sub.get_node_names_and_namespaces())
   if len(got)>=5 and 'minimal_pub' in graph:break
  return {'separate_contexts':separate,'sent':sent,'received':len(got),'all_payloads_equal':all(t==token for t in got),'subscriber_graph':graph,'writer_count':sub.count_publishers('/isolated_dds_probe'),'status':'PASS' if len(got)>=5 and 'minimal_pub' in graph and all(t==token for t in got) else 'FAIL'}
 finally:
  es.shutdown();ep.shutdown();sub.destroy_node();pub.destroy_node();c1.shutdown()
  if separate:c2.shutdown()
if a.worker:
 c=ctx();n=Node('minimal_'+a.worker,context=c);ex=SingleThreadedExecutor(context=c);ex.add_node(n);got=[];sent=0
 w=n.create_publisher(String,'/isolated_dds_cross_process',10) if a.worker=='publisher' else None
 if a.worker=='subscriber':n.create_subscription(String,'/isolated_dds_cross_process',lambda m:got.append(m.data),10)
 end=time.monotonic()+22
 try:
  while time.monotonic()<end:
   if w:w.publish(String(data=a.token));sent+=1
   ex.spin_once(timeout_sec=.05)
  write({'role':a.worker,'pid':os.getpid(),'sent':sent,'received':len(got),'all_payloads_equal':all(t==a.token for t in got),'graph':sorted(name for name,ns in n.get_node_names_and_namespaces()),'publisher_count':n.count_publishers('/isolated_dds_cross_process')})
 finally:ex.shutdown();n.destroy_node();c.shutdown()
 sys.exit(0)
result={'architecture':platform.machine(),'rmw':get_rmw_implementation_identifier(),'domain':a.domain,'localhost_only':True,'environment':{k:os.environ.get(k) for k in ['RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','ROS_DISCOVERY_SERVER','RMW_FASTRTPS_USE_QOS_FROM_XML']},'scope':'finite minimal std_msgs probe only; no vehicle acceptance','cases':[]}
try:
 for separate in [False,True]:result['cases'].append(run_pair(separate))
 token=str(uuid.uuid4());children=[]
 for role in ['subscriber','publisher']:
  path=out.with_name(out.stem+'-'+role+'.json')
  children.append((role,path,subprocess.Popen([sys.executable,__file__,'--output',str(path),'--domain',str(a.domain+1),'--worker',role,'--token',token],start_new_session=True)))
 for role,path,child in children:
  try:child.wait(timeout=40)
  except subprocess.TimeoutExpired:
   import signal
   os.killpg(child.pid,signal.SIGTERM)
   try:child.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
  result[role]=json.loads(path.read_text()) if path.exists() else {'exit':child.returncode,'receipt_missing':True}
 result['cases'].append({'cross_process':True,'status':'PASS' if result['subscriber'].get('received',0)>=5 and result['subscriber'].get('all_payloads_equal') and 'minimal_publisher' in result['subscriber'].get('graph',[]) else 'FAIL'})
 result['status']='PASS' if all(x['status']=='PASS' for x in result['cases']) else 'FAIL'
except Exception as e:
 import traceback
 result['status']='FAIL';result['exception']=repr(e);result['traceback']=traceback.format_exc()
finally:write(result);print(json.dumps(result),flush=True)
sys.exit(0 if result['status']=='PASS' else 1)
