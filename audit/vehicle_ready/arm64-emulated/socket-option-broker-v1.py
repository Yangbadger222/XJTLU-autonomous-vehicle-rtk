#!/usr/bin/env python3
import array,json,os,pathlib,socket,struct,time,argparse
p=argparse.ArgumentParser();p.add_argument('--container-id',required=True);p.add_argument('--duration-s',type=float,default=100);p.add_argument('--output',required=True);a=p.parse_args()
path=pathlib.Path('/dev/shm/codex-arm64-work/socket-broker.sock');assert not path.exists()
out=pathlib.Path(a.output);assert not out.exists()
s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);s.bind(str(path));path.chmod(0o600);s.listen(32);s.settimeout(.2)
start=time.monotonic();records=[];stop=False
import signal
def terminate(_signal,_frame):
 global stop
 stop=True
signal.signal(signal.SIGINT,terminate);signal.signal(signal.SIGTERM,terminate)
print('FINITE_LOOPBACK_FD_BROKER_READY',flush=True)
try:
 while not stop and time.monotonic()-start<a.duration_s:
  try:c,_=s.accept()
  except socket.timeout:continue
  fds=[];rc=0;record={}
  try:
   c.settimeout(2)
   pid,uid,gid=struct.unpack('3i',c.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,12))
   cg=pathlib.Path('/proc/'+str(pid)+'/cgroup').read_text()
   record={'peer_pid':pid,'peer_uid':uid,'container_match':a.container_id in cg}
   assert record['container_match'],'not task container'
   payload,anc,flags,_=c.recvmsg(8,socket.CMSG_SPACE(4*4))
   for level,name,data in anc:
    if level==socket.SOL_SOCKET and name==socket.SCM_RIGHTS:
     values=array.array('i');values.frombytes(data[:len(data)//4*4]);fds.extend(values)
   assert flags==0 and payload==b'CABI'+socket.inet_aton('127.0.0.1') and len(fds)==1,'invalid request'
   dup=socket.fromfd(fds[0],socket.AF_INET,socket.SOCK_DGRAM)
   try:
    assert dup.family==socket.AF_INET and dup.getsockopt(socket.SOL_SOCKET,socket.SO_TYPE)==socket.SOCK_DGRAM
    dup.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_IF,socket.inet_aton('127.0.0.1'))
    actual=dup.getsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_IF,4)
    assert actual==socket.inet_aton('127.0.0.1')
    record.update(status='PASS',actual_interface=socket.inet_ntoa(actual),native_setsockopt=True)
   finally:dup.close()
  except Exception as e:rc=getattr(e,'errno',None) or 22;record.update(status='FAIL',exception=repr(e))
  finally:
   for fd in fds:os.close(fd)
   try:c.sendall(struct.pack('!I',rc))
   except OSError:pass
   c.close();records.append(record)
finally:
 s.close();path.unlink(missing_ok=True)
 result={'scope':'finite emulator-only native socket option broker; not vehicle code','container_id':a.container_id,'duration_s':time.monotonic()-start,'configured_requests':sum(r.get('status')=='PASS' for r in records),'failed_requests':sum(r.get('status')!='PASS' for r in records),'records':records}
 out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='records'}),flush=True)
