#!/usr/bin/env python3
"""Finite emulator-only actual UDP socket option broker, never a vehicle node."""
import argparse,array,errno,json,math,os,pathlib,re,signal,socket,struct,time,uuid
p=argparse.ArgumentParser()
p.add_argument('--container-id',required=True)
p.add_argument('--duration-s',type=float,default=100.)
p.add_argument('--output',required=True)
p.add_argument('--socket',default='/dev/shm/codex-arm64-work/socket-broker.sock')
a=p.parse_args()
if re.fullmatch(r'[0-9a-f]{64}',a.container_id) is None:
 p.error('requires full64 lowercase hexadecimal task container ID')
if not math.isfinite(a.duration_s) or not 1.<=a.duration_s<=600.:
 p.error('requires finite duration in[1,600]seconds')
path=pathlib.Path(a.socket);out=pathlib.Path(a.output)
if not path.is_absolute() or len(os.fsencode(str(path)))>=108 or not out.is_absolute():
 p.error('requires absolute bounded Unix socket and output paths')
if path.exists():p.error('existing socket refused')
attempt=str(uuid.uuid4())
with out.open('x') as f:
 json.dump({'state':'INCOMPLETE','attempt_id':attempt},f);f.flush();os.fsync(f.fileno())
s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
start=time.monotonic();records=[];stop=False;bound=False
def terminate(_signal,_frame):
 global stop
 stop=True
signal.signal(signal.SIGINT,terminate);signal.signal(signal.SIGTERM,terminate)
print('FINITE_LOOPBACK_FD_BROKER_RESERVED',flush=True)
failure=None
try:
 s.bind(str(path));bound=True;path.chmod(0o600);s.listen(32);s.settimeout(.2)
 print('FINITE_LOOPBACK_FD_BROKER_READY',flush=True)
 while not stop and time.monotonic()-start<a.duration_s:
  try:c,_=s.accept()
  except socket.timeout:continue
  fds=[];rc=0;record={}
  try:
   c.settimeout(2)
   pid,uid,gid=struct.unpack('3i',c.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,12))
   cg=pathlib.Path('/proc/'+str(pid)+'/cgroup').read_text()
   components=[part for line in cg.splitlines() if len(line.split(':',2))==3
               for part in line.split(':',2)[2].split('/')]
   match=any(part in {a.container_id,'docker-'+a.container_id+'.scope'} for part in components)
   record={'peer_pid':pid,'peer_uid':uid,'container_match':match}
   if not match:raise OSError(errno.EACCES,'not the task container')
   payload,anc,flags,_=c.recvmsg(8,socket.CMSG_SPACE(4*4))
   for level,name,data in anc:
    if level==socket.SOL_SOCKET and name==socket.SCM_RIGHTS:
     values=array.array('i');values.frombytes(data[:len(data)//4*4]);fds.extend(values)
   record.update(received_fd_count=len(fds),request_hex=payload.hex())
   if flags or payload!=b'CABI'+socket.inet_aton('127.0.0.1') or len(fds)!=1:
    raise OSError(errno.EINVAL,'invalid fixed loopback request')
   dup=socket.fromfd(fds[0],socket.AF_INET,socket.SOCK_DGRAM)
   try:
    if (dup.getsockopt(socket.SOL_SOCKET,socket.SO_DOMAIN)!=socket.AF_INET or
        dup.getsockopt(socket.SOL_SOCKET,socket.SO_TYPE)!=socket.SOCK_DGRAM or
        dup.getsockopt(socket.SOL_SOCKET,socket.SO_PROTOCOL)!=socket.IPPROTO_UDP):
     raise OSError(errno.EINVAL,'requires actual IPv4 UDP descriptor')
    dup.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_IF,socket.inet_aton('127.0.0.1'))
    actual=dup.getsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_IF,4)
    if actual!=socket.inet_aton('127.0.0.1'):raise OSError(errno.EIO,'actual interface verification failed')
    record.update(status='PASS',actual_interface=socket.inet_ntoa(actual),native_setsockopt=True)
   finally:dup.close()
  except Exception as e:
   rc=getattr(e,'errno',None) or errno.EINVAL;record.update(status='FAIL',error=rc,exception=repr(e))
  finally:
   for fd in fds:os.close(fd)
   try:c.sendall(struct.pack('!I',rc))
   except OSError:pass
   c.close();records.append(record)
except BaseException as e:failure=repr(e);raise
finally:
 s.close()
 if bound:path.unlink(missing_ok=True)
 result={'state':'FAILED' if failure else 'COMPLETED','attempt_id':attempt,
   'scope':'finite emulator-only native socket option broker; not vehicle code',
   'container_id':a.container_id,'duration_s':time.monotonic()-start,
   'configured_requests':sum(r.get('status')=='PASS' for r in records),
   'failed_requests':sum(r.get('status')!='PASS' for r in records),
   'python_optimize':os.environ.get('PYTHONOPTIMIZE'),'records':records,'failure':failure}
 out.write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({k:v for k,v in result.items() if k!='records'}),flush=True)
