#!/usr/bin/env python3
import array,json,pathlib,socket,struct
path='/research-ws/socket-broker.sock';cases=[]
def request(name,payload,fds):
 c=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);c.settimeout(3);c.connect(path)
 try:
  ancillary=[(socket.SOL_SOCKET,socket.SCM_RIGHTS,array.array('i',fds))] if fds else []
  c.sendmsg([payload],ancillary)
  reply=c.recv(4);code=struct.unpack('!I',reply)[0] if len(reply)==4 else None
  cases.append({'case':name,'reply':code,'status':'PASS' if code and code!=0 else 'FAIL'})
 except OSError as e:cases.append({'case':name,'reply_error':repr(e),'status':'PASS'})
 finally:c.close()
d=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);second=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
try:
 request('wrong_magic',b'BADC'+socket.inet_aton('127.0.0.1'),[d.fileno()])
 request('wrong_interface',b'CABI'+socket.inet_aton('127.0.0.2'),[d.fileno()])
 request('missing_fd',b'CABI'+socket.inet_aton('127.0.0.1'),[])
 request('extra_fd',b'CABI'+socket.inet_aton('127.0.0.1'),[d.fileno(),second.fileno()])
 request('non_ipv4_udp_fd',b'CABI'+socket.inet_aton('127.0.0.1'),[s.fileno()])
finally:d.close();s.close();second.close()
r={'scope':'native broker request denial fixture only; no ROS/control','cases':cases,'status':'PASS' if all(c['status']=='PASS' for c in cases) else 'FAIL'}
pathlib.Path('/research-ws/qualification/fd-broker-denials-optimize.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r),flush=True)
raise SystemExit(0 if r['status']=='PASS' else 1)
