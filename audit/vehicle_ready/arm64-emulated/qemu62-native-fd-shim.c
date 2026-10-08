#define _GNU_SOURCE
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <dlfcn.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
/* Finite QEMU 6.2 diagnosis only. A native host broker configures the actual
 * duplicated UDP fd to loopback and verifies it. No fabricated success.
 * Not installed in or linked into any vehicle package. */
int setsockopt(int fd,int level,int name,const void *value,socklen_t length) {
 typedef int (*call_t)(int,int,int,const void*,socklen_t);
 call_t call=(call_t)dlsym(RTLD_NEXT,"setsockopt");
 if(!call){errno=ENOSYS;return -1;}
 if(level!=IPPROTO_IP || name!=IP_MULTICAST_IF ||
    length!=sizeof(struct in_addr) || !value)
  return call(fd,level,name,value,length);
 const unsigned char lo[4]={127,0,0,1};
 if(memcmp(value,lo,4)!=0) return call(fd,level,name,value,length);
 const char *path=getenv("RESEARCH_QEMU_FD_BROKER");
 if(!path || strlen(path)>=sizeof(((struct sockaddr_un*)0)->sun_path)){
  errno=EINVAL;return -1;
 }
 int client=socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);
 if(client<0)return -1;
 struct timeval tv={3,0};
 if(call(client,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv))!=0){int e=errno;close(client);errno=e;return -1;}
 struct sockaddr_un address;
 memset(&address,0,sizeof(address));address.sun_family=AF_UNIX;
 memcpy(address.sun_path,path,strlen(path)+1);
 if(connect(client,(struct sockaddr*)&address,sizeof(address))!=0){int e=errno;close(client);errno=e;return -1;}
 unsigned char payload[8]={'C','A','B','I',127,0,0,1};
 struct iovec vector={payload,sizeof(payload)};
 union {struct cmsghdr align;unsigned char buffer[CMSG_SPACE(sizeof(int))];} ancillary;
 memset(&ancillary,0,sizeof(ancillary));
 struct msghdr message;memset(&message,0,sizeof(message));
 message.msg_iov=&vector;message.msg_iovlen=1;
 message.msg_control=ancillary.buffer;message.msg_controllen=sizeof(ancillary.buffer);
 struct cmsghdr *control=CMSG_FIRSTHDR(&message);
 control->cmsg_level=SOL_SOCKET;control->cmsg_type=SCM_RIGHTS;control->cmsg_len=CMSG_LEN(sizeof(int));
 memcpy(CMSG_DATA(control),&fd,sizeof(fd));
 if(sendmsg(client,&message,MSG_NOSIGNAL)!=(ssize_t)sizeof(payload)){int e=errno?errno:EIO;close(client);errno=e;return -1;}
 unsigned char reply[4];size_t received=0;
 while(received<sizeof(reply)){
  ssize_t n=recv(client,reply+received,sizeof(reply)-received,0);
  if(n<=0){int e=errno?errno:EIO;close(client);errno=e;return -1;}
  received+=(size_t)n;
 }
 close(client);
 unsigned int error=((unsigned int)reply[0]<<24)|((unsigned int)reply[1]<<16)|((unsigned int)reply[2]<<8)|reply[3];
 if(error){errno=(int)error;return -1;}
 return 0;
}
