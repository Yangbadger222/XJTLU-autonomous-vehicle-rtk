#!/usr/bin/env python3
"""Loopback test proxy: fail state GETs while operator POSTs remain reachable.

Only for the allocated-PTY browser test, never part of the runtime launch.
Logs contain action names/counts only, never headers/cookies/CSRF tokens.
"""
import argparse
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--fault-file",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    records=[]
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def forward(self):
            if self.path=="/api/state" and args.fault_file.exists():
                records.append({"time":time.time(),"kind":"GET_state_blocked"})
                body=b'{"reason":"TEST ONLY: state GET outage"}'
                self.send_response(503);self.send_header("Content-Length",str(len(body)));self.end_headers();self.wfile.write(body)
                args.output.write_text(json.dumps(records[-500:]))
                return
            body=self.rfile.read(int(self.headers.get("Content-Length","0"))) if self.command=="POST" else None
            headers=dict(self.headers)
            headers["Host"]="127.0.0.1:8765"
            if "Origin" in headers:headers["Origin"]="http://127.0.0.1:8765"
            if body:
                action=json.loads(body).get("action","")
                records.append({"time":time.time(),"kind":"POST_"+action})
            connection=http.client.HTTPConnection("127.0.0.1",8765,timeout=1)
            try:
                connection.request(self.command,self.path,body,headers)
                response=connection.getresponse();data=response.read()
                self.send_response(response.status)
                for key,value in response.getheaders():
                    if key.lower() not in ("server","date","connection"):self.send_header(key,value)
                self.end_headers();self.wfile.write(data)
            except OSError:self.send_error(502,"test upstream unavailable")
            finally:connection.close()
            args.output.write_text(json.dumps(records[-500:]))
        do_GET=forward
        do_POST=forward
    server=ThreadingHTTPServer(("127.0.0.1",8787),Handler)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()


if __name__=="__main__":main()
