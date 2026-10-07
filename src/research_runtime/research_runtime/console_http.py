"""Loopback-only, same-origin browser transport for the operator console."""
import json
from http import cookies
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import secrets
import re
import threading
import time


class BoundedHTTPServer(ThreadingHTTPServer):
    daemon_threads=True
    def __init__(self,*args):
        self.slots=threading.BoundedSemaphore(16)
        super().__init__(*args)

    def process_request(self,request,address):
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        request.settimeout(.5)
        try:super().process_request(request,address)
        except BaseException:
            self.slots.release()
            raise

    def process_request_thread(self,request,address):
        try:super().process_request_thread(request,address)
        finally:self.slots.release()


class ConsoleHTTP:
    def __init__(self, console, assets, port=8765):
        self.console, self.assets = console, Path(assets)
        self.clients, self.lock = {}, threading.Lock()
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args): pass  # Never log session/CSRF credentials.

            def _host(self):
                return self.headers.get("Host") in {"127.0.0.1:"+str(outer.port), "localhost:"+str(outer.port)}

            def _client(self):
                try:
                    jar = cookies.SimpleCookie(self.headers.get("Cookie", ""))
                    key = jar["research_console"].value
                    with outer.lock:
                        client = outer.clients.get(key)
                        if client and time.monotonic()-client["created"] < 12*3600: return key, client
                except (KeyError, cookies.CookieError): pass
                return None, None

            def _reply(self, status, data, mime="application/json", cookie=None):
                body = json.dumps(data, ensure_ascii=False, allow_nan=False).encode() if mime == "application/json" else data
                self.send_response(status)
                self.send_header("Content-Type", mime+"; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'self'")
                if cookie: self.send_header("Set-Cookie", cookie)
                self.end_headers()
                self.wfile.write(body)

            def _identity(self,key):
                window=self.headers.get("X-Console-Window","")
                return key+":"+window if key and re.fullmatch(r"[0-9a-f-]{36}",window) else None

            def do_GET(self):
                if not self._host(): return self._reply(403, {"reason": "invalid host"})
                routes = {"/": ("index.html", "text/html"), "/app.js": ("app.js", "application/javascript"), "/style.css": ("style.css", "text/css")}
                if self.path in routes:
                    filename, mime = routes[self.path]
                    return self._reply(200, (outer.assets/filename).read_bytes(), mime)
                key, client = self._client()
                if self.path == "/api/session":
                    if not client:
                        with outer.lock:
                            outer.clients = {k:v for k,v in outer.clients.items() if time.monotonic()-v["created"] < 12*3600}
                            if len(outer.clients) >= 64: return self._reply(429, {"reason": "session capacity"})
                            key, client = secrets.token_hex(24), {"csrf": secrets.token_hex(24), "created": time.monotonic()}
                            outer.clients[key] = client
                    return self._reply(200, {"csrf": client["csrf"]}, cookie="research_console="+key+"; HttpOnly; SameSite=Strict; Path=/; Max-Age=43200")
                if self.path == "/api/state": return self._reply(200, outer.console.snapshot(self._identity(key)))
                self._reply(404, {"reason": "unknown route"})

            def do_POST(self):
                key, client = self._client()
                origin = "http://"+self.headers.get("Host", "")
                if (not self._host() or self.path != "/api/command" or not client or not self._identity(key) or
                        self.headers.get("Origin") != origin or
                        not secrets.compare_digest(self.headers.get("X-Console-CSRF", ""), client["csrf"])):
                    return self._reply(403, {"reason": "same-origin operator session required"})
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if not 0 < length <= 8192 or self.headers.get("Content-Type") != "application/json": raise ValueError()
                    data = json.loads(self.rfile.read(length))
                    if not isinstance(data, dict) or set(data)-{"action", "payload", "request_id"}: raise ValueError()
                    if not isinstance(data.get("action"), str) or not isinstance(data.get("payload", {}), dict) or not isinstance(data.get("request_id", ""), str): raise ValueError()
                    result = outer.console.command(self._identity(key), data["action"], data.get("payload"), data.get("request_id", ""))
                except TimeoutError:return self._reply(408,{"reason":"request read deadline"})
                except (ValueError, TypeError, KeyError): return self._reply(400, {"reason": "invalid bounded request"})
                self._reply(200 if result["accepted"] else 409, result)

        self.server = BoundedHTTPServer(("127.0.0.1", int(port)), Handler)
        self.server.daemon_threads = True
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def start(self): self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
