"""
=============================================================================
 fanuc_mock_controller/server.py — FOROS Fanuc Karel Mock Controller — AMEFORGE
=============================================================================
 Simule un contrôleur Fanuc R-30iB :
   - Accepte le code Karel via TCP (port 7002)
   - Parse et simule les mouvements
   - Expose /api/status HTTP (port 8768)
=============================================================================
"""
from __future__ import annotations
import socket, threading, time, json, math, re, http.server
from datetime import datetime


class FanucRobotState:
    def __init__(self):
        self.joint_angles = [0.0]*6   # J1-J6 degrés
        self.tcp_pos = [0.0, 0.0, 500.0, 0.0, 0.0, 0.0]  # X,Y,Z,W,P,R mm/deg
        self.mode = "AUTO"
        self.program_running = False
        self.last_cmd = ""
        self.log_buffer = []
        self._lock = threading.Lock()

    def move_to(self, x, y, z, w=0, p=0, r=0, speed=50):
        with self._lock:
            ox, oy, oz = self.tcp_pos[:3]
            steps = max(10, speed // 5)
            for i in range(steps):
                t = (i+1)/steps
                self.tcp_pos[0] = ox+(x-ox)*t
                self.tcp_pos[1] = oy+(y-oy)*t
                self.tcp_pos[2] = oz+(z-oz)*t
                time.sleep(0.02)
            self.tcp_pos = [x, y, z, w, p, r]
            self._log(f"L P[...] {speed}% CNT0 → X={x:.1f} Y={y:.1f} Z={z:.1f}")

    def _log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:12]
        entry = f"[{ts}] {msg}"
        self.log_buffer = (self.log_buffer + [entry])[-200:]
        print(f"\033[33m[FANUC]\033[0m {entry}")

    def get_foros_status(self) -> dict:
        j_rad = [math.radians(a) for a in self.joint_angles]
        tcp_m = [v/1000 for v in self.tcp_pos[:3]] + [math.radians(v) for v in self.tcp_pos[3:]]
        return {
            "connected": True, "simulation": True,
            "robot_mode": f"FANUC_R30iB_{self.mode}",
            "tcp_pose": tcp_m, "joints_rad": j_rad,
            "joints_deg": self.joint_angles[:],
            "program_running": self.program_running,
        }


class KarelParser:
    def __init__(self, state: FanucRobotState):
        self.state = state

    def execute_program(self, karel_code: str):
        self.state.program_running = True
        self.state._log("Karel program started")
        for line in karel_code.split("\n"):
            line = line.strip()
            if not line or line.startswith("--") or line.startswith("PROGRAM") or line == "END":
                continue
            # L P[n] speed% CNT n
            m = re.search(r'L\s+P\[(\d+)\]\s+(\d+)%', line)
            if m:
                # Simulate incremental move
                self.state.move_to(
                    self.state.tcp_pos[0]+50, self.state.tcp_pos[1],
                    self.state.tcp_pos[2], speed=int(m.group(2))
                )
                continue
            # WAIT n.n(sec)
            m = re.search(r'WAIT\s+([\d.]+)\(sec\)', line)
            if m:
                self.state._log(f"WAIT {m.group(1)}s")
                time.sleep(min(float(m.group(1)), 5.0))
                continue
            # DO[n]
            if "DO[" in line:
                self.state._log(f"DIGITAL OUTPUT: {line}")
                continue
        self.state.program_running = False
        self.state._log("Karel program finished")


def run_tcp_server(state: FanucRobotState, port=7002):
    parser = KarelParser(state)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port)); srv.listen(5)
    print(f"\033[33m[Fanuc Mock]\033[0m Karel TCP server on port {port}")
    while True:
        conn, addr = srv.accept()
        conn.sendall(b"FOROS Fanuc Mock Controller v1.0\nReady.\n")
        def handle(c=conn):
            buf = b""
            try:
                while True:
                    chunk = c.recv(4096)
                    if not chunk: break
                    buf += chunk
                    if b"END\n" in buf or (b"\n\n" in buf and len(buf) > 20):
                        code = buf.decode("utf-8", errors="replace")
                        state.last_cmd = code[:80].replace("\n"," ")
                        threading.Thread(target=parser.execute_program, args=(code,), daemon=True).start()
                        c.sendall(b"OK\n"); buf = b""
            except: pass
            finally: c.close()
        threading.Thread(target=handle, daemon=True).start()


def run_http_server(state: FanucRobotState, port=8768):
    class H(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def do_GET(self):
            if self.path == "/api/status":
                b = json.dumps(state.get_foros_status()).encode()
                self.send_response(200); self.send_header("Content-Type","application/json")
                self.send_header("Access-Control-Allow-Origin","*")
                self.send_header("Content-Length",len(b)); self.end_headers(); self.wfile.write(b)
            else:
                self.send_response(404); self.end_headers()
        def do_POST(self):
            if self.path == "/api/stop":
                state._log("EMERGENCY STOP"); state.program_running = False
                self.send_response(200); self.send_header("Content-Type","application/json")
                self.send_header("Access-Control-Allow-Origin","*")
                self.end_headers(); self.wfile.write(b'{"success":true}')
            else:
                self.send_response(404); self.end_headers()
    http.server.HTTPServer(("0.0.0.0", port), H).serve_forever()


if __name__ == "__main__":
    state = FanucRobotState()
    state._log("FOROS Fanuc Karel Mock Controller starting...")
    state._log("Karel TCP: port 7002 | HTTP feedback: port 8768")
    threading.Thread(target=run_http_server, args=(state, 8768), daemon=True).start()
    run_tcp_server(state, port=7002)
