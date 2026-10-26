"""
=============================================================================
 doosan_mock_controller/server.py — FOROS Doosan DRL Mock Controller — AMEFORGE
=============================================================================
 Simule un robot Doosan :
   - Accepte le code DRL via TCP (port 7003)
   - Parse et simule les mouvements
   - Expose /api/status HTTP (port 8769)
=============================================================================
"""
from __future__ import annotations
import socket, threading, time, json, math, re, http.server
from datetime import datetime


class DoosanRobotState:
    def __init__(self):
        self.joint_angles = [0.0]*6
        self.tcp_pos = [0.0, 0.0, 500.0, 0.0, 0.0, 0.0]
        self.mode = "AUTONOMOUS"
        self.program_running = False
        self.last_cmd = ""
        self.log_buffer = []
        self._lock = threading.Lock()

    def move_to(self, x, y, z, rx=0, ry=0, rz=0):
        with self._lock:
            ox, oy, oz = self.tcp_pos[:3]
            for i in range(20):
                t = (i+1)/20
                self.tcp_pos[0] = ox+(x-ox)*t
                self.tcp_pos[1] = oy+(y-oy)*t
                self.tcp_pos[2] = oz+(z-oz)*t
                time.sleep(0.025)
            self.tcp_pos = [x, y, z, rx, ry, rz]
            self._log(f"movel posx({x:.1f},{y:.1f},{z:.1f},{rx:.1f},{ry:.1f},{rz:.1f})")

    def _log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:12]
        entry = f"[{ts}] {msg}"
        self.log_buffer = (self.log_buffer + [entry])[-200:]
        print(f"\033[34m[DOOSAN]\033[0m {entry}")

    def get_foros_status(self) -> dict:
        j_rad = [math.radians(a) for a in self.joint_angles]
        tcp_m = [v/1000 for v in self.tcp_pos[:3]] + [math.radians(v) for v in self.tcp_pos[3:]]
        return {
            "connected": True, "simulation": True,
            "robot_mode": f"DOOSAN_{self.mode}",
            "tcp_pose": tcp_m, "joints_rad": j_rad,
            "joints_deg": self.joint_angles[:],
            "program_running": self.program_running,
        }


class DRLParser:
    def __init__(self, state: DoosanRobotState):
        self.state = state

    def execute_program(self, drl_code: str):
        self.state.program_running = True
        self.state._log("DRL program started")
        for line in drl_code.split("\n"):
            line = line.strip()
            if not line or line.startswith("#"): continue
            # movel(posx(x,y,z,rx,ry,rz))
            m = re.search(r'movel\s*\(\s*posx\s*\(([^)]+)\)', line, re.IGNORECASE)
            if m:
                vals = [float(v.strip()) for v in m.group(1).split(",")]
                while len(vals) < 6: vals.append(0.0)
                self.state.move_to(*vals[:6])
                continue
            # movej(posj(j1,j2,j3,j4,j5,j6))
            m = re.search(r'movej\s*\(\s*posj\s*\(([^)]+)\)', line, re.IGNORECASE)
            if m:
                joints = [float(v.strip()) for v in m.group(1).split(",")]
                with self.state._lock:
                    self.state.joint_angles = joints[:6]
                self.state._log(f"movej → {joints[:6]}")
                continue
            # wait(sec)
            m = re.search(r'wait\s*\(([\d.]+)\)', line, re.IGNORECASE)
            if m:
                t = float(m.group(1))
                self.state._log(f"wait({t})")
                time.sleep(min(t, 5.0))
                continue
            # set_digital_output / gripper
            if "digital_output" in line.lower() or "grip" in line.lower():
                self.state._log(f"I/O: {line}")
        self.state.program_running = False
        self.state._log("DRL program finished")


def run_tcp_server(state: DoosanRobotState, port=7003):
    parser = DRLParser(state)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port)); srv.listen(5)
    print(f"\033[34m[Doosan Mock]\033[0m DRL TCP server on port {port}")
    while True:
        conn, addr = srv.accept()
        conn.sendall(b"FOROS Doosan Mock Controller v1.0\nReady.\n")
        def handle(c=conn):
            buf = b""
            try:
                while True:
                    chunk = c.recv(4096)
                    if not chunk: break
                    buf += chunk
                    if b"\n\n" in buf and len(buf) > 20:
                        code = buf.decode("utf-8", errors="replace")
                        state.last_cmd = code[:80].replace("\n"," ")
                        threading.Thread(target=parser.execute_program, args=(code,), daemon=True).start()
                        c.sendall(b"OK\n"); buf = b""
            except: pass
            finally: c.close()
        threading.Thread(target=handle, daemon=True).start()


def run_http_server(state: DoosanRobotState, port=8769):
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
    state = DoosanRobotState()
    state._log("FOROS Doosan DRL Mock Controller starting...")
    state._log("DRL TCP: port 7003 | HTTP feedback: port 8769")
    threading.Thread(target=run_http_server, args=(state, 8769), daemon=True).start()
    run_tcp_server(state, port=7003)
