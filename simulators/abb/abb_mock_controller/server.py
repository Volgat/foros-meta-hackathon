"""
=============================================================================
 abb_mock_controller/server.py — FOROS ABB RAPID Mock Controller — AMEFORGE
=============================================================================
 Simule un contrôleur ABB IRC5 / OmniCore :
   - Accepte le code RAPID via TCP (port 7001)
   - Parse et exécute une simulation cinématique simplifiée
   - Expose /api/status HTTP (port 8767) pour le feedback FOROS
=============================================================================
"""
from __future__ import annotations
import socket, threading, time, json, math, re, http.server
from datetime import datetime


class ABBRobotState:
    def __init__(self):
        self.joint_angles = [0.0] * 6          # J1-J6 degrés
        self.tcp_pos = [0.0, 0.0, 500.0]       # X, Y, Z mm
        self.tcp_orient = [1.0, 0.0, 0.0, 0.0] # Quaternion w,x,y,z
        self.speed = "v250"
        self.mode = "AUTO_REMOTE"
        self.program_running = False
        self.last_cmd = ""
        self.log_buffer = []
        self._lock = threading.Lock()

    def move_to(self, x, y, z, speed_factor=0.5):
        with self._lock:
            steps = max(10, int(30 * speed_factor))
            ox, oy, oz = self.tcp_pos
            for i in range(steps):
                t = (i+1)/steps
                self.tcp_pos = [ox+(x-ox)*t, oy+(y-oy)*t, oz+(z-oz)*t]
                time.sleep(0.02)
            self._log(f"MoveL → X={x:.1f} Y={y:.1f} Z={z:.1f}")

    def set_joints(self, angles):
        with self._lock:
            self.joint_angles = angles[:6]
            self._log(f"MoveAbsJ → [{', '.join(f'{a:.1f}' for a in angles[:6])}]°")

    def _log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:12]
        entry = f"[{ts}] {msg}"
        self.log_buffer = (self.log_buffer + [entry])[-200:]
        print(f"\033[31m[ABB]\033[0m {entry}")

    def get_foros_status(self) -> dict:
        j_rad = [math.radians(a) for a in self.joint_angles]
        tcp_m = [v/1000 for v in self.tcp_pos]
        return {
            "connected": True, "simulation": True,
            "robot_mode": f"ABB_IRC5_{self.mode}",
            "tcp_pose": tcp_m + [0.0, 0.0, 0.0],
            "joints_rad": j_rad, "joints_deg": self.joint_angles[:],
            "program_running": self.program_running,
            "last_command": self.last_cmd,
        }


class RAPIDParser:
    def __init__(self, state: ABBRobotState):
        self.state = state

    def execute_line(self, line: str):
        line = line.strip()
        if not line or line.startswith("!") or line.startswith("MODULE") or line == "ENDMODULE":
            return

        # MoveL p1, v250, z10, tool0;
        m = re.search(r'MoveL\s+\[([^\]]+)\]', line, re.IGNORECASE)
        if m:
            coords = [float(v.strip()) for v in m.group(1).split(",") if v.strip().replace("-","").replace(".","").isdigit()]
            if len(coords) >= 3:
                self.state.move_to(coords[0], coords[1], coords[2], 0.5)
            return

        # MoveJ p1, v250, z10, tool0;
        m = re.search(r'MoveJ\s+\[([^\]]+)\]', line, re.IGNORECASE)
        if m:
            coords = [float(v.strip()) for v in m.group(1).split(",") if v.strip().replace("-","").replace(".","").isdigit()]
            if len(coords) >= 3:
                self.state.move_to(coords[0], coords[1], coords[2], 0.7)
            return

        # MoveAbsJ — joint move
        m = re.search(r'MoveAbsJ\s+\[\s*\[([^\]]+)\]', line, re.IGNORECASE)
        if m:
            angles = [float(v.strip()) for v in m.group(1).split(",")]
            self.state.set_joints(angles)
            return

        # WaitTime
        m = re.search(r'WaitTime\s+([\d.]+)', line, re.IGNORECASE)
        if m:
            t = float(m.group(1))
            self.state._log(f"WaitTime {t}s")
            time.sleep(min(t, 5.0))
            return

        # SetDO / gripper
        if "SetDO" in line or "grip" in line.lower():
            self.state._log(f"I/O: {line}")

    def execute_program(self, rapid_code: str):
        self.state.program_running = True
        self.state._log("RAPID program started")
        for line in rapid_code.split("\n"):
            self.execute_line(line)
        self.state.program_running = False
        self.state._log("RAPID program finished")


def run_tcp_server(state: ABBRobotState, port=7001):
    parser = RAPIDParser(state)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(5)
    print(f"\033[31m[ABB Mock]\033[0m RAPID TCP server on port {port}")
    while True:
        conn, addr = srv.accept()
        conn.sendall(b"FOROS ABB Mock Controller v1.0\nReady.\n")
        def handle(c=conn):
            buf = b""
            try:
                while True:
                    chunk = c.recv(4096)
                    if not chunk: break
                    buf += chunk
                    if b"ENDPROC\n" in buf or (b"\n\n" in buf and len(buf) > 20):
                        rapid = buf.decode("utf-8", errors="replace")
                        state.last_cmd = rapid[:80].replace("\n"," ")
                        threading.Thread(target=parser.execute_program, args=(rapid,), daemon=True).start()
                        c.sendall(b"OK\n")
                        buf = b""
            except: pass
            finally: c.close()
        threading.Thread(target=handle, daemon=True).start()


def run_http_server(state: ABBRobotState, port=8767):
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
    state = ABBRobotState()
    state._log("FOROS ABB RAPID Mock Controller starting...")
    state._log("RAPID TCP: port 7001 | HTTP feedback: port 8767")
    threading.Thread(target=run_http_server, args=(state, 8767), daemon=True).start()
    run_tcp_server(state, port=7001)
