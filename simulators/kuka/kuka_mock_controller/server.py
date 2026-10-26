"""
=============================================================================
 kuka_mock_controller/server.py — FOROS KUKA KRL Mock Controller — AMEFORGE
=============================================================================
 Simule un contrôleur KUKA KRC4 :
   - Accepte le code KRL via TCP (port 7000)
   - Parse et exécute une simulation cinématique simplifiée
   - Expose /api/status HTTP (port 8766) pour le feedback FOROS
   - Affiche les commandes reçues dans un terminal visuel (pour VNC)

 Architecture:
   FOROS bridge → TCP:7000 → KRL parser → Kinematics sim → Feedback
=============================================================================
"""
from __future__ import annotations
import socket, threading, time, json, math, re, os, sys
import http.server
from datetime import datetime

# =============================================================================
# Simulated Robot State
# =============================================================================
class KUKARobotState:
    def __init__(self):
        self.joint_angles = [0.0] * 6          # A1-A6 en degrés
        self.tcp_pos = [0.0, 0.0, 500.0, 0.0, 0.0, 0.0]  # X,Y,Z,A,B,C (mm, deg)
        self.speed_override = 100               # %
        self.mode = "AUTO"
        self.program_running = False
        self.last_cmd = ""
        self.log_buffer = []
        self._lock = threading.Lock()

    def move_to(self, x, y, z, a=0.0, b=0.0, c=0.0, speed=1.0):
        with self._lock:
            # Smooth interpolation
            steps = max(10, int(speed * 30))
            ox, oy, oz = self.tcp_pos[0], self.tcp_pos[1], self.tcp_pos[2]
            for i in range(steps):
                t = (i + 1) / steps
                self.tcp_pos[0] = ox + (x - ox) * t
                self.tcp_pos[1] = oy + (y - oy) * t
                self.tcp_pos[2] = oz + (z - oz) * t
                time.sleep(0.02)
            self.tcp_pos = [x, y, z, a, b, c]
            self._log(f"PTP MOVE → X={x:.1f} Y={y:.1f} Z={z:.1f} A={a:.1f}")

    def set_joints(self, angles):
        with self._lock:
            for i, a in enumerate(angles[:6]):
                self.joint_angles[i] = a
            self._log(f"JOINTS SET → {[f'{a:.1f}°' for a in self.joint_angles]}")

    def _log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:12]
        entry = f"[{ts}] {msg}"
        self.log_buffer.append(entry)
        if len(self.log_buffer) > 200:
            self.log_buffer = self.log_buffer[-200:]
        print(f"\033[36m[KUKA]\033[0m {entry}")

    def get_foros_status(self) -> dict:
        with self._lock:
            j_rad = [math.radians(a) for a in self.joint_angles]
            # Rough FK: convert joint angles to approximate TCP (mm → m)
            tcp_m = [v / 1000.0 for v in self.tcp_pos[:3]] + self.tcp_pos[3:]
            return {
                "connected": True,
                "simulation": True,
                "robot_mode": f"KUKA_KRC4_{self.mode}",
                "tcp_pose": [tcp_m[0], tcp_m[1], tcp_m[2],
                             math.radians(self.tcp_pos[3]),
                             math.radians(self.tcp_pos[4]),
                             math.radians(self.tcp_pos[5])],
                "joints_rad": j_rad,
                "joints_deg": self.joint_angles[:],
                "speed_override": self.speed_override,
                "program_running": self.program_running,
                "last_command": self.last_cmd,
            }


# =============================================================================
# KRL Parser
# =============================================================================
class KRLParser:
    """Minimal KRL interpreter for FOROS-generated programs."""

    def __init__(self, state: KUKARobotState):
        self.state = state

    def execute_line(self, line: str):
        line = line.strip()
        if not line or line.startswith(";") or line.startswith("DEF ") or line in ("END", "BEGIN"):
            return

        # PTP {X, Y, Z, A, B, C}
        m = re.search(r'PTP\s*\{([^}]+)\}', line, re.IGNORECASE)
        if m:
            coords = self._parse_coords(m.group(1))
            self.state.move_to(
                coords.get("X", 0), coords.get("Y", 0), coords.get("Z", 500),
                coords.get("A", 0), coords.get("B", 0), coords.get("C", 0),
                speed=0.5
            )
            return

        # LIN {X, Y, Z, A, B, C}
        m = re.search(r'LIN\s*\{([^}]+)\}', line, re.IGNORECASE)
        if m:
            coords = self._parse_coords(m.group(1))
            self.state.move_to(
                coords.get("X", 0), coords.get("Y", 0), coords.get("Z", 500),
                coords.get("A", 0), coords.get("B", 0), coords.get("C", 0),
                speed=0.3
            )
            return

        # WAIT SEC x.x
        m = re.search(r'WAIT\s+SEC\s+([\d.]+)', line, re.IGNORECASE)
        if m:
            t = float(m.group(1))
            self.state._log(f"WAIT {t}s")
            time.sleep(min(t, 5.0))  # Cap at 5s in simulation
            return

        # $ANOUT[1] = ... (gripper analog)
        if "ANOUT" in line.upper() or "GRIP" in line.upper():
            self.state._log(f"GRIPPER: {line}")
            return

        # $OV_PRO = speed
        m = re.search(r'\$OV_PRO\s*=\s*(\d+)', line, re.IGNORECASE)
        if m:
            self.state.speed_override = int(m.group(1))
            self.state._log(f"SPEED OVERRIDE → {self.state.speed_override}%")
            return

    def _parse_coords(self, s: str) -> dict:
        result = {}
        for pair in s.split(","):
            pair = pair.strip()
            if ":" in pair:
                k, v = pair.split(":", 1)
                try:
                    result[k.strip().upper()] = float(v.strip())
                except ValueError:
                    pass
        return result

    def execute_program(self, krl_code: str):
        """Execute a KRL program string."""
        self.state.program_running = True
        self.state._log("Program started")
        for line in krl_code.split("\n"):
            self.execute_line(line)
        self.state.program_running = False
        self.state._log("Program finished")


# =============================================================================
# TCP Server (port 7000)
# =============================================================================
class KUKATCPServer:
    def __init__(self, state: KUKARobotState, host="0.0.0.0", port=7000):
        self.state = state
        self.host = host
        self.port = port
        self.parser = KRLParser(state)

    def start(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((self.host, self.port))
        server_sock.listen(5)
        print(f"\033[32m[KUKA Mock]\033[0m KRL TCP server listening on {self.host}:{self.port}")

        while True:
            conn, addr = server_sock.accept()
            print(f"\033[32m[KUKA Mock]\033[0m Connection from {addr}")
            threading.Thread(target=self._handle_client, args=(conn,), daemon=True).start()

    def _handle_client(self, conn: socket.socket):
        conn.sendall(b"FOROS KUKA Mock Controller v1.0\nReady.\n")
        buf = b""
        try:
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk
                # Process complete programs (terminated by double-newline or END)
                if b"END\n" in buf or (b"\n\n" in buf and len(buf) > 20):
                    krl = buf.decode("utf-8", errors="replace")
                    self.state.last_cmd = krl[:80].replace("\n", " ")
                    threading.Thread(target=self.parser.execute_program, args=(krl,), daemon=True).start()
                    conn.sendall(b"OK\n")
                    buf = b""
        except Exception as e:
            print(f"[KUKA Mock] Client error: {e}")
        finally:
            conn.close()


# =============================================================================
# HTTP Feedback Server (port 8766)
# =============================================================================
def start_http_server(state: KUKARobotState, port=8766):
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def do_OPTIONS(self):
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
        def do_GET(self):
            if self.path == "/api/status":
                body = json.dumps(state.get_foros_status()).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Content-Length", len(body))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/api/logs":
                body = json.dumps(state.log_buffer[-50:]).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_response(404)
                self.end_headers()
        def do_POST(self):
            if self.path == "/api/stop":
                state._log("EMERGENCY STOP received")
                state.program_running = False
                body = b'{"success": true}'
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_response(404)
                self.end_headers()

    server = http.server.HTTPServer(("0.0.0.0", port), Handler)
    print(f"\033[32m[KUKA Mock]\033[0m HTTP feedback server on port {port}")
    server.serve_forever()


# =============================================================================
# Entry Point
# =============================================================================
if __name__ == "__main__":
    state = KUKARobotState()
    state._log("FOROS KUKA Mock Controller starting...")
    state._log("KRL TCP server: port 7000 | Feedback HTTP: port 8766")

    # Start HTTP feedback server in background
    threading.Thread(target=start_http_server, args=(state, 8766), daemon=True).start()

    # Start TCP server (blocking)
    tcp_server = KUKATCPServer(state, port=7000)
    tcp_server.start()
