"""
=============================================================================
 server.py — FOROS ROS 2 Mock Controller — AMEFORGE
=============================================================================
 Simulates a ROS 2 / rosbridge node + feedback server.
 Exposes:
   - WebSocket port 9091 for mock rosbridge messages
   - HTTP feedback port 8770 for FOROS telemetry
   - noVNC on port 6082
=============================================================================
"""
from __future__ import annotations
import socket, threading, time, json, math, re, os, sys
import http.server
from datetime import datetime

# Simple WebSocket server implementation in pure Python to avoid extra library dependencies
# (or we can use standard socket server to handle basic WebSocket frames)
import struct
import hashlib
import base64

class ROS2RobotState:
    def __init__(self):
        self.joint_angles = [0.0] * 6
        self.tcp_pos = [0.0, 0.0, 500.0, 0.0, 0.0, 0.0]
        self.speed_override = 100
        self.mode = "AUTO"
        self.program_running = False
        self.last_cmd = ""
        self.log_buffer = []
        self._lock = threading.Lock()

    def move_to(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0, speed=1.0):
        with self._lock:
            steps = max(10, int(speed * 30))
            ox, oy, oz = self.tcp_pos[0], self.tcp_pos[1], self.tcp_pos[2]
            for i in range(steps):
                t = (i + 1) / steps
                self.tcp_pos[0] = ox + (x - ox) * t
                self.tcp_pos[1] = oy + (y - oy) * t
                self.tcp_pos[2] = oz + (z - oz) * t
                time.sleep(0.02)
            self.tcp_pos = [x * 1000.0, y * 1000.0, z * 1000.0, 0.0, 0.0, 0.0]  # Store internally in mm
            self._log(f"ROS2 MOVE → X={x:.3f} Y={y:.3f} Z={z:.3f}")

    def set_joints(self, angles):
        with self._lock:
            for i, a in enumerate(angles[:6]):
                self.joint_angles[i] = math.degrees(a)
            self._log(f"ROS2 JOINTS SET → {[f'{math.degrees(a):.1f}°' for a in angles[:6]]}")

    def _log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:12]
        entry = f"[{ts}] {msg}"
        self.log_buffer.append(entry)
        if len(self.log_buffer) > 200:
            self.log_buffer = self.log_buffer[-200:]
        print(f"\033[35m[ROS2]\033[0m {entry}")

    def get_foros_status(self) -> dict:
        with self._lock:
            j_rad = [math.radians(a) for a in self.joint_angles]
            tcp_m = [v / 1000.0 for v in self.tcp_pos[:3]]
            return {
                "connected": True,
                "simulation": True,
                "robot_mode": f"ROS2_HUMBLE_{self.mode}",
                "tcp_pose": [tcp_m[0], tcp_m[1], tcp_m[2], 0.0, 0.0, 0.0],
                "joints_rad": j_rad,
                "joints_deg": self.joint_angles[:],
                "speed_override": self.speed_override,
                "program_running": self.program_running,
                "last_command": self.last_cmd,
            }

# =============================================================================
# WebSocket Server for Mock ros2bridge (port 9091)
# =============================================================================
class MockROS2BridgeServer:
    def __init__(self, state: ROS2RobotState, host="0.0.0.0", port=9091):
        self.state = state
        self.host = host
        self.port = port

    def start(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, self.port))
        s.listen(5)
        print(f"\033[35m[ROS2 Mock]\033[0m WebSocket ros2bridge server listening on {self.host}:{self.port}")

        while True:
            conn, addr = s.accept()
            threading.Thread(target=self._handle_client, args=(conn,), daemon=True).start()

    def _handle_client(self, conn: socket.socket):
        # 1) Handshake
        data = conn.recv(1024).decode('utf-8')
        key_match = re.search(r"Sec-WebSocket-Key:\s+(.+)\r\n", data)
        if not key_match:
            conn.close()
            return
        key = key_match.group(1).strip()
        guid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
        accept_key = base64.b64encode(hashlib.sha1((key + guid).encode('utf-8')).digest()).decode('utf-8')
        
        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Accept: {}\r\n\r\n"
        ).format(accept_key)
        conn.sendall(response.encode('utf-8'))
        self.state._log("WebSocket connection established")

        # 2) Frame reader
        try:
            while True:
                header = conn.recv(2)
                if not header:
                    break
                b1, b2 = header[0], header[1]
                opcode = b1 & 0x0f
                if opcode == 8:  # Close frame
                    break
                masked = (b2 & 0x80) != 0
                payload_len = b2 & 0x7f
                if payload_len == 126:
                    payload_len = struct.unpack(">H", conn.recv(2))[0]
                elif payload_len == 127:
                    payload_len = struct.unpack(">Q", conn.recv(8))[0]
                
                masks = conn.recv(4) if masked else b""
                payload = conn.recv(payload_len)
                
                if masked:
                    decoded = bytearray(payload_len)
                    for i in range(payload_len):
                        decoded[i] = payload[i] ^ masks[i % 4]
                    payload = decoded
                
                msg_str = payload.decode('utf-8', errors='replace')
                self._process_message(msg_str)
        except Exception as e:
            self.state._log(f"WS client error: {e}")
        finally:
            conn.close()
            self.state._log("WebSocket disconnected")

    def _process_message(self, msg_str):
        try:
            msg = json.loads(msg_str)
            op = msg.get("op", "")
            if op == "publish":
                topic = msg.get("topic", "")
                data = msg.get("msg", {})
                self.state.last_cmd = f"Topic {topic}"
                if topic == "/foros/cmd":
                    cmd = data.get("data", "")
                    self.state._log(f"ROS2 cmd: {cmd}")
                elif topic == "/foros/joint_target":
                    pos = data.get("position", [])
                    if pos:
                        self.state.set_joints(pos)
                elif topic == "/foros/target_pose":
                    pose = data.get("pose", {}).get("position", {})
                    x = pose.get("x", 0.0)
                    y = pose.get("y", 0.0)
                    z = pose.get("z", 0.0)
                    self.state.move_to(x, y, z)
        except Exception as e:
            self.state._log(f"Error parsing WS message: {e}")

# =============================================================================
# HTTP Feedback Server (port 8770)
# =============================================================================
def start_http_server(state: ROS2RobotState, port=8770):
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
    print(f"\033[35m[ROS2 Mock]\033[0m HTTP feedback server on port {port}")
    server.serve_forever()

if __name__ == "__main__":
    state = ROS2RobotState()
    state._log("FOROS ROS 2 Mock Controller starting...")
    state._log("WebSocket: port 9091 | Feedback HTTP: port 8770")

    # Start HTTP feedback server in background
    threading.Thread(target=start_http_server, args=(state, 8770), daemon=True).start()

    # Start WebSocket server
    ws_server = MockROS2BridgeServer(state, port=9091)
    ws_server.start()
