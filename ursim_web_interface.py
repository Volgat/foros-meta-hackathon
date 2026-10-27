"""
=============================================================================
 ursim_web_interface.py — FOROS Web Interface & Control Server — AMEFORGE
=============================================================================
 Serves the Dark-Theme Industrial Dashboard on port 8080 and handles
 all API requests to control simulated or physical UR robots.

 Usage:
   python ursim_web_interface.py --token YOUR_HF_TOKEN
   python ursim_web_interface.py --token YOUR_HF_TOKEN --dry-run
   python ursim_web_interface.py --ip 192.168.1.100 --token ...

 Shortcuts:
   Open browser at: http://localhost:8080
=============================================================================
"""

from __future__ import annotations
import os, sys, time, json, socket, threading, webbrowser, argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Load .env file if present
_env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
if os.path.isfile(_env_path):
    with open(_env_path, "r", encoding="utf-8") as _ef:
        for _line in _ef:
            _line = _line.strip()
            if _line and not _line.startswith("#") and "=" in _line:
                _k, _v = _line.split("=", 1)
                os.environ.setdefault(_k.strip(), _v.strip().strip('"').strip("'"))

from ursim_bridge   import ForosBridge, BridgeResult
from ursim_feedback import URSimFeedback
from foros_adapter  import ForosAdapter
import foros_db

# Shared state
bridge: Optional[ForosBridge] = None
feedback: Optional[URSimFeedback] = None
dry_run_global = False

class WebServerHandler(BaseHTTPRequestHandler):
    """Custom HTTP Request Handler for the FOROS Control Dashboard API."""

    def log_message(self, format, *args):
        # Suppress verbose standard library HTTP logging to keep terminal clean
        pass

    def send_json(self, status_code: int, data: dict):
        self.send_response(status_code)
        self.send_header('Content-Type', 'application/json')
        # Enable CORS
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode('utf-8'))

    def do_OPTIONS(self):
        # CORS preflight requests
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        print(f"[web] GET {self.path}")
        # Serve frontend files
        if self.path == '/' or self.path == '/index.html':
            script_dir = os.path.dirname(os.path.abspath(__file__))
            filepath = os.path.join(script_dir, 'web', 'index.html')
            self.serve_file(filepath, 'text/html')
        elif self.path == '/favicon.jpg':
            script_dir = os.path.dirname(os.path.abspath(__file__))
            filepath = os.path.join(script_dir, 'web', 'favicon.jpg')
            self.serve_file(filepath, 'image/jpeg')
        elif self.path == '/logo_dark.png':
            script_dir = os.path.dirname(os.path.abspath(__file__))
            filepath = os.path.join(script_dir, 'web', 'logo_dark.png')
            self.serve_file(filepath, 'image/png')
        elif self.path == '/logo_light.png':
            script_dir = os.path.dirname(os.path.abspath(__file__))
            filepath = os.path.join(script_dir, 'web', 'logo_light.png')
            self.serve_file(filepath, 'image/png')
        elif self.path.startswith('/api/admin/metrics'):
            metrics = foros_db.get_metrics()
            self.send_json(200, metrics)
        elif self.path.startswith('/api/admin/logs'):
            logs = foros_db.get_logs(100)
            self.send_json(200, logs)
        elif self.path.startswith('/api/admin/training_stats'):
            # Inference quality statistics — correction rate, avg latency, etc.
            stats = foros_db.get_inference_stats()
            self.send_json(200, stats)
        elif self.path.startswith('/api/admin/export_corpus'):
            # Export production inference log as a training JSONL corpus.
            # Query params: ?approved_only=1  and/or  ?clean_only=1
            from urllib.parse import urlparse, parse_qs
            params = parse_qs(urlparse(self.path).query)
            only_approved    = params.get('approved_only', ['0'])[0] == '1'
            clean_only       = params.get('clean_only',    ['0'])[0] == '1'
            include_corrected = not clean_only
            export_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       'training_data', 'production_corpus.txt')
            n = foros_db.export_training_corpus(
                output_path=export_path,
                only_approved=only_approved,
                include_corrected=include_corrected
            )
            if n > 0:
                # Stream the file back as plain text
                with open(export_path, 'rb') as f:
                    content = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'text/plain; charset=utf-8')
                self.send_header('Content-Disposition',
                                 'attachment; filename="production_corpus.txt"')
                self.send_header('Content-Length', str(len(content)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(content)
            else:
                self.send_json(404, {"success": False,
                                     "error": "No inference logs found matching criteria"})
        elif self.path.startswith('/api/status'):

            from urllib.parse import urlparse, parse_qs
            query = urlparse(self.path).query
            params = parse_qs(query)
            fmt = params.get('format', ['urscript'])[0].strip()

            if fmt == 'urscript':
                if feedback:
                    state = feedback.get_state()
                else:
                    state = {
                        "connected": False,
                        "simulation": True,
                        "robot_mode": "NO_FEEDBACK_ENGINE",
                        "tcp_pose": [0.0]*6,
                        "joints_rad": [0.0]*6,
                        "joints_deg": [0.0]*6,
                    }
            else:
                port_map = {
                    'krl': 8766,
                    'rapid': 8767,
                    'fanuc': 8768,
                    'drl': 8769,
                    'ros1': 8765,
                    'ros2': 8770
                }
                port = port_map.get(fmt)
                if port:
                    import urllib.request
                    try:
                        with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/status", timeout=0.8) as resp:
                            state = json.loads(resp.read().decode('utf-8'))
                    except Exception:
                        state = {
                            "connected": False,
                            "simulation": True,
                            "robot_mode": f"{fmt.upper()}_OFFLINE",
                            "tcp_pose": [0.0]*6,
                            "joints_rad": [0.0]*6,
                            "joints_deg": [0.0]*6,
                        }
                else:
                    state = {
                        "connected": False,
                        "simulation": True,
                        "robot_mode": f"UNKNOWN_FORMAT_{fmt.upper()}",
                        "tcp_pose": [0.0]*6,
                        "joints_rad": [0.0]*6,
                        "joints_deg": [0.0]*6,
                    }
            if bridge:
                state["engine_label"] = bridge.active_engine_name
                state["muse_api_active"] = bool(bridge.muse_api_key)
                state["vision_model"] = f"Meta {bridge.sam_model}"
            self.send_json(200, state)
        elif self.path.startswith('/api/vision/segment'):
            from urllib.parse import urlparse, parse_qs
            query = parse_qs(urlparse(self.path).query).get('query', [''])[0]
            if not bridge:
                self.send_json(503, {"success": False, "error": "Robot bridge disconnected"})
                return
            self.send_json(200, bridge.segment_workspace(query=query))
        else:
            # Fallback 404
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"Not Found")

    def do_POST(self):
        print(f"[web] POST {self.path}")
        # Read content length for JSON body parsing
        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length)
        
        try:
            body = json.loads(post_data) if post_data else {}
        except Exception:
            body = {}
        
        if body:
            print(f"  Body: {body}")

        # Auth and Admin Endpoints
        if self.path == '/api/auth/register':
            email = body.get('email', '').strip()
            password = body.get('password', '').strip()
            if not email or not password:
                self.send_json(400, {"success": False, "error": "Email and password required"})
                return
            success = foros_db.register_user(email, password)
            if success:
                self.send_json(200, {"success": True})
            else:
                self.send_json(400, {"success": False, "error": "Email already registered or invalid"})
            return

        elif self.path == '/api/auth/login':
            email = body.get('email', '').strip()
            password = body.get('password', '').strip()
            if not email or not password:
                self.send_json(400, {"success": False, "error": "Email and password required"})
                return
            user = foros_db.authenticate_user(email, password)
            if user:
                if email != 'admin@foros.run' and user.get('subscription_status') == 'inactive':
                    self.send_json(403, {"success": False, "error": "Subscription suspended. Contact support."})
                    return
                session_id = foros_db.open_session(user['id'])
                self.send_json(200, {"success": True, "user": user, "session_id": session_id})
            else:
                self.send_json(401, {"success": False, "error": "Invalid email or password"})
            return

        elif self.path == '/api/auth/logout':
            session_id = body.get('session_id')
            if session_id:
                foros_db.close_session(session_id)
            self.send_json(200, {"success": True})
            return

        elif self.path == '/api/auth/forgot-password':
            email = body.get('email', '').strip()
            if not email:
                self.send_json(400, {"success": False, "error": "Email required"})
                return
            foros_db.generate_reset_token(email)
            self.send_json(200, {"success": True, "message": "If the email is registered, a recovery code has been generated."})
            return

        elif self.path == '/api/auth/reset-password':
            email = body.get('email', '').strip()
            code = body.get('code', '').strip()
            new_password = body.get('new_password', '').strip()
            if not email or not code or not new_password:
                self.send_json(400, {"success": False, "error": "Email, code, and new password are required"})
                return
            success = foros_db.verify_and_reset_password(email, code, new_password)
            if success:
                self.send_json(200, {"success": True, "message": "Password reset successfully."})
            else:
                self.send_json(400, {"success": False, "error": "Invalid verification code or email."})
            return

        elif self.path == '/api/admin/toggle_subscription':
            requester_email = body.get('requester_email', '').strip()
            target_user_id = body.get('user_id')
            if requester_email != 'admin@foros.run':
                self.send_json(403, {"success": False, "error": "Unauthorized. Admin only."})
                return
            if not target_user_id:
                self.send_json(400, {"success": False, "error": "User ID required"})
                return
            success = foros_db.toggle_user_subscription(target_user_id)
            if success:
                self.send_json(200, {"success": True})
            else:
                self.send_json(400, {"success": False, "error": "User not found or operation failed"})
            return

        # 1) Natural Language Translation + Meta Muse Spatial Reasoning + SAM 3.1 Grounding
        elif self.path == '/api/process':
            prompt = body.get('prompt', '').strip()
            fmt = body.get('format', 'urscript').strip()
            user_id = body.get('user_id')
            
            if not prompt:
                self.send_json(400, {"success": False, "error": "Empty directive provided"})
                return

            if not bridge or not bridge.foros_ready:
                self.send_json(503, {"success": False, "error": "FOROS inference engine not loaded"})
                return

            t0 = time.time()
            try:
                # Step 1: Unified Muse + FOROS Inference & SAM 3.1 Grounding
                infer_res = bridge.infer_with_reasoning(prompt)
                ops = infer_res.get("ops")
                raw_json = infer_res.get("raw", "")

                if ops is None:
                    err_msg = infer_res.get("error") or f"Model generated invalid JSON output: {raw_json}"
                    self.send_json(422, {
                        "success": False,
                        "error": err_msg,
                        "engine_mode": infer_res.get("engine_mode", "iso10218_safety_gate"),
                    })
                    return

                import json as _json
                ops_after_str = _json.dumps(ops, separators=(',', ':'))

                # Step 2: Convert to target format using ForosAdapter
                adapter = ForosAdapter(format=fmt, speed_override=None)
                code = adapter.convert(ops)

                latency_ms = (time.time() - t0) * 1000.0

                foros_db.log_inference(
                    prompt=prompt,
                    raw_model_json=ops_after_str,
                    corrected_json=ops_after_str,
                    was_corrected=False,
                    coord_fix_count=0,
                    final_code=code,
                    format=fmt,
                    latency_ms=latency_ms,
                    user_id=user_id
                )

                foros_db.log_operation(
                    user_id=user_id,
                    robot_brand=fmt,
                    op_type='inference',
                    prompt=prompt,
                    latency_ms=latency_ms
                )

                self.send_json(200, {
                    "success": True,
                    "json_ops": ops,
                    "robot_code": code,
                    "reasoning_steps": infer_res.get("reasoning_steps", []),
                    "grounded_objects": infer_res.get("grounded_objects", []),
                    "safety_rejected": infer_res.get("safety_rejected", False),
                    "rejection_reason": infer_res.get("rejection_reason", ""),
                    "engine_mode": infer_res.get("engine_mode", "foros_hybrid"),
                    "engine_label": infer_res.get("engine_label", bridge.active_engine_name),
                    "latency_ms": latency_ms
                })
            except Exception as e:
                self.send_json(500, {"success": False, "error": str(e)})

        # 1b) Meta SAM 3.1 Workspace Perception Segmentation
        elif self.path == '/api/vision/segment':
            query = body.get('query', '').strip()
            if not bridge:
                self.send_json(503, {"success": False, "error": "Robot bridge disconnected"})
                return
            self.send_json(200, bridge.segment_workspace(query=query))

        # 1c) Meta Muse Voice Transcribe (muse-voice-transcribe-1.0)
        elif self.path == '/api/voice/transcribe':
            audio_b64 = body.get('audio_base64', '').strip()
            mime_type = body.get('mime_type', 'audio/wav').strip()
            if not audio_b64:
                self.send_json(400, {"success": False, "error": "No audio payload provided"})
                return
            if not bridge:
                self.send_json(503, {"success": False, "error": "Robot bridge disconnected"})
                return
            import base64
            try:
                if "," in audio_b64:
                    audio_b64 = audio_b64.split(",", 1)[1]
                audio_bytes = base64.b64decode(audio_b64)
                res = bridge.transcribe_voice(audio_bytes, mime_type=mime_type)
                self.send_json(200 if res.get("success") else 502, res)
            except Exception as e:
                self.send_json(500, {"success": False, "error": str(e)})

        # 2) Code execution
        elif self.path == '/api/send':
            code = body.get('code', '').strip()
            fmt = body.get('format', 'urscript').strip()
            user_id = body.get('user_id')
            if not code:
                self.send_json(400, {"success": False, "error": "No robot code provided"})
                return

            try:
                if fmt == 'urscript':
                    if not bridge:
                        self.send_json(503, {"success": False, "error": "Robot bridge disconnected"})
                        return
                    sent = bridge.send_urscript(code)
                elif fmt in ('krl', 'rapid', 'fanuc', 'drl'):
                    tcp_ports = {'krl': 7000, 'rapid': 7001, 'fanuc': 7002, 'drl': 7003}
                    port = tcp_ports[fmt]
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(2.0)
                    s.connect(("127.0.0.1", port))
                    s.recv(256)
                    s.sendall((code + "\nEND\n").encode("utf-8"))
                    resp = s.recv(256).decode("utf-8", errors="replace").strip()
                    s.close()
                    sent = (resp == "OK")
                elif fmt in ('ros1', 'ros2'):
                    ws_ports = {'ros1': 9090, 'ros2': 9091}
                    port = ws_ports[fmt]
                    import websocket
                    ws = websocket.create_connection(f"ws://127.0.0.1:{port}", timeout=2.0)
                    msg = {
                        "op": "publish",
                        "topic": "/foros/cmd",
                        "msg": {"data": code}
                    }
                    ws.send(json.dumps(msg))
                    ws.close()
                    sent = True
                else:
                    sent = False
                    print(f"[web] send code format {fmt} ignored or unhandled")
                
                print(f"[web] send_code ({fmt}) -> success={sent}")
                
                # Log operation
                foros_db.log_operation(
                    user_id=user_id,
                    robot_brand=fmt,
                    op_type='send_code',
                    prompt=None,
                    latency_ms=0.0
                )
                
                self.send_json(200, {"success": sent, "error": None if sent else "Transmission to robot controller failed"})
            except Exception as e:
                print(f"[web] send_code ({fmt}) ERROR: {e}")
                self.send_json(500, {"success": False, "error": str(e)})

        # 3) Emergency Stop
        elif self.path == '/api/stop':
            fmt = body.get('format', 'urscript').strip()
            user_id = body.get('user_id')
            try:
                if fmt == 'urscript':
                    if not bridge:
                        self.send_json(503, {"success": False, "error": "Robot bridge disconnected"})
                        return
                    r = bridge.send_urscript_direct("stopj(2.0)")
                    sent = r.sent
                elif fmt in ('krl', 'rapid', 'fanuc', 'drl'):
                    http_ports = {'krl': 8766, 'rapid': 8767, 'fanuc': 8768, 'drl': 8769}
                    port = http_ports[fmt]
                    import urllib.request
                    req = urllib.request.Request(f"http://127.0.0.1:{port}/api/stop", data=b'', method='POST')
                    with urllib.request.urlopen(req, timeout=1.0) as resp:
                        res = json.loads(resp.read().decode('utf-8'))
                        sent = res.get('success', False)
                elif fmt in ('ros1', 'ros2'):
                    ws_ports = {'ros1': 9090, 'ros2': 9091}
                    port = ws_ports[fmt]
                    import websocket
                    ws = websocket.create_connection(f"ws://127.0.0.1:{port}", timeout=2.0)
                    ws.send(json.dumps({"op": "publish", "topic": "/foros/cmd", "msg": {"data": "EMERGENCY_STOP"}}))
                    ws.close()
                    sent = True
                else:
                    sent = False
                
                # Log operation
                foros_db.log_operation(
                    user_id=user_id,
                    robot_brand=fmt,
                    op_type='stop',
                    prompt=None,
                    latency_ms=0.0
                )
                
                self.send_json(200, {"success": sent})
            except Exception as e:
                print(f"[web] emergency stop ({fmt}) ERROR: {e}")
                self.send_json(500, {"success": False, "error": str(e)})

        # 4) Return to safe Home position
        elif self.path == '/api/home':
            fmt = body.get('format', 'urscript').strip()
            user_id = body.get('user_id')
            try:
                if fmt == 'urscript':
                    if not bridge:
                        self.send_json(503, {"success": False, "error": "Robot bridge disconnected"})
                        return
                    r = bridge.send_urscript_direct("movej(p[0.0,-0.32,0.42,0,3.14159,0], a=1.2, v=0.6)")
                    sent = r.sent
                elif fmt in ('krl', 'rapid', 'fanuc', 'drl'):
                    home_codes = {
                        'krl': "DEF FOROS_HOME()\nPTP HOME Vel=30% DEFAULT\nEND",
                        'rapid': "! HOME COMMAND\nMoveJ pHome, v100, fine, tool0;",
                        'fanuc': "-- HOME COMMAND\nMOVE TO home_pos",
                        'drl': "# HOME COMMAND\nmovej(pos_home, vel=30, acc=30)"
                    }
                    tcp_ports = {'krl': 7000, 'rapid': 7001, 'fanuc': 7002, 'drl': 7003}
                    port = tcp_ports[fmt]
                    code = home_codes[fmt]
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(2.0)
                    s.connect(("127.0.0.1", port))
                    s.recv(256)
                    s.sendall((code + "\nEND\n").encode("utf-8"))
                    resp = s.recv(256).decode("utf-8", errors="replace").strip()
                    s.close()
                    sent = (resp == "OK")
                elif fmt in ('ros1', 'ros2'):
                    ws_ports = {'ros1': 9090, 'ros2': 9091}
                    port = ws_ports[fmt]
                    import websocket
                    ws = websocket.create_connection(f"ws://127.0.0.1:{port}", timeout=2.0)
                    ws.send(json.dumps({"op": "publish", "topic": "/foros/cmd", "msg": {"data": "HOME"}}))
                    ws.close()
                    sent = True
                else:
                    sent = False
                
                # Log operation
                foros_db.log_operation(
                    user_id=user_id,
                    robot_brand=fmt,
                    op_type='home',
                    prompt=None,
                    latency_ms=0.0
                )
                
                self.send_json(200, {"success": sent})
            except Exception as e:
                print(f"[web] home command ({fmt}) ERROR: {e}")
                self.send_json(500, {"success": False, "error": str(e)})

        # 5) Power On / Release brakes
        elif self.path == '/api/power':
            fmt = body.get('format', 'urscript').strip()
            user_id = body.get('user_id')
            try:
                if fmt == 'urscript':
                    if not bridge:
                        self.send_json(503, {"success": False, "error": "Robot bridge disconnected"})
                        return
                    bridge.power_on()
                
                # Log operation
                foros_db.log_operation(
                    user_id=user_id,
                    robot_brand=fmt,
                    op_type='power',
                    prompt=None,
                    latency_ms=0.0
                )
                
                self.send_json(200, {"success": True})
            except Exception as e:
                print(f"[web] power on ({fmt}) ERROR: {e}")
                self.send_json(500, {"success": False, "error": str(e)})

        else:
            self.send_response(404)
            self.end_headers()

    def serve_file(self, filepath: str, content_type: str):
        try:
            with open(filepath, 'rb') as f:
                content = f.read()
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(content)))
            self.end_headers()
            self.wfile.write(content)
        except Exception as e:
            self.send_response(500)
            self.end_headers()
            self.wfile.write(f"Error loading file: {e}".encode('utf-8'))


def run_web_server(port: int):
    server = ThreadingHTTPServer(('0.0.0.0', port), WebServerHandler)
    print(f"\n[web] Dashboard ready at http://localhost:{port}/")
    
    # Launch browser automatically
    def open_browser():
        time.sleep(1.0)
        webbrowser.open(f"http://localhost:{port}/")
    threading.Thread(target=open_browser, daemon=True).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        print("[web] Web server stopped.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="FOROS Action Engine Web Interface & Control Server — AMEFORGE")
    parser.add_argument("--port", type=int, default=8090, help="Web server port (default: 8090)")
    parser.add_argument("--ip", default="127.0.0.1", help="Robot controller IP (default: 127.0.0.1)")
    parser.add_argument("--token", help="Hugging Face token for AMFORGE/foros_tok")
    parser.add_argument("--dry-run", action="store_true", help="Generate robot programs but do not send to physical sockets")
    args = parser.parse_args()
    # Initialize database
    foros_db.init_db()

    # 1) Start RTDE / Simulation Feedback Thread
    feedback = URSimFeedback(args.ip)
    print("[feedback] Starting telemetries poll engine...")
    feedback.start()

    # 2) Initialize and load FOROS model
    bridge = ForosBridge(
        robot_ip=args.ip,
        hf_token=args.token,
        dry_run=args.dry_run
    )
    
    success = bridge.load_foros()
    if not success:
        print("[ERROR] Could not load FOROS model. Ensure your HF token is correct and repo is accessible.")
        feedback.stop()
        sys.exit(1)

    print("[bridge] Connecting to URSim robot socket...")
    bridge.connect()

    # 3) Start Web Server in main thread
    run_web_server(args.port)

    # Cleanup on exit
    print("[cleanup] Stopping all background loops...")
    feedback.stop()
    bridge.disconnect()
    print("Done.")
