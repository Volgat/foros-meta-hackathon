"""
=============================================================================
 ros1_feedback_server.py — FOROS ROS1 Feedback WebSocket Server
=============================================================================
 Exposes simulated joint/TCP state over WebSocket (port 8765) so the
 FOROS web dashboard can poll telemetry the same way as URSim RTDE.
=============================================================================
"""
import asyncio, json, math, socket, time
import websockets

# Simulated state (updated by incoming command execution)
sim_state = {
    "connected": True,
    "simulation": True,
    "robot_mode": "ROS1_GAZEBO",
    "tcp_pose": [0.0, 0.0, 0.5, 0.0, 0.0, 0.0],
    "joints_rad": [0.0] * 6,
    "joints_deg": [0.0] * 6,
}

async def status_handler(websocket, path):
    try:
        async for message in websocket:
            # Accept joint updates from the bridge
            try:
                data = json.loads(message)
                if "joints_rad" in data:
                    sim_state["joints_rad"] = data["joints_rad"]
                    sim_state["joints_deg"] = [math.degrees(r) for r in data["joints_rad"]]
                if "tcp_pose" in data:
                    sim_state["tcp_pose"] = data["tcp_pose"]
            except Exception:
                pass
            await websocket.send(json.dumps(sim_state))
    except websockets.exceptions.ConnectionClosed:
        pass

async def http_status_server():
    """Minimal HTTP server for /api/status polling from FOROS dashboard."""
    import http.server, threading

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def do_GET(self):
            if self.path == '/api/status':
                body = json.dumps(sim_state).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-Length', len(body))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_response(404)
                self.end_headers()

    server = http.server.HTTPServer(('0.0.0.0', 8765), Handler)
    print("[ROS1 Feedback] HTTP status server on port 8765")
    threading.Thread(target=server.serve_forever, daemon=True).start()

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(http_status_server())
    print("[ROS1 Feedback] Running. Waiting for connections...")
    # Keep running
    while True:
        time.sleep(10)
