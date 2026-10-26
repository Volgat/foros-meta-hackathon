"""
=============================================================================
 ros1_bridge.py — FOROS → ROS 1 Pipeline — AMEFORGE
=============================================================================
 Pipeline:
   Natural Language → Foros (10M params) → JSON ops → ForosAdapter
   → ROS1 YAML/Topics → rosbridge WebSocket (port 9090) → Gazebo

 Communication ROS1:
   Port 9090 → rosbridge WebSocket server (publishes to /foros/cmd)
   Port 8765 → FOROS feedback HTTP endpoint (/api/status)

 Usage:
   bridge = ROS1Bridge(ros_host="127.0.0.1")
   bridge.connect()
   result = bridge.send_nl("move to x=0.5 y=-0.3 z=0.6")
=============================================================================
"""
from __future__ import annotations
import os, sys, json, time, math, threading
from typing import Optional, List
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from foros_adapter import ForosAdapter

try:
    import websocket
except ImportError:
    import subprocess
    subprocess.run([sys.executable, "-m", "pip", "install", "-q", "websocket-client"], check=False)
    import websocket


# =============================================================================
# ROS1 Bridge — FOROS JSON → rosbridge WebSocket
# =============================================================================

@dataclass
class BridgeResult:
    nl_input: str
    json_ops: Optional[list]
    ros_yaml: str
    sent: bool
    latency_ms: float
    error: Optional[str]

    def __str__(self):
        status = "SENT OK" if self.sent else f"FAILED: {self.error}"
        return (
            f"  NL input  : {self.nl_input}\n"
            f"  JSON ops  : {json.dumps(self.json_ops) if self.json_ops else 'N/A'}\n"
            f"  ROS YAML  : {self.ros_yaml[:100]}...\n"
            f"  Status    : {status}\n"
            f"  Latency   : {self.latency_ms:.1f} ms"
        )


class ROS1Bridge:
    """
    Full pipeline: Natural Language → Foros → JSON → ROS1 Topics → Gazebo.

    Parameters
    ----------
    ros_host : str
        Hostname of the ROS1 container (default 127.0.0.1).
    rosbridge_port : int
        Port of rosbridge WebSocket (default 9090).
    hf_token : str | None
        Hugging Face token for Foros model download.
    speed : float
        Default movement speed (m/s).
    dry_run : bool
        Generate commands but do NOT send to rosbridge.
    """

    def __init__(
        self,
        ros_host:       str   = "127.0.0.1",
        rosbridge_port: int   = 9090,
        hf_token:       Optional[str] = None,
        speed:          float = 0.25,
        dry_run:        bool  = False,
    ):
        self.ros_host       = ros_host
        self.rosbridge_port = rosbridge_port
        self.hf_token       = hf_token or os.environ.get("HF_TOKEN", "")
        self.speed          = speed
        self.dry_run        = dry_run

        self._ws: Optional[websocket.WebSocket] = None
        self._adapter = ForosAdapter(format="ros1", speed_override=speed)
        self.history: List[BridgeResult] = []

        # Foros model (lazy-loaded)
        self._model = None
        self._tokenizer = None
        self._cfg = None

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """Connect to rosbridge WebSocket server."""
        try:
            url = f"ws://{self.ros_host}:{self.rosbridge_port}"
            self._ws = websocket.create_connection(url, timeout=5)
            print(f"[ros1_bridge] Connected to rosbridge → {url}")

            # Advertise the /foros/cmd topic
            advertise_msg = json.dumps({
                "op": "advertise",
                "topic": "/foros/cmd",
                "type": "std_msgs/String"
            })
            self._ws.send(advertise_msg)

            # Advertise /foros/joint_target topic
            advertise_joints = json.dumps({
                "op": "advertise",
                "topic": "/foros/joint_target",
                "type": "sensor_msgs/JointState"
            })
            self._ws.send(advertise_joints)
            return True

        except Exception as e:
            print(f"[ros1_bridge] Connection failed: {e}")
            self._ws = None
            if not self.dry_run:
                return False
            return True

    def disconnect(self):
        if self._ws:
            try:
                self._ws.close()
            except Exception:
                pass
            self._ws = None
        print("[ros1_bridge] Disconnected.")

    def is_connected(self) -> bool:
        return self._ws is not None or self.dry_run

    # ── Foros model loading ───────────────────────────────────────────────────

    def load_foros(self) -> bool:
        """Load Foros model from HuggingFace. Reuses ursim_bridge logic."""
        # Import from parent bridge
        try:
            parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            sys.path.insert(0, parent)
            from ursim_bridge import ForosBridge
            _tmp = ForosBridge(hf_token=self.hf_token, dry_run=True)
            ok = _tmp.load_foros()
            if ok:
                self._model = _tmp._model
                self._tokenizer = _tmp._tokenizer
                self._cfg = _tmp._cfg
                self._device = _tmp._device
                self._tmp_bridge = _tmp
            return ok
        except Exception as e:
            print(f"[ros1_bridge] load_foros error: {e}")
            return False

    @property
    def foros_ready(self) -> bool:
        return self._model is not None

    # ── ROS publishing ────────────────────────────────────────────────────────

    def _json_ops_to_ros_messages(self, ops: list) -> List[dict]:
        """Convert FOROS JSON ops to a list of rosbridge publish messages."""
        messages = []
        joint_angles = [0.0] * 6  # running state

        for op in ops:
            op_type = op.get("op", "")

            if op_type == "move_tcp":
                x = op.get("x", 0.0)
                y = op.get("y", 0.0)
                z = op.get("z", 0.3)
                # Publish as geometry_msgs/PoseStamped to /foros/target_pose
                messages.append({
                    "op": "publish",
                    "topic": "/foros/target_pose",
                    "msg": {
                        "header": {"stamp": {"secs": 0, "nsecs": 0}, "frame_id": "world"},
                        "pose": {
                            "position": {"x": x, "y": y, "z": z},
                            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                        }
                    }
                })
                # Also publish string command
                messages.append({
                    "op": "publish",
                    "topic": "/foros/cmd",
                    "msg": {"data": f"MOVE_TCP x={x} y={y} z={z}"}
                })

            elif op_type == "move_joints":
                joints = op.get("joints", [0.0]*6)
                while len(joints) < 6:
                    joints.append(0.0)
                joint_angles = joints[:6]
                messages.append({
                    "op": "publish",
                    "topic": "/foros/joint_target",
                    "msg": {
                        "header": {"stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""},
                        "name": ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"],
                        "position": joint_angles,
                        "velocity": [0.0]*6,
                        "effort": [0.0]*6
                    }
                })

            elif op_type == "wait":
                duration = op.get("duration", 1.0)
                messages.append({
                    "op": "publish",
                    "topic": "/foros/cmd",
                    "msg": {"data": f"WAIT {duration}"}
                })

            elif op_type == "gripper":
                state = op.get("state", "open")
                messages.append({
                    "op": "publish",
                    "topic": "/foros/gripper",
                    "msg": {"data": state.upper()}
                })

            elif op_type == "home":
                messages.append({
                    "op": "publish",
                    "topic": "/foros/cmd",
                    "msg": {"data": "HOME"}
                })

            elif op_type == "stop":
                messages.append({
                    "op": "publish",
                    "topic": "/foros/cmd",
                    "msg": {"data": "STOP"}
                })

        return messages

    def send_ros_messages(self, messages: List[dict]) -> bool:
        """Publish a list of rosbridge messages to the WebSocket."""
        if self.dry_run:
            print("[ros1_bridge] DRY RUN — Messages:")
            for m in messages:
                print(f"  → {m}")
            return True

        if not self._ws:
            print("[ros1_bridge] Not connected to rosbridge.")
            return False

        try:
            for msg in messages:
                self._ws.send(json.dumps(msg))
                time.sleep(0.05)  # Small delay between publishes
            return True
        except Exception as e:
            print(f"[ros1_bridge] Send failed: {e}")
            return False

    # ── Inference ─────────────────────────────────────────────────────────────

    def _foros_infer(self, text: str) -> Optional[str]:
        if hasattr(self, "_tmp_bridge"):
            return self._tmp_bridge._foros_infer(text)
        return None

    def _parse_ops(self, raw: str) -> Optional[list]:
        try:
            ops = json.loads(raw)
            if isinstance(ops, list) and ops:
                return ops
        except Exception:
            pass
        return None

    # ── Main pipeline ─────────────────────────────────────────────────────────

    def send_nl(self, text: str) -> BridgeResult:
        """Natural Language → Foros → JSON → ROS1 Topics."""
        t0 = time.time()
        error = None
        json_ops = None
        ros_yaml = ""
        sent = False

        if not self.foros_ready:
            return BridgeResult(text, None, "", False, 0.0, "Foros model not loaded")

        try:
            raw = self._foros_infer(text)
            json_ops = self._parse_ops(raw)

            if not json_ops:
                error = f"Invalid JSON from Foros: {raw!r}"
            else:
                # Convert to ROS YAML representation
                adapter = ForosAdapter(format="ros1", speed_override=self.speed)
                ros_yaml = adapter.convert(json_ops)

                # Send via rosbridge
                messages = self._json_ops_to_ros_messages(json_ops)
                sent = self.send_ros_messages(messages)
                if not sent:
                    error = "rosbridge send failed"

        except Exception as e:
            error = str(e)

        result = BridgeResult(text, json_ops, ros_yaml, sent,
                              (time.time() - t0) * 1000.0, error)
        self.history.append(result)
        return result

    def send_direct(self, topic: str, message_type: str, data: dict) -> bool:
        """Publish a direct ROS message to any topic."""
        msg = {"op": "publish", "topic": topic, "msg": data}
        return self.send_ros_messages([msg])

    def emergency_stop(self) -> bool:
        return self.send_ros_messages([{
            "op": "publish",
            "topic": "/foros/cmd",
            "msg": {"data": "EMERGENCY_STOP"}
        }])

    def home(self) -> bool:
        return self.send_ros_messages([{
            "op": "publish",
            "topic": "/foros/cmd",
            "msg": {"data": "HOME"}
        }])


# =============================================================================
# Standalone test
# =============================================================================
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="FOROS → ROS1 Bridge test")
    parser.add_argument("--host",    default="127.0.0.1", help="ROS host IP")
    parser.add_argument("--port",    type=int, default=9090, help="rosbridge port")
    parser.add_argument("--token",   default="", help="HF token")
    parser.add_argument("--dry-run", action="store_true", help="Don't send to rosbridge")
    parser.add_argument("--cmd",     default="move to x=0.4 y=-0.2 z=0.5")
    args = parser.parse_args()

    bridge = ROS1Bridge(ros_host=args.host, rosbridge_port=args.port,
                        hf_token=args.token, dry_run=args.dry_run)

    if not bridge.connect():
        print("[test] Connection failed.")
        sys.exit(1)

    print("[test] Loading Foros model...")
    if not bridge.load_foros():
        print("[test] Failed to load Foros model.")
        sys.exit(1)

    print(f"[test] Sending: {args.cmd!r}")
    result = bridge.send_nl(args.cmd)
    print(f"\n[test] Result:\n{result}")
    bridge.disconnect()
