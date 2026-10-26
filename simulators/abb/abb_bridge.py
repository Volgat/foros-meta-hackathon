"""
=============================================================================
 abb_bridge.py — FOROS → ABB RAPID Pipeline — AMEFORGE
=============================================================================
 Pipeline:
   Natural Language → Foros → JSON ops → ForosAdapter
   → RAPID code → TCP:7001 → ABB Mock Controller

 Usage:
   bridge = ABBBridge(robot_ip="127.0.0.1")
   bridge.connect()
   bridge.load_foros()
   result = bridge.send_nl("move to x=0.5 y=-0.3 z=0.6")
=============================================================================
"""
from __future__ import annotations
import os, sys, json, socket, time
from typing import Optional, List
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from foros_adapter import ForosAdapter


@dataclass
class BridgeResult:
    nl_input: str
    json_ops: Optional[list]
    rapid_code: str
    sent: bool
    latency_ms: float
    error: Optional[str]

    def __str__(self):
        return (
            f"  NL input : {self.nl_input}\n"
            f"  RAPID code : {self.rapid_code[:120]}...\n"
            f"  Status   : {'SENT OK' if self.sent else f'FAILED: {self.error}'}\n"
            f"  Latency  : {self.latency_ms:.1f} ms"
        )


class ABBBridge:
    """FOROS → ABB RAPID bridge. Sends RAPID programs to the mock controller via TCP."""

    def __init__(
        self,
        robot_ip:  str   = "127.0.0.1",
        port:      int   = 7001,
        hf_token:  Optional[str] = None,
        speed:     float = 0.5,
        dry_run:   bool  = False,
    ):
        self.robot_ip = robot_ip
        self.port     = port
        self.hf_token = hf_token or os.environ.get("HF_TOKEN", "")
        self.speed    = speed
        self.dry_run  = dry_run
        self._sock: Optional[socket.socket] = None
        self._adapter = ForosAdapter(format="rapid", speed_override=speed)
        self.history: List[BridgeResult] = []
        self._model = self._tokenizer = self._cfg = None

    def connect(self) -> bool:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(5.0)
            self._sock.connect((self.robot_ip, self.port))
            welcome = self._sock.recv(256).decode("utf-8", errors="replace").strip()
            print(f"[abb_bridge] Connected: {welcome}")
            return True
        except Exception as e:
            print(f"[abb_bridge] Connection failed: {e}")
            self._sock = None
            return self.dry_run

    def disconnect(self):
        if self._sock:
            try: self._sock.close()
            except: pass
            self._sock = None
        print("[abb_bridge] Disconnected.")

    def load_foros(self) -> bool:
        try:
            parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            sys.path.insert(0, parent)
            from ursim_bridge import ForosBridge
            tmp = ForosBridge(hf_token=self.hf_token, dry_run=True)
            ok = tmp.load_foros()
            if ok:
                self._model = tmp._model
                self._tokenizer = tmp._tokenizer
                self._cfg = tmp._cfg
                self._device = tmp._device
                self._tmp = tmp
            return ok
        except Exception as e:
            print(f"[abb_bridge] load_foros: {e}")
            return False

    @property
    def foros_ready(self): return self._model is not None

    def _foros_infer(self, text): return self._tmp._foros_infer(text)

    def _parse_ops(self, raw):
        try:
            ops = json.loads(raw)
            return ops if isinstance(ops, list) and ops else None
        except: return None

    def send_rapid(self, rapid_code: str) -> bool:
        if self.dry_run:
            print("[abb_bridge] DRY RUN:\n" + rapid_code)
            return True
        if not self._sock:
            return False
        try:
            # The mock server expects a complete file block or END marker
            self._sock.sendall((rapid_code + "\nEND\n").encode("utf-8"))
            resp = self._sock.recv(256).decode("utf-8", errors="replace").strip()
            return resp == "OK"
        except Exception as e:
            print(f"[abb_bridge] Send error: {e}")
            return False

    def send_nl(self, text: str) -> BridgeResult:
        t0 = time.time()
        error = None; json_ops = None; rapid = ""
        if not self.foros_ready:
            return BridgeResult(text, None, "", False, 0, "Foros not loaded")
        try:
            raw = self._foros_infer(text)
            json_ops = self._parse_ops(raw)
            if not json_ops:
                error = f"Invalid JSON: {raw!r}"
            else:
                adapter = ForosAdapter(format="rapid", speed_override=self.speed)
                rapid = adapter.convert(json_ops)
                sent = self.send_rapid(rapid)
                if not sent: error = "RAPID send failed"
        except Exception as e:
            error = str(e)
        result = BridgeResult(text, json_ops, rapid, error is None,
                              (time.time()-t0)*1000, error)
        self.history.append(result)
        return result

    def emergency_stop(self) -> bool:
        return self.send_rapid("! EMERGENCY STOP\nExit;")

    def home(self) -> bool:
        return self.send_rapid("! HOME COMMAND\nMoveJ pHome, v100, fine, tool0;")


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=7001)
    p.add_argument("--token", default="")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--cmd", default="move to x=500 y=-200 z=600")
    args = p.parse_args()

    bridge = ABBBridge(args.host, args.port, args.token, dry_run=args.dry_run)
    bridge.connect()
    bridge.load_foros()
    result = bridge.send_nl(args.cmd)
    print(result)
    bridge.disconnect()
