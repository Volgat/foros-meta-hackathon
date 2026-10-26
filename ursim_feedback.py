"""
=============================================================================
 ursim_feedback.py — FOROS URSim Real-Time Feedback — AMEFORGE
=============================================================================
 Reads robot state in real time via RTDE (port 30004):
   - TCP pose (x, y, z, rx, ry, rz) in metres / radians
   - Joint positions (radians + degrees)
   - Robot mode (RUNNING, IDLE, PROTECTIVE_STOP, ...)
   - Safety status
   - Is-moving flag (derived from joint speeds)

 Falls back to a synthetic simulation mode if ur_rtde is not installed,
 so the demo CLI works even without the library present.

 Usage:
   fb = URSimFeedback("127.0.0.1")
   fb.start()
   state = fb.get_state()
   fb.stop()
=============================================================================
"""

from __future__ import annotations
import math, time, threading
from typing import Optional

# ── Try to import ur_rtde (optional — fallback to simulation if missing) ──────
try:
    import rtde_receive
    _RTDE_AVAILABLE = True
except ImportError:
    _RTDE_AVAILABLE = False


# =============================================================================
# Robot mode / safety constants (RTDE values from UR documentation)
# =============================================================================
_ROBOT_MODES = {
    -1: "NO_CONTROLLER",
     0: "DISCONNECTED",
     1: "CONFIRM_SAFETY",
     2: "BOOTING",
     3: "POWER_OFF",
     4: "POWER_ON",
     5: "IDLE",
     6: "BACKDRIVE",
     7: "RUNNING",
     8: "UPDATING_FIRMWARE",
}

_SAFETY_MODES = {
    1: "NORMAL",
    2: "REDUCED",
    3: "PROTECTIVE_STOP",
    4: "RECOVERY",
    5: "SAFEGUARD_STOP",
    6: "SYSTEM_EMERGENCY_STOP",
    7: "ROBOT_EMERGENCY_STOP",
    8: "VIOLATION",
    9: "FAULT",
}


# =============================================================================
# URSimFeedback
# =============================================================================
class URSimFeedback:
    """
    Real-time robot state reader via RTDE (port 30004).

    If ur_rtde is not installed, automatically enters simulation mode:
    the state values animate slowly to mimic a robot moving, which lets
    the demo UI run without requiring the full ur_rtde stack.

    Parameters
    ----------
    robot_ip : str
        IP address of the URSim (or real UR robot). Default: 127.0.0.1
    poll_hz : float
        State polling frequency in Hz. RTDE supports up to 500Hz (e-Series).
        125Hz is comfortable for a display loop.
    """

    def __init__(self, robot_ip: str = "127.0.0.1", poll_hz: float = 10.0):
        self.robot_ip  = robot_ip
        self.poll_hz   = poll_hz
        self._interval = 1.0 / poll_hz

        self._lock     = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._running  = False
        self._rtde     = None
        self.simulation_mode = False

        # State dict — always available
        self._state = {
            "tcp_pose":      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # x,y,z,rx,ry,rz [m/rad]
            "joints_rad":    [0.0] * 6,
            "joints_deg":    [0.0] * 6,
            "joint_speeds":  [0.0] * 6,
            "robot_mode":    "DISCONNECTED",
            "safety_mode":   "NORMAL",
            "is_moving":     False,
            "timestamp":     0.0,
            "connected":     False,
            "simulation":    False,
        }

    # ── Public API ────────────────────────────────────────────────────────────

    def start(self) -> bool:
        """
        Connect to RTDE and start the background polling thread.
        Returns True if connected (real or simulation), False on fatal error.
        """
        if not _RTDE_AVAILABLE:
            print("[feedback] ur_rtde not installed -> simulation mode")
            self.simulation_mode = True
            self._state["simulation"] = True
            self._state["robot_mode"] = "RUNNING (simulated)"
            self._state["connected"]  = True
            self._running = True
            self._thread  = threading.Thread(target=self._simulate_loop, daemon=True)
            self._thread.start()
            return True

        try:
            self._rtde = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            print(f"[feedback] RTDE connected to {self.robot_ip}:30004")
            self._running = True
            self._thread  = threading.Thread(target=self._rtde_loop, daemon=True)
            self._thread.start()
            return True
        except Exception as e:
            print(f"[feedback] RTDE connection failed: {e}")
            print("[feedback] Falling back to simulation mode")
            self.simulation_mode = True
            self._state["simulation"] = True
            self._state["robot_mode"] = "RUNNING (simulated)"
            self._state["connected"]  = True
            self._running = True
            self._thread  = threading.Thread(target=self._simulate_loop, daemon=True)
            self._thread.start()
            return True

    def stop(self):
        """Stop the background thread and disconnect RTDE."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._rtde:
            try:
                self._rtde.disconnect()
            except Exception:
                pass
            self._rtde = None
        with self._lock:
            self._state["connected"] = False

    def get_state(self) -> dict:
        """Return a copy of the current robot state (thread-safe)."""
        with self._lock:
            return dict(self._state)

    def is_connected(self) -> bool:
        return self._state.get("connected", False)

    # ── Real RTDE polling loop ────────────────────────────────────────────────

    def _rtde_loop(self):
        while self._running:
            try:
                tcp   = self._rtde.getActualTCPPose()       # [x,y,z,rx,ry,rz]
                jrad  = self._rtde.getActualQ()              # 6 joint angles [rad]
                jspd  = self._rtde.getActualQd()             # 6 joint speeds [rad/s]
                rmode = self._rtde.getRobotMode()            # int
                smode = self._rtde.getSafetyMode()           # int

                jdeg    = [math.degrees(j) for j in jrad]
                moving  = any(abs(s) > 0.01 for s in jspd)

                with self._lock:
                    self._state.update({
                        "tcp_pose":     list(tcp),
                        "joints_rad":   list(jrad),
                        "joints_deg":   jdeg,
                        "joint_speeds": list(jspd),
                        "robot_mode":   _ROBOT_MODES.get(rmode, f"MODE_{rmode}"),
                        "safety_mode":  _SAFETY_MODES.get(smode, f"SAFETY_{smode}"),
                        "is_moving":    moving,
                        "timestamp":    time.time(),
                        "connected":    True,
                        "simulation":   False,
                    })
            except Exception as e:
                with self._lock:
                    self._state["connected"]   = False
                    self._state["robot_mode"]  = f"ERROR: {e}"
                time.sleep(1.0)
                # Try to reconnect
                try:
                    self._rtde = rtde_receive.RTDEReceiveInterface(self.robot_ip)
                    with self._lock:
                        self._state["connected"] = True
                except Exception:
                    pass

            time.sleep(self._interval)

    # ── Simulation loop (synthetic animated values) ───────────────────────────

    def _simulate_loop(self):
        """
        Animates fake robot state for offline testing.
        The TCP pose slowly oscillates so the UI looks alive.
        """
        t0 = time.time()
        home_q = [0.0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0.0]

        while self._running:
            t = time.time() - t0
            # Slow sinusoidal oscillation on TCP
            tcp = [
                0.3 + 0.05 * math.sin(t * 0.3),
                -0.2 + 0.03 * math.cos(t * 0.4),
                0.5 + 0.04 * math.sin(t * 0.25),
                0.0, math.pi, 0.0
            ]
            jrad = [home_q[i] + 0.05 * math.sin(t * 0.2 + i) for i in range(6)]
            jdeg = [math.degrees(j) for j in jrad]
            jspd = [0.05 * math.cos(t * 0.2 + i) for i in range(6)]

            with self._lock:
                self._state.update({
                    "tcp_pose":     tcp,
                    "joints_rad":   jrad,
                    "joints_deg":   jdeg,
                    "joint_speeds": jspd,
                    "robot_mode":   "RUNNING (simulated)",
                    "safety_mode":  "NORMAL",
                    "is_moving":    True,
                    "timestamp":    time.time(),
                    "connected":    True,
                    "simulation":   True,
                })
            time.sleep(self._interval)


# =============================================================================
# Quick standalone test
# =============================================================================
if __name__ == "__main__":
    import sys
    ip = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    print(f"[test] Connecting RTDE feedback to {ip}...")
    fb = URSimFeedback(ip, poll_hz=5.0)
    ok = fb.start()
    if not ok:
        print("[test] Could not start feedback."); sys.exit(1)

    print("[test] Reading state for 5 seconds (Ctrl+C to stop):")
    try:
        for _ in range(20):
            s = fb.get_state()
            sim = " [SIM]" if s["simulation"] else ""
            tcp = s["tcp_pose"]
            print(f"  Mode: {s['robot_mode']}{sim} | "
                  f"TCP: x={tcp[0]:.3f} y={tcp[1]:.3f} z={tcp[2]:.3f} | "
                  f"Moving: {s['is_moving']}")
            time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        fb.stop()
        print("[test] Feedback stopped.")
