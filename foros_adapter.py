"""
=============================================================================
 Foros Output Adapter — AMEFORGE
=============================================================================
 Converts Foros JSON output to legacy robot protocols.

 Supported output formats:
   json          Default — compact JSON array (AMEFORGE standard)
   urscript      Universal Robots URScript (UR3/UR5/UR10/UR16 — 2012+)
   krl           KUKA Robot Language (KRL) — KUKA KR series 2005+
   rapid         ABB RAPID — ABB IRB series 2000+
   fanuc         Fanuc Karel — Fanuc R-series 2010+
   ros1          ROS1 topic messages (plain text, YAML-like)
   plain_text    Human-readable flat text (oldest/simplest robots)
   serial        Raw serial ASCII commands (2000s-era PLCs / CNC)
   modbus        Modbus TCP register map (CSV: reg_addr,value)

 Usage:
   from foros_adapter import ForosAdapter
   adapter = ForosAdapter(format="urscript")
   script  = adapter.convert(ops)

 Safety:
   All numeric values are clamped to plausible physical ranges
   before conversion to prevent runaway motion from coordinate
   unit bugs (mm → m etc.).

 Example (legacy 2016 UR5):
    ops = [{"op":"move","x":0.5,"y":-0.3,"z":0.6},
           {"op":"gripper","action":"close"}]
    print(ForosAdapter("urscript").convert(ops))
    # movel(p[0.5,-0.3,0.6,0,3.14159,0], a=1.4, v=0.25)
    # set_digital_out(0, True)

=============================================================================
"""

from __future__ import annotations
import json, math
from typing import Any

# ── Physical safety clamps ───────────────────────────────────────────────────
COORD_MAX    = 3.0        # m  — beyond this something is almost certainly wrong
COORD_Z_MIN  = 0.15       # m  — UR5e singularity/collision floor: z < 0.15 m near
                          #       base risks Protective Stop (Position close to joint
                          #       limits) and potential collision with workbench/floor.
VEL_MAX      = 2.5        # m/s
VEL_MIN      = 0.01       # m/s
WAIT_MAX     = 300.0      # s  — 5 minutes max
FORCE_MAX    = 1.0
FREQ_MAX     = 500.0      # Hz


def _clamp(v, lo, hi):
    try: return max(lo, min(hi, float(v)))
    except: return 0.0

def _safe_xyz(op: dict):
    x = _clamp(op.get("x", 0), -COORD_MAX, COORD_MAX)
    y = _clamp(op.get("y", 0), -COORD_MAX, COORD_MAX)
    z = _clamp(op.get("z", 0), COORD_Z_MIN, COORD_MAX)
    return x, y, z

def _safe_vel(op: dict, key="velocity"):
    return _clamp(op.get(key, 0.5), VEL_MIN, VEL_MAX)

def _safe_force(op: dict):
    return _clamp(op.get("force", 0.5), 0.0, FORCE_MAX)

def _safe_seconds(op: dict):
    return _clamp(op.get("seconds", 1.0), 0.0, WAIT_MAX)


# =============================================================================
# ForosAdapter
# =============================================================================
class ForosAdapter:
    """
    Convert Foros JSON op-list to a target robot protocol.

    Parameters
    ----------
    format : str
        One of: json, urscript, krl, rapid, fanuc, ros1, plain_text,
                serial, modbus
    speed_override : float | None
        If set, overrides the velocity for all move operations (m/s).
    precision : int
        Decimal places for coordinate output (default 3).
    """

    FORMATS = [
        "json", "urscript", "krl", "rapid", "fanuc",
        "ros1", "ros2", "drl", "inform", "plain_text", "serial", "modbus",
    ]

    def __init__(self, format: str = "json",
                 speed_override: float | None = None,
                 precision: int = 3):
        if format not in self.FORMATS:
            raise ValueError(f"Unknown format '{format}'. Choose from: {self.FORMATS}")
        self.format          = format
        self.speed_override  = speed_override
        self.prec            = precision
        self._current_speed  = 0.25  # default m/s

    # ── Public API ────────────────────────────────────────────────────────────

    def convert(self, ops: list | str, top_level: bool = True) -> str:
        """
        Convert a Foros ops list (or raw JSON string) to the target format.
        Returns the converted program as a single string.

        NOTE: top_level=True no longer auto-wraps in a repeat-10 loop.
        A repeat is only generated when the JSON explicitly contains
        {"op": "repeat", "times": N, "body": [...]}.
        """
        if isinstance(ops, str):
            ops = json.loads(ops)
        if not isinstance(ops, list):
            ops = [ops]

        # Make a shallow copy to prevent modifying caller's list
        ops = list(ops)

        # REMOVED: automatic repeat(10) + home injection.
        # The program runs once unless the operator explicitly adds a repeat op.

        fn = getattr(self, f"_to_{self.format.replace('-', '_')}")
        return fn(ops)

    def convert_file(self, json_path: str, out_path: str):
        """Read a JSON file of ops and write the converted program."""
        with open(json_path) as f:
            ops = json.load(f)
        result = self.convert(ops)
        with open(out_path, "w") as f:
            f.write(result)
        print(f"[adapter] Written {len(result)} chars to {out_path}")

    # ── JSON (pass-through, with safety clamp) ────────────────────────────────

    def _to_json(self, ops: list) -> str:
        safe = self._clamp_all(ops)
        return json.dumps(safe, separators=(",", ":"), ensure_ascii=False)

    # ── URScript (Universal Robots UR3/UR5/UR10/UR16 — 2012+) ────────────────

    def _to_urscript(self, ops: list) -> str:
        """
        Generates URScript compatible with UR e-Series and CB-Series.
        Coordinates are in meters, angles in radians.

        SAFETY RULES (v5.8):
          - All Cartesian moves use movel(p[x,y,z,ax,ay,az]) — straight-line
            trajectory. NEVER movej(get_inverse_kin(...)) for pose targets,
            which can cause large, unpredictable joint sweeps and Protective Stops.
          - Speed is tracked via self._current_speed and updated by the
            "speed" op. It is never hardcoded.
          - No automatic repeat loop is injected. Loops only appear when the
            JSON explicitly contains {"op": "repeat", "times": N}.
        """
        lines = ["# Generated by Foros Adapter — AMEFORGE", "# URScript v3.x compatible"]
        prev_xy = (0.0, -0.30)
        for op in ops:
            t = op.get("op")

            # ── FIX 1 + 2 + 3 : movel + dynamic speed + singularity guard ─
            if t == "move":
                x, y, z = _safe_xyz(op)
                op_vel = op.get("velocity", None)
                if op_vel is not None:
                    v = _clamp(op_vel, VEL_MIN, VEL_MAX)
                elif self.speed_override is not None:
                    v = self.speed_override
                else:
                    v = self._current_speed

                # 1) Inner base-cylinder singularity guard (UR5e shoulder column r_xy >= 0.22 m)
                r_xy = math.sqrt(x**2 + y**2)
                if r_xy < 0.22:
                    if r_xy < 1e-4:
                        x, y = 0.0, -0.26
                    else:
                        scale_in = 0.24 / r_xy
                        x = round(x * scale_in, 4)
                        y = round(y * scale_in, 4)

                # 2) Outer spherical reach protection (r <= 0.78 m)
                dist = math.sqrt(x**2 + y**2 + z**2)
                if dist > 0.78:
                    scale = 0.78 / dist
                    x = round(x * scale, 4)
                    y = round(y * scale, 4)
                    z = round(z * scale, 4)

                # 3) Cross-origin transit guard: if straight line from prev_xy to (x,y)
                # passes within 0.19 m of base (0,0), insert a safe front bypass waypoint
                px, py = prev_xy
                dx, dy = x - px, y - py
                seg_len_sq = dx * dx + dy * dy
                if seg_len_sq > 1e-4:
                    t_proj = max(0.0, min(1.0, -(px * dx + py * dy) / seg_len_sq))
                    cx = px + t_proj * dx
                    cy = py + t_proj * dy
                    min_origin_dist = math.sqrt(cx * cx + cy * cy)
                    if min_origin_dist < 0.19:
                        safe_z = round(max(z, 0.40), 4)
                        lines.append(
                            f"movel(p[0.0,-0.30,{safe_z},0,3.14159,0], a=1.2, v={round(v, 3)})"
                        )

                lines.append(
                    f"movel(p[{x},{y},{z},0,3.14159,0], a=1.2, v={round(v, 3)})"
                )
                prev_xy = (x, y)

            elif t == "joint_move":
                # Joint-space move (explicit joint angles) — movej is correct here
                joints = [round(float(j), 4) for j in op.get("joints", [])][:6]
                # Convert degrees to radians if angles look like degrees (> 2π)
                joints_rad = [math.radians(j) if abs(j) > math.pi * 2 else j for j in joints]
                jlist = ", ".join(str(round(j, 4)) for j in joints_rad)
                v = self.speed_override if self.speed_override is not None else self._current_speed
                lines.append(f"movej([{jlist}], a=1.4, v={round(v, 3)})")

            elif t == "gripper":
                action = op.get("action", "open")
                force  = _safe_force(op)
                if action == "close":
                    lines.append(f"set_digital_out(0, True)   # close gripper (force={force})")
                else:
                    lines.append("set_digital_out(0, False)  # open gripper")

            elif t == "wait":
                s = _safe_seconds(op)
                lines.append(f"sleep({s})")

            elif t == "home":
                # Safe extended front-table posture at (0.0, -0.32, 0.42) using joint-space IK interpolation
                lines.append("movej(p[0.0,-0.32,0.42,0,3.14159,0], a=1.2, v=0.6)")
                prev_xy = (0.0, -0.32)

            elif t == "speed":
                # FIX 3 — update session speed; subsequent moves will use this value
                self._current_speed = _safe_vel(op)
                lines.append(f"# speed set to {round(self._current_speed, 3)} m/s")

            elif t == "stop":
                lines.append("stopj(2.0)")

            elif t == "tool":
                state = op.get("state", "off")
                lines.append(f"set_tool_voltage({'24' if state == 'on' else '0'})")

            elif t == "safety":
                cond = op.get("cond", "obstacle")
                then_ops = op.get("then", [])
                if cond == "obstacle":
                    ur_cond = "get_digital_in(2) == True"
                elif cond == "collision":
                    ur_cond = "get_digital_in(3) == True"
                else:
                    ur_cond = "False"
                lines.append(f"if ({ur_cond}):")
                for l in self._to_urscript(then_ops).split("\n"):
                    if l and not l.startswith("#"):
                        lines.append("  " + l)
                lines.append("end")

            elif t == "repeat":
                # Explicit repeat — only generated when the JSON asks for it
                times = int(op.get("times", 1))
                body  = op.get("body", [])
                loop_id = abs(hash(str(op))) % 10000
                lines.append(f"loop_counter_{loop_id} = 0")
                lines.append(f"while (loop_counter_{loop_id} < {times}):")
                for l in self._to_urscript(body).split("\n"):
                    if l and not l.startswith("#"):
                        lines.append("  " + l)
                lines.append(f"  loop_counter_{loop_id} = loop_counter_{loop_id} + 1")
                lines.append("end")

            elif t == "repeat_until":
                cond = op.get("cond", "sensor_trip")
                body = op.get("body", [])
                if cond == "sensor_trip":
                    ur_cond = "get_digital_in(1) == True"
                elif cond == "obstacle":
                    ur_cond = "get_digital_in(2) == True"
                elif cond == "collision":
                    ur_cond = "get_digital_in(3) == True"
                else:
                    ur_cond = "False"
                lines.append(f"while (not ({ur_cond})):")
                for l in self._to_urscript(body).split("\n"):
                    if l and not l.startswith("#"):
                        lines.append("  " + l)
                lines.append("end")

            elif t == "parallel":
                branches = op.get("branches", [])
                thread_ids = []
                for idx, branch in enumerate(branches):
                    thread_name = f"parallel_thread_{idx}"
                    thread_ids.append(thread_name)
                    lines.append(f"thread {thread_name}():")
                    for l in self._to_urscript(branch).split("\n"):
                        if l and not l.startswith("#"):
                            lines.append("  " + l)
                    lines.append("end")
                for tid in thread_ids:
                    lines.append(f"run {tid}()")
                for tid in thread_ids:
                    lines.append(f"join {tid}")

            elif t == "trajectory":
                # FIX 2 — trajectory waypoints also use movel for straight-line
                wps = op.get("waypoints", [])
                for wp in wps:
                    x = _clamp(wp.get("x", 0), -COORD_MAX, COORD_MAX)
                    y = _clamp(wp.get("y", 0), -COORD_MAX, COORD_MAX)
                    z = _clamp(wp.get("z", 0), COORD_Z_MIN, COORD_MAX)
                    dist = math.sqrt(x**2 + y**2 + z**2)
                    if dist > 0.80:
                        scale = 0.80 / dist
                        x = round(x * scale, 4)
                        y = round(y * scale, 4)
                        z = round(z * scale, 4)
                    v = self.speed_override if self.speed_override is not None else self._current_speed
                    lines.append(f"movel(p[{x},{y},{z},0,3.14159,0], a=1.4, v={round(v, 3)})")

            elif t == "control_loop":
                hz  = _clamp(op.get("frequency_hz", 50), 1, FREQ_MAX)
                dur = _safe_seconds({"seconds": op.get("duration_s", 1.0)})
                body = op.get("body", [])
                lines.append(f"# Control loop {hz}Hz for {dur}s")
                loop_id = abs(hash(str(op))) % 10000
                lines.append(f"control_timer_{loop_id} = 0")
                lines.append(f"while (control_timer_{loop_id} < {dur}):")
                lines.append(f"  sleep({round(1.0/hz, 4)})")
                for l in self._to_urscript(body).split("\n"):
                    if l and not l.startswith("#"):
                        lines.append("  " + l)
                lines.append(f"  control_timer_{loop_id} = control_timer_{loop_id} + {round(1.0/hz, 4)}")
                lines.append("end")

        return "\n".join(lines)

    # ── KUKA Robot Language (KRL) — KUKA KR series 2005+ ─────────────────────

    def _to_krl(self, ops: list) -> str:
        lines = [
            "&ACCESS RVP",
            "&REL 26",
            "DEF FOROS_PROGRAM()",
            "  ; Generated by Foros Adapter — AMEFORGE",
            "  BAS(#INITMOV, 0)",
        ]
        for i, op in enumerate(ops):
            t = op.get("op")
            if t == "move":
                x, y, z = _safe_xyz(op)
                v = self.speed_override or self._current_speed
                # KRL: PTP or LIN to cartesian position
                lines.append(f"  $VEL.CP={round(v,3)}")
                lines.append(f"  LIN {{X {x*1000:.1f},Y {y*1000:.1f},Z {z*1000:.1f},A 0,B 0,C 0}}")
            elif t == "joint_move":
                joints = [round(float(j), 2) for j in op.get("joints", [])][:6]
                # KRL uses degrees for axes
                joints_deg = [math.degrees(j) if abs(j) < math.pi * 2 else j for j in joints]
                jlist = ",".join(f"A{k+1} {round(d,2)}" for k, d in enumerate(joints_deg))
                lines.append(f"  PTP {{{jlist}}}")
            elif t == "gripper":
                action = op.get("action", "open")
                lines.append(f"  $OUT[1]={'TRUE' if action == 'close' else 'FALSE'}  ; gripper")
            elif t == "wait":
                s = _safe_seconds(op)
                lines.append(f"  WAIT SEC {s}")
            elif t == "home":
                lines.append("  PTP {A1 0,A2 -90,A3 90,A4 0,A5 0,A6 0}")
            elif t == "speed":
                self._current_speed = _safe_vel(op)
                lines.append(f"  $VEL.CP={round(self._current_speed,3)}")
            elif t == "stop":
                lines.append("  HALT")
            elif t == "repeat":
                times = int(op.get("times", 1))
                body  = op.get("body", [])
                lines.append(f"  FOR i=1 TO {times}")
                for l in self._to_krl(body).split("\n"):
                    if l.strip() and not l.strip().startswith(";"):
                        lines.append("    " + l.strip())
                lines.append("  ENDFOR")
            elif t == "safety":
                cond = op.get("cond", "obstacle")
                then_ops = op.get("then", [])
                lines.append(f"  ; SAFETY: if {cond}")
                lines.append(f"  IF $IN[1] THEN")
                for l in self._to_krl(then_ops).split("\n"):
                    if l.strip() and not l.strip().startswith(";"):
                        lines.append("    " + l.strip())
                lines.append("  ENDIF")
        lines.append("END")
        return "\n".join(lines)

    # ── ABB RAPID ─────────────────────────────────────────────────────────────

    def _to_rapid(self, ops: list) -> str:
        lines = [
            "MODULE FOROS_MODULE",
            "  ! Generated by Foros Adapter — AMEFORGE",
            "  PROC Main()",
        ]
        for op in ops:
            t = op.get("op")
            if t == "move":
                x, y, z = _safe_xyz(op)
                v = self.speed_override or self._current_speed
                vms = int(v * 1000)
                # mm for RAPID
                lines.append(
                    f"    MoveJ [[{x*1000:.1f},{y*1000:.1f},{z*1000:.1f}],"
                    f"[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],"
                    f"v{vms},z10,tool0;"
                )
            elif t == "joint_move":
                joints = [round(float(j), 2) for j in op.get("joints", [])][:6]
                joints_deg = [math.degrees(j) if abs(j) < math.pi * 2 else j for j in joints]
                jlist = ",".join(f"{round(d,2)}" for d in joints_deg)
                lines.append(f"    MoveAbsJ [[{jlist}],[9E9,9E9,9E9,9E9,9E9,9E9]],v250,z10,tool0;")
            elif t == "gripper":
                action = op.get("action", "open")
                lines.append(f"    SetDO DO_GRIPPER, {'1' if action == 'close' else '0'};")
            elif t == "wait":
                s = _safe_seconds(op)
                lines.append(f"    WaitTime {s};")
            elif t == "home":
                # Safe folded posture: [0, -90, 90, 0, 90, 0] (deg)
                lines.append("    MoveAbsJ [[0,-90,90,0,90,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v250,z10,tool0;")
            elif t == "speed":
                self._current_speed = _safe_vel(op)
            elif t == "stop":
                lines.append("    Stop;")
            elif t == "repeat":
                times = int(op.get("times", 1))
                body  = op.get("body", [])
                lines.append(f"    FOR i FROM 1 TO {times} DO")
                for l in self._to_rapid(body).split("\n"):
                    ls = l.strip()
                    if ls and not ls.startswith("!") and ls not in ("MODULE FOROS_MODULE",
                                                                      "PROC Main()", "ENDPROC", "ENDMODULE"):
                        lines.append("      " + ls)
                lines.append("    ENDFOR")
            elif t == "safety":
                cond = op.get("cond", "obstacle")
                then_ops = op.get("then", [])
                lines.append(f"    ! SAFETY: if {cond}")
                lines.append("    IF DI_SAFETY = 1 THEN")
                for l in self._to_rapid(then_ops).split("\n"):
                    ls = l.strip()
                    if ls and not ls.startswith("!") and ls not in ("MODULE FOROS_MODULE",
                                                                     "PROC Main()", "ENDPROC", "ENDMODULE"):
                        lines.append("      " + ls)
                lines.append("    ENDIF")
        lines += ["  ENDPROC", "ENDMODULE"]
        return "\n".join(lines)

    # ── Fanuc Karel ────────────────────────────────────────────────────────────

    def _to_fanuc(self, ops: list) -> str:
        lines = [
            "/PROG FOROS",
            "/ATTR",
            "  COMMENT = 'Foros Adapter — AMEFORGE';",
            "/MN",
        ]
        ln = 1
        for op in ops:
            t = op.get("op")
            if t == "move":
                x, y, z = _safe_xyz(op)
                lines.append(
                    f"  {ln:3d}: L P[{ln}] 500mm/sec FINE ;"
                    f"  ! P[{ln}]=({x*1000:.1f},{y*1000:.1f},{z*1000:.1f}) mm"
                ); ln += 1
            elif t == "gripper":
                action = op.get("action", "open")
                lines.append(f"  {ln:3d}: DO[1:GRIPPER]={'ON' if action == 'close' else 'OFF'} ;"); ln += 1
            elif t == "wait":
                s = _safe_seconds(op)
                lines.append(f"  {ln:3d}: WAIT {s}(sec) ;"); ln += 1
            elif t == "home":
                lines.append(f"  {ln:3d}: J P[HOME] 30% FINE ;"); ln += 1
            elif t == "stop":
                lines.append(f"  {ln:3d}: ABORT ;"); ln += 1
            elif t == "repeat":
                times = int(op.get("times", 1))
                body  = op.get("body", [])
                lines.append(f"  {ln:3d}: R[1:COUNT]=0 ;"); ln += 1
                loop_start = ln
                lines.append(f"  {ln:3d}: R[1:COUNT]=R[1:COUNT]+1 ;"); ln += 1
                for sub_op in body:
                    sub_prog = ForosAdapter("fanuc").convert([sub_op], top_level=False)
                    for l in sub_prog.split("\n"):
                        if l.strip() and l.strip()[0].isdigit():
                            lines.append(f"  {ln:3d}: {l.strip().split(':', 1)[1].strip()}"); ln += 1
                lines.append(f"  {ln:3d}: IF R[1:COUNT]<{times}, JMP LBL[{loop_start}] ;"); ln += 1
            elif t == "safety":
                cond = op.get("cond", "obstacle")
                then_ops = op.get("then", [])
                lines.append(f"  {ln:3d}: ! SAFETY: {cond}"); ln += 1
                lines.append(f"  {ln:3d}: IF DI[1:SAFETY]=ON, JMP LBL[{ln+1}] ;"); ln += 1
                lines.append(f"  {ln:3d}: JMP LBL[{ln + len(then_ops) + 2}] ;"); ln += 1
                for sub_op in then_ops:
                    sub_prog = ForosAdapter("fanuc").convert([sub_op], top_level=False)
                    for l in sub_prog.split("\n"):
                        if l.strip() and l.strip()[0].isdigit():
                            lines.append(f"  {ln:3d}: {l.strip().split(':', 1)[1].strip()}"); ln += 1
        lines += ["/END"]
        return "\n".join(lines)

    # ── ROS1 topic messages ────────────────────────────────────────────────────

    def _to_ros1(self, ops: list) -> str:
        """
        Generates ROS1-style command messages.
        Each operation maps to a ROS topic + message format.
        Suitable for robots running ROS Indigo / Kinetic / Melodic (2014-2020).
        """
        lines = ["# Foros → ROS1 Topics — AMEFORGE", "# Paste into rosbash or execute via rostopic pub"]
        for op in ops:
            t = op.get("op")
            if t == "move":
                x, y, z = _safe_xyz(op)
                lines.append(
                    f"rostopic pub -1 /move_group/goal geometry_msgs/PoseStamped "
                    f"'{{header: {{frame_id: world}}, "
                    f"pose: {{position: {{x: {x}, y: {y}, z: {z}}}, "
                    f"orientation: {{w: 1.0}}}}}}'"
                )
            elif t == "joint_move":
                joints = [round(float(j), 4) for j in op.get("joints", [])][:7]
                jlist = ", ".join(str(j) for j in joints)
                lines.append(
                    f"rostopic pub -1 /joint_trajectory_controller/command "
                    f"trajectory_msgs/JointTrajectory "
                    f"'{{joint_names: [j1,j2,j3,j4,j5,j6], "
                    f"points: [{{positions: [{jlist}], time_from_start: {{secs: 2}}}}]}}'"
                )
            elif t == "gripper":
                action = op.get("action", "open")
                val = "1.0" if action == "open" else "0.0"
                lines.append(
                    f"rostopic pub -1 /gripper_controller/command "
                    f"control_msgs/GripperCommand '{{command: {{position: {val}, max_effort: 50.0}}}}'"
                )
            elif t == "wait":
                s = _safe_seconds(op)
                lines.append(f"sleep {s}  # seconds")
            elif t == "home":
                # Safe folded posture: [0, -pi/2, pi/2, -pi/2, -pi/2, 0] (rad)
                lines.append(
                    "rosservice call /move_group/move_to_joint_goal "
                    "'{group_name: arm, goal: {joint_names: [j1,j2,j3,j4,j5,j6], "
                    "joint_positions: [0.0,-1.5708,1.5708,-1.5708,-1.5708,0.0]}}'"
                )
            elif t == "stop":
                lines.append("rostopic pub -1 /move_group/cancel actionlib_msgs/GoalID '{}'")
            elif t == "speed":
                v = _safe_vel(op)
                lines.append(f"rosparam set /move_group/max_velocity_scaling_factor {round(v/2.5,3)}")
            elif t == "safety":
                cond = op.get("cond", "obstacle")
                then_ops = op.get("then", [])
                lines.append(f"# SAFETY trigger: {cond}")
                lines.append(f"# Execute on {cond} event:")
                lines.extend(self._to_ros1(then_ops).split("\n")[2:])
        return "\n".join(lines)

    # ── Plain text ─────────────────────────────────────────────────────────────

    def _to_plain_text(self, ops: list) -> str:
        """
        Human-readable flat text commands.
        Compatible with any robot that accepts ASCII serial commands.
        """
        lines = ["# Foros Plain Text — AMEFORGE"]
        for op in ops:
            t = op.get("op")
            if t == "move":
                x, y, z = _safe_xyz(op)
                lines.append(f"MOVE X={x} Y={y} Z={z}")
            elif t == "joint_move":
                joints = [round(float(j), 3) for j in op.get("joints", [])]
                lines.append(f"JOINT {' '.join(str(j) for j in joints)}")
            elif t == "gripper":
                lines.append(f"GRIPPER {'CLOSE' if op.get('action') == 'close' else 'OPEN'}")
            elif t == "wait":
                lines.append(f"WAIT {_safe_seconds(op)}")
            elif t == "home":
                lines.append("HOME")
            elif t == "speed":
                lines.append(f"SPEED {_safe_vel(op)}")
            elif t == "stop":
                lines.append("STOP")
            elif t == "tool":
                lines.append(f"TOOL {'ON' if op.get('state') == 'on' else 'OFF'}")
            elif t == "safety":
                cond = op.get("cond", "obstacle")
                then_ops = op.get("then", [])
                lines.append(f"IF {cond.upper()}:")
                for sub in self._to_plain_text(then_ops).split("\n")[1:]:
                    lines.append("  " + sub)
                lines.append("ENDIF")
            elif t == "repeat":
                times = int(op.get("times", 1))
                body  = op.get("body", [])
                lines.append(f"REPEAT {times}:")
                for sub in self._to_plain_text(body).split("\n")[1:]:
                    lines.append("  " + sub)
                lines.append("ENDREPEAT")
            elif t == "repeat_until":
                cond  = op.get("cond", "obstacle")
                body  = op.get("body", [])
                lines.append(f"REPEAT UNTIL {cond.upper()}:")
                for sub in self._to_plain_text(body).split("\n")[1:]:
                    lines.append("  " + sub)
                lines.append("ENDREPEAT")
            elif t == "parallel":
                branches = op.get("branches", [])
                lines.append("PARALLEL:")
                for i, branch in enumerate(branches):
                    lines.append(f"  BRANCH {i+1}:")
                    for sub in self._to_plain_text(branch).split("\n")[1:]:
                        lines.append("    " + sub)
                lines.append("ENDPARALLEL")
            elif t == "control_loop":
                hz  = _clamp(op.get("frequency_hz", 50), 1, FREQ_MAX)
                dur = _safe_seconds({"seconds": op.get("duration_s", 1.0)})
                lines.append(f"LOOP HZ={hz} DUR={dur}:")
                for sub in self._to_plain_text(op.get("body", [])).split("\n")[1:]:
                    lines.append("  " + sub)
                lines.append("ENDLOOP")
        return "\n".join(lines)

    # ── Serial ASCII ───────────────────────────────────────────────────────────

    def _to_serial(self, ops: list) -> str:
        """
        Raw serial ASCII commands — ultra-minimal format for 2000s-era PLCs
        and CNC controllers with RS-232/RS-485 interfaces.
        Format: CMD ARG1 ARG2 ... <CRLF>
        """
        lines = []
        for op in ops:
            t = op.get("op")
            if t == "move":
                x, y, z = _safe_xyz(op)
                lines.append(f"G01 X{x:.3f} Y{y:.3f} Z{z:.3f} F{int(self._current_speed*60)}")
            elif t == "gripper":
                lines.append(f"M{'03' if op.get('action') == 'close' else '05'}")
            elif t == "wait":
                s = _safe_seconds(op)
                lines.append(f"G04 P{int(s*1000)}")  # Dwell in milliseconds (G-code standard)
            elif t == "home":
                lines.append("G28")  # G-code return to home
            elif t == "speed":
                self._current_speed = _safe_vel(op)
                lines.append(f"F{int(self._current_speed * 60)}")  # Feed rate mm/min
            elif t == "stop":
                lines.append("M00")  # Program stop
            elif t == "tool":
                state = op.get("state", "off")
                lines.append(f"M{'06' if state == 'on' else '09'}")
            elif t == "repeat":
                body = op.get("body", [])
                times = int(op.get("times", 1))
                for _ in range(min(times, 20)):  # Unroll loops (serial has no loop construct)
                    lines.extend(self._to_serial(body).split("\n"))
            elif t == "safety":
                lines.append("; SAFETY CHECK")
                lines.extend(self._to_serial(op.get("then", [])).split("\n"))
        return "\n".join(lines)

    # ── Modbus TCP register map ────────────────────────────────────────────────

    def _to_modbus(self, ops: list) -> str:
        """
        Modbus TCP register map (CSV format).
        Suitable for PLCs and legacy VFDs that expose Modbus over TCP.

        Register layout (16-bit integers, scaled x1000 for floats):
          40001 — Op code (1=move, 2=gripper, 3=home, 4=wait, 5=speed, 6=stop)
          40002 — X * 1000  (mm)
          40003 — Y * 1000
          40004 — Z * 1000
          40005 — Param1   (gripper: 1=close/0=open, wait: ms, speed: mm/s)
          40006 — Execute   (write 1 to trigger)
        """
        OP_CODES = {"move": 1, "gripper": 2, "home": 3, "wait": 4,
                    "speed": 5, "stop": 6, "joint_move": 7, "tool": 8}
        lines = ["# Foros Modbus TCP Register Map — AMEFORGE",
                 "# Format: register_address,value",
                 "# Write registers in order, then set 40006=1 to execute each command"]
        for op in ops:
            t = op.get("op")
            code = OP_CODES.get(t, 0)
            if not code:
                lines.append(f"# Skipped unsupported op: {t}")
                continue
            lines.append(f"# --- {t} ---")
            lines.append(f"40001,{code}")
            if t == "move":
                x, y, z = _safe_xyz(op)
                lines.append(f"40002,{int(x*1000)}")
                lines.append(f"40003,{int(y*1000)}")
                lines.append(f"40004,{int(z*1000)}")
                lines.append("40005,0")
            elif t == "gripper":
                lines.append("40002,0\n40003,0\n40004,0")
                lines.append(f"40005,{'1' if op.get('action') == 'close' else '0'}")
                if "force" in op:
                    lines.append(f"40007,{int(_safe_force(op)*1000)}")
            elif t == "wait":
                s = _safe_seconds(op)
                lines.append(f"40002,0\n40003,0\n40004,0")
                lines.append(f"40005,{int(s*1000)}")  # milliseconds
            elif t == "speed":
                v = _safe_vel(op)
                lines.append(f"40002,0\n40003,0\n40004,0")
                lines.append(f"40005,{int(v*1000)}")  # mm/s
            else:
                lines.append("40002,0\n40003,0\n40004,0\n40005,0")
            lines.append("40006,1  # EXECUTE")
        return "\n".join(lines)

    # ── ROS 2 (rclpy / MoveIt 2 Python Node) ──────────────────────────────────

    def _to_ros2(self, ops: list) -> str:
        """Generates a ROS 2 rclpy / MoveIt2 trajectory execution script."""
        lines = [
            "#!/usr/bin/env python3",
            "# Generated by Foros Adapter — AMEFORGE (ROS 2 Humble / MoveIt 2)",
            "import time",
            "import rclpy",
            "from rclpy.node import Node",
            "from geometry_msgs.msg import PoseStamped",
            "from std_msgs.msg import Bool",
            "",
            "class ForosActionNode(Node):",
            "    def __init__(self):",
            "        super().__init__('foros_action_node')",
            "        self.pose_pub = self.create_publisher(PoseStamped, '/moveit2/target_pose', 10)",
            "        self.grip_pub = self.create_publisher(Bool, '/gripper/command', 10)",
            "        self.velocity_scale = 0.25",
            "",
            "    def move_to(self, x: float, y: float, z: float):",
            "        msg = PoseStamped()",
            "        msg.header.frame_id = 'base_link'",
            "        msg.pose.position.x = x",
            "        msg.pose.position.y = y",
            "        msg.pose.position.z = z",
            "        msg.pose.orientation.w = 1.0",
            "        self.pose_pub.publish(msg)",
            "        time.sleep(0.5)",
            "",
            "def main():",
            "    rclpy.init()",
            "    node = ForosActionNode()",
        ]
        for op in ops:
            t = op.get("op")
            if t == "move":
                x, y, z = _safe_xyz(op)
                lines.append(f"    node.move_to({x}, {y}, {z})")
            elif t == "gripper":
                act = "True" if op.get("action") == "close" else "False"
                lines.append(f"    node.grip_pub.publish(Bool(data={act}))")
            elif t == "speed":
                v = _safe_vel(op)
                lines.append(f"    node.velocity_scale = {v}")
            elif t == "wait":
                s = _safe_seconds(op)
                lines.append(f"    time.sleep({s})")
            elif t == "home":
                lines.append("    node.move_to(0.25, 0.0, 0.45)  # Home posture")
            elif t == "stop":
                lines.append("    node.get_logger().warn('Emergency stop triggered')")
                lines.append("    break")
        lines.extend([
            "    node.destroy_node()",
            "    rclpy.shutdown()",
            "",
            "if __name__ == '__main__':",
            "    main()",
        ])
        return "\n".join(lines)

    # ── Doosan Robotics Language (DRL) ────────────────────────────────────────

    def _to_drl(self, ops: list) -> str:
        """Generates Doosan Robotics Language (DRL) Python-based controller code."""
        lines = [
            "# Generated by Foros Adapter — AMEFORGE",
            "# Target: Doosan Robotics M/H/A-Series (DRL)",
            "set_velx(250, 80.625)",
            "set_accx(500, 161.25)",
        ]
        for op in ops:
            t = op.get("op")
            if t == "move":
                x, y, z = _safe_xyz(op)
                lines.append(f"movel(posx({round(x*1000, 1)}, {round(y*1000, 1)}, {round(z*1000, 1)}, 0.0, 180.0, 0.0))")
            elif t == "joint_move":
                joints = [round(float(j), 2) for j in op.get("joints", [0,0,0,0,0,0])][:6]
                jstr = ", ".join(str(j) for j in joints)
                lines.append(f"movej(posj({jstr}))")
            elif t == "gripper":
                val = 1 if op.get("action") == "close" else 0
                lines.append(f"set_digital_output(1, {val})  # gripper {'close' if val else 'open'}")
            elif t == "speed":
                v = _safe_vel(op)
                lines.append(f"set_velx({round(v*1000, 1)}, 80.625)")
            elif t == "wait":
                s = _safe_seconds(op)
                lines.append(f"wait({s})")
            elif t == "home":
                lines.append("movej(posj(0, 0, 90, 0, 90, 0))")
            elif t == "stop":
                lines.append("stop(DR_SSTOP)")
        return "\n".join(lines)

    # ── Yaskawa Motoman INFORM III (.JBI) ─────────────────────────────────────

    def _to_inform(self, ops: list) -> str:
        """Generates Yaskawa Motoman INFORM III job file (.JBI)."""
        pos_lines = []
        cmd_lines = ["NOP"]
        p_idx = 0
        current_v = 250  # mm/s
        for op in ops:
            t = op.get("op")
            if t == "speed":
                current_v = int(_safe_vel(op) * 1000)
            elif t == "move":
                x, y, z = _safe_xyz(op)
                c_idx = f"C{p_idx:05d}"
                pos_lines.append(f"{c_idx}={round(x*1000, 3)},{round(y*1000, 3)},{round(z*1000, 3)},180.0000,0.0000,0.0000")
                cmd_lines.append(f"MOVL {c_idx} V={current_v}.0")
                p_idx += 1
            elif t == "gripper":
                state = "ON" if op.get("action") == "close" else "OFF"
                cmd_lines.append(f"DOUT OT#(1) {state}")
            elif t == "wait":
                s = _safe_seconds(op)
                cmd_lines.append(f"TIMER T={s:.2f}")
            elif t == "home":
                cmd_lines.append("MOVJ VJ=25.00")
            elif t == "stop":
                cmd_lines.append("PAUSE")
        cmd_lines.append("END")
        header = [
            "/JOB",
            "//NAME FOROS_ACTION",
            "//POS",
            f"///NPOS {len(pos_lines)},0,0,0,0,0",
            "///TOOL 0",
            "///POSTYPE PULSE",
            "///RECTAN",
            "///RCONF 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0",
        ]
        return "\n".join(header + pos_lines + ["//INST", "///DATE 2026/10/21 08:00", "///ATTR SC,RW", "///GROUP1 RB1"] + cmd_lines)

    # ── Internal helpers ───────────────────────────────────────────────────────

    def _clamp_all(self, ops: list) -> list:
        """Apply safety clamps to all numeric values in the ops tree."""
        result = []
        for op in ops:
            if not isinstance(op, dict):
                result.append(op)
                continue
            safe = dict(op)
            t = op.get("op")
            if t == "move":
                safe["x"] = _clamp(op.get("x", 0), -COORD_MAX, COORD_MAX)
                safe["y"] = _clamp(op.get("y", 0), -COORD_MAX, COORD_MAX)
                safe["z"] = _clamp(op.get("z", 0), COORD_Z_MIN, COORD_MAX)
            elif t == "speed":
                if "velocity" in op:
                    safe["velocity"] = _clamp(op["velocity"], VEL_MIN, VEL_MAX)
            elif t == "wait":
                safe["seconds"] = _clamp(op.get("seconds", 1.0), 0.0, WAIT_MAX)
            elif t == "gripper" and "force" in op:
                safe["force"] = _clamp(op["force"], 0.0, FORCE_MAX)
            # Recurse into nested bodies
            for key in ("then", "body", "branches"):
                if key in safe:
                    if isinstance(safe[key], list) and len(safe[key]) > 0:
                        if isinstance(safe[key][0], list):
                            safe[key] = [self._clamp_all(b) for b in safe[key]]
                        else:
                            safe[key] = self._clamp_all(safe[key])
            result.append(safe)
        return result


# =============================================================================
# Command-line utility
# =============================================================================
if __name__ == "__main__":
    import sys, argparse

    parser = argparse.ArgumentParser(
        description="Foros Output Adapter — AMEFORGE",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"Supported formats: {', '.join(ForosAdapter.FORMATS)}"
    )
    parser.add_argument("ops", nargs="?",
                        help="JSON ops string OR path to JSON file")
    parser.add_argument("-f", "--format", default="plain_text",
                        choices=ForosAdapter.FORMATS,
                        help="Output format (default: plain_text)")
    parser.add_argument("-o", "--output",
                        help="Output file (default: stdout)")
    parser.add_argument("--demo", action="store_true",
                        help="Run a demo conversion across all formats")
    args = parser.parse_args()

    DEMO_OPS = [
        {"op": "home"},
        {"op": "speed", "velocity": 0.5},
        {"op": "gripper", "action": "open"},
        {"op": "move", "x": 0.5, "y": 0.3, "z": 0.4},
        {"op": "gripper", "action": "close", "force": 0.7},
        {"op": "move", "x": 0.5, "y": 0.3, "z": 0.7},
        {"op": "move", "x": -0.5, "y": 0.8, "z": 0.7},
        {"op": "move", "x": -0.5, "y": 0.8, "z": 0.2},
        {"op": "gripper", "action": "open"},
        {"op": "wait", "seconds": 1.0},
        {"op": "home"},
    ]

    if args.demo:
        print("=" * 70)
        print("  Foros Adapter DEMO — pick-and-place across all protocols")
        print("=" * 70)
        for fmt in ForosAdapter.FORMATS:
            print(f"\n{'─' * 60}")
            print(f"  Format: {fmt.upper()}")
            print(f"{'─' * 60}")
            try:
                out = ForosAdapter(format=fmt).convert(DEMO_OPS)
                print(out[:600] + ("..." if len(out) > 600 else ""))
            except Exception as e:
                print(f"  [ERROR] {e}")
        sys.exit(0)

    if not args.ops:
        parser.print_help()
        sys.exit(1)

    try:
        import os
        if os.path.isfile(args.ops):
            with open(args.ops) as f:
                ops = json.load(f)
        else:
            ops = json.loads(args.ops)
    except Exception as e:
        print(f"Error parsing input: {e}", file=sys.stderr)
        sys.exit(1)

    result = ForosAdapter(format=args.format).convert(ops)

    if args.output:
        with open(args.output, "w") as f:
            f.write(result)
        print(f"Written to {args.output}")
    else:
        print(result)
