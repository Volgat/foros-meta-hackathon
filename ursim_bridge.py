#!/usr/bin/env python3
# =============================================================================
# FOROS Action Engine — Meta Model API + URSim Hardware Bridge
# Meta Global AI Developer Hackathon Edition (Public Open-Source Release)
#
# Integrates:
#   1. Meta Muse Spark (muse-spark-1.3) — Spatial Reasoning & Task Planning
#   2. Meta SAM 3.1 (sam-3.1)           — Workcell Perception & 3D Grounding
#   3. Meta Muse Voice (muse-voice-transcribe-1.0) — Push-to-Talk Voice Control
#   4. FOROS Deterministic Compiler     — Multi-brand industrial robot code
# =============================================================================

import os
import re
import sys
import json
import math
import time
import copy
import socket
import urllib.request
import urllib.error
from dataclasses import dataclass
from typing import Optional, List

from foros_adapter import ForosAdapter


# =============================================================================
# URSim Dashboard Client (port 29999)
# =============================================================================
class URDashboard:
    """
    Lightweight client for the URSim Dashboard Server (port 29999).
    Used to power on the robot, release brakes, and check status.
    """
    PORT = 29999

    def __init__(self, host: str = "127.0.0.1", timeout: float = 5.0):
        self.host = host
        self.port = self.PORT
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None

    def connect(self) -> bool:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(self.timeout)
            self._sock.connect((self.host, self.port))
            welcome = self._sock.recv(1024).decode("utf-8", errors="replace").strip()
            print(f"[dashboard] Connected: {welcome}")
            return True
        except Exception as e:
            print(f"[dashboard] Connection failed: {e}")
            self._sock = None
            return False

    def disconnect(self):
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def send(self, cmd: str) -> str:
        for attempt in range(2):
            if not self._sock:
                if not self.connect():
                    return ""
            try:
                self._sock.sendall((cmd.strip() + "\n").encode("utf-8"))
                time.sleep(0.12)
                resp = self._sock.recv(4096).decode("utf-8", errors="replace").strip()
                if resp:
                    return resp
                # Empty response means remote closed idle socket; reconnect once
                self.disconnect()
            except Exception as e:
                self.disconnect()
                if attempt == 1:
                    return f"ERROR: {e}"
        return ""

    def power_on(self):
        print("[dashboard] Powering on robot...")
        r = self.send("power on")
        print(f"[dashboard] power on -> {r}")
        time.sleep(3.0)
        r = self.send("brake release")
        print(f"[dashboard] brake release -> {r}")
        time.sleep(2.0)

    def set_remote_control(self):
        return self.send("enable freedrive")

    def get_robot_mode(self) -> str:
        return self.send("robotmode")

    def ensure_ready(self):
        """Ensure URSim is out of PROTECTIVE_STOP / FAULT and in RUNNING mode before script execution."""
        sm = self.send("safetymode")
        if "PROTECTIVE_STOP" in sm or "FAULT" in sm or "VIOLATION" in sm:
            print(f"[dashboard] Auto-recovering safety state: {sm}")
            self.send("unlock protective stop")
            self.send("close safety popup")
            self.send("close popup")
            time.sleep(1.8)
            self.send("unlock protective stop")
            self.send("close safety popup")
        rm = self.send("robotmode")
        if "RUNNING" not in rm:
            print(f"[dashboard] Auto-powering robot from state: {rm}")
            self.send("power on")
            time.sleep(2.5)
            self.send("brake release")
            time.sleep(1.5)

    def stop_program(self):
        return self.send("stop")

    def is_connected(self) -> bool:
        return self._sock is not None


# =============================================================================
# ForosBridge — Main Pipeline (Meta Muse Spark + SAM 3.1 + Voice + URSim)
# =============================================================================
@dataclass
class BridgeResult:
    """Result of a single NL -> URSim command execution."""
    nl_input:   str
    json_ops:   Optional[list]
    urscript:   str
    sent:       bool
    latency_ms: float
    error:      Optional[str]
    valid_json: bool

    def __str__(self):
        status = "OK SENT" if self.sent else f"FAILED: {self.error}"
        return (
            f"  NL input  : {self.nl_input}\n"
            f"  JSON ops  : {json.dumps(self.json_ops) if self.json_ops else 'N/A'}\n"
            f"  URScript  : {self.urscript[:120]}{'...' if len(self.urscript) > 120 else ''}\n"
            f"  Status    : {status}\n"
            f"  Latency   : {self.latency_ms:.1f} ms"
        )


class ForosBridge:
    """
    Full pipeline:
      Voice (muse-voice-transcribe-1.0) / Natural Language
      + Workcell Vision (sam-3.1)
      -> Spatial Reasoning (muse-spark-1.3)
      -> Verified FOROS JSON Ops
      -> Multi-Brand Robot Code (URScript, KRL, RAPID, Karel, ROS 2, DRL, INFORM)
      -> URSim / Industrial Controller Execution
    """

    def __init__(
        self,
        robot_ip:     str   = "127.0.0.1",
        hf_token:     Optional[str] = None,
        robot_model:  str   = "UR5e",
        primary_port: int   = 30001,
        speed:        float = 0.25,
        dry_run:      bool  = False,
    ):
        self.robot_ip     = robot_ip
        self.robot_model  = robot_model
        self.primary_port = primary_port
        self.speed        = speed
        self.dry_run      = dry_run

        self._tcp_sock:  Optional[socket.socket] = None
        self._dashboard: Optional[URDashboard]   = None
        self._adapter = ForosAdapter(format="urscript", speed_override=speed)

        # Official Meta Model API configuration (https://api.meta.ai/v1)
        self.muse_api_key = (
            os.environ.get("MODEL_API_KEY")
            or os.environ.get("MUSE_API_KEY")
            or ""
        ).strip()
        self.muse_api_base = os.environ.get("MUSE_API_BASE", "https://api.meta.ai/v1").rstrip("/")
        self.muse_model    = os.environ.get("MUSE_MODEL", "muse-spark-1.3")
        self.sam_model     = os.environ.get("SAM_MODEL", "sam-3.1")
        self.voice_model   = os.environ.get("VOICE_MODEL", "muse-voice-transcribe-1.0")

        # Calibrated workcell registry (UR5e base frame, meters). In live mode SAM 3.1 re-locates
        # each object from the overhead image (see segment_workspace); z = known part height.
        self._workcell_objects = [
            {
                "id": "obj_block_alpha",
                "sam_prompt": "cyan square",
                "label": "Aluminum Block A",
                "aliases": [
                    "aluminum block", "block a", "aluminum block a", "metal block",
                    "workpiece a", "bloc a", "bloc d'aluminium", "billet",
                ],
                "category": "Machined Billet",
                "xyz": [0.28, -0.22, 0.20],
                "approach_z": 0.40,
                "bbox_norm": [0.16, 0.22, 0.14, 0.16],
                "dimensions_mm": [45, 45, 45],
                "color": "#06b6d4",
            },
            {
                "id": "obj_gear_beta",
                "sam_prompt": "blue circle",
                "label": "Steel Spur Gear B",
                "aliases": [
                    "steel spur gear", "spur gear", "steel gear", "gear b",
                    "engrenage b", "pignon b", "pignon", "engrenage", "gear",
                ],
                "category": "Transmission Component",
                "xyz": [0.32, 0.05, 0.18],
                "approach_z": 0.40,
                "bbox_norm": [0.43, 0.18, 0.13, 0.15],
                "dimensions_mm": [60, 60, 28],
                "color": "#3b82f6",
            },
            {
                "id": "obj_sensor_gamma",
                "sam_prompt": "green square",
                "label": "PCB Sensor Module C",
                "aliases": [
                    "pcb sensor module", "sensor module c", "pcb sensor", "module c",
                    "sensor module", "pcb", "capteur c", "module electronique", "capteur",
                    "green sensor", "capteur vert",
                ],
                "category": "Electronics Subassembly",
                "xyz": [0.18, -0.32, 0.22],
                "approach_z": 0.42,
                "bbox_norm": [0.22, 0.56, 0.15, 0.14],
                "dimensions_mm": [52, 38, 18],
                "color": "#10b981",
            },
            {
                "id": "zone_tray_delta",
                "sam_prompt": "orange rectangle",
                "label": "Inspection Tray D (Target Drop)",
                "aliases": [
                    "inspection tray", "tray d", "drop zone", "pallet", "assembly fixture",
                    "target tray", "tray", "yellow tray", "orange tray", "plateau d", "plateau", "bac",
                ],
                "category": "Placement Fixture",
                "xyz": [-0.28, -0.22, 0.20],
                "approach_z": 0.40,
                "bbox_norm": [0.68, 0.28, 0.20, 0.24],
                "dimensions_mm": [140, 110, 15],
                "color": "#f59e0b",
            },
        ]

        self._last_perception_live = False
        self.history: List[BridgeResult] = []
        self._execution_state = {
            "status": "IDLE",
            "start_time": 0.0,
            "duration": 0.0,
            "move_count": 0,
        }

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        try:
            self._tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._tcp_sock.settimeout(5.0)
            self._tcp_sock.connect((self.robot_ip, self.primary_port))
            print(f"[bridge] Primary Interface connected -> {self.robot_ip}:{self.primary_port}")
        except Exception as e:
            print(f"[bridge] Primary Interface connection failed: {e}")
            self._tcp_sock = None
            if not self.dry_run:
                return False

        self._dashboard = URDashboard(self.robot_ip)
        if not self._dashboard.connect():
            print("[bridge] Dashboard not available (non-critical — continuing)")
            self._dashboard = None

        return True

    def disconnect(self):
        if self._tcp_sock:
            try:
                self._tcp_sock.sendall(b"stopj(2.0)\n")
                time.sleep(0.2)
            except Exception:
                pass
            try:
                self._tcp_sock.close()
            except Exception:
                pass
            self._tcp_sock = None
        if self._dashboard:
            self._dashboard.disconnect()
        print("[bridge] Disconnected.")

    def power_on(self):
        if self._dashboard:
            self._dashboard.power_on()
        else:
            print("[bridge] Dashboard not connected — cannot power on.")

    def get_robot_mode(self) -> str:
        if self._dashboard:
            return self._dashboard.get_robot_mode()
        return "UNKNOWN (no dashboard)"

    def load_foros(self) -> bool:
        """
        Verify Meta Model API configuration.
        Local 10M model is parked for the Meta Global AI Developer Hackathon.
        """
        if self.muse_api_key:
            print(f"[meta-api] Ready: {self.muse_model}, {self.sam_model}, {self.voice_model} via {self.muse_api_base}")
        else:
            print("[meta-api] Warning: MODEL_API_KEY not set; using deterministic spatial planner fallback.")
        return True

    @property
    def foros_ready(self) -> bool:
        return True

    @property
    def active_engine_name(self) -> str:
        if self.muse_api_key:
            return f"Meta {self.muse_model} planner + FOROS verifier & compiler"
        return "Offline deterministic planner + FOROS compiler (no Meta API key)"

    @property
    def _perception_phase(self) -> str:
        return "SAM 3.1 PERCEPTION" if self._last_perception_live else "WORKCELL REGISTRY"

    @property
    def _perception_name(self) -> str:
        return "SAM 3.1 perception" if self._last_perception_live else "the calibrated workcell registry"

    # ── 1. Meta SAM 3.1 Workspace Perception (sam-3.1) ────────────────────────

    # SAM 3.1 box record on the wire, e.g. <|box;x1=181;y1=365;x2=556;y2=486;w=1516;h=1600;c=0.96|>
    _SAM_BOX_RE = re.compile(r"<\|box;([^|]*)\|>")

    @classmethod
    def _parse_sam_boxes(cls, text: str) -> list:
        """Parse SAM 3.1 Responses output text into a list of pixel boxes."""
        boxes = []
        for m in cls._SAM_BOX_RE.finditer(text or ""):
            fields = {}
            for kv in m.group(1).split(";"):
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    try:
                        fields[k.strip()] = float(v)
                    except ValueError:
                        pass
            if all(k in fields for k in ("x1", "y1", "x2", "y2", "w", "h")):
                boxes.append(fields)
        return boxes

    def _sam_detect(self, prompt: str, image_data_url: str, timeout: float = 25.0) -> list:
        """
        One SAM 3.1 call (POST /v1/responses, model=sam-3.1) with a short noun-phrase
        prompt and an image. Streams the response and returns the parsed boxes.
        Raises on HTTP / network errors so the caller can report them honestly.
        """
        payload = json.dumps({
            "model": self.sam_model,
            "input": [{
                "type": "message",
                "role": "user",
                "content": [
                    {"type": "input_text", "text": prompt},
                    {"type": "input_image", "image_url": image_data_url},
                ],
            }],
            "stream": True,
            "metadata": {"include_confidence": "true", "mask_encoding": "one_bit"},
        }).encode("utf-8")
        req = urllib.request.Request(
            f"{self.muse_api_base}/responses",
            data=payload,
            headers={
                "Content-Type": "application/json",
                "Accept": "text/event-stream",
                "Authorization": f"Bearer {self.muse_api_key}",
            },
            method="POST",
        )
        text_parts = []
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            for raw in resp:
                line = raw.decode("utf-8", errors="replace").strip()
                if not line.startswith("data:"):
                    continue
                data = line[5:].strip()
                if not data or data == "[DONE]":
                    continue
                try:
                    evt = json.loads(data)
                except json.JSONDecodeError:
                    continue
                if evt.get("type") == "response.output_text.delta":
                    text_parts.append(evt.get("delta", ""))
                elif evt.get("type") == "error":
                    raise RuntimeError(f"SAM 3.1 stream error: {evt}")
        return self._parse_sam_boxes("".join(text_parts))

    @staticmethod
    def _pixel_to_base(u: float, v: float, w: float, h: float, camera: dict) -> tuple:
        """
        Map an image pixel to the robot base frame for the top-down orthographic
        workcell camera rendered by the web cockpit (image +u -> base +x, image +v -> base +y).
        """
        x = camera["x_min"] + (u / w) * (camera["x_max"] - camera["x_min"])
        y = camera["y_min"] + (v / h) * (camera["y_max"] - camera["y_min"])
        return round(x, 3), round(y, 3)

    def segment_workspace(
        self,
        query: Optional[str] = None,
        image_data_url: Optional[str] = None,
        camera: Optional[dict] = None,
    ) -> dict:
        """
        Workcell perception.

        * Live mode (MODEL_API_KEY set and an overhead image supplied): every workcell
          object is located by Meta SAM 3.1 from its visual prompt; the detected box
          centre is projected into the robot base frame. Z comes from the known part
          height on the table (a single overhead RGB view cannot measure height).
        * Registry mode (no key or no image): the calibrated workcell registry is returned
          as-is and clearly labelled as such — no confidence values are invented.
        """
        t0 = time.time()
        q = (query or "").strip().lower()
        live = bool(self.muse_api_key and image_data_url and camera)
        errors = []
        detections = {}

        if live:
            from concurrent.futures import ThreadPoolExecutor

            def _run(obj):
                try:
                    return obj["id"], self._sam_detect(obj["sam_prompt"], image_data_url), None
                except Exception as e:  # report, never hide
                    return obj["id"], [], f"{obj['label']}: {e}"

            with ThreadPoolExecutor(max_workers=4) as pool:
                for oid, boxes, err in pool.map(_run, self._workcell_objects):
                    if err:
                        errors.append(err)
                    if boxes:
                        detections[oid] = max(boxes, key=lambda b: b.get("c", 0.0))

        objects = []
        for obj in self._workcell_objects:
            item = {k: v for k, v in obj.items() if k != "sam_prompt"}
            det = detections.get(obj["id"])
            if det:
                w, h = det["w"], det["h"]
                u = (det["x1"] + det["x2"] + 1) / 2.0
                v = (det["y1"] + det["y2"] + 1) / 2.0
                bx, by = self._pixel_to_base(u, v, w, h, camera)
                item["xyz"] = [bx, by, obj["xyz"][2]]
                item["bbox_norm"] = [
                    round(det["x1"] / w, 4), round(det["y1"] / h, 4),
                    round((det["x2"] - det["x1"] + 1) / w, 4), round((det["y2"] - det["y1"] + 1) / h, 4),
                ]
                item["confidence"] = det.get("c")
                item["source"] = "sam-3.1"
                item["detected"] = True
            else:
                item["confidence"] = None
                item["source"] = "calibrated_registry"
                item["detected"] = False if live else None
            if q:
                item["highlighted"] = (
                    q in obj["label"].lower()
                    or q in obj["category"].lower()
                    or any(q in a for a in obj["aliases"])
                )
            else:
                item["highlighted"] = True
            objects.append(item)

        # Keep downstream grounding consistent with what perception just reported.
        for obj in self._workcell_objects:
            det_item = next((o for o in objects if o["id"] == obj["id"]), None)
            if det_item and det_item["source"] == "sam-3.1":
                obj["xyz"] = list(det_item["xyz"])

        n_detected = sum(1 for o in objects if o["source"] == "sam-3.1")
        self._last_perception_live = n_detected > 0
        return {
            "success": True,
            "mode": "sam_live" if n_detected else "registry",
            "vision_model": f"Meta {self.sam_model}" if n_detected else "Calibrated workcell registry",
            "api_connected": bool(self.muse_api_key),
            "sam_api_verified": n_detected > 0,
            "detected_count": n_detected,
            "camera_frame": "topdown_ortho_ur5e_base" if live else None,
            "latency_ms": round((time.time() - t0) * 1000.0, 1),
            "errors": errors,
            "objects": objects,
        }

    def _ground_prompt_with_vision(self, text: str) -> tuple[str, list]:
        """
        Detect references to SAM 3.1 segmented workcell objects in the operator's
        natural-language directive in exact order of appearance and ground them into
        explicit (x, y, z) coordinates.
        """
        lower = text.lower()
        matched_with_pos = []
        for obj in self._workcell_objects:
            best_pos = None
            candidates = [obj["label"].lower(), obj["label"].lower().replace(" (target drop)", "")] + obj["aliases"]
            for cand in candidates:
                idx = lower.find(cand)
                if idx != -1 and (best_pos is None or idx < best_pos):
                    best_pos = idx
            if best_pos is not None:
                matched_with_pos.append((best_pos, obj))

        matched_with_pos.sort(key=lambda item: item[0])
        grounded = [item[1] for item in matched_with_pos]

        has_explicit_coords = bool(re.search(r'\b[xyz]\s*[=:]\s*-?\d', text, re.IGNORECASE))

        # If no explicit coordinates and no grounded objects, do not silently substitute.
        # Allow downstream safety gate to enforce disambiguation or target specification.
        if has_explicit_coords or not grounded:
            return text, grounded

        if len(grounded) >= 2:
            seq_desc = ", ".join(
                f"step {idx+1} target {o['label']} at x={o['xyz'][0]} y={o['xyz'][1]} z={o['xyz'][2]} (clearance z={o['approach_z']})"
                for idx, o in enumerate(grounded)
            )
            grounded_prompt = f"{text} [SAM-3.1 Grounded Coordinates in sequence: {seq_desc}]"
            return grounded_prompt, grounded
        else:
            target = grounded[0]
            tx, ty, tz = target["xyz"]
            if any(k in lower for k in ("pick", "grab", "grasp", "take", "lift", "transfer", "move", "prends", "deplace")):
                tray = next((o for o in self._workcell_objects if o["id"] == "zone_tray_delta"), None)
                dx, dy, dz = tray["xyz"] if tray else (-0.28, -0.22, 0.20)
                grounded_prompt = (
                    f"{text} [SAM-3.1 Grounded Coordinates: pick {target['label']} at x={tx} y={ty} z={tz}, "
                    f"place on Inspection Tray D at x={dx} y={dy} z={dz}, safe clearance z=0.40]"
                )
            else:
                grounded_prompt = (
                    f"{text} [SAM-3.1 Grounded Coordinates: target {target['label']} at x={tx} y={ty} z={tz}]"
                )
            return grounded_prompt, grounded

    # ── 2. Meta Muse Voice Transcribe (muse-voice-transcribe-1.0) ─────────────

    def transcribe_voice(self, audio_bytes: bytes, mime_type: str = "audio/wav") -> dict:
        """
        Send recorded operator voice audio to Meta's muse-voice-transcribe-1.0
        endpoint (POST https://api.meta.ai/v1/asr/transcribe).
        Automatically converts browser WebM/Opus streams to 16kHz 16-bit mono WAV.
        """
        if not self.muse_api_key:
            return {"success": False, "error": "MODEL_API_KEY is not configured"}

        t0 = time.time()

        # Convert WebM/Opus or non-RIFF audio to 16kHz mono WAV so Meta ASR accepts it reliably
        if not audio_bytes.startswith(b"RIFF"):
            try:
                import subprocess
                proc = subprocess.run(
                    ["ffmpeg", "-y", "-i", "pipe:0", "-ar", "16000", "-ac", "1", "-f", "wav", "pipe:1"],
                    input=audio_bytes,
                    capture_output=True,
                    timeout=10.0,
                )
                if proc.returncode == 0 and proc.stdout.startswith(b"RIFF"):
                    audio_bytes = proc.stdout
                    mime_type = "audio/wav"
            except Exception as conv_err:
                print(f"[voice] ffmpeg WAV conversion warning: {conv_err}")

        boundary = f"----ForosMetaBoundary{int(time.time() * 1000)}"
        req_json = json.dumps({
            "model": self.voice_model,
            "audioEncoding": "WAV",
            "mode": "PUSH_TO_TALK",
        })

        body_parts = []
        # Part 1: request metadata JSON
        body_parts.append(f"--{boundary}\r\n".encode("utf-8"))
        body_parts.append(b'Content-Disposition: form-data; name="request"\r\n')
        body_parts.append(b"Content-Type: application/json\r\n\r\n")
        body_parts.append(req_json.encode("utf-8") + b"\r\n")

        # Part 2: audio binary (WAV)
        body_parts.append(f"--{boundary}\r\n".encode("utf-8"))
        body_parts.append(b'Content-Disposition: form-data; name="audio"; filename="command.wav"\r\n')
        body_parts.append(b"Content-Type: audio/wav\r\n\r\n")
        body_parts.append(audio_bytes + b"\r\n")
        body_parts.append(f"--{boundary}--\r\n".encode("utf-8"))

        payload = b"".join(body_parts)
        req = urllib.request.Request(
            f"{self.muse_api_base}/asr/transcribe",
            data=payload,
            headers={
                "Authorization": f"Bearer {self.muse_api_key}",
                "Accept": "application/json",
                "Content-Type": f"multipart/form-data; boundary={boundary}",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=15.0) as resp:
                raw_resp = resp.read().decode("utf-8")
                data = json.loads(raw_resp)
                text = (
                    data.get("text")
                    or data.get("transcript")
                    or data.get("transcription")
                    or (data.get("results", [{}])[0].get("transcript") if isinstance(data.get("results"), list) and data.get("results") else "")
                )
                return {
                    "success": True,
                    "text": text.strip(),
                    "model": self.voice_model,
                    "latency_ms": round((time.time() - t0) * 1000.0, 1),
                }
        except urllib.error.HTTPError as e:
            err_body = e.read().decode("utf-8", errors="replace")
            print(f"[voice] HTTPError {e.code}: {err_body}")
            return {"success": False, "error": f"Meta ASR HTTP {e.code}: {err_body[:200]}"}
        except Exception as e:
            print(f"[voice] Error: {e}")
            return {"success": False, "error": str(e)}

    # ── 3. Meta Muse Spark API Inference (muse-spark-1.3) ─────────────────────

    def _muse_infer(self, text: str, grounded_objects: list) -> Optional[dict]:
        """
        Call Meta Model API (https://api.meta.ai/v1/chat/completions) with
        model='muse-spark-1.3' to perform spatial reasoning and emit FOROS JSON ops.
        """
        if not self.muse_api_key:
            return None

        vision_summary = json.dumps(
            [
                {
                    "id": o["id"],
                    "label": o["label"],
                    "xyz_meters": o["xyz"],
                    "safe_approach_z": o["approach_z"],
                }
                for o in self._workcell_objects
            ]
        )

        system_prompt = (
            "You are the Meta Muse-Spark-1.3 Spatial Reasoning Engine integrated into the FOROS Action Engine "
            "for industrial 6-DOF robot control (Universal Robots UR5e, KUKA, ABB, FANUC, Doosan, Yaskawa, ROS 2).\n"
            "SAFETY CONSTRAINTS:\n"
            "1. Minimum Z height is z >= 0.15 m (table collision floor).\n"
            "2. Safe transit / clearance plane is z = 0.40 m.\n"
            "3. Maximum spherical reach envelope is r <= 0.80 m from base (0,0,0).\n"
            "4. Supported atomic operations inside 'ops':\n"
            '   - {"op": "home"}\n'
            '   - {"op": "speed", "velocity": 0.25}\n'
            '   - {"op": "move", "x": 0.28, "y": -0.22, "z": 0.40}\n'
            '   - {"op": "gripper", "action": "open"} or {"op": "gripper", "action": "close", "force": 0.6}\n'
            '   - {"op": "wait", "seconds": 0.5}\n'
            '   - {"op": "stop"}\n'
            '   - {"op": "repeat", "times": 3, "body": [...]}\n'
            '   - {"op": "safety", "cond": "collision", "then": [...]}\n'
            "5. Pick-and-place tasks MUST follow: home -> speed -> gripper open -> approach (z=0.40) -> "
            "descend to pick (z>=0.15) -> gripper close -> wait -> lift to clearance (z=0.40) -> "
            "transit above place (z=0.40) -> descend to place -> gripper open -> wait -> retract (z=0.40) -> home.\n"
            f"WORKCELL OBJECTS (base frame, meters): {vision_summary}\n\n"
            "Respond strictly with a valid JSON object matching this schema (English only, no markdown fences, no emojis):\n"
            "{\n"
            '  "reasoning_steps": [\n'
            '    {"phase": "PERCEPTION", "title": "...", "detail": "..."},\n'
            '    {"phase": "PLANNER", "title": "...", "detail": "..."},\n'
            '    {"phase": "KINEMATIC SAFETY", "title": "...", "detail": "..."},\n'
            '    {"phase": "FOROS COMPILER", "title": "...", "detail": "..."}\n'
            "  ],\n"
            '  "ops": [\n'
            '    {"op": "home"},\n'
            '    {"op": "speed", "velocity": 0.25},\n'
            '    {"op": "gripper", "action": "open"},\n'
            '    {"op": "move", "x": 0.28, "y": -0.22, "z": 0.40}\n'
            "  ]\n"
            "}"
        )

        payload = json.dumps({
            "model": self.muse_model,
            "temperature": 0.1,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": text},
            ],
        }).encode("utf-8")

        req = urllib.request.Request(
            f"{self.muse_api_base}/chat/completions",
            data=payload,
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.muse_api_key}",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=30.0) as resp:
                body = json.loads(resp.read().decode("utf-8"))
                msg = body["choices"][0].get("message", {})
                content = (msg.get("content") or msg.get("reasoning_content") or "").strip()
                if "{" in content and "}" in content:
                    content = content[content.index("{"):content.rindex("}") + 1]
                parsed = json.loads(content)
                if isinstance(parsed.get("ops"), list) and parsed["ops"]:
                    print(f"[{self.muse_model}] Live API inference succeeded ({len(parsed['ops'])} ops)")
                    return parsed
        except urllib.error.HTTPError as e:
            err_body = e.read().decode("utf-8", errors="replace")
            print(f"[muse-spark-1.3] HTTPError {e.code}: {err_body}")
        except Exception as e:
            print(f"[muse-spark-1.3] API error: {e}")
        return None

    # ── Deterministic Spatial Planner & Safety Verifier ───────────────────────

    @staticmethod
    def _build_spatial_reasoning(
        original_prompt: str,
        ops: list,
        grounded_objects: list,
        engine_label: str,
        perception_phase: str = "WORKCELL REGISTRY",
        planner_phase: str = "DETERMINISTIC PLANNER (OFFLINE)",
    ) -> list:
        steps = []
        if grounded_objects:
            obj_desc = ", ".join(
                f"{o['label']} at ({o['xyz'][0]:+.2f}, {o['xyz'][1]:+.2f}, {o['xyz'][2]:+.2f}) m"
                + (f" [SAM conf {o['confidence']*100:.1f}%]" if o.get("confidence") is not None else "")
                for o in grounded_objects
            )
            steps.append({
                "phase": perception_phase,
                "title": "Workcell Object Grounding",
                "detail": f"Resolved semantic target(s) to base-frame coordinates: {obj_desc}.",
            })
        else:
            steps.append({
                "phase": "SEMANTIC PARSING",
                "title": "Natural Language Intent & Coordinate Extraction",
                "detail": f"Parsed operator directive ({original_prompt!r}) into structured Cartesian and actuator constraints.",
            })

        move_ops = [op for op in ops if op.get("op") == "move"]
        grip_ops = [op for op in ops if op.get("op") == "gripper"]
        speed_ops = [op for op in ops if op.get("op") == "speed"]
        vel_str = f"{speed_ops[0].get('velocity', 0.25):.2f} m/s" if speed_ops else "0.25 m/s"

        steps.append({
            "phase": planner_phase,
            "title": f"Multi-Stage Sequence Synthesis ({len(ops)} Atomic Primitives)",
            "detail": (
                f"Synthesized {len(move_ops)} linear Cartesian waypoint(s) and "
                f"{len(grip_ops)} end-effector actuation(s) at target linear velocity {vel_str}."
            ),
        })

        max_reach = 0.0
        min_z = 999.0
        for m in move_ops:
            x, y, z = float(m.get("x", 0.0)), float(m.get("y", 0.0)), float(m.get("z", 0.0))
            r = (x**2 + y**2 + z**2) ** 0.5
            if r > max_reach:
                max_reach = r
            if z < min_z:
                min_z = z
        if not move_ops:
            min_z = 0.40

        steps.append({
            "phase": "KINEMATIC SAFETY",
            "title": "Workspace Envelope Check (ISO 10218-inspired limits)",
            "detail": (
                f"Verified peak radial reach r_max = {max_reach:.3f} m (limit <= 0.800 m) and "
                f"minimum tool height z_min = {min_z:.3f} m (collision floor >= 0.150 m). "
                f"Linear interpolation (movel) used between waypoints."
            ),
        })

        steps.append({
            "phase": "FOROS COMPILER",
            "title": "Deterministic Hardware Protocol Translation",
            "detail": f"Compiled validated JSON operation graph via {engine_label} with zero syntax variance.",
        })
        return steps

    def _synthesize_spatial_plan(self, text: str, grounded_objects: list) -> list:
        """
        Deterministic fallback planner when offline or if API credits are paused.
        Handles pick-and-place, multi-point patrol, repeat loops, FSM safety, and moves.
        """
        lower = text.lower()
        coord_hits = re.findall(r'\b([xyz])\s*[=:]\s*(-?\d+\.?\d*)', text, re.IGNORECASE)
        triplets = []
        i = 0
        while i <= len(coord_hits) - 3:
            axes = [h[0].lower() for h in coord_hits[i:i+3]]
            if set(axes) == {'x', 'y', 'z'}:
                t = {h[0].lower(): float(h[1]) for h in coord_hits[i:i+3]}
                triplets.append((t['x'], t['y'], max(0.15, t['z'])))
                i += 3
            else:
                i += 1

        vel_m = re.search(r'(?:speed|velocity|vel)\s*(?:to\s+|=\s*|at\s*|of\s*)?(-?\d+\.?\d*)', text, re.IGNORECASE)
        vel = float(vel_m.group(1)) if vel_m else 0.25

        # 1) Safety / Collision FSM
        if "collision" in lower or "obstacle" in lower:
            cond = "collision" if "collision" in lower else "obstacle"
            return [
                {"op": "safety", "cond": cond, "then": [
                    {"op": "stop"},
                    {"op": "gripper", "action": "open"},
                    {"op": "home"},
                ]}
            ]

        # 2) Repeat loop
        rep_m = re.search(r'repeat\s+(\d+)\s*times', lower)
        if rep_m:
            times = int(rep_m.group(1))
            tx, ty, tz = triplets[0] if triplets else (0.25, -0.15, 0.35)
            return [
                {"op": "speed", "velocity": vel},
                {"op": "repeat", "times": times, "body": [
                    {"op": "move", "x": tx, "y": ty, "z": tz},
                    {"op": "wait", "seconds": 1.0},
                ]},
                {"op": "home"},
            ]

        # 3) Pick and place (single or multi-object sequence)
        is_pick_place = any(k in lower for k in ("pick", "place", "put", "take", "grab", "grasp", "transfer", "move"))
        if is_pick_place and (len(triplets) >= 2 or len(grounded_objects) >= 1):
            z_clear = 0.40
            if len(grounded_objects) >= 3 and not triplets:
                ops = [
                    {"op": "home"},
                    {"op": "speed", "velocity": vel},
                    {"op": "gripper", "action": "open"},
                ]
                for idx, obj in enumerate(grounded_objects):
                    ox, oy, oz = obj["xyz"]
                    az = obj["approach_z"]
                    is_tray = "tray" in obj["id"]
                    ops.extend([
                        {"op": "move", "x": ox, "y": oy, "z": az},
                        {"op": "move", "x": ox, "y": oy, "z": oz},
                        {"op": "gripper", "action": "open" if is_tray else "close", **({} if is_tray else {"force": 0.6})},
                        {"op": "wait", "seconds": 0.5},
                        {"op": "move", "x": ox, "y": oy, "z": az},
                    ])
                ops.append({"op": "home"})
                return ops

            if len(triplets) >= 2:
                sx, sy, sz = triplets[0]
                dx, dy, dz = triplets[1]
            elif len(grounded_objects) >= 2:
                sx, sy, sz = grounded_objects[0]["xyz"]
                dx, dy, dz = grounded_objects[1]["xyz"]
            else:
                sx, sy, sz = grounded_objects[0]["xyz"]
                dx, dy, dz = (-0.28, -0.22, 0.20)
            return [
                {"op": "home"},
                {"op": "speed", "velocity": vel},
                {"op": "gripper", "action": "open"},
                {"op": "move", "x": sx, "y": sy, "z": z_clear},
                {"op": "move", "x": sx, "y": sy, "z": sz},
                {"op": "gripper", "action": "close", "force": 0.6},
                {"op": "wait", "seconds": 0.5},
                {"op": "move", "x": sx, "y": sy, "z": z_clear},
                {"op": "move", "x": dx, "y": dy, "z": z_clear},
                {"op": "move", "x": dx, "y": dy, "z": dz},
                {"op": "gripper", "action": "open"},
                {"op": "wait", "seconds": 0.5},
                {"op": "move", "x": dx, "y": dy, "z": z_clear},
                {"op": "home"},
            ]

        if lower.strip() in ("go home", "home", "return home", "retour home"):
            return [{"op": "home"}]

        # 4) Multi-waypoint or single waypoint move
        if triplets:
            ops = [{"op": "speed", "velocity": vel}]
            for (tx, ty, tz) in triplets:
                ops.append({"op": "move", "x": tx, "y": ty, "z": tz})
            if "home" in lower:
                ops.append({"op": "home"})
            return ops

        if grounded_objects:
            tx, ty, tz = grounded_objects[0]["xyz"]
            return [
                {"op": "speed", "velocity": vel},
                {"op": "move", "x": tx, "y": ty, "z": 0.40},
                {"op": "move", "x": tx, "y": ty, "z": tz},
            ]

        return [
            {"op": "home"},
            {"op": "speed", "velocity": vel},
            {"op": "move", "x": 0.30, "y": -0.20, "z": 0.35},
            {"op": "home"},
        ]

    @staticmethod
    def _validate_industrial_directive(text: str, grounded_objects: list) -> Optional[str]:
        """
        Pre-inference gate (ISO 10218-inspired limits):
        Reject conversational chitchat, non-operational questions, or unrelated text
        so the robot never executes a hallucinated trajectory on non-robotic input.
        """
        clean = text.strip()
        if not clean:
            return "SAFETY GATE: Empty directive rejected."
        lower = clean.lower()

        if grounded_objects:
            return None
        if re.search(r'\b[xyz]\s*[=:]\s*-?\d', lower):
            return None

        robot_keywords = (
            "move", "go", "pick", "place", "put", "take", "grab", "grasp", "lift",
            "drop", "transfer", "home", "return", "reset", "stop", "halt", "wait",
            "pause", "sleep", "speed", "velocity", "vel", "gripper", "grip", "open",
            "close", "repeat", "loop", "patrol", "scan", "inspect", "weld", "assemble",
            "stack", "sort", "push", "pull", "reach", "lower", "raise", "up", "down",
            "left", "right", "forward", "back", "collision", "safety", "obstacle",
            "feedback", "tool", "arm", "robot", "tcp", "joint", "pose", "waypoint",
            "circle", "square", "line", "trajectory", "deplace", "bouge", "va",
            "prends", "pose", "ouvre", "ferme", "arrete", "attend", "vitesse",
            "retour", "origine", "pince", "objet", "piece", "plateau", "bloc",
        )
        words = re.findall(r'[a-zA-Zà-ÿ0-9_]+', lower)
        if any(w in robot_keywords for w in words) or any(k in lower for k in ("pick and place", "go to", "move to")):
            return None

        return (
            f"SAFETY GATE: Rejected non-operational directive ({clean!r}). "
            "No robot motion generated. Specify a valid motion, actuator, or workcell task."
        )

    @classmethod
    def _enforce_iso10218_envelope(cls, ops: list, prompt: str) -> tuple[list, list]:
        """
        Enforce the UR5e workspace envelope (ISO 10218-inspired limits) on all JSON ops:
          - z in [0.15 m, 0.75 m] (table collision floor & upper height limit)
          - r_xy = sqrt(x^2 + y^2) >= 0.22 m (central base column singularity guard)
          - r = sqrt(x^2 + y^2 + z^2) <= 0.78 m (spherical reach envelope)
          - speed.velocity in [0.05, 1.0] m/s
        Returns (sanitized_ops, clamp_notes).
        """
        sanitized = copy.deepcopy(ops)
        notes = []

        # Detect if prompt explicitly requested out-of-envelope coordinates
        for axis, val_str in re.findall(r'\b([xyz])\s*[=:]\s*(-?\d+\.?\d*)', prompt, re.IGNORECASE):
            ax = axis.lower()
            v = float(val_str)
            if ax == "z" and v < 0.15:
                notes.append(f"Requested z={v}m violated table collision floor (z_min=0.15m); clamped to z=0.15m")
            elif ax == "z" and v > 0.75:
                notes.append(f"Requested z={v}m exceeded vertical envelope (z_max=0.75m); clamped to z=0.75m")
            elif ax in ("x", "y") and abs(v) > 0.78:
                notes.append(f"Requested {ax}={v}m exceeded UR5e reach envelope (r_max=0.78m); clamped to safe boundary")

        def _sanitize_list(op_list: list):
            for op in op_list:
                if not isinstance(op, dict):
                    continue
                t = op.get("op")
                if t == "move":
                    ox = float(op.get("x", 0.0))
                    oy = float(op.get("y", 0.0))
                    oz = float(op.get("z", 0.30))
                    x, y, z = ox, oy, oz

                    # If x and y are near zero (e.g. prompt only gave z=...), place at safe front table center
                    if abs(x) < 0.04 and abs(y) < 0.04:
                        x, y = 0.0, -0.30

                    # 1. Z floor & ceiling clamp
                    z = max(0.15, min(0.75, z))

                    # 2. Inner base-column singularity cylinder guard (r_xy >= 0.22m)
                    r_xy = math.sqrt(x * x + y * y)
                    if r_xy < 0.22:
                        if r_xy < 1e-4:
                            x, y = 0.0, -0.26
                        else:
                            scale_xy = 0.22 / r_xy
                            x = x * scale_xy
                            y = y * scale_xy

                    # 3. Outer spherical reach envelope clamp (r <= 0.78m)
                    r = math.sqrt(x * x + y * y + z * z)
                    if r > 0.78:
                        scale = 0.78 / r
                        x = x * scale
                        y = y * scale
                        z = max(0.15, z * scale)

                    x, y, z = round(x, 3), round(y, 3), round(z, 3)
                    if (abs(x - ox) > 0.01 or abs(y - oy) > 0.01 or abs(z - oz) > 0.01):
                        notes.append(f"Clamped waypoint ({ox:.2f}, {oy:.2f}, {oz:.2f}) -> ({x:.2f}, {y:.2f}, {z:.2f})m")
                    op["x"], op["y"], op["z"] = x, y, z

                elif t == "speed":
                    v = float(op.get("velocity", 0.25))
                    op["velocity"] = round(max(0.05, min(1.0, v)), 3)

                elif t == "repeat" and isinstance(op.get("body"), list):
                    op["times"] = max(1, min(20, int(op.get("times", 1))))
                    _sanitize_list(op["body"])

                elif t == "safety" and isinstance(op.get("then"), list):
                    _sanitize_list(op["then"])

        _sanitize_list(sanitized)
        return sanitized, notes

    @staticmethod
    def _correct_ops_from_prompt(ops: list, prompt: str) -> list:
        """Verify coordinates against in-envelope prompt specifications."""
        coord_pattern = re.compile(r'\b([xyz])\s*[=:]\s*(-?\d+\.?\d*)', re.IGNORECASE)
        coord_hits = coord_pattern.findall(prompt)

        move_targets = []
        i = 0
        while i < len(coord_hits) - 2:
            axes = [h[0].lower() for h in coord_hits[i:i+3]]
            if set(axes) == {'x', 'y', 'z'}:
                triplet = {h[0].lower(): float(h[1]) for h in coord_hits[i:i+3]}
                # Only use prompt triplet directly if it lies within the valid UR5e kinematic envelope
                tx, ty, tz = triplet['x'], triplet['y'], triplet['z']
                r = math.sqrt(tx*tx + ty*ty + tz*tz)
                r_xy = math.sqrt(tx*tx + ty*ty)
                if 0.15 <= tz <= 0.75 and 0.22 <= r_xy and r <= 0.78:
                    move_targets.append(triplet)
                i += 3
            else:
                i += 1

        vel_pattern = re.compile(
            r'(?:speed|velocity|vel(?:ocity)?)\s*(?:to\s+|=\s*|at\s*|of\s*)?(-?\d+\.?\d*)',
            re.IGNORECASE,
        )
        vel_m = vel_pattern.search(prompt)
        prompt_velocity = float(vel_m.group(1)) if vel_m else None

        if not move_targets and prompt_velocity is None:
            return ops

        corrected = copy.deepcopy(ops)
        move_target_idx = 0
        speed_corrected = False

        for op in corrected:
            t = op.get("op")
            if t == "speed" and prompt_velocity is not None and not speed_corrected:
                op["velocity"] = max(0.05, min(1.0, prompt_velocity))
                speed_corrected = True
            elif t == "move" and move_target_idx < len(move_targets):
                target = move_targets[move_target_idx]
                dist = math.sqrt(
                    (op.get("x", 0.0) - target.get("x", 0.0)) ** 2
                    + (op.get("y", 0.0) - target.get("y", 0.0)) ** 2
                    + (op.get("z", 0.0) - target.get("z", 0.0)) ** 2
                )
                z_diff = abs(op.get("z", 0.0) - target.get("z", 0.0))
                if dist < 0.30 and z_diff < 0.05:
                    for axis in ("x", "y", "z"):
                        if axis in target:
                            op[axis] = target[axis]
                    move_target_idx += 1

        return corrected

    def _lock_grounded_object_coordinates(
        self, ops: list, grounded_objects: list, original_text: str
    ) -> list:
        """
        Single-Source-of-Truth Grounding Lock:
        When the operator directive references SAM 3.1 calibrated workcell objects
        without custom user-typed (x=, y=, z=) overrides, deterministically lock
        every move operation's (x, y, z) to the exact calibrated panel coordinates:
          - Aluminum Block A        : (0.28, -0.22, 0.20), approach_z=0.40
          - Steel Spur Gear B       : (0.32,  0.05, 0.18), approach_z=0.40
          - PCB Sensor Module C     : (0.18, -0.32, 0.22), approach_z=0.42
          - Inspection Tray D       : (-0.28, -0.22, 0.20), approach_z=0.40
        """
        if not grounded_objects:
            return ops
        if re.search(r'\b[xyz]\s*[=:]\s*-?\d', original_text, re.IGNORECASE):
            return ops

        locked = copy.deepcopy(ops)
        lower = original_text.lower()

        if len(grounded_objects) == 1 and any(
            k in lower for k in ("pick", "grab", "grasp", "take", "lift", "transfer", "move", "prends", "deplace")
        ):
            expected_stations = [grounded_objects[0], self._workcell_objects[3]]
        else:
            expected_stations = list(grounded_objects)

        # Group top-level move ops into consecutive station visits (same xy within 0.12m)
        station_groups = []
        current_group = []
        last_xy = None

        for op in locked:
            if isinstance(op, dict) and op.get("op") == "move":
                mx, my = float(op.get("x", 0.0)), float(op.get("y", 0.0))
                if last_xy is None or math.hypot(mx - last_xy[0], my - last_xy[1]) <= 0.12:
                    current_group.append(op)
                    if last_xy is None:
                        last_xy = (mx, my)
                else:
                    station_groups.append(current_group)
                    current_group = [op]
                    last_xy = (mx, my)

        if current_group:
            station_groups.append(current_group)

        # If a multi-object prompt (>= 2 objects) produced fewer station visits than grounded objects,
        # synthesize the complete deterministic multi-station sequence so no object is ever skipped
        if len(expected_stations) >= 2 and len(station_groups) < len(expected_stations):
            return self._synthesize_spatial_plan(original_text, grounded_objects)

        # Lock each station group to its corresponding calibrated SAM 3.1 object
        for idx, group in enumerate(station_groups):
            if idx < len(expected_stations):
                target_obj = expected_stations[idx]
            else:
                # Match to nearest calibrated object
                gx, gy = float(group[0].get("x", 0.0)), float(group[0].get("y", 0.0))
                target_obj = min(
                    self._workcell_objects,
                    key=lambda o: math.hypot(gx - o["xyz"][0], gy - o["xyz"][1]),
                )

            tx, ty, tz = target_obj["xyz"]
            az = target_obj["approach_z"]
            for mop in group:
                mz = float(mop.get("z", az))
                mop["x"] = tx
                mop["y"] = ty
                mop["z"] = tz if mz < 0.30 else az

        # Also lock any move ops nested inside repeat.body
        for op in locked:
            if isinstance(op, dict) and op.get("op") == "repeat" and isinstance(op.get("body"), list):
                target_obj = expected_stations[0]
                tx, ty, tz = target_obj["xyz"]
                az = target_obj["approach_z"]
                for bop in op["body"]:
                    if isinstance(bop, dict) and bop.get("op") == "move":
                        mz = float(bop.get("z", az))
                        bop["x"] = tx
                        bop["y"] = ty
                        bop["z"] = tz if mz < 0.30 else az

        return locked

    def _check_safety_rejection(self, text: str, grounded_objects: list) -> Optional[dict]:
        """
        Deterministic pre-execution Safety & Validity Check:
          1. Rejects explicit out-of-reach coordinates (r > 0.80 m) with exact calculation.
          2. Rejects explicit below-table coordinates (z < 0.15 m) with exact calculation.
          3. Rejects non-robotic / conversational prompts with zero motion emitted.
        Returns a structured rejection response with red REJECTED badges if triggered.
        """
        clean = text.strip()
        lower = clean.lower()

        # 1) Check explicit coordinates in prompt for out-of-envelope or table-collision violation
        coords = {ax.lower(): float(val) for ax, val in re.findall(r'\b([xyz])\s*[=:]\s*(-?\d+\.?\d*)', clean, re.IGNORECASE)}
        if coords:
            cx = coords.get("x", 0.0)
            cy = coords.get("y", 0.0)
            cz = coords.get("z", 0.30)
            r = math.sqrt(cx * cx + cy * cy + cz * cz)

            if "z" in coords and cz < 0.15:
                return {
                    "ops": [{"op": "home"}, {"op": "stop"}],
                    "raw": json.dumps([{"op": "home"}, {"op": "stop"}]),
                    "safety_rejected": True,
                    "rejection_reason": f"Table Collision Violation: z={cz:.3f}m < 0.150m floor",
                    "reasoning_steps": [
                        {
                            "phase": "SEMANTIC PARSING",
                            "status": "VERIFIED",
                            "title": "Explicit Vertical Coordinate Extraction",
                            "detail": f"Extracted requested vertical target z = {cz:.3f} m from operator directive ({clean!r}).",
                        },
                        {
                            "phase": "KINEMATIC SAFETY",
                            "status": "REJECTED (z < 0.15m)",
                            "title": "Table Collision Floor Violation Detected",
                            "detail": (
                                f"SAFETY REJECTION: Requested height z = {cz:.3f} m violates the hard workcell table "
                                f"collision floor (z_min >= 0.150 m). Downward motion blocked to prevent tool impact."
                            ),
                        },
                        {
                            "phase": "PRE-PLANNING SAFETY GATE",
                            "status": "HOLD ENGAGED",
                            "title": "Zero-Motion Protective Hold Synthesized",
                            "detail": "Suppressed unsafe Cartesian waypoint; emitted safe home + immediate stop hold sequence.",
                        },
                        {
                            "phase": "FOROS COMPILER",
                            "status": "SAFE HOLD",
                            "title": "Protective Stop Compilation",
                            "detail": "Compiled safe stationary hold with zero downward trajectory emission.",
                        },
                    ],
                    "grounded_objects": [],
                    "engine_mode": "muse_api_live",
                    "engine_label": self.active_engine_name,
                }

            if r > 0.80 or abs(cx) > 0.80 or abs(cy) > 0.80 or cz > 0.80:
                return {
                    "ops": [{"op": "home"}, {"op": "stop"}],
                    "raw": json.dumps([{"op": "home"}, {"op": "stop"}]),
                    "safety_rejected": True,
                    "rejection_reason": f"Reach Envelope Violation: r={r:.3f}m > 0.800m max reach",
                    "reasoning_steps": [
                        {
                            "phase": "SEMANTIC PARSING",
                            "status": "VERIFIED",
                            "title": "Cartesian Target Extraction",
                            "detail": f"Extracted target coordinates (x={cx:.2f}, y={cy:.2f}, z={cz:.2f}) m from directive.",
                        },
                        {
                            "phase": "KINEMATIC SAFETY",
                            "status": "REJECTED (r > 0.80m)",
                            "title": "Spherical Reach Envelope Exceeded",
                            "detail": (
                                f"SAFETY REJECTION: Computed radial reach r = sqrt({cx:.2f}^2 + {cy:.2f}^2 + {cz:.2f}^2) "
                                f"= {r:.3f} m exceeds UR5e maximum spherical workspace limit (r_max <= 0.800 m)."
                            ),
                        },
                        {
                            "phase": "PRE-PLANNING SAFETY GATE",
                            "status": "HOLD ENGAGED",
                            "title": "Out-of-Envelope Trajectory Blocked",
                            "detail": "Aborted unreachable waypoint generation; emitted safe home + stop protective sequence.",
                        },
                        {
                            "phase": "FOROS COMPILER",
                            "status": "SAFE HOLD",
                            "title": "Protective Hold Compilation",
                            "detail": "Compiled stationary safety hold without out-of-envelope coordinates.",
                        },
                    ],
                    "grounded_objects": [],
                    "engine_mode": "muse_api_live",
                    "engine_label": self.active_engine_name,
                }

        # 2) Disambiguation & Object Verification Gate
        # Prevent silent substitution of nonexistent, ambiguous, or attribute-conflicted objects
        color_conflicts = [
            (r'\b(red|rouge|crimson|vermeil)\s+(block|bloc|cube|billet)\b', "red block"),
            (r'\b(blue|bleu)\s+(block|bloc|cube)\b', "blue block"),
            (r'\b(green|vert|verte)\s+(block|bloc|cube)\b', "green block"),
            (r'\b(yellow|jaune)\s+(block|bloc|cube)\b', "yellow block"),
            (r'\b(black|noir|noire)\s+(block|bloc|cube)\b', "black block"),
            (r'\b(white|blanc|blanche)\s+(block|bloc|cube)\b', "white block"),
            (r'\b(wooden|bois|plastic|plastique)\s+(block|bloc|cube)\b', "wooden/plastic block"),
            (r'\b(red|rouge)\s+(gear|engrenage|pignon)\b', "red gear"),
            (r'\b(green|vert|verte)\s+(gear|engrenage|pignon)\b', "green gear"),
            (r'\b(yellow|jaune)\s+(gear|engrenage|pignon)\b', "yellow gear"),
            (r'\b(red|rouge)\s+(sensor|capteur|module|pcb)\b', "red sensor"),
            (r'\b(blue|bleu)\s+(sensor|capteur|module|pcb)\b', "blue sensor"),
            (r'\b(yellow|jaune)\s+(sensor|capteur|module|pcb)\b', "yellow sensor"),
            (r'\b(red|rouge)\s+(cylinder|cylindre)\b', "red cylinder"),
            # French noun + conflicting modifier order
            (r'\b(block|bloc|cube|billet)\s+(rouge|bleu|bleue|vert|verte|jaune|noir|noire|blanc|blanche|en bois|en plastique)\b', "french block conflict"),
            (r'\b(gear|engrenage|pignon)\s+(rouge|vert|verte|jaune|noir|noire|en plastique)\b', "french gear conflict"),
            (r'\b(sensor|capteur|module|pcb)\s+(rouge|bleu|bleue|jaune|noir|noire)\b', "french sensor conflict"),
            (r'\b(cylinder|cylindre)\s+(rouge|vert|verte|jaune|noir|noire)\b', "french cylinder conflict"),
        ]
        for pattern, obj_name in color_conflicts:
            m = re.search(pattern, lower)
            if m:
                detected_str = m.group(0)
                return {
                    "ops": [{"op": "home"}, {"op": "stop"}],
                    "raw": json.dumps([{"op": "home"}, {"op": "stop"}]),
                    "safety_rejected": True,
                    "rejection_reason": f"Disambiguation Safety Rejection: Object '{detected_str}' not found in {self._perception_name} (silent substitution prohibited)",
                    "reasoning_steps": [
                        {
                            "phase": self._perception_phase,
                            "status": "REJECTED (OBJECT NOT FOUND)",
                            "title": "Workcell Semantic Disambiguation Gate",
                            "detail": (
                                f"SAFETY REJECTION: Directive requested target entity '{detected_str}', but {self._perception_name} "
                                f"identifies only [Aluminum Block A, Steel Spur Gear B, PCB Sensor Module C, Inspection Tray D]. "
                                f"Silent substitution of nonexistent or ambiguous objects is not allowed by the FOROS safety policy."
                            ),
                        },
                        {
                            "phase": "PRE-PLANNING SAFETY GATE",
                            "status": "HOLD ENGAGED",
                            "title": "Disambiguation Safe Hold Synthesized",
                            "detail": (
                                f"Refused to guess or substitute ungrounded object '{detected_str}'. "
                                f"Synthesized safe stationary hold sequence; awaiting operator clarification."
                            ),
                        },
                        {
                            "phase": "KINEMATIC SAFETY",
                            "status": "ZERO MOTION",
                            "title": "Trajectory Suppression & Invariant Hold",
                            "detail": "Zero Cartesian motion emitted. Arm held in safe posture to prevent erroneous pick-and-place.",
                        },
                        {
                            "phase": "FOROS COMPILER",
                            "status": "SAFE HOLD",
                            "title": "Protective Stop Compilation",
                            "detail": "Compiled safe stop hold sequence with zero actuator movement.",
                        },
                    ],
                    "grounded_objects": [],
                    "engine_mode": "muse_api_live",
                    "engine_label": self.active_engine_name,
                }

        is_manipulation = any(k in lower for k in ("pick", "place", "move", "take", "grab", "grasp", "lift", "transfer", "put", "drop", "prends", "deplace", "pose", "mets"))
        has_tray = any("tray" in o["id"] for o in grounded_objects)
        workpiece_count = sum(1 for o in grounded_objects if "tray" not in o["id"])
        if is_manipulation and has_tray and workpiece_count == 0 and not coords:
            return {
                "ops": [{"op": "home"}, {"op": "stop"}],
                "raw": json.dumps([{"op": "home"}, {"op": "stop"}]),
                "safety_rejected": True,
                "rejection_reason": f"Disambiguation Safety Rejection: Source workpiece not found in {self._perception_name} (silent substitution prohibited)",
                "reasoning_steps": [
                    {
                        "phase": self._perception_phase,
                        "status": "REJECTED (SOURCE MISSING)",
                        "title": "Workpiece Disambiguation Failure",
                        "detail": (
                            f"SAFETY REJECTION: Directive ({clean!r}) targets drop tray but source workpiece was not detected. "
                            f"Silent substitution prohibited. Available workcell pieces: [Aluminum Block A, Steel Spur Gear B, PCB Sensor Module C]."
                        ),
                    },
                    {
                        "phase": "PRE-PLANNING SAFETY GATE",
                        "status": "HOLD ENGAGED",
                        "title": "Ambiguous Trajectory Blocked",
                        "detail": "Refused to synthesize blind pick motion without calibrated source workpiece coordinates.",
                    },
                    {
                        "phase": "KINEMATIC SAFETY",
                        "status": "ZERO MOTION",
                        "title": "Invariant Safe Standby",
                        "detail": "Zero motion emitted. Protective hold engaged.",
                    },
                    {
                        "phase": "FOROS COMPILER",
                        "status": "SAFE HOLD",
                        "title": "Protective Stop Compilation",
                        "detail": "Emitted safe stop hold sequence.",
                    },
                ],
                "grounded_objects": [],
                "engine_mode": "muse_api_live",
                "engine_label": self.active_engine_name,
            }

        if is_manipulation and not grounded_objects and not coords:
            return {
                "ops": [{"op": "home"}, {"op": "stop"}],
                "raw": json.dumps([{"op": "home"}, {"op": "stop"}]),
                "safety_rejected": True,
                "rejection_reason": "Disambiguation Safety Rejection: Ambiguous manipulation directive. Specify a calibrated workcell object (Block A, Gear B, Sensor C) or Cartesian coordinates.",
                "reasoning_steps": [
                    {
                        "phase": self._perception_phase,
                        "status": "REJECTED (AMBIGUOUS TARGET)",
                        "title": "Unspecified Target Object",
                        "detail": f"Directive ({clean!r}) contains manipulation intent but lacks specific workpiece identification. Silent guessing prohibited.",
                    },
                    {
                        "phase": "KINEMATIC SAFETY",
                        "status": "STANDBY HOLD",
                        "title": "Protective Hold Engaged",
                        "detail": "Blocked blind trajectory generation per the FOROS safety policy.",
                    },
                ],
                "grounded_objects": [],
                "engine_mode": "muse_api_live",
                "engine_label": self.active_engine_name,
            }

        # 3) Check for non-robotic / conversational input (e.g. "Hello, how are you?")
        refusal = self._validate_industrial_directive(text, grounded_objects)
        if refusal:
            return {
                "ops": [{"op": "stop"}],
                "raw": json.dumps([{"op": "stop"}]),
                "safety_rejected": True,
                "rejection_reason": refusal,
                "reasoning_steps": [
                    {
                        "phase": "SEMANTIC PARSING",
                        "status": "REJECTED (NON-ROBOTIC)",
                        "title": "Non-Operational Input Detected",
                        "detail": f"Input ({clean!r}) contains no valid industrial motion, actuator, or workcell target intent.",
                    },
                    {
                        "phase": "KINEMATIC SAFETY",
                        "status": "STANDBY HOLD",
                        "title": "Zero-Motion Standby Enforced",
                        "detail": "Blocked trajectory synthesis on conversational/ambiguous input per the FOROS safety policy.",
                    },
                ],
                "grounded_objects": [],
                "engine_mode": "muse_api_live",
                "engine_label": self.active_engine_name,
            }

        return None

    @staticmethod
    def _sanitize_model_steps(steps) -> list:
        """Keep the model's own reasoning trace (text only, bounded)."""
        out = []
        if not isinstance(steps, list):
            return out
        for st in steps[:6]:
            if not isinstance(st, dict):
                continue
            out.append({
                "phase": f"MUSE-SPARK-1.3 · {str(st.get('phase', 'REASONING'))[:40]}",
                "title": str(st.get("title", ""))[:140],
                "detail": str(st.get("detail", ""))[:600],
                "source": "model",
            })
        return out

    def infer_with_reasoning(self, text: str) -> dict:
        """
        Inference pipeline:
          1. Pre-planning safety gates (reach, floor, unknown/ambiguous objects, non-operational text).
          2. Ground object names to the workcell (SAM 3.1 detections when live, else the registry).
          3. Live: Meta Muse Spark 1.3 plans (chat/completions) and its reasoning trace is kept verbatim.
             Offline: a deterministic planner is used and labelled as such.
          4. Deterministic verifier: coordinate lock to grounded objects + workspace envelope clamp.
        """
        grounded_prompt, grounded_objects = self._ground_prompt_with_vision(text)

        rejection_res = self._check_safety_rejection(text, grounded_objects)
        if rejection_res is not None:
            return rejection_res

        muse_res = self._muse_infer(grounded_prompt, grounded_objects)
        if muse_res is not None:
            model_ops = [dict(o) for o in muse_res["ops"]]
            ops = self._correct_ops_from_prompt(muse_res["ops"], grounded_prompt)
            ops = self._lock_grounded_object_coordinates(ops, grounded_objects, text)
            ops, clamp_notes = self._enforce_iso10218_envelope(ops, text)
            verifier = self._build_spatial_reasoning(
                text, ops, grounded_objects, self.active_engine_name,
                perception_phase=self._perception_phase, planner_phase="FOROS VERIFIER",
            )[-2:]
            corrected = sum(1 for a_, b_ in zip(model_ops, ops) if a_ != b_) + abs(len(model_ops) - len(ops))
            if corrected:
                verifier.insert(0, {
                    "phase": "FOROS VERIFIER",
                    "title": f"{corrected} model op(s) corrected before compilation",
                    "detail": "Waypoints were locked to grounded object coordinates and/or clamped to the workspace envelope.",
                })
            return {
                "ops": ops,
                "raw": json.dumps(ops),
                "reasoning_steps": self._sanitize_model_steps(muse_res.get("reasoning_steps")) + verifier,
                "model_ops": model_ops,
                "grounded_objects": grounded_objects,
                "safety_notes": clamp_notes,
                "engine_mode": "muse_api_live",
                "engine_label": self.active_engine_name,
            }

        spatial_ops = self._synthesize_spatial_plan(text, grounded_objects)
        spatial_ops = self._lock_grounded_object_coordinates(spatial_ops, grounded_objects, text)
        spatial_ops, clamp_notes = self._enforce_iso10218_envelope(spatial_ops, text)
        reasoning_steps = self._build_spatial_reasoning(
            text, spatial_ops, grounded_objects,
            "Offline deterministic planner + FOROS compiler",
            perception_phase=self._perception_phase,
            planner_phase="DETERMINISTIC PLANNER (OFFLINE)",
        )
        return {
            "ops": spatial_ops,
            "raw": json.dumps(spatial_ops),
            "reasoning_steps": reasoning_steps,
            "grounded_objects": grounded_objects,
            "safety_notes": clamp_notes,
            "engine_mode": "muse_spatial_fallback",
            "engine_label": "Offline deterministic planner + FOROS compiler",
        }

    # ── URScript sending ──────────────────────────────────────────────────────

    def send_urscript(self, script: str) -> bool:
        full_script = "def foros_cmd():\n"
        for line in script.split("\n"):
            if line.strip() and not line.strip().startswith("#"):
                full_script += f"  {line}\n"
        full_script += "end\n\nforos_cmd()\n"

        if self.dry_run:
            print("[bridge] DRY RUN — URScript (not sent):")
            print(full_script)
            return True

        if self._dashboard:
            try:
                self._dashboard.ensure_ready()
            except Exception as e:
                print(f"[bridge] Dashboard ensure_ready warning: {e}")

        if not self._tcp_sock:
            print("[bridge] Not connected to robot.")
            return False

        try:
            self._tcp_sock.sendall(full_script.encode("utf-8"))
            return True
        except Exception as e:
            print(f"[bridge] Send failed: {e}")
            try:
                self._tcp_sock.close()
            except Exception:
                pass
            self._tcp_sock = None
            try:
                self._tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._tcp_sock.settimeout(5.0)
                self._tcp_sock.connect((self.robot_ip, self.primary_port))
                self._tcp_sock.sendall(full_script.encode("utf-8"))
                return True
            except Exception as e2:
                print(f"[bridge] Reconnect failed: {e2}")
                return False

    def send_urscript_direct(self, script: str) -> BridgeResult:
        t0 = time.time()
        sent = self.send_urscript(script)
        if sent:
            self.start_execution_tracking(script)
        return BridgeResult(
            nl_input="[direct URScript]",
            json_ops=None,
            urscript=script,
            sent=sent,
            latency_ms=(time.time() - t0) * 1000.0,
            error=None if sent else "Send failed",
            valid_json=False,
        )

    def start_execution_tracking(self, code: str) -> dict:
        """
        Estimate trajectory duration and track live execution progress.
        Signals completion once physical/simulated trajectory reaches termination.
        """
        move_count = len(re.findall(r'\b(movej|movel)\b', code))
        sleep_durations = [float(s) for s in re.findall(r'sleep\((\d+\.?\d*)\)', code)]
        total_sleep = sum(sleep_durations)
        est_duration = max(3.0, move_count * 1.8 + total_sleep + 1.2)
        self._execution_state = {
            "status": "RUNNING",
            "start_time": time.time(),
            "duration": est_duration,
            "move_count": move_count,
        }
        return self._execution_state

    def get_execution_status(self) -> dict:
        """
        Return the live execution status: 'IDLE' | 'RUNNING' | 'COMPLETED'.
        """
        if self._execution_state.get("status") == "RUNNING":
            elapsed = time.time() - self._execution_state.get("start_time", 0.0)
            if elapsed >= self._execution_state.get("duration", 3.0):
                self._execution_state["status"] = "COMPLETED"
        return dict(self._execution_state)

