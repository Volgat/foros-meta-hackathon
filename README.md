<div align="center">
  <img src="web/logo_dark.png" alt="FOROS Action Engine" width="320" />
  <h1>FOROS — Talk to Robots</h1>
  <p><strong>Multimodal Spatial Reasoning, Workcell Perception, and Deterministic Multi-Vendor Robot Control Powered by the Meta Model API</strong></p>
  <p>
    <a href="https://foros.run"><strong>Live Interactive Demo (https://foros.run)</strong></a> ·
    <a href="#what-was-built-during-the-hackathon-oct-26--nov-4-2026"><strong>Hackathon Contributions</strong></a> ·
    <a href="#meta-model-api-architecture"><strong>Meta AI Pipeline</strong></a> ·
    <a href="#60-second-judge-walkthrough"><strong>Judge Walkthrough</strong></a>
  </p>
</div>

---

## Overview

**FOROS Action Engine (Meta Hackathon Edition)** is an industrial robotics control platform built for the **Meta Global AI Developer Hackathon** under the **Push the Frontier** problem statement (*"Give a machine a new capability... unlock a better way to understand and control a physical system"*).

Industrial robot arms from Universal Robots, KUKA, ABB, FANUC, Doosan, and Yaskawa each require proprietary, low-level programming languages (`URScript`, `KRL`, `RAPID`, `Karel`, `DRL`, `INFORM III`). At the same time, raw LLM code generation is unsafe for physical machinery due to coordinate hallucinations, joint-space sweeping collisions, and syntax variance.

**FOROS solves this by separating multimodal spatial reasoning from deterministic hardware compilation:**
1. **Voice Directive (`muse-voice-transcribe-1.0`)**: Factory operators issue spoken commands via Push-to-Talk ASR.
2. **Workcell Perception (`sam-3.1`)**: Overhead RGB-D camera segmentation identifies workpieces on the robot table and grounds them into calibrated 3D base-frame coordinates $(x, y, z)$ in meters.
3. **Spatial Reasoning & Task Planning (`muse-spark-1.3`)**: Decomposes high-level natural language into a verified 4-stage spatial reasoning trace (`reasoning_steps`) and a sequence of atomic operation primitives (`ops`), enforcing safe approach/retract clearance planes ($z = 0.40\text{ m}$).
4. **Deterministic Compiler & Live URSim Execution (`ForosAdapter` + `URSim RTDE`)**: Clamps coordinates to the ISO-10218 workspace envelope ($z \ge 0.15\text{ m}$, $r \le 0.80\text{ m}$), compiles the verified JSON graph into 12 industrial robot formats with zero syntax variance, and executes the trajectory live on a Universal Robots **UR5e Polyscope** controller streamed via noVNC.

---

## What Was Built During the Hackathon (Oct 26 – Nov 4, 2026)

> [!IMPORTANT]
> **Compliance with Section 4 (Hackathon Rules — New Work Only):**  
> This section explicitly identifies the code, features, and integrations developed during the Meta Global AI Developer Hackathon building period.

| Module | File(s) | Newly Built Hackathon Capabilities |
| :--- | :--- | :--- |
| **1. Meta `muse-spark-1.3` Spatial Reasoning Engine** | [`ursim_bridge.py`](./ursim_bridge.py) (`_muse_infer`, `infer_with_reasoning`) | Built the live OpenAI-compatible connector to `POST https://api.meta.ai/v1/chat/completions` (`model="muse-spark-1.3"`) with a domain-specific spatial grounding system prompt that outputs both a structured 4-stage engineering trace (`reasoning_steps`) and executable FOROS JSON primitives (`ops`). |
| **2. Meta `sam-3.1` Workcell Perception & 3D Grounding** | [`ursim_bridge.py`](./ursim_bridge.py) (`segment_workspace`, `_ground_prompt_with_vision`), [`ursim_web_interface.py`](./ursim_web_interface.py) (`/api/vision/segment`) | Integrated `POST https://api.meta.ai/v1/responses` (`model="sam-3.1"`) with an extrinsic ISO-9283 camera-to-base calibration layer that maps segmented workcell objects (`Aluminum Block A`, `Steel Spur Gear B`, `PCB Sensor Module C`, `Inspection Tray D`) to 3D Cartesian coordinates $(x, y, z)$ and injects them into natural-language directives. |
| **3. Meta `muse-voice-transcribe-1.0` Push-to-Talk ASR** | [`ursim_bridge.py`](./ursim_bridge.py) (`transcribe_voice`), [`ursim_web_interface.py`](./ursim_web_interface.py) (`/api/voice/transcribe`), [`web/index.html`](./web/index.html) | Built the multipart audio streaming bridge to `POST https://api.meta.ai/v1/asr/transcribe` (`model="muse-voice-transcribe-1.0"`, `mode="PUSH_TO_TALK"`) and the browser Push-to-Talk microphone capture interface. |
| **4. Interactive Perception & Spatial Plan Cockpit** | [`web/index.html`](./web/index.html) | Added the **SAM 3.1 Workcell Perception** interactive SVG segmentation viewport, 3D Three.js workcell object synchronization, and the **Muse Spatial Plan** inspection tab displaying verified perception, planning, kinematic safety, and compilation steps. |
| **5. Expanded Multi-Vendor Compiler (`ROS 2`, `DRL`, `INFORM`)** | [`foros_adapter.py`](./foros_adapter.py) (`_to_ros2`, `_to_drl`, `_to_inform`) | Added deterministic code generators for **ROS 2 Humble / MoveIt 2 (`rclpy`)**, **Doosan Robotics Language (`DRL`)**, and **Yaskawa Motoman (`INFORM III .JBI`)**, bringing total supported industrial protocols to 12. |

---

## Meta Model API Architecture

```mermaid
flowchart LR
    subgraph Input["1. Multimodal Operator Input"]
        Mic["Push-to-Talk Audio"] -->|"POST /v1/asr/transcribe\n(muse-voice-transcribe-1.0)"| Prompt["Natural Language Directive"]
        Cam["Overhead RGB-D Camera"] -->|"POST /v1/responses\n(sam-3.1)"| Seg["Calibrated 3D Workcell Map\n(x, y, z, confidence)"]
    end

    subgraph Reasoning["2. Agentic Spatial Reasoning"]
        Prompt --> Ground["Semantic Object Grounding"]
        Seg --> Ground
        Ground -->|"POST /v1/chat/completions\n(muse-spark-1.3)"| Plan["Spatial Reasoning Trace\n+ Atomic JSON Ops"]
    end

    subgraph Execution["3. Deterministic Safety & Hardware Execution"]
        Plan --> Clamp["ISO-10218 Envelope Verifier\n(z >= 0.15m, r <= 0.80m)"]
        Clamp --> Compiler["FOROS Deterministic Adapter\n(12 Robot Protocols)"]
        Compiler -->|"TCP 30001 / RTDE 30004"| URSim["Live UR5e Polyscope Simulator\n+ 50Hz 3D Telemetry"]
    end
```

### Why Three Meta Models Together?
* **`muse-voice-transcribe-1.0`** enables hands-free voice operation in industrial environments (`PUSH_TO_TALK` mode).
* **`sam-3.1`** eliminates manual teach-pendant coordinate jogging by resolving semantic object names (*"Aluminum Block A"*, *"Inspection Tray D"*) into millimeter-accurate 3D coordinates.
* **`muse-spark-1.3`** performs multi-step spatial reasoning—synthesizing safe approach heights ($z = 0.40\text{ m}$), grasp actuations, and retract trajectories before any machine code is emitted.

---

## 60-Second Judge Walkthrough

The live deployment at **[https://foros.run](https://foros.run)** is open in **Public Judge Mode** (no account or login required):

1. **Inspect Workcell Perception (`sam-3.1`)**:
   - In the left panel under **SAM 3.1 Workcell Perception**, click **Scan Workcell** or click directly on any segmented object mask (e.g., **Aluminum Block A** at `x=0.28, y=-0.22, z=0.20m`).
   - Notice how the directive box automatically grounds the semantic object into a complete pick-and-place command targeting **Inspection Tray D**.
2. **Test Voice or Natural Language Input (`muse-voice-transcribe-1.0` & `muse-spark-1.3`)**:
   - Click **VOICE** to speak a directive (or type a custom command such as `"Pick up Steel Spur Gear B and place it on Inspection Tray D"`), then click **Generate Action**.
3. **Review the Spatial Reasoning Trace (`Muse Spatial Plan`)**:
   - In the right panel, switch between **Muse Spatial Plan** (4-stage verified reasoning trace from `sam-3.1` and `muse-spark-1.3`), **JSON Ops (Model)**, and **Robot Script**.
   - Change the **Protocol** dropdown (`URScript`, `KUKA KRL`, `ABB RAPID`, `FANUC Karel`, `Doosan DRL`, `Yaskawa INFORM`, `ROS 2`) to see deterministic cross-brand compilation.
4. **Execute Live on the Universal Robots UR5e Simulator**:
   - In the center panel, watch the **3D Render** (synchronized at 50 Hz via RTDE) or switch to the **VNC Simulator** tab to see the real **Universal Robots Polyscope** controller.
   - Click **Send & Execute on Robot** to watch the UR5e arm execute the generated trajectory in real time.

---

## Supported Industrial Protocols (12 Formats)

| Manufacturer / Standard | Protocol / Language | Format Key | Execution / Simulation Interface |
| :--- | :--- | :--- | :--- |
| **Universal Robots** | URScript (`movel` / `movej`) | `urscript` | Live URSim Polyscope (TCP `30001`, RTDE `30004`, Dashboard `29999`) |
| **KUKA** | KUKA Robot Language (`.src`) | `krl` | KUKA.OfficeLite / Mock Controller (`7000`) |
| **ABB** | RAPID (`MODULE ForosModule`) | `rapid` | ABB RobotStudio / Mock Controller (`7001`) |
| **FANUC** | Karel / TP | `fanuc` | FANUC ROBOGUIDE / Mock Controller (`7002`) |
| **Doosan Robotics** | Doosan Robotics Language | `drl` | Doosan Virtual Robot (`7003`) |
| **Yaskawa Motoman** | INFORM III (`.JBI`) | `inform` | MotoSim EG-VRC |
| **Open Robotics (ROS 2)** | `rclpy` / MoveIt 2 Node | `ros2` | ROS 2 Humble WebSocket Bridge (`9091`) |
| **Open Robotics (ROS 1)** | `rospy` / MoveIt Commander | `ros1` | ROS 1 Noetic Rosbridge (`9090`) |
| **CNC / Motion Stages** | RS-274 G-Code | `serial` | Serial / GRBL / LinuxCNC |
| **Industrial PLC** | Modbus TCP Register Map | `modbus` | Holding Registers `40001–40007` |
| **Audit / Verification** | Human-Readable Step Trace | `plain_text` | Safety Inspector Log |
| **Intermediate Graph** | Clamped FOROS JSON Schema | `json` | ISO-10218 Validated AST |

---

## Local Installation & Setup

### Prerequisites
* Python 3.10+
* A valid **Meta Model API Key** from [dev.meta.ai](https://dev.meta.ai) (`muse-spark-1.3`, `sam-3.1`, `muse-voice-transcribe-1.0`)
* *(Optional)* Docker for running the local Universal Robots URSim container (`universalrobots/ursim_e-series`)

### 1. Clone the Repository
```bash
git clone https://github.com/Volgat/foros-meta-hackathon.git
cd foros-meta-hackathon
pip install -r requirements.txt
```

### 2. Configure `.env`
```bash
cp .env.example .env
```
Edit `.env` and insert your Meta Model API key:
```ini
MODEL_API_KEY=your_meta_model_api_key_here
MUSE_API_BASE=https://api.meta.ai/v1
MUSE_MODEL=muse-spark-1.3
SAM_MODEL=sam-3.1
VOICE_MODEL=muse-voice-transcribe-1.0
```

### 3. Run the Control Server
```bash
# With a live URSim container on 127.0.0.1:
python ursim_web_interface.py --port 8080 --ip 127.0.0.1

# Or in dry-run simulation mode (no Docker container required):
python ursim_web_interface.py --port 8080 --dry-run
```
Open `http://localhost:8080` in your browser.

---

## Repository Structure

```text
foros-meta-hackathon/
├── ursim_bridge.py          # Meta Model API client (muse-spark-1.3, sam-3.1, muse-voice-transcribe-1.0) + URSim TCP/Dashboard bridge
├── ursim_web_interface.py   # HTTP/REST control server (/api/process, /api/vision/segment, /api/voice/transcribe, /api/send)
├── foros_adapter.py         # Deterministic multi-brand compiler (12 industrial robot protocols + ISO-10218 safety clamps)
├── ursim_feedback.py        # 50 Hz Real-Time Data Exchange (RTDE port 30004) telemetry engine
├── foros_db.py              # SQLite telemetry & audit logger
├── web/
│   ├── index.html           # Industrial 3D/VNC control cockpit, SAM 3.1 viewport, and Muse Spatial Plan inspector
│   ├── logo_dark.png        # FOROS brand assets
│   └── logo_light.png
└── simulators/              # Docker Compose definitions for URSim and multi-vendor mock controllers
```

---

## Team & Acknowledgments

* **Team Name:** Volgat (Indie Developer Track — Solo Participant)
* **Built With:** Meta Model API (`muse-spark-1.3`, `sam-3.1`, `muse-voice-transcribe-1.0`), Universal Robots URSim, Three.js, noVNC.
