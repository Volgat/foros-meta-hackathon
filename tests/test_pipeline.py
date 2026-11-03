"""
Deterministic pipeline tests (no network, no Meta API key).

Covers the safety scenarios D1-D10 documented in the README, the SAM 3.1 wire-format
parser, the pixel->base projection, and the 12 compiler targets.
Run:  python -m pytest -q
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.pop("MODEL_API_KEY", None)
os.environ.pop("MUSE_API_KEY", None)

from ursim_bridge import ForosBridge  # noqa: E402
from foros_adapter import ForosAdapter  # noqa: E402

REACH, FLOOR = 0.80, 0.15


@pytest.fixture()
def bridge():
    return ForosBridge(dry_run=True)


def moves(ops):
    out = []
    for op in ops:
        if op.get("op") == "move":
            out.append(op)
        for key in ("body", "then"):
            if isinstance(op.get(key), list):
                out.extend(moves(op[key]))
    return out


def obj(bridge, oid):
    return next(o for o in bridge._workcell_objects if o["id"] == oid)


def assert_inside_envelope(ops):
    for m in moves(ops):
        r = math.sqrt(m["x"] ** 2 + m["y"] ** 2 + m["z"] ** 2)
        assert r <= REACH + 1e-6, m
        assert m["z"] >= FLOOR - 1e-6, m


# ---------- D1-D6: grounded pick & place, coordinates locked to the workcell ----------
@pytest.mark.parametrize("prompt,src,dst", [
    ("Pick up the steel spur gear and place it on the inspection tray", "obj_gear_beta", "zone_tray_delta"),
    ("Pick up the aluminum block and place it on the inspection tray", "obj_block_alpha", "zone_tray_delta"),
    ("Pick up the PCB sensor module and place it on the tray", "obj_sensor_gamma", "zone_tray_delta"),
])
def test_pick_place_locked_to_grounded_objects(bridge, prompt, src, dst):
    res = bridge.infer_with_reasoning(prompt)
    assert not res.get("safety_rejected"), res.get("rejection_reason")
    assert res["engine_mode"] == "muse_spatial_fallback"  # honest: no API key in tests
    ops = res["ops"]
    xy = {(round(m["x"], 3), round(m["y"], 3)) for m in moves(ops)}
    s, d = obj(bridge, src)["xyz"], obj(bridge, dst)["xyz"]
    assert (round(s[0], 3), round(s[1], 3)) in xy, "pick waypoint not locked to source object"
    assert (round(d[0], 3), round(d[1], 3)) in xy, "place waypoint not locked to destination"
    assert any(o.get("op") == "gripper" and o.get("action") == "close" for o in ops)
    assert_inside_envelope(ops)


def test_offline_trace_is_labelled_honestly(bridge):
    res = bridge.infer_with_reasoning("Pick up the steel spur gear and place it on the inspection tray")
    phases = " ".join(s["phase"] for s in res["reasoning_steps"])
    assert "MUSE" not in phases and "SAM 3.1" not in phases
    assert "OFFLINE" in phases and "REGISTRY" in phases


# ---------- D7: out of reach ----------
def test_out_of_reach_is_rejected(bridge):
    res = bridge.infer_with_reasoning("Move to x=2.0 y=0.5 z=0.3")
    assert res.get("safety_rejected")
    assert len(moves(res.get("ops", []))) == 0 or all(
        math.sqrt(m["x"] ** 2 + m["y"] ** 2 + m["z"] ** 2) <= REACH for m in moves(res["ops"]))
    assert "Reach" in res.get("rejection_reason", "")


# ---------- D8: below the table floor ----------
def test_below_floor_is_rejected(bridge):
    res = bridge.infer_with_reasoning("Move to x=0.3 y=0.1 z=0.05")
    assert res.get("safety_rejected")
    assert "Table" in res.get("rejection_reason", "") or "floor" in res.get("rejection_reason", "").lower()


# ---------- D9: unknown object, no silent substitution ----------
def test_unknown_object_is_rejected(bridge):
    res = bridge.infer_with_reasoning("Move the red block to the tray")
    assert res.get("safety_rejected")
    assert "red block" in res.get("rejection_reason", "")
    assert all(o.get("op") != "gripper" or o.get("action") != "close" for o in res.get("ops", []))


# ---------- D10: conversational input ----------
def test_conversational_input_emits_no_motion(bridge):
    res = bridge.infer_with_reasoning("Hello, how are you ?")
    assert len(moves(res.get("ops", []))) == 0


# ---------- SAM 3.1 wire format + projection ----------
def test_sam_box_parser():
    text = ("<0f>0<|box;x1=181;y1=365;x2=556;y2=486;w=1516;h=1600;c=0.96|><|mask;x=0;y=0;data=1,2,!xx|>,"
            "1<|box;x1=10;y1=20;x2=30;y2=40;w=1516;h=1600|><|mask;x=0;y=0;data=3,4,!yy|>")
    boxes = ForosBridge._parse_sam_boxes(text)
    assert len(boxes) == 2
    assert boxes[0]["x1"] == 181 and boxes[0]["c"] == pytest.approx(0.96)
    assert "c" not in boxes[1]
    assert ForosBridge._parse_sam_boxes("") == []


def test_pixel_to_base_projection():
    cam = {"x_min": -0.6, "x_max": 0.6, "y_min": -0.6, "y_max": 0.6}
    assert ForosBridge._pixel_to_base(384, 384, 768, 768, cam) == (0.0, 0.0)
    assert ForosBridge._pixel_to_base(0, 768, 768, 768, cam) == (-0.6, 0.6)


def test_registry_mode_invents_no_confidence(bridge):
    res = bridge.segment_workspace()
    assert res["mode"] == "registry" and res["sam_api_verified"] is False
    assert all(o["confidence"] is None and o["source"] == "calibrated_registry" for o in res["objects"])


def test_sam_detection_updates_coordinates(bridge, monkeypatch):
    bridge.muse_api_key = "test-key"
    cam = {"x_min": -0.6, "x_max": 0.6, "y_min": -0.6, "y_max": 0.6, "width": 768, "height": 768}

    def fake_detect(prompt, image):
        if prompt == "blue circle":  # gear moved to (+0.30, +0.12)
            u, v = (0.30 + 0.6) / 1.2 * 768, (0.12 + 0.6) / 1.2 * 768
            return [{"x1": u - 10.5, "y1": v - 10.5, "x2": u + 9.5, "y2": v + 9.5, "w": 768, "h": 768, "c": 0.91}]
        return []

    monkeypatch.setattr(bridge, "_sam_detect", fake_detect)
    res = bridge.segment_workspace(image_data_url="data:image/png;base64,AAAA", camera=cam)
    gear = next(o for o in res["objects"] if o["id"] == "obj_gear_beta")
    assert gear["source"] == "sam-3.1" and gear["confidence"] == pytest.approx(0.91)
    assert gear["xyz"][0] == pytest.approx(0.30, abs=2e-3) and gear["xyz"][1] == pytest.approx(0.12, abs=2e-3)
    block = next(o for o in res["objects"] if o["id"] == "obj_block_alpha")
    assert block["detected"] is False and block["confidence"] is None
    # the planner now uses the SAM position
    plan = bridge.infer_with_reasoning("Pick up the steel spur gear and place it on the inspection tray")
    assert (0.3, 0.12) in {(round(m["x"], 2), round(m["y"], 2)) for m in moves(plan["ops"])}


# ---------- 12 compiler targets ----------
@pytest.mark.parametrize("fmt", ["urscript", "krl", "rapid", "fanuc", "inform", "drl",
                                 "ros1", "ros2", "serial", "modbus", "plain_text", "json"])
def test_all_compiler_targets(bridge, fmt):
    ops = bridge.infer_with_reasoning("Pick up the steel spur gear and place it on the inspection tray")["ops"]
    code = ForosAdapter(format=fmt, speed_override=None).convert(ops)
    assert isinstance(code, str) and len(code.strip()) > 20


# ---------- "holds the robot still": refusals emit no motion at all ----------
@pytest.mark.parametrize("prompt", ["Move the red block to the tray", "Move to x=2.0 y=0.5 z=0.3",
                                    "Move to x=0.3 y=0.1 z=0.05", "Hello, how are you ?"])
def test_refusals_hold_still(bridge, prompt):
    res = bridge.infer_with_reasoning(prompt)
    assert res.get("safety_rejected")
    assert res["ops"] == [{"op": "stop"}]  # no 'home', no move


# ---------- "only what passes gets compiled": model plans are verified, not clamped ----------
def _fake_model(bridge, monkeypatch, ops):
    bridge.muse_api_key = "test-key"
    monkeypatch.setattr(bridge, "_muse_infer", lambda t, g: {
        "reasoning_steps": [{"phase": "PLANNER", "title": "model trace", "detail": "d"}], "ops": ops})


def test_model_plan_descending_on_unknown_spot_is_rejected(bridge, monkeypatch):
    _fake_model(bridge, monkeypatch, [{"op": "move", "x": 0.3, "y": -0.3, "z": 0.40},
                                      {"op": "move", "x": 0.10, "y": 0.45, "z": 0.16}])
    res = bridge.infer_with_reasoning("Move to x=0.3 y=-0.3 z=0.25")
    assert res.get("safety_rejected") and res["ops"] == [{"op": "stop"}]
    assert "no known object" in res["rejection_reason"]
    assert res["reasoning_steps"][0]["source"] == "model"  # the model's own trace is kept


def test_model_plan_out_of_reach_is_rejected_not_clamped(bridge, monkeypatch):
    _fake_model(bridge, monkeypatch, [{"op": "move", "x": 0.3, "y": -0.3, "z": 0.25},
                                      {"op": "move", "x": 0.75, "y": 0.40, "z": 0.40}])
    res = bridge.infer_with_reasoning("Move to x=0.3 y=-0.3 z=0.25")
    assert res.get("safety_rejected") and "reach" in res["rejection_reason"]


def test_valid_model_plan_passes_with_model_trace(bridge, monkeypatch):
    _fake_model(bridge, monkeypatch, [{"op": "move", "x": 0.3, "y": -0.3, "z": 0.25}])
    res = bridge.infer_with_reasoning("Move to x=0.3 y=-0.3 z=0.25")
    assert not res.get("safety_rejected") and res["engine_mode"] == "muse_api_live"
    assert res["reasoning_steps"][0]["phase"].startswith("MUSE-SPARK-1.3")


@pytest.mark.parametrize("prompt", [
    "Pick up Steel Spur Gear B and place it on Inspection Tray D",
    "If collision detected then stop immediately, open gripper, and return home",
    "Repeat 3 times: move to PCB Sensor Module C at x=0.18 y=-0.32 z=0.30 then wait 1.0s",
    "Pick up Aluminum Block A, place it on Inspection Tray D, then pick up Steel Spur Gear B",
    "Patrol at speed=0.30: move to x=0.28 y=-0.22 z=0.40, move to x=0.18 y=-0.32 z=0.40, move to x=-0.28 y=-0.22 z=0.40, then go home",
])
def test_ui_example_tasks_pass_the_verifier(bridge, prompt):
    assert not bridge.infer_with_reasoning(prompt).get("safety_rejected")


# ---------- SAM guards: bad boxes never move the workcell ----------
def _box_at(x, y, c, cam_w=768):
    u, v = (x + 0.6) / 1.2 * cam_w, (y + 0.6) / 1.2 * cam_w
    return [{"x1": u - 10.5, "y1": v - 10.5, "x2": u + 9.5, "y2": v + 9.5, "w": cam_w, "h": cam_w, "c": c}]


CAM = {"x_min": -0.6, "x_max": 0.6, "y_min": -0.6, "y_max": 0.6, "width": 768, "height": 768}


def test_far_detection_is_rejected_and_registry_kept(bridge, monkeypatch):
    bridge.muse_api_key = "k"
    # "orange rectangle" reported where the aluminium block is (the swap seen in production)
    monkeypatch.setattr(bridge, "_sam_detect", lambda p, i: _box_at(0.279, -0.219, 0.88) if p == "orange rectangle" else [])
    res = bridge.segment_workspace(image_data_url="data:image/png;base64,AAAA", camera=CAM)
    tray = next(o for o in res["objects"] if o["id"] == "zone_tray_delta")
    assert tray["detected"] is False and tray["xyz"][:2] == [-0.28, -0.22]
    assert "expected position" in tray["rejected_detection"]["reason"]
    assert obj(bridge, "zone_tray_delta")["xyz"][:2] == [-0.28, -0.22]
    assert any("rejected" in e for e in res["errors"])


def test_low_confidence_detection_is_rejected(bridge, monkeypatch):
    bridge.muse_api_key = "k"
    monkeypatch.setattr(bridge, "_sam_detect", lambda p, i: _box_at(0.32, 0.06, 0.2) if p == "blue circle" else [])
    res = bridge.segment_workspace(image_data_url="data:image/png;base64,AAAA", camera=CAM)
    gear = next(o for o in res["objects"] if o["id"] == "obj_gear_beta")
    assert gear["detected"] is False and gear["xyz"][:2] == [0.32, 0.05]


def test_detections_do_not_accumulate_across_scans(bridge, monkeypatch):
    bridge.muse_api_key = "k"
    # each scan reports the gear 7 cm further: must be judged against the registry, not the last scan
    shifts = iter([0.07, 0.14])
    monkeypatch.setattr(bridge, "_sam_detect",
                        lambda p, i: _box_at(0.32 + next(shifts), 0.05, 0.9) if p == "blue circle" else [])
    bridge.segment_workspace(image_data_url="data:image/png;base64,AAAA", camera=CAM)
    assert obj(bridge, "obj_gear_beta")["xyz"][0] == pytest.approx(0.39, abs=3e-3)
    res = bridge.segment_workspace(image_data_url="data:image/png;base64,AAAA", camera=CAM)
    gear = next(o for o in res["objects"] if o["id"] == "obj_gear_beta")
    assert gear["detected"] is False
    assert obj(bridge, "obj_gear_beta")["xyz"][:2] == [0.32, 0.05]  # back to registry, not drifting
