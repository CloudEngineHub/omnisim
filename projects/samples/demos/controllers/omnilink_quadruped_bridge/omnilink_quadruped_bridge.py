# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""omnilink_quadruped_bridge — config-driven quadruped bridge.

`--robot <key>` selects an entry of `_quadruped_configs.QUADRUPED_CONFIGS`
(OmniQuad, and the Deep Robotics Lite3 / X30 / M20 / M20S / M20 Piper). The
config names the leg motors, the stand / sit poses and the sign conventions;
this file is the motion and intent surface:

    stand       -- hold the standing stance with a tiny sway
    sit         -- crouch low (knees bent further)
    wave        -- exaggerate the sway for ~6 s
    walk        -- per config: "gait" = wave-gait leg cycle + supervisor-
                   driven forward body translation (OmniQuad: the only way
                   to keep it upright while it locomotes -- pure-physics
                   walking on that URDF reliably topples after 5-10 s; see
                   `omniquad_simple_pose.py`); "wheels" = hold the stance
                   and drive the wheel motors (the M20 family, real
                   physics); "march" = the leg cycle in place
    stop        -- freeze the legs (and wheels) in current position
    home        -- alias for stand

Every robot starts with a settle: the motors are ramped from the pose the
engine spawned them in to the stand pose over the config's `settle_s` (the
Deep Robotics package notes why: projects/robots/deep_robotics/PROVENANCE.md).

The HTTP surface is intentionally minimal: list_robots, get_robot_state,
stop_robot, reset_to_home, prompt. set_joint_positions is also exposed
for advanced users who want to send raw joint vectors.

Robot window: same omnilink_chat plugin.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Tuple

from omnisim import Supervisor

import os as _os
import sys as _sys
_THIS_DIR = _os.path.dirname(_os.path.abspath(__file__))
_RELAY_PARENT = _os.path.abspath(_os.path.join(_THIS_DIR, ".."))
if _RELAY_PARENT not in _sys.path:
    _sys.path.insert(0, _RELAY_PARENT)

from _omnilink_relay.http_security import (  # noqa: E402
    RequestError,
    RequestIdGuard,
    allowed_origins,
    check_authorization,
    check_protocol_version,
    checked_origin,
    configured_token,
    error_envelope,
    nonempty_string,
    read_json,
    require_field,
    validate_request_id,
    WIRE_SERVICE,
    WIRE_VERSION,
)

try:
    from _omnilink_relay import OmniLinkRelay, Tool, is_enabled as omnilink_enabled, get_omni_key
except Exception:
    OmniLinkRelay = None  # type: ignore[assignment]
    Tool = None  # type: ignore[assignment]
    def omnilink_enabled() -> bool: return False
    def get_omni_key() -> str: return ""


from _quadruped_configs import LEGS, QUADRUPED_CONFIGS  # noqa: E402

# Walk parameters (wave gait + supervisor-driven body translation; see
# omniquad_simple_pose for the longer rationale and physics caveats). The
# deltas are MAGNITUDES; the config's hip_sweep_dir / knee_flex_dir give them
# a sign per leg.
WALK_PERIOD_S = 4.0
SWING_FRACTION = 0.20
SWING_KNEE_DELTA = 0.30        # extra knee flexion at mid-swing
HIP_SWEEP_AMP = 0.18
WALK_VELOCITY_RAMP_S = 2.0     # accelerate from 0 to the config's walk_velocity_ms
COM_SHIFT_AMP_Y = 0.08
SETTLE_HOLD_S = 1.5            # OmniQuad only: hold the `extend` pose before the ramp

LEG_PHASES = {
    "front_right": 0.00,
    "rear_left":   0.25,
    "front_left":  0.50,
    "rear_right":  0.75,
}


def _smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--robot", default="omniquad", choices=sorted(QUADRUPED_CONFIGS))
    p.add_argument("--port", type=int, default=8765)
    args, _ = p.parse_known_args()
    return args


class QuadrupedBridge:
    def __init__(self, robot: Supervisor, robot_id: str = "omniquad") -> None:
        self.robot = robot
        self.robot_id = robot_id
        self.cfg = QUADRUPED_CONFIGS[robot_id]
        self.model = self.cfg["model"]
        self.timestep = int(robot.getBasicTimeStep())

        # 12 leg motors, named by the config (URDF joint name + "_motor",
        # falling back to the bare joint name), plus their position sensors
        # so the settle can start from the pose the engine actually spawned.
        self.motors = {}
        self.sensors = {}
        for leg in LEGS:
            for joint in ("hip_x", "hip_y", "knee"):
                name = self.cfg["legs"][leg][joint]
                m = robot.getDevice(name)
                if m is None and name.endswith("_motor"):
                    m = robot.getDevice(name[:-len("_motor")])
                if m is None:
                    print(f"[omnilink_quadruped_bridge] WARNING: missing {name}")
                self.motors[(leg, joint)] = m
                s = m.getPositionSensor() if m is not None else None
                if s is not None:
                    s.enable(self.timestep)
                self.sensors[(leg, joint)] = s
        # Wheel motors (M20 family): velocity-controlled, parked at spawn.
        self.wheels = {}
        for leg, name in self.cfg.get("wheels", {}).items():
            w = robot.getDevice(name)
            if w is None and name.endswith("_motor"):
                w = robot.getDevice(name[:-len("_motor")])
            if w is None:
                print(f"[omnilink_quadruped_bridge] WARNING: missing wheel {name}")
                continue
            w.setPosition(float("inf"))
            w.setVelocity(0.0)
            self.wheels[leg] = w

        # Motion state.
        self.lock = threading.RLock()
        # ("settle"|"crouch"|"stand"|"sit"|"wave"|"walk"|"stop", params)
        self.motion = ("settle", {"t0": robot.getTime()})
        # Per-(leg, joint) pose the settle ramps FROM: the config's `extend`
        # pose when it has one (OmniQuad), else the measured spawn pose.
        self.settle_from: Dict[Tuple[str, str], float] = {}
        self.last_tick_at = time.time()

        # Supervisor handles for walk-phase body translation + orientation lock.
        self.self_node = None
        self.translation_field = None
        self.rotation_field = None
        try:
            self.self_node = robot.getSelf()
        except Exception:
            self.self_node = None
        if self.self_node is not None:
            try:
                self.translation_field = self.self_node.getField("translation")
            except Exception:
                self.translation_field = None
            try:
                self.rotation_field = self.self_node.getField("rotation")
            except Exception:
                self.rotation_field = None

        # Persistent floating-base anchor (x, y, z), captured lazily on the
        # first body-lock. The body is supervisor-pinned to this every tick so
        # the stiff-legged stance holds upright under any physics backend
        # (without it the unbalanced stance topples under Newton/XPBD; ODE's
        # softer solver merely tolerated it).
        self.body_anchor: Optional[Tuple[float, float, float]] = None

        # wwi outbox.
        self.window_outbox: List[str] = []
        self.window_configured = False

        self.capabilities = {
            "model": self.model,
            "legs": list(LEGS),
            "joints_per_leg": ["hip_x", "hip_y", "knee"],
            "stand_pose": {leg: {"hip_y": hy, "knee": kn} for leg, (hy, kn) in self.cfg["stand"].items()},
            "sit_pose": {leg: {"hip_y": hy, "knee": kn} for leg, (hy, kn) in self.cfg["sit"].items()},
            "walk": self.cfg["walk"],
            "walk_velocity_ms": self.cfg["walk_velocity_ms"],
            "body_lock": bool(self.cfg["body_lock"]),
        }

    def queue_window(self, line: str) -> None:
        with self.lock:
            self.window_outbox.append(line)

    def _pose(self, leg: str, base: str, hip_y_delta: float = 0.0, knee_delta: float = 0.0,
              hip_x_extra: float = 0.0) -> Tuple[float, float, float]:
        """Absolute (hip_x, hip_y, knee) for `leg`: the config's `base` pose
        ("stand" / "sit") plus signed deltas. `hip_y_delta` > 0 moves the
        foot backward, `knee_delta` > 0 flexes the knee, whatever the
        robot's joint convention."""
        hy, kn = self.cfg[base][leg]
        return (self.cfg["hip_x"][leg] + hip_x_extra,
                hy + self.cfg["hip_sweep_dir"][leg] * hip_y_delta,
                kn + self.cfg["knee_flex_dir"][leg] * knee_delta)

    def _apply(self, leg: str, hip_x: float, hip_y: float, knee: float) -> None:
        for joint, value in (("hip_x", hip_x), ("hip_y", hip_y), ("knee", knee)):
            m = self.motors[(leg, joint)]
            if m is not None:
                m.setPosition(value)

    def _set_all(self, base: str = "stand", hip_y_delta: float = 0.0, knee_delta: float = 0.0,
                 hip_x_extra: float = 0.0) -> None:
        for leg in LEGS:
            self._apply(leg, *self._pose(leg, base, hip_y_delta, knee_delta, hip_x_extra))

    def _capture_settle_from(self) -> None:
        """Where the settle ramp starts: the config's `extend` pose, else the
        joints as the engine spawned them (they start at q=0 clamped into
        range whatever the URDF's rest pose says -- PROVENANCE.md)."""
        extend = self.cfg.get("extend")
        for leg in LEGS:
            for joint in ("hip_x", "hip_y", "knee"):
                if extend is not None:
                    hy, kn = extend[leg]
                    v = {"hip_x": self.cfg["hip_x"][leg], "hip_y": hy, "knee": kn}[joint]
                else:
                    s = self.sensors[(leg, joint)]
                    v = s.getValue() if s is not None else float("nan")
                    if not math.isfinite(v):
                        v = {"hip_x": self.cfg["hip_x"][leg], "hip_y": self.cfg["stand"][leg][0],
                             "knee": self.cfg["stand"][leg][1]}[joint]
                self.settle_from[(leg, joint)] = v

    def _set_wheels(self, velocity_radps: float) -> None:
        for w in self.wheels.values():
            w.setVelocity(velocity_radps)

    def _ensure_anchor(self) -> None:
        """Capture the floating-base anchor once, from the live body pose."""
        if self.body_anchor is not None:
            return
        pos0 = None
        if self.self_node is not None:
            try:
                pos0 = self.self_node.getPosition()
            except Exception:
                pos0 = None
        if pos0 is not None and len(pos0) >= 3:
            self.body_anchor = (float(pos0[0]), float(pos0[1]), float(pos0[2]))
        else:
            self.body_anchor = (0.0, 0.0, 0.85)

    def _write_body_pose(self, x: float, y: float, z: float) -> None:
        """Rewrite the floating-base translation + level orientation.

        Only the body link's pose is rewritten; the leg joints are NOT
        zeroed (that would need Node.resetPhysics), so position-controlled
        gaits/poses keep animating underneath.
        """
        if self.translation_field is None:
            return
        try:
            self.translation_field.setSFVec3f([x, y, z])
            if self.rotation_field is not None:
                self.rotation_field.setSFRotation([0.0, 0.0, 1.0, 0.0])
        except Exception:
            pass

    def _lock_body_upright(self, z_extra: float = 0.0) -> None:
        """Hold the body at its anchor, level — the same supervisor pose-lock
        _tick_walk uses, applied to the standing/idle poses too. Without it the
        stiff-legged stance has no balance feedback and topples under
        Newton/XPBD (it stood only because ODE's softer solver tolerated it)."""
        if self.self_node is None or self.translation_field is None:
            return
        self._ensure_anchor()
        ax, ay, az = self.body_anchor
        self._write_body_pose(ax, ay, az + z_extra)

    def tick(self, sim_t: float) -> None:
        with self.lock:
            kind, p = self.motion
            self.last_tick_at = time.time()

        if kind == "settle":
            # OmniQuad: hold its `extend` pose for a soft drop, then ramp.
            # Every other robot: ramp from the measured spawn pose at once.
            if not self.settle_from:
                self._capture_settle_from()
            hold = SETTLE_HOLD_S if self.cfg.get("extend") is not None else 0.0
            if sim_t - p["t0"] < hold:
                for leg in LEGS:
                    self._apply(leg, *(self.settle_from[(leg, j)] for j in ("hip_x", "hip_y", "knee")))
            else:
                with self.lock:
                    self.motion = ("crouch", {"t0": sim_t})
        elif kind == "crouch":
            # Smoothstep from the settle-from pose into the stand over settle_s.
            a = _smoothstep((sim_t - p["t0"]) / max(1e-6, float(self.cfg["settle_s"])))
            for leg in LEGS:
                target = self._pose(leg, "stand")
                start = tuple(self.settle_from[(leg, j)] for j in ("hip_x", "hip_y", "knee"))
                self._apply(leg, *(s + (t - s) * a for s, t in zip(start, target)))
            if a >= 1.0:
                with self.lock:
                    self.motion = ("stand", {"t0": sim_t})
        elif kind == "stand":
            sway = 0.04 * math.sin(2 * math.pi * (sim_t - p["t0"]) / 3.0)
            self._set_all("stand", hip_y_delta=sway)
        elif kind == "sit":
            self._set_all("sit")
        elif kind == "wave":
            t = sim_t - p["t0"]
            sway = 0.10 * math.sin(2 * math.pi * 0.8 * t)
            self._set_all("stand", hip_y_delta=sway, hip_x_extra=sway * 0.5)
            if t > p["duration_s"]:
                with self.lock:
                    self.motion = ("stand", {"t0": sim_t})
        elif kind == "walk":
            if self.cfg["walk"] == "wheels":
                self._tick_drive(sim_t, p)
            else:
                self._tick_walk(sim_t, p, translate=(self.cfg["walk"] == "gait"))
        elif kind == "stop":
            # Do nothing -- last setpoints stay applied.
            pass

        # Pin the floating base upright for every non-walk pose (walk runs its
        # own translation+rotation lock in _tick_walk). This is what keeps OmniQuad
        # standing under Newton/XPBD; see _lock_body_upright. Robots whose config
        # says body_lock False stand on their own physics.
        if kind != "walk" and self.cfg["body_lock"]:
            self._lock_body_upright()

    def _tick_drive(self, sim_t: float, p: Dict[str, Any]) -> None:
        """Wheeled-legged walk: hold the stance and roll the wheels. Real
        physics -- no supervisor writes."""
        self._set_all("stand")
        v_cmd = self.cfg["walk_velocity_ms"] * max(0.0, min(1.0, (sim_t - p["t0"]) / WALK_VELOCITY_RAMP_S))
        self._set_wheels(self.cfg.get("wheel_forward_sign", -1.0) * v_cmd / self.cfg["wheel_radius_m"])

    def _tick_walk(self, sim_t: float, p: Dict[str, Any], translate: bool = True) -> None:
        """Joint-space wave gait (+ supervisor-driven forward translation when
        `translate`).

        Each leg cycles realistically through stance/swing (animated by
        position-controlled motors). With `translate`, the body's X position
        is locked to a straight forward line from the walk-start anchor,
        advancing at the config's walk_velocity_ms (ramped from 0). Pure
        physics walking is not stable on the OmniQuad URDF; supervisor
        translation lets it visibly walk while keeping the gait visually
        realistic. Without it ("march") the legs cycle in place.
        """
        walk_t = sim_t - p["t0"]
        cycle_phase = (walk_t / WALK_PERIOD_S) % 1.0
        amp_scale = max(0.0, min(1.0, walk_t / (2.0 * WALK_PERIOD_S)))

        # Lateral CoM shift via hip_x delta during single-leg swing.
        com_shift_y = -COM_SHIFT_AMP_Y * math.cos(2.0 * math.pi * cycle_phase) * amp_scale

        for leg in LEGS:
            offset = (cycle_phase - LEG_PHASES[leg]) % 1.0
            if offset < SWING_FRACTION:
                # Swing: the foot travels from back (+A) to front (-A); knee flexes.
                pp = offset / SWING_FRACTION
                smooth = pp * pp * (3.0 - 2.0 * pp)
                hip_y_delta = HIP_SWEEP_AMP * (1.0 - 2.0 * smooth) * amp_scale
                knee_delta = SWING_KNEE_DELTA * math.sin(math.pi * pp) * amp_scale
            else:
                # Stance: the foot travels from front (-A) to back (+A); planted
                # foot -> body moves forward in world.
                pp = (offset - SWING_FRACTION) / (1.0 - SWING_FRACTION)
                hip_y_delta = HIP_SWEEP_AMP * (-1.0 + 2.0 * pp) * amp_scale
                knee_delta = 0.0
            self._apply(leg, *self._pose(leg, "stand", hip_y_delta, knee_delta, hip_x_extra=com_shift_y))

        if not translate:
            return
        # Advance the shared body anchor along +x by this tick's ramped
        # distance, then write it (level). Using the persistent anchor (rather
        # than a per-walk start offset) means a later 'stand' holds the
        # advanced position instead of snapping back to the spawn point.
        if self.self_node is not None and self.translation_field is not None:
            self._ensure_anchor()
            v_cmd = self.cfg["walk_velocity_ms"] * max(0.0, min(1.0, walk_t / WALK_VELOCITY_RAMP_S))
            dt = max(0.0, self.timestep / 1000.0)
            ax, ay, az = self.body_anchor
            ax += v_cmd * dt
            self.body_anchor = (ax, ay, az)
            body_bob = 0.015 * math.cos(4.0 * math.pi * cycle_phase) * amp_scale
            self._write_body_pose(ax, ay, az + body_bob)

    # ── Actions ──────────────────────────────────────────────────

    def act_stop(self) -> dict:
        with self.lock:
            self.motion = ("stop", {})
        self._set_wheels(0.0)
        return {"halted_at": time.time()}

    def act_stand(self) -> dict:
        with self.lock:
            self.motion = ("stand", {"t0": self.robot.getTime()})
        self._set_wheels(0.0)
        return {"accepted": True, "pose": "stand"}

    def act_sit(self) -> dict:
        with self.lock:
            self.motion = ("sit", {})
        self._set_wheels(0.0)
        return {"accepted": True, "pose": "sit"}

    def act_wave(self, duration_s: float = 6.0) -> dict:
        with self.lock:
            self.motion = ("wave", {"t0": self.robot.getTime(), "duration_s": duration_s})
        return {"accepted": True, "duration_s": duration_s}

    def act_walk(self) -> dict:
        with self.lock:
            self.motion = ("walk", {"t0": self.robot.getTime()})
        return {"accepted": True, "pose": "walk", "walk": self.cfg["walk"],
                "velocity_ms": self.cfg["walk_velocity_ms"]}

    def act_reset_to_home(self) -> dict:
        return self.act_stand()

    def get_state(self) -> dict:
        with self.lock:
            kind = self.motion[0]
        pos = None
        if self.self_node is not None:
            try:
                pos = [float(v) for v in self.self_node.getPosition()]
            except Exception:
                pos = None
        return {
            "id": self.robot_id,
            "model": self.model,
            "mode": kind,
            "position": pos,
            "last_tick_at": self.last_tick_at,
            "sim_time": self.robot.getTime(),
        }


# ── Intent ───────────────────────────────────────────────────────────

class IntentRouter:
    def __init__(self, bridge: QuadrupedBridge):
        self.bridge = bridge

    def dispatch(self, text: str) -> dict:
        s = text.strip().lower()
        if re.search(r"\b(stop|halt|freeze)\b", s):
            self.bridge.act_stop()
            return {"agent": "Holding position.", "tools": [("stop_robot", "ok", "frozen")]}
        if re.search(r"\b(sit|crouch|down|low)\b", s):
            self.bridge.act_sit()
            return {"agent": "Sitting down.", "tools": [("set_joint_positions", "ok", "sit pose")]}
        if re.search(r"\b(stand|up|home|reset)\b", s):
            self.bridge.act_stand()
            return {"agent": "Standing.", "tools": [("reset_to_home", "ok", "stand pose")]}
        if re.search(r"\b(wave|hello|dance|demo|show)\b", s):
            self.bridge.act_wave()
            return {"agent": "Waving — give me ~6 seconds.", "tools": [("wave", "ok", "0.8 Hz sway")]}
        if re.search(r"\b(walk|forward|move|go|drive|roll)\b", s):
            r = self.bridge.act_walk()
            v = r["velocity_ms"]
            verb = {"wheels": "Driving", "march": "Marching in place", "gait": "Walking"}[r["walk"]]
            msg = f"{verb} forward at {v:.2f} m/s." if v > 0 else f"{verb} (this robot's bridge does not translate the body)."
            return {"agent": msg, "tools": [("walk", "ok", f"{r['walk']} v={v} m/s")]}
        if re.search(r"\b(status|state|where|pose)\b", s):
            st = self.bridge.get_state()
            return {"agent": f"mode={st['mode']}", "tools": [("get_robot_state", "ok", st["mode"])]}
        return {
            "agent": "Try: \"stand\", \"sit\", \"wave hello\", \"stop\".",
            "tools": [],
        }


# ── HTTP ─────────────────────────────────────────────────────────────

def _json_finite(obj: Any) -> Any:
    """Recursively replace non-finite floats (NaN/Inf) with None.

    Sensor / clock reads can be NaN before the first robot.step()
    completes (the HTTP server is up BEFORE the main loop starts), and
    json.dumps happily emits bare `NaN` -- which is NOT valid JSON, so
    clients (jq, JSON.parse, json.loads) reject the whole body and the
    endpoint LOOKS empty/broken."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _json_finite(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_finite(v) for v in obj]
    return obj


def make_handler(bridge: QuadrupedBridge, router: IntentRouter, relay: Any = None):
    action_lock = threading.RLock()
    request_ids = RequestIdGuard()
    trusted_origins = allowed_origins()
    bridge_token = configured_token()

    class _H(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def _json(self, code, obj):
            # allow_nan=False guarantees strictly valid JSON on the wire;
            # the sanitizer maps any NaN/Inf field to null instead.
            try:
                data = json.dumps(obj, default=str, allow_nan=False).encode("utf-8")
            except ValueError:
                data = json.dumps(_json_finite(obj), default=str,
                                  allow_nan=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("X-OmniSim-Wire", WIRE_VERSION)
            self.send_header("X-OmniSim-Service", WIRE_SERVICE)
            origin = getattr(self, "_response_origin", None)
            if origin:
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _read_json(self):
            return read_json(self, allow_empty=True)

        def _guard(self) -> None:
            check_protocol_version(self.headers)
            self._response_origin = checked_origin(self.headers, trusted_origins)
            check_authorization(self.headers, bridge_token)

        def do_OPTIONS(self):
            try:
                self._response_origin = checked_origin(self.headers, trusted_origins)
                self.send_response(204)
                if self._response_origin:
                    self.send_header("Access-Control-Allow-Origin", self._response_origin)
                    self.send_header("Vary", "Origin")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header(
                    "Access-Control-Allow-Headers", "Content-Type, Authorization, X-OmniSim-Token"
                )
                self.end_headers()
            except RequestError as exc:
                self._json(exc.status, error_envelope(exc.code, exc.message, exc.details))

        # Top-level guards: an uncaught exception in a route used to
        # propagate into socketserver, which logs it server-side and closes
        # the connection with ZERO bytes sent -- the client sees an empty
        # reply (curl error 52) with no clue why. Every route now answers
        # with a JSON body: data, or a clear {"error": ...}.
        def do_GET(self):
            try:
                self._guard()
                self._route_get()
            except RequestError as e:
                self._json(e.status, error_envelope(e.code, e.message, e.details))
            except Exception as e:
                self._error_response(e)

        def do_POST(self):
            try:
                self._guard()
                path = self.path.split("?", 1)[0].rstrip("/") or "/"
                body = self._read_json()
                request_id = validate_request_id(body.pop("id", None))
                if path not in ("/state", "/get_robot_state", "/list_robots", "/capabilities"):
                    request_ids.claim(path, request_id)
                if path == "/stop_robot":
                    self._route_post(body)
                else:
                    with action_lock:
                        self._route_post(body)
            except RequestError as e:
                self._json(e.status, error_envelope(e.code, e.message, e.details))
            except Exception as e:
                self._error_response(e)

        def _error_response(self, e: Exception) -> None:
            import traceback
            print(f"[omnilink_quadruped_bridge] HTTP {self.command} {self.path} "
                  f"failed: {e!r}\n{traceback.format_exc()}")
            try:
                self._json(500, error_envelope("internal_error", "The bridge could not complete the request."))
            except Exception:
                pass  # headers already sent / socket gone -- nothing to add

        def _route_get(self):
            if self.path == "/protocol":
                return self._json(200, {
                    "ok": True, "omnisim_wire": WIRE_VERSION,
                    "service": WIRE_SERVICE,
                    "service_versions": {WIRE_SERVICE: WIRE_VERSION},
                    "instance": {"name": "omnilink_quadruped_bridge", "robot_id": bridge.robot_id},
                    "extensions": [],
                })
            if self.path in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if self.path in ("/capabilities", "/list_robots"):
                return self._json(200, [{
                    "id": bridge.robot_id, "model": bridge.model,
                    "capabilities": bridge.capabilities,
                }])
            if self.path == "/usage":
                if relay is None:
                    return self._json(200, {"enabled": False})
                return self._json(200, {
                    "enabled": True,
                    "latest": relay.latest_usage(),
                })
            return self._json(404, error_envelope("not_found", "Endpoint not found."))

        def _route_post(self, body):
            p = self.path.rstrip("/")
            if p in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if p in ("/list_robots", "/capabilities"):
                return self._json(200, [{
                    "id": bridge.robot_id, "model": bridge.model,
                    "capabilities": bridge.capabilities,
                }])
            if p == "/stop_robot":
                return self._json(200, bridge.act_stop())
            if p == "/reset_to_home":
                return self._json(200, bridge.act_reset_to_home())
            if p == "/prompt":
                text = nonempty_string(require_field(body, "text"), "text")
                if relay is not None:
                    return self._json(200, relay.dispatch_sync(text))
                result = router.dispatch(text)
                return self._json(200, {
                    "response": result["agent"],
                    "actions": [{"tool": t[0], "result": t[1], "summary": t[2]}
                                for t in result["tools"]],
                })
            if p == "/tool":
                # Platform-side tool callback. omnilink-agents.com web UI
                # POSTs {"tool": "<name>", ...args} after producing tool
                # calls on its side; dispatch via the relay-registered Tool.
                tool_name = nonempty_string(require_field(body, "tool"), "tool")
                body.pop("tool", None)
                if relay is None or tool_name not in getattr(relay, "tools", {}):
                    return self._json(503, {
                        "status": "err",
                        "tool": tool_name,
                        "error": "tool_not_registered",
                    })
                try:
                    result = relay.tools[tool_name].dispatch(body)
                    return self._json(200, {
                        "status": "ok",
                        "tool": tool_name,
                        "result": result,
                    })
                except Exception as e:
                    return self._json(500, {
                        "status": "err",
                        "tool": tool_name,
                        "error": "tool_execution_failed",
                    })
            return self._json(404, error_envelope("not_found", "Endpoint not found.", {"path": p}))
    return _H


def start_http(bridge: QuadrupedBridge, router: IntentRouter, port: int, relay: Any = None):
    server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(bridge, router, relay))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[omnilink_quadruped_bridge] HTTP on http://127.0.0.1:{port}")


def build_quadruped_tools(bridge: QuadrupedBridge) -> List[Any]:
    if Tool is None:
        return []
    return [
        Tool(name="stand", description="Hold a standing stance with gentle sway.",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_stand()),
        Tool(name="sit", description="Crouch / sit pose (knees bent further).",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_sit()),
        Tool(name="wave", description="Exaggerated sway for ~6 s -- 'hello' gesture.",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_wave()),
        Tool(name="walk", description=(
                {"gait": "Walk forward at %.2f m/s with a wave-gait leg cycle.",
                 "wheels": "Drive forward at %.2f m/s on the wheels, legs held in the stance.",
                 "march": "Cycle the legs through a walking gait in place (%.2f m/s of body motion)."}
                [bridge.cfg["walk"]] % bridge.cfg["walk_velocity_ms"]
                + " Continues until 'stand' or 'stop' is called."),
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_walk()),
        Tool(name="stop_robot", description="Freeze the legs at their current pose.",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_stop()),
        Tool(name="get_robot_state", description="Read current pose mode.",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.get_state()),
    ]


def setup_omnilink_relay(bridge: QuadrupedBridge, http_port: int = 8765) -> Optional[Any]:
    if OmniLinkRelay is None or not omnilink_enabled():
        return None
    try:
        agent_name = f"OmniSim-{bridge.robot_id}"
        tools = build_quadruped_tools(bridge)
        main_task = (
            f"You operate the {bridge.model} in OmniSim through the OmniLink bridge. "
            f"Available actions: stand, sit, wave, walk ({bridge.cfg['walk']}, "
            f"{bridge.cfg['walk_velocity_ms']:.2f} m/s), stop_robot. Translate operator "
            "requests into one tool call. Keep responses short."
        )
        relay = OmniLinkRelay(
            omni_key=get_omni_key(),
            agent_name=agent_name,
            main_task=main_task,
            tools=tools,
        )
        # Push a OmniQuad profile so operators can chat to it from the
        # omnilink-agents.com web UI; tool calls round-trip via /tool.
        from _omnilink_relay import profile_sync
        if profile_sync.is_enabled():
            profile_sync.ensure_profile(
                client=relay._client,
                agent_name=agent_name,
                main_task=main_task,
                tool_defs=[t.to_definition() for t in tools],
                engine=relay.engine,
                tool_callback_url=f"http://127.0.0.1:{http_port}/tool",
            )
        print(f"[omnilink_quadruped_bridge] OmniLink relay ON (agent='{agent_name}')")
        return relay
    except Exception as e:
        print(f"[omnilink_quadruped_bridge] OmniLink relay setup failed: {e}")
        return None


def push_configure(bridge: QuadrupedBridge, relay: Any) -> None:
    agent_label = (
        f"OmniLink relay ({_os.environ.get('OMNILINK_ENGINE', 'g4-engine')})"
        if relay is not None else "local intent (regex)"
    )
    cfg = {
        "robot": bridge.model,
        "robot_class": "quadruped",
        "agent": agent_label,
        "suggestions": ["stand", "sit", "wave hello",
                        "drive forward" if bridge.cfg["walk"] == "wheels" else "walk", "stop"],
    }
    bridge.queue_window("configure:" + json.dumps(cfg))
    bridge.queue_window("status:connected")
    bridge.window_configured = True


def _on_relay_event(bridge: QuadrupedBridge, kind: str, payload: Dict[str, Any]) -> None:
    if kind == "status":
        bridge.queue_window(f"status:{payload.get('state', 'idle')}")
    elif kind == "tool":
        bridge.queue_window(
            f"tool:{payload.get('name', '?')}:{payload.get('status', 'ok')}:{payload.get('summary', '')}")
    elif kind == "agent":
        bridge.queue_window("agent:" + str(payload.get("text", "")))
    elif kind == "usage":
        bridge.queue_window("usage:" + json.dumps(payload, default=str))
    elif kind == "audio_out":
        bridge.queue_window("audio_out:" + json.dumps(payload, default=str))
    elif kind == "error":
        bridge.queue_window("error:" + str(payload.get("text", "")))


def handle_wwi(bridge: QuadrupedBridge, router: IntentRouter, relay: Any, msg: str) -> None:
    if not msg:
        return
    if msg.startswith("configure"):
        push_configure(bridge, relay); return
    if msg.startswith("stop"):
        bridge.act_stop()
        bridge.queue_window("agent:Stop received.")
        bridge.queue_window("tool:stop_robot:ok:frozen")
        bridge.queue_window("status:idle"); return
    if msg.startswith("prompt:"):
        text = msg[len("prompt:"):]
        if relay is not None:
            relay.dispatch_async(text, lambda k, p: _on_relay_event(bridge, k, p))
            return
        bridge.queue_window("status:thinking")
        result = router.dispatch(text)
        for (t, st, sm) in result["tools"]:
            bridge.queue_window(f"tool:{t}:{st}:{sm}")
        bridge.queue_window("agent:" + result["agent"])
        bridge.queue_window("status:idle"); return
    if msg.startswith("audio_in:"):
        if relay is None:
            bridge.queue_window("error:audio_in requires OMNI_KEY (no relay attached)"); return
        import base64 as _b64
        payload = msg[len("audio_in:"):]
        try:
            info = json.loads(payload)
            audio = _b64.b64decode(info.get("audio_b64", ""))
            mime = info.get("mime_type", "audio/webm")
        except Exception as e:
            bridge.queue_window(f"error:audio_in decode failed: {e}"); return
        bridge.queue_window("status:transcribing")

        def _stt_worker():
            text = relay.transcribe(audio, mime_type=mime)
            if not text:
                bridge.queue_window("error:could not transcribe audio")
                bridge.queue_window("status:idle"); return
            bridge.queue_window("transcript:" + text)
            relay.dispatch_async(text, lambda k, p: _on_relay_event(bridge, k, p))

        threading.Thread(target=_stt_worker, name="omnilink-stt", daemon=True).start()
        return


def main() -> int:
    args = _parse_args()
    robot = Supervisor()
    bridge = QuadrupedBridge(robot, args.robot)
    router = IntentRouter(bridge)
    relay = setup_omnilink_relay(bridge, http_port=args.port)
    start_http(bridge, router, args.port, relay)
    print(f"[omnilink_quadruped_bridge] {bridge.model} ready ({'OmniLink' if relay else 'local'}).")

    timestep = bridge.timestep
    while robot.step(timestep) != -1:
        sim_t = robot.getTime()
        while True:
            msg = robot.wwiReceiveText()
            if msg is None or msg == "":
                break
            try:
                handle_wwi(bridge, router, relay, msg)
            except Exception as e:
                bridge.queue_window(f"error:bridge_exception: {e!r}")
        with bridge.lock:
            outbox = bridge.window_outbox
            bridge.window_outbox = []
        for line in outbox:
            try:
                robot.wwiSendText(line)
            except Exception:
                pass
        bridge.tick(sim_t)
    return 0


if __name__ == "__main__":
    sys.exit(main())
