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

"""A URDF <rest> pose on a joint whose range reaches past +/-pi is read correctly.

The defect this pins (2026-10-07, found on the UFACTORY xArm lane 2026-10-04):
the importer writes a joint's child link ALREADY POSED at its initial angle
(the <rest> value, or the midpoint of a range that excludes 0), and must then
tell the joint it starts there (HingeJointParameters `position`). That line was
written only inside the branch for ranges that fit in (-pi, pi). A range that
reaches past +/-pi at one end (xArm joint3 -3.927..0.192, joint5
-1.693..3.14159) took the other branch, so the link was posed at the rest angle
while the joint believed it was at 0: the PositionSensor read 0 at the rest
pose, every setPosition() landed `rest` radians off, and the xArm controller
drove joint3 to a physical -245.6 deg, past the URDF's own -225 deg stop.

Probe: three horizontal fingers on a fixed base, each with <rest>, one with a
range inside +/-pi (the control), one past -pi, one past +pi. Each finger
carries a tip link 0.2 m out, whose world position gives the PHYSICAL angle.
The controller reads the sensors on the first step (they must equal <rest>),
then commands rest + 0.3 and checks sensor and physical angle agree.

    python -m pytest tests/test_urdf_rest_wide_range.py -v

OMNISIM_BINARY selects a scratch build. One engine launch, a few seconds.
"""

from __future__ import annotations

import math
import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)
TOL = 0.02
STEP = 0.3

# name: (lower, upper, rest, y offset of the finger on the base)
JOINTS = {
    "inside": (-1.5, 1.5, 0.7, 0.15),
    "past_lo": (-3.927, 0.192, -1.2, 0.0),
    "past_hi": (-1.693, 3.14159, 1.0, -0.15),
}

_I = '<inertia ixx=".001" iyy=".001" izz=".001" ixy="0" ixz="0" iyz="0"/>'


def _urdf():
    parts = ['<?xml version="1.0"?>', '<robot name="rest_probe">',
             f'<link name="base"><inertial><mass value="2"/>{_I}</inertial></link>']
    for n, (lo, hi, rest, y) in JOINTS.items():
        parts += [
            f'<link name="{n}_finger"><inertial><origin xyz=".1 0 0"/><mass value=".2"/>{_I}</inertial></link>',
            f'<link name="{n}_tip"><inertial><mass value=".01"/>{_I}</inertial></link>',
            f'<joint name="{n}" type="revolute"><parent link="base"/><child link="{n}_finger"/>'
            f'<origin xyz="0 {y} 0"/><axis xyz="0 1 0"/><limit lower="{lo}" upper="{hi}" effort="50" '
            f'velocity="3"/><rest>{rest}</rest></joint>',
            f'<joint name="{n}_tip_fix" type="fixed"><parent link="{n}_finger"/><child link="{n}_tip"/>'
            f'<origin xyz=".2 0 0"/></joint>',
        ]
    parts.append("</robot>")
    return "\n".join(parts)


WORLD = """#VRML_SIM R2025a utf8
WorldInfo { basicTimeStep 4 newtonSolver "mujoco" }
Viewpoint { orientation 0 0 1 0 position -2 0 1 }
URDFRobot { url "rest.urdf" translation 0 0 1 staticBase TRUE supervisor TRUE controller "rest_probe" }
"""

CONTROLLER = """import os
from omnisim import Supervisor
NAMES = %r
STEP = %r
r = Supervisor(); dt = int(r.getBasicTimeStep())
motors = {n: r.getDevice(n + "_motor") for n in NAMES}
sensors = {n: r.getDevice(n + "_sensor") for n in NAMES}
for s in sensors.values():
    s.enable(dt)
r.step(dt)
first = {n: sensors[n].getValue() for n in NAMES}
for n in NAMES:
    motors[n].setPosition(first[n] + STEP)
for _ in range(375):
    r.step(dt)
with open(os.environ["PROBE_OUT"], "w") as f:
    for n in NAMES:
        f.write("%%s %%.9g %%.9g %%.9g\\n" %% (n, first[n], first[n] + STEP, sensors[n].getValue()))
r.simulationQuit(0)
""" % (list(JOINTS), STEP)


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(
    _binary() is None, reason="no simulator binary in this clone; build first")


@pytest.fixture(scope="module")
def run(tmp_path_factory):
    root = tmp_path_factory.mktemp("rest_wide")
    worlds, ctrl = root / "worlds", root / "controllers" / "rest_probe"
    worlds.mkdir(parents=True)
    ctrl.mkdir(parents=True)
    (worlds / "rest.urdf").write_text(_urdf(), encoding="utf-8")
    (worlds / "rest.omniworld").write_text(WORLD, encoding="utf-8")
    (ctrl / "rest_probe.py").write_text(CONTROLLER, encoding="utf-8")
    out, log, traj = root / "out.txt", root / "engine.log", root / "traj.tsv"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log), PROBE_OUT=str(out),
               OMNISIM_PROBE_TRAJ=str(traj), OMNISIM_PROBE_TRAJ_ALL="1", OMNISIM_PROBE_TRAJ_MS="3000")
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        str(worlds / "rest.omniworld")], env=env, timeout=240, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    if not out.is_file():
        for sig in _BRINGUP_SIGNATURES:
            if sig in text:
                pytest.skip("Newton did not come up (%r); the run produced no data" % sig)
        pytest.fail("the rest probe produced no output:\n%s" % text[-1500:])
    readings = {}
    for line in out.read_text(encoding="utf-8").splitlines():
        n, first, target, final = line.split()
        readings[n] = (float(first), float(target), float(final))
    # Physical angle of each finger from its tip's last recorded world position
    # (joint axis +y: a finger at angle q puts its tip at x = 0.2 cos q, z = -0.2 sin q
    # relative to the joint).
    last = {}
    for row in traj.read_text(encoding="utf-8").splitlines() if traj.is_file() else []:
        r = row.split("\t")
        if len(r) == 6:
            last[r[1]] = (float(r[2]), float(r[4]))
    physical = {}
    for n in JOINTS:
        if n + "_tip" in last:
            x, z = last[n + "_tip"]
            physical[n] = math.atan2(-(z - 1.0), x)
    return readings, physical, text


@pytest.mark.parametrize("name", list(JOINTS))
def test_sensor_reads_rest_on_the_first_step(run, name):
    readings, _, _ = run
    rest = JOINTS[name][2]
    assert abs(readings[name][0] - rest) < TOL, (
        "%s: the PositionSensor must read the <rest> pose (%.3f) the link was spawned at; got %.4f"
        % (name, rest, readings[name][0]))


@pytest.mark.parametrize("name", list(JOINTS))
def test_sensor_and_physical_angle_agree(run, name):
    readings, physical, log = run
    _, target, final = readings[name]
    assert abs(final - target) < TOL, "%s: the joint must reach its command %.3f; sensor %.4f" % (
        name, target, final)
    assert name in physical, "no trajectory for %s_tip:\n%s" % (name, log[-1500:])
    assert abs(physical[name] - target) < TOL, (
        "%s: commanded %.3f and the sensor agrees, but the link is physically at %.4f rad"
        % (name, target, physical[name]))
