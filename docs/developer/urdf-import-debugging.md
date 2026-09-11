# URDF Import Debugging

This note is the fast path for debugging robot models loaded through the URDF importer.

The key constraint is that URDF and Webots do not describe every joint and collision detail in exactly the same way. A model can look "almost right" while still losing important information at import time. The importer now exposes that loss instead of hiding it.

## What To Check First

- Joint axes: URDF defines the joint axis in the joint frame, while Webots expects the axis in the parent solid frame. The importer rotates the axis through the joint origin's RPY before emission.
- Collision coverage: a link with one supported collision emits a single `boundingObject`; a link with multiple supported collisions emits a `boundingObject Group { children [...] }`.
- Mesh geometry: visual and collision meshes are imported when the path resolves (supports `package://`, `file://`, absolute, and relative paths). Unresolved meshes are skipped with a warning. Webots cannot apply a non-unit scale inside a `boundingObject`, so collision-mesh scale is dropped (visual-mesh scale is honoured via a `Transform` wrapper).
- Inertia fidelity: the URDF `<inertia ixx ixy ixz iyy iyz izz>` is passed through to Webots' `Physics.inertiaMatrix` when the tensor is positive definite. Non-PD tensors are rejected with a warning and the boundingObject-derived inertia is used instead. `centerOfMass` is always emitted alongside the matrix.
- Sensors: `<gazebo reference="LINK"><sensor>` blocks (modern Gazebo) are mapped to Webots devices — `imu` → `InertialUnit` + `Gyro` + `Accelerometer`, `gps` → `GPS`, `camera`/`depth` → `Camera`, `ray`/`gpu_ray`/`lidar` → `Lidar`. Legacy `<gazebo><plugin filename="libhector_gazebo_ros_imu.so">` / `libhector_gazebo_ros_gps.so` patterns are also recognized so unmodified Clearpath URDFs (Jackal, Husky) import with their full sensor suite.
- Joint damping and friction: imported when present and materially affect whether parts settle cleanly.

## Static Import Report

Run the developer importer with a JSON report before loading the model in the simulator:

```powershell
python scripts/dev/urdf_import.py path\to\robot.urdf --report urdf-report.json --strict > robot.wbt.snippet
```

The report includes:

- root-link selection
- per-link counts (`supported_visual_count`, `supported_collision_count`), `has_inertia_matrix` flag, and any unresolved mesh paths
- per-joint `axis_joint_frame` and `axis_parent_frame`
- imported limits, damping, and friction
- per-sensor kind, target link, and update rate

`--strict` exits non-zero when the report contains warnings. That makes it suitable for agent loops and CI gates. The Jackal URDF (`projects/robots/clearpath/jackal_description/urdf/jackal.urdf`) is the reference URDF that exercises all four import paths — primitive collisions, mesh visuals, full inertia tensors, and legacy plugin-form sensors — and should always pass `--strict`.

## Runtime Joint Logging

When loading through native `URDFRobot { ... }` expansion, enable importer logging:

```powershell
$env:OMNISIM_URDF_DEBUG = "1"
%OMNISIM_HOME%\msys64\mingw64\bin\omnisim-bin.exe path\to\world.omniworld --stdout --stderr
```

The parser logs:

- robot root-link selection
- per-joint origin xyz/rpy
- raw axis from the URDF joint frame
- transformed axis emitted to the Webots joint
- imported limits, damping, friction, and max velocity when present

Search the log for `URDF_DEBUG`.

## Recommended Agent Workflow

1. Generate the JSON report with `scripts/dev/urdf_import.py --report --strict`.
2. Resolve report warnings that change physics behavior first: missing collision primitives, missing limits, missing damping, and unsupported geometry.
3. Load the model through `URDFRobot { ... }` with `OMNISIM_URDF_DEBUG=1`.
4. Compare each suspicious joint's `axis_joint_frame` and `axis_parent_frame`. If the joint origin has rotation, they should usually differ.
5. If the model still does not settle, inspect the imported collision primitives and inertial blocks before adjusting controller code.

## Current Limits

- `<transmission>` blocks are ignored — motors come from the joint type. Mechanical reduction must be applied in the controller.
- ROS `xacro` is not expanded as a full template language. However, a common partial-expansion pattern — URDFs that ship with literal `${variable}` placeholders that were meant to be substituted by an outer xacro include — is handled by stripping any unexpanded `${...}` pattern at parse time, with a one-time warning. TurtleBot3's `turtlebot3_burger.urdf`, `turtlebot3_waffle.urdf`, and `turtlebot3_waffle_pi.urdf` rely on this: their joint and link names literally contain `${namespace}` because they were designed for inclusion under a namespaced xacro parent.
- Individual COLLADA (`.dae`) meshes may crash the mesh loader at world load. Most DAE files load fine (Husky uses many), but a SketchUp-authored file whose submeshes carry vertices and no normals can take the loader down. Strip the offending `<visual>` block from the URDF if you hit this. ⚠️ **The worked example this bullet used to give is gone**: TurtleBot3 waffle's `meshes/sensors/r200.dae` was removed on 2026-08-24 for licence reasons (it modelled an **Intel** RealSense R200, and ROBOTIS' Apache-2.0 cannot convey rights in Intel's design), so there is no longer a reproducer in the tree. The engine-side guard it prompted stays — `OmMesh.cpp` null-checks `mesh->mNormals` and skips such submeshes with a warning — and it was never specific to that file.
- Collision-mesh scale (`<mesh scale="...">` inside `<collision>`) is dropped because Webots' `boundingObject` does not honour `Transform.scale`. Visual-mesh scale is honoured.
- Sensor noise / drift parameters from `<gazebo><plugin>` blocks are not propagated — only the device type and the link it attaches to. Tune noise on the Webots side once imported if it matters for your demo.
- Multi-shape collision is supported via `Group`, but Webots auto-computes inertia from the *first* shape only unless an explicit `Physics.inertiaMatrix` is supplied.

## Inertia import (ON by default since 2026-09-10)

### `OMNISIM_URDF_USE_INERTIA` — emit `Physics.inertiaMatrix` from URDF `<inertia>` tensors **(default ON)**

What it does: passes `<inertia ixx ixy ixz iyy iyz izz>` straight into `Physics.inertiaMatrix` (2-line MFVector3), together with the `<inertial><origin>` as `Physics.centerOfMass`, for any link with a physically admissible tensor.

⚠️ **This used to be OFF by default, and the consequence was that a URDF's `<inertia>` NEVER reached the solver.** Measured 2026-09-10 on a two-link cart-pole: a pole declaring `iyy = 0.0666667` about its COM was integrated at `0.00334` — the Newton runtime's Husky-tuned mass preset `m*(0.0094, 0.0167, 0.0094)` — and *multiplying the URDF tensor by 4.5× produced a byte-identical `mjModel`*. The tensor was not merely ignored; it was masked by a number with no relation to the robot. The OFF default was an ODE-era workaround for a `dMassSetParameters` crash, and ODE was deleted on 2026-08-08 (`bdc02139`), so the reason had already outlived the code. **`OMNISIM_URDF_USE_INERTIA=0` restores the bounding-object-derived import** (value-parsed) for a bisect.

Two guards remain, and both now WARN instead of dropping a tensor silently:

- **Small-tensor clamp, re-derived at `1e-9`** (it was `1e-4`). The old value was the ODE crash threshold; MuJoCo's own floor is `mjMINVAL` (1e-15), so `1e-4` was a second silent mask — a 100 g gripper finger's real tensor is ~1e-5 and was being thrown away. `1e-9` is a numerical-sanity floor, not a crash guard.
- **Triangle inequality on the principal moments (`a + b >= c`)**, mirroring `principal_moments()` in `scripts/dev/urdf_import.py`. A tensor can be positive definite and still describe no rigid body; MuJoCo's compiler REJECTS such a body, so — now that the tensor actually reaches the solver — an unchecked one would turn a sloppy URDF into a hard load failure. Such links fall back to bounding-object inertia with a named warning.

Where the tensor lands is reported once per load: `[OmNewtonBackend] inertia provenance: N declared, M from geometry, K from the mass preset`. A non-zero `preset` count names bodies whose rotational inertia no line of the world declares and no geometry implies.

Verified end-to-end: `python -m omnisim run-headless projects/samples/demos/worlds/physics/newton_husky_smoke_test.omniworld --duration 15` drives the Husky 12.53 m against 12.51 m with the tensors reverted (0.17%).

## Opt-in features

### `OMNISIM_URDF_USE_SENSORS=1` — emit OmniSim devices from `<gazebo><sensor>` and `<plugin>` blocks **(works for short runs; long-run crash is in the engine, not the importer)**

What it does: maps `<gazebo reference="LINK"><sensor type="imu/gps/camera/ray">` and legacy `<plugin filename="libhector_gazebo_ros_*.so">` patterns into `InertialUnit`/`Gyro`/`Accelerometer`/`GPS`/`Camera`/`Lidar` nodes. Devices are wrapped in a per-cluster carrier `Solid` (the pattern from `projects/samples/devices/worlds/imu.omniworld`) which is emitted inline as a child of the sensor's URDF link. Carrier has a tiny `Box` bounding object + `Physics`; SolidMerger absorbs it into the parent body.

What works:

- Static `urdf_import.py --report` lists all sensors with their kind / link / update_rate.
- The runtime importer emits the carrier Solids cleanly.
- World load + first ~5 seconds of fast-mode simulation pass; controllers start; sensors are registered and queryable.

The delayed-crash isolation finding (this is the key conclusion):

- Around 5-10 seconds into a longer run, OmniSim crashes with `ACCESS_VIOLATION` (0xC0000005) regardless of what URDF you import or how the carrier is emitted.
- **The stock `projects/samples/devices/worlds/imu.omniworld` sample world — Cyberbotics' own hand-authored example with `Accelerometer`/`Gyro`/`Compass`/`InertialUnit` siblings inside a `Solid` — crashes at the same ~5-8 second mark with the same exit code**, both in `--mode=fast` and `--mode=realtime`. Reproduce with `python scripts/dev/headless_runner.py projects/samples/devices/worlds/imu.omniworld --duration 25`.
- Bisects ruled out as causes: URDFRobot expansion, override injection, carrier mass/size, in-link vs root emission, controller startup, floor contact (suspending the robot 5 m in the air doesn't help), specific sensor type (IMU-only or GPS-only both reproduce).


What's safe to do today: use the sensor gate for short integration runs (`--duration 3-5`), to verify the imported devices register with the expected names + transforms before a controller pipeline goes upstream. Avoid long-running simulations until the bug is patched.

### Further isolation (2026-05-18 session)

Even tighter minimal repro — no URDF, no Physics, no controller:

```
Robot { children [ Accelerometer {} ] controller "<none>" }
```

Crashes with the same exit code at `t ≈ 2s` wall in `--mode=fast --no-rendering`. The crash time tracks **step count**, not wall clock — simpler worlds run more steps per wall second and hit the crash faster. The same world in larger scenes (e.g. `accelerometer.wbt` with its full geometry) crashes later in wall time because each step costs more, but the crash step count is comparable.

Additional negatives ruled out this session:

- `OMNISIM_WITH_NEWTON=OFF` rebuild still crashes. **Newton is not the cause** — earlier suspicion that the postPhysicsStep parent-walk for Newton-backed-ancestor detection was dereferencing stale pointers turned out to be wrong; short-circuiting that walk delayed the crash (per-step cost dropped) but did not eliminate it.
- Plain `Robot { children [ Solid {} ] }` runs 10s+ clean. **The bug is specifically in the OmSolidDevice path**, not in nested-Solid handling.
- `PositionSensor` (extends `OmJointDevice`) and `Motor`/`LinearMotor` worlds run clean. The crash is exclusive to subclasses of `OmSolidDevice` (Accelerometer, Camera, GPS, IMU, Gyro, Compass, TouchSensor, Lidar, Display, Connector, Emitter, Receiver, LED, Speaker, Pen, Radar, RangeFinder, VacuumGripper, etc.). *(This list also named `Radio` when it was written; the `Radio` node has since been retired — there is no `Radio.wrl` and no `OmRadio` class, so it is not a device you can author.)*
- Removing `Background {}` doesn't help. `--no-rendering` doesn't help.
- `controller "<none>"` and `controller "<extern>"` both crash with the same signature, so it isn't in the controller-comms dispatch path.

Where to look next:

- Code that runs per-step for `OmSolidDevice` but **not** for plain `OmSolid` or `OmJointDevice`. The most likely suspects are in `OmRobot::dispatchAnswer` / `writeAnswer` (foreach device → `d->writeAnswer(stream)` runs even when controller is `<none>` since the robot is still serializing state), the physics-object registration done at `OmSolid::createOdeObjects` for a device-Solid that has no physics body (⚠ 2026-08-08: that hook still exists and still runs — it kept its legacy name — but the **ODE space/geom registration it used to perform is gone with the backend (`bdc02139`) and is now a no-op stub**, so the live suspect is what replaced it: the solid-merger + bounding-object walk it still drives, and the Newton registration performed at world build / `finalizeWorld()` in [`OmNewtonBackend`](../../src/omnisim/physics/OmNewtonBackend.cpp). Re-run the minimal repro before trusting any of this suspect list — it was assembled against an engine with two backends), and any sensor-update side path triggered by `OmSimulationCluster::step` for the dual-inheritance OmDevice+OmSolid intersection.
- A debug-symbols OmniSim build + Windows debugger (windbg / Visual Studio) is the fastest next move — the crash is reproducible in seconds with the minimal world above, so a single attach-and-step session should pinpoint the faulting frame.

A device-smoke harness for re-validating after a fix is already in place: `python scripts/dev/device_smoke.py` walks every world in `projects/samples/devices/worlds/` headlessly and records PASS/FAIL/TIMEOUT per world to `device_smoke_results.json`. Pre-fix baseline: 8/45 PASS (motor/brake/encoders/supervisor — none of them exercise the OmSolidDevice path).

### How the gates behave

Both gates default off. Setting the env var to anything other than `0`, `false`, or `off` enables the feature. The `urdf_import.py --report` Python tool always reports what the C++ importer *would* emit if the gates were on, so reports remain useful for planning. The `OMNISIM_URDF_DEBUG=1` runtime log indicates which gates are active by emitting the `inertiaMatrix [...]` and sensor device lines (or omitting them).

## Headless launch (no visible OmniSim window)

OmniSim requires an OpenGL context even with `--no-rendering`, so `QT_QPA_PLATFORM=offscreen` triggers a fatal "could not initialize the rendering system" error. The working pattern is to launch the window normally then hide it via Win32 `ShowWindow(SW_HIDE)`:

```powershell
Add-Type @"using System; using System.Runtime.InteropServices; public class WinHide { [DllImport(\"user32.dll\")] public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow); public const int SW_HIDE = 0; }"@ -PassThru | Out-Null
$proc = Start-Process -FilePath "msys64\mingw64\bin\omnisim-bin.exe" -ArgumentList "<world.omniworld>","--mode=fast","--batch","--minimize" -PassThru
for ($i = 0; $i -lt 10; $i++) {
  Start-Sleep -Milliseconds 500
  $bin = Get-Process -Name omnisim-bin -ErrorAction SilentlyContinue
  if ($bin -and $bin.MainWindowHandle -ne [IntPtr]::Zero) {
    [WinHide]::ShowWindow($bin.MainWindowHandle, [WinHide]::SW_HIDE) | Out-Null
    break
  }
}
```

Use this for any CI / diagnostic loop where the GUI window should not appear.

## Fixed: TurtleBot3 chassis didn't translate

This was the "wheels spin at commanded speed, chassis stays at exactly (0,0,0)" bug. Bisected to: when the URDF root is a pure-frame link with no visuals / collisions / inertial (e.g. `base_footprint` in all three TB3 URDFs), the importer would synthesize a 0.001 kg Physics + 1 mm sphere bounding object on the Robot's root. That synthetic body's tiny ODE inertia tensor (~1e-10) appeared to leave the merged root body unable to receive friction-induced acceleration from the wheels.

**Fix** (in `emitRobot`): when the URDF root link is empty (no visuals, no collisions, no inertial) AND has a single fixed-joint outgoing edge, walk past it and promote the child link to be the Robot's root. Any fixed-joint offset is dropped — the new root sits at the original Robot's `translation`. The synthetic-Physics mass and bounding-sphere size were also bumped from 0.001 / 0.001 m to 0.1 / 0.01 m for cases where the empty-root rewrite doesn't apply, as belt-and-braces against the small-body ODE issue.

Verified: `python scripts/dev/headless_runner.py projects/samples/demos/worlds/showcase/turtlebot3_drive.omniworld --duration 30` drives all three TB3 variants — burger 11+ m, waffle 6+ m, waffle_pi 4+ m, no crashes.

The diagnostic harness (`tb3_drive_straight` controller — `turtlebot3_fall_test.wbt` was removed when it stopped earning its keep; reconstruct a minimal one if a "robot loaded but won't drive" regression returns) commands both wheels at a fixed velocity and logs commanded vs measured wheel rotation vs chassis position to `C:\tmp\husky_trace\<name>_straight.log`.

## Running the Jackal demo on Windows

Python controllers (including `husky_random` which the Jackal demo uses) require `python.exe` to be reachable on PATH. On most Windows boxes the only `python` on PATH is the Microsoft Store stub at `%LOCALAPPDATA%\Microsoft\WindowsApps\python.exe`, which prints "install from Store" and exits. Symptom: the world loads, the robot stays still, and `omnisim_log.txt` contains `WARNING: husky_random: failed to start:`. To fix, prepend a real Python install to PATH before launching OmniSim:

```powershell
$env:PATH = "C:\Users\<you>\AppData\Local\Programs\Python\Python314;$env:PATH"
msys64\mingw64\bin\omnisim-bin.exe projects\samples\demos\worlds\showcase\jackal_drive.omniworld --mode=realtime
```

Per-controller fix (alternative): edit `projects/default/controllers/husky_random/runtime.ini` to set `COMMAND = "C:/path/to/python.exe"` instead of the bare `python`.
