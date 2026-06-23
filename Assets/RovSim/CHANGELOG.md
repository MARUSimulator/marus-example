# OceanScale ROV Sim — Changelog

Standalone Unity ROV/AUV pilot-training simulator (a fork of MARUSimulator/marus-example,
branch `rov-sim-v1`). Vehicle control only (no manipulator arm) for v1.

Stack: Unity 6000.0.42f1 LTS · HDRP 17.0.4 · org.mujoco 3.8.1 · free/native assets only.

## 1.0.0-mvp — 2026-06-23
First reproducible demo MVP.

### Vehicle & physics
- 6-DOF ROV driven by MuJoCo `xfrc_applied` force-integration (not teleport), shared
  `RovDynamics.ComputeWrench` used by both the player and the headless test.
- Anisotropic body-frame hydro drag (terminal surge ~1.02, sway ~0.30, heave ~0.33 m/s),
  passive self-righting, command low-pass smoothing, and an angular-rate governor
  (roll/pitch 0.2, yaw 0.8 rad/s) for smooth, stable motion (no fast-spin tumble).
- Headless oracle suite `RovSimTest.RunHeadless` — 20 physics + joystick oracles, ALL_PASS.

### Sensors, HUD & cameras
- Real-data dashboard: heading compass tape, artificial-horizon attitude indicator,
  depth/altitude gauge, bipolar thruster-command bars, forward-looking sonar scope.
- Multi-camera: 3rd-person chase (main) + onboard pilot FPV + oblique inspection cam (PiPs).
- Forward multibeam sonar (64 rays / 18 m), altimeter, telemetry CSV.

### Scene & tasks
- Two selectable environments via `ROVSIM_SCENE` (`harbor` turbid green / `deepwater` clear blue).
- Subsea scene: textured CC0 seabed (low relief), recognizable Gazebo Fuel ROV (CC-BY),
  wellhead, export pipeline + anchors, secondary manifold, native HDRP underwater water,
  volumetric fog, marine snow.
- Scripted teaching/pitch tour: intro → 6-DOF control → forward sonar + altimeter →
  onboard pilot camera → Task 1 Wellhead Inspection (autonomous mission) → Task 2 Pipeline
  Survey → Task 3 Sonar Grid Survey (lawnmower) → Task 4 Ascent & Recovery.

### Input
- Logitech gamepads F310 / F710 via Unity Input System (XInput); flight-stick path for
  HOTAS sticks; keyboard fallback (W/S A/D R/F Q/E). Pure mappings unit-tested.

### Reproduce
- `tools/reproduce_demo.sh` builds, runs the oracles, records and encodes the demo from the
  repo (Unity + ffmpeg + an X display required).

### Known limitations (honest)
- Visual is free-stack (independent review ~5–7/10); production photorealism needs licensed
  water shader + paid subsea assets.
- Two environments × four scripted tasks; full scenario library and Win/macOS packaging are future work.
- HOTAS throttle unit (e.g. X56) not yet bound — stick only.
