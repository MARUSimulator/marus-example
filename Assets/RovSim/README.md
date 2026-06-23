# OceanScale ROV Sim (v1)

A standalone Unity ROV/AUV **pilot-training** simulator — vehicle control only (no arm in v1).
Forked from [MARUSimulator/marus-example](https://github.com/MARUSimulator/marus-example)
(`rov-sim-v1` branch). Real MuJoCo physics, real-data dashboard, multi-camera, scripted
teaching/pitch tour. See [`CHANGELOG.md`](CHANGELOG.md) · version in [`VERSION`](VERSION).

## Layout (this is all in-repo)
```
Assets/RovSim/            controller, dynamics, input, cameras, HUD, headless test
Assets/RovSim/Editor/     RovSimBuild (scene+player build) · RovSimTest (oracles)
Assets/RovSim/Models/     Gazebo Fuel ROV (CC-BY) + CC0 seabed/rock textures
staging/rov_v1.xml        vehicle-only MJCF (MuJoCo, z-up, neutral buoyancy)
tools/reproduce_demo.sh   one-command reproduce: build → oracle → record → encode
Builds/ , Logs/           generated outputs (git-ignored)
```

## Reproduce the demo
Requirements: Unity **6000.0.42f1** (HDRP + org.mujoco packages), `ffmpeg`, and an X display
for the recording step.
```bash
UNITY=/path/to/Unity/6000.0.42f1/Editor/Unity  DISPLAY_ID=:1  tools/reproduce_demo.sh
# environment variant: prefix with  ROVSIM_SCENE=deepwater  (default: harbor)
```
Produces `Builds/OceanScale_ROV_Sim_demo.mp4` (64 s, two tasks) and the telemetry/logs.

## Run / pilot interactively
Build once (`tools/reproduce_demo.sh` or `-executeMethod RovSimBuild.BuildStandalone`), then:
```bash
DISPLAY=:1 Builds/rov_sim_v1.x86_64 -screen-width 1280 -screen-height 720
```
Controls: **W/S** surge · **A/D** sway · **R/F** heave · **Q/E** yaw, or a Logitech gamepad
(F310/F710) / flight stick.

## Verify physics (headless, no display)
```bash
$UNITY -batchmode -nographics -quit -projectPath . -executeMethod RovSimTest.RunHeadless -logFile Logs/oracle.log
grep ROVTEST_RESULT Logs/oracle.log     # -> ALL_PASS 20/20
```

## Honesty
Visual quality is the free/native-stack ceiling (independent review ~5–7/10). Production
photorealism needs a licensed water shader + paid subsea assets. Provenance: MARUS = Apache-2.0,
Fuel ROV = CC-BY 4.0, textures = CC0; no paid/pirated asset is used or committed.
