# OceanScale ROV Sim — Gap Analysis vs PaleBlue + Build Plan

Date: 2026-06-25. Base: `marus-example` branch `rov-sim-v1` (Unity 6000.0.42f1 + HDRP + org.mujoco 3.8.1).
Reference: PaleBlue product spec (`46-marine/artifacts/reports/paleblue-product-reproduction-spec-2026-06-25.html`).
Verify gate for every phase: `RovSimTest.RunHeadless` (headless MuJoCo oracles, ALL_PASS) + commit.

## Current state (verified, v1.0.0-mvp)
- 6-DOF vehicle on MuJoCo `xfrc_applied` (force-integrated, not teleport); anisotropic Fossen drag; angular-rate governor.
- Forward sonar (64-beam FLS) + altimeter; HUD (compass/attitude/depth/thruster bars); 3 cameras (chase + onboard FPV + inspection PiP).
- 1 piloted guided mission (Descend→Transit→Hold→Complete) + 4-task scripted autonomous tour; scoring; CSV telemetry.
- Headless oracle: **20/20 ALL_PASS**. Two environment variants. Joystick + keyboard input.

## Gap table — PaleBlue ROV Trainer/Subsea-Access feature → our status → phase
| PaleBlue feature | Our status | Action | Phase |
|---|---|---|---|
| **Manipulator arm (full physics, multiple types)** | **NONE (vehicle only)** | build articulated arm + jaw + control + manip mission | **P1** |
| Mission library (Salvage, Pipeline Survey, Flying Lead, Suction Pile, Torque Tool, Cathodic Protection, Water Jet…) | 1 piloted + 4 autonomous | build a **piloted, scored mission library** (data-driven) | **P2** |
| Tether + TMS (physically-based, entangles) | NONE | build tether catenary/drag + TMS payout | P3 |
| Instructor Seat + Fault Insertion + Recording/replay | scoring + CSV + frame capture | build instructor mode + fault injection + session replay | P4 |
| Subsea current + heave + visibility (instructor-tunable) | drag + depth only | environment-force layer (current/heave) + visibility presets | P5 |
| Control matrix (gamepad/joystick/HOTAS/master-arm/pedal) | joystick + keyboard | device-mapping layer + master-arm path | P6 |
| Work-class vs Inspection-class ROV | 1 generic vehicle | vehicle-class variants (MJCF + params) | P6 |
| Collision envelope (color-coded breach) — Subsea Access | NONE | per-part collision envelope + breach viz | P5 |
| Multi-camera split-screen | onboard+inspection+chase PiP | ✅ HAVE | — |
| **Forward sonar / altimeter** | ✅ 64-beam FLS + altimeter | ✅ **we EXCEED** (PaleBlue ROV Trainer has no sonar) | — |
| IMCA Class A accreditation | none | business/cert (long-horizon) | — |
| Multiplayer team training (VR Connect) | none | later (out of MVP scope) | P7 |

## Structural advantages to keep (don't lose these)
- Real MuJoCo physics oracle gate (CI-checkable) — PaleBlue ships +0% placeholder KPIs; we assert real numbers.
- Sonar/altimeter sensing already exceeds PaleBlue's ROV Trainer.
- Apache-2.0 free stack; headless-reproducible (`tools/reproduce_demo.sh`).

## Build phases (each: edit MJCF/C# → `RovSimTest.RunHeadless` ALL_PASS → commit)
- **P1 Manipulator arm** ← *building now*. `staging/rov_v2.xml` (4-DOF arm + jaw, position actuators); `RovManipulator.cs` (input→ctrl, telemetry, end-effector); HUD arm panel; a grab/operate manipulation mission; oracle: arm reaches pose, jaw open/close, EE moves, joint limits respected, no-NaN with arm.
- **P2 Mission library** — data-driven piloted missions (waypoints + task objectives + scoring rubric) matching PaleBlue's named taxonomy; mission-select; per-mission oracles.
- **P3 Tether + TMS** — cable model (segmented or catenary drag) anchored to TMS; payout; entanglement/contact; oracle: tether tension grows with distance, payout bounded.
- **P4 Instructor + fault insertion + replay** — instructor console (env/fault control), fault injection (thruster loss, current gust, sensor dropout), CSV→replay; oracle: injected thruster-fault reduces that axis' authority; replay reproduces trajectory.
- **P5 Environment + collision envelope** — current/heave forces (Fossen current in drag), visibility presets, collision-envelope breach detection; oracles.
- **P6 Control matrix + vehicle classes** — device mapping (HOTAS/master-arm), work/inspection variants.
- **P7 (later)** multiplayer, course authoring.

## Notes / constraints
- Disk `/mnt/storage` ~95% — keep build artifacts lean; reuse one Builds/ dir.
- Unity↔MuJoCo frame trap: all control stays in the MuJoCo frame (see RovPilotController homing).
- Arm appended AFTER base in MJCF so base body id (1) + base dof adr (0) stay valid → existing oracles unaffected.
