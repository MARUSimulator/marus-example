# Engine setup — OceanScale ROV Sim (Unity)

This is a standalone Unity project (a fork of `MARUSimulator/marus-example`, Apache-2.0) that
OceanScale tracks **as a git submodule** under the monorepo `sims/`. The Unity Editor and the
regenerable caches/builds are NOT in the repo — only source + scenes + configs + (for now) the
content textures/meshes are versioned.

## Engine (NOT committed — install separately, document version)
- **Unity 6000.0.42f1 LTS** (install via Unity Hub or use an equivalent offline editor).
- Render: **HDRP 17.0.4** · Physics: **org.mujoco 3.8.1** — both pinned in `Packages/manifest.json`
  + `Packages/packages-lock.json` (versioned). `git clone` + open in the editor restores them.

## What's in the repo vs not (3-tier policy)
| Tier | Paths | In repo? |
|---|---|---|
| Source | `Assets/RovSim/**`, `staging/*.xml`, `ProjectSettings/`, `Packages/manifest*.json`, `docs/`, `tools/` | ✅ yes |
| Regenerable (gitignored) | `Library/` (2.7G), `Builds/` (7.3G), `Temp/`, `Logs/`, `obj/` | ❌ never |
| Large binaries | content textures/meshes (~150M, currently in-repo for self-containment); future multi-GB renders/datasets/demo-video archives | → **HuggingFace** + fetch script |

## Build / verify (headless, no display)
```bash
UNITY=/path/to/Unity/Editor/Unity
# physics + manipulator oracles (CI gate, ALL_PASS 25/25):
"$UNITY" -batchmode -nographics -quit -projectPath . -executeMethod RovSimTest.RunHeadless -logFile Logs/oracle.log
# full standalone build + the demo video:
UNITY=$UNITY DISPLAY_ID=:1 tools/reproduce_demo.sh
```
