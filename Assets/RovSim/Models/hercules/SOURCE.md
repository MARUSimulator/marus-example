# Hercules ROV — asset provenance

- **Model:** "ROV Hercules" — EV Nautilus (E/V Nautilus, Ocean Exploration Trust).
- **Source:** Sketchfab — https://sketchfab.com/3d-models/rov-hercules-b182d28a62854465b6574e7edb743dfc
- **Author / credit (required):** EVNautilus — https://sketchfab.com/EVNautilus
- **License:** CC-BY-4.0 (commercial use allowed; author MUST be credited). Full text: `LICENSE_CC-BY_EVNautilus.txt`.
- **Provided as:** `rov_hercules.zip` (glTF: `scene.gltf` + `scene.bin`), no UV / no texture (STL-derived).

## Regenerate `hercules.obj` (the Unity-importable asset)
`hercules.obj` (382k faces, ~52 MB) is **derived** from the glTF and treated as a large binary
(HuggingFace / local, not committed raw to keep git lean). To regenerate:

```bash
python - <<'PY'
import trimesh
s = trimesh.load("rov_hercules/scene.gltf", force="scene")
m = trimesh.util.concatenate(list(s.geometry.values()))
m.export("hercules.obj")
PY
```
Material is applied in code (`RovSimBuild.SwapInHerculesRov`) since the model has no UV/texture.
