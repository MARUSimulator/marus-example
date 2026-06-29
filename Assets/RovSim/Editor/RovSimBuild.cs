using System;
using System.Linq;
using Mujoco;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

// Builds the v1 ROV sim: injects the org.mujoco ROV + pilot controller into MARUS's HDRP ExampleScene
// (reusing MARUS's working lighting/sky/water), then optionally builds a standalone Linux player.
public static class RovSimMjImportAssets
{
    public static void CleanForMjcf(string mjcfPath)
    {
        var modelPrefix = System.IO.Path.GetFileNameWithoutExtension(mjcfPath);
        if (string.IsNullOrEmpty(modelPrefix)) return;

        var importsDir = System.IO.Path.GetFullPath(
            System.IO.Path.Combine(Application.dataPath, "Local", "MjImports"));
        if (!System.IO.Directory.Exists(importsDir)) return;

        int removed = 0;
        foreach (var dir in System.IO.Directory.GetDirectories(importsDir, modelPrefix + "*"))
        {
            var assetPath = "Assets" + dir.Substring(Application.dataPath.Length).Replace('\\', '/');
            if (AssetDatabase.DeleteAsset(assetPath))
                removed++;
            else if (System.IO.Directory.Exists(dir))
            {
                System.IO.Directory.Delete(dir, true);
                removed++;
            }
        }
        if (removed > 0)
        {
            AssetDatabase.Refresh();
            Debug.Log("ROVIMPORT_CACHE_CLEARED prefix=" + modelPrefix + " dirs=" + removed);
        }
    }
}

public static class RovSimBuild
{
    // repo-relative resolution so the demo reproduces from a clone (falls back to the dev path)
    private static string RepoFile(string rel)
    {
        return System.IO.Path.GetFullPath(System.IO.Path.Combine(Application.dataPath, "..", rel));
    }
    private static string Mjcf => RepoFile("staging/rov_v2.xml");   // v2 = vehicle + manipulator arm
    // environment selector (set ROVSIM_SCENE=deepwater for the clear blue-water site)
    private static string Scene => (Environment.GetEnvironmentVariable("ROVSIM_SCENE") ?? "harbor").ToLowerInvariant();
    private static bool DeepWater => Scene == "deepwater";
    // vehicle visual selector (ROVSIM_VEHICLE=hercules for the EV Nautilus dual-arm work-class ROV; default = fuel)
    private static string Vehicle => (Environment.GetEnvironmentVariable("ROVSIM_VEHICLE") ?? "hercules").ToLowerInvariant();
    private const string BaseScene = "Assets/Scenes/ExampleScene.unity";
    private const string OutScene = "Assets/RovSim/rov_sim_v1.unity";
    private static string BuildPath => System.IO.Path.GetFullPath(System.IO.Path.Combine(Application.dataPath, "..", "Builds", "rov_sim_v1.x86_64"));

    public static void BuildScene()
    {
        try { CreateScene(); EditorApplication.Exit(0); }
        catch (Exception ex) { Debug.LogError("ROVBUILD_SCENE_EXCEPTION " + ex); EditorApplication.Exit(1); }
    }

    public static void BuildStandalone()
    {
        try
        {
            CreateScene();
            System.IO.Directory.CreateDirectory(System.IO.Path.GetDirectoryName(BuildPath));
            PlayerSettings.productName = "OceanScale ROV Sim";
            try { PlayerSettings.bundleVersion = System.IO.File.ReadAllText(RepoFile("Assets/RovSim/VERSION")).Trim(); } catch { }
            PlayerSettings.defaultScreenWidth = 1280;
            PlayerSettings.defaultScreenHeight = 720;
            PlayerSettings.fullScreenMode = FullScreenMode.Windowed;
            var opts = new BuildPlayerOptions
            {
                scenes = new[] { OutScene },
                locationPathName = BuildPath,
                target = BuildTarget.StandaloneLinux64,
                options = BuildOptions.None
            };
            var report = BuildPipeline.BuildPlayer(opts);
            Debug.Log("ROVBUILD_RESULT result=" + report.summary.result +
                      " size=" + report.summary.totalSize + " out=" + BuildPath);
            EditorApplication.Exit(report.summary.result == BuildResult.Succeeded ? 0 : 1);
        }
        catch (Exception ex) { Debug.LogError("ROVBUILD_EXCEPTION " + ex); EditorApplication.Exit(1); }
    }

    // Editor-side HDRP still capture — bypasses BuildPlayer (whose ILPP step crashes in this env).
    public static void CaptureStill()
    {
        try
        {
            CreateScene();
            var pilePos = new Vector3(9.1f, -0.3f, -3.6f);     // monopile center (radius ~3 m -> face at x~6.1)
            var rovPos  = new Vector3(4.9f, -1.4f, -3.6f);      // ROV ~1.2 m off the pile face, mid-water
            var baseT = UnityEngine.Object.FindObjectsByType<Transform>(FindObjectsInactive.Include, FindObjectsSortMode.None)
                .FirstOrDefault(t => t.name.IndexOf("base_link", StringComparison.OrdinalIgnoreCase) >= 0);
            if (baseT != null) { var root = baseT.root; root.position += (rovPos - baseT.position); }
            var cam = Camera.main ?? UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsSortMode.None).FirstOrDefault();
            var follow = cam.GetComponent<RovFollowCamera>(); if (follow != null) follow.enabled = false;
            // POLISH: make every spot light cast a visible volumetric shaft; ROV work-lights as bright beams
            foreach (var wl in UnityEngine.Object.FindObjectsByType<Light>(FindObjectsSortMode.None))
            {
                var h = wl.GetComponent<HDAdditionalLightData>(); if (h == null) continue;
                if (wl.type == LightType.Spot) { h.affectsVolumetric = true; h.volumetricDimmer = 10f; }
                if (wl.name.StartsWith("rov_worklight")) { h.SetIntensity(1600f, LightUnit.Lumen); wl.shadows = LightShadows.Soft; }
            }
            // densify the existing underwater volumetric fog so light shafts/god-rays read
            foreach (var v in UnityEngine.Object.FindObjectsByType<Volume>(FindObjectsSortMode.None))
            {
                var prf = v.sharedProfile ?? v.profile;
                if (prf != null && prf.TryGet<Fog>(out var fog))
                {
                    fog.meanFreePath.Override(6.5f);              // denser -> visible shafts (not so dense it hazes the ROV)
                    fog.multipleScatteringIntensity.Override(0.9f);
                    fog.anisotropy.Override(0.86f);              // strong forward scatter = pronounced god-rays
                }
            }
            cam.transform.position = rovPos + new Vector3(-1.65f, 0.5f, 1.4f);
            cam.transform.LookAt(rovPos + new Vector3(0.45f, -0.05f, -0.15f));   // ROV subject, pile face as backdrop
            // key light BEHIND the camera (out of frame) as a wide spot toward the ROV+pile — reveals without bloom hotspot
            var aim = rovPos + new Vector3(1.4f, -0.4f, 0f);
            var kgo = new GameObject("cap_key");
            kgo.transform.position = cam.transform.position + cam.transform.up * 0.5f - cam.transform.right * 0.6f;
            kgo.transform.rotation = Quaternion.LookRotation((aim - kgo.transform.position).normalized);
            var KL = kgo.AddComponent<Light>(); KL.type = LightType.Spot; KL.spotAngle = 70f; KL.innerSpotAngle = 30f;
            KL.range = 22f; KL.spotAngle = 96f; KL.color = new Color(0.92f, 0.96f, 1f);
            var khd = kgo.AddComponent<HDAdditionalLightData>(); khd.SetIntensity(2700f, LightUnit.Lumen);
            khd.affectsVolumetric = true; khd.volumetricDimmer = 0.4f; KL.shadows = LightShadows.None;
            var pgo = new GameObject("cap_pile_light");
            pgo.transform.position = new Vector3(6.0f, -1.2f, -3.6f);
            pgo.transform.rotation = Quaternion.LookRotation((pilePos - pgo.transform.position).normalized);
            var PL = pgo.AddComponent<Light>(); PL.type = LightType.Spot; PL.spotAngle = 80f; PL.range = 10f; PL.color = new Color(0.85f, 0.92f, 1f);
            var phd = pgo.AddComponent<HDAdditionalLightData>(); phd.SetIntensity(1500f, LightUnit.Lumen); phd.affectsVolumetric = true; PL.shadows = LightShadows.None;
            // fixed exposure — auto-exposure keys off the dark water and blows out the lit ROV
            var evgo = new GameObject("cap_exposure");
            var vol = evgo.AddComponent<Volume>(); vol.isGlobal = true; vol.priority = 1000f;
            var prof = ScriptableObject.CreateInstance<VolumeProfile>(); vol.profile = prof;
            var exp = prof.Add<UnityEngine.Rendering.HighDefinition.Exposure>(true);
            exp.mode.Override(UnityEngine.Rendering.HighDefinition.ExposureMode.Fixed);
            exp.fixedExposure.Override(4.3f);   // EV; higher = darker (tame the bright ROV)
            // god-ray shaft from above-behind, angled down through the frame (volumetric spot)
            var sgo = new GameObject("cap_godray");
            sgo.transform.position = rovPos + new Vector3(-1.2f, 5.5f, 1.2f);
            sgo.transform.rotation = Quaternion.LookRotation(((rovPos + new Vector3(0.6f, -1f, -0.3f)) - sgo.transform.position).normalized);
            var SL = sgo.AddComponent<Light>(); SL.type = LightType.Spot; SL.spotAngle = 40f; SL.innerSpotAngle = 8f;
            SL.range = 20f; SL.color = new Color(0.82f, 0.9f, 1f);
            var shd = sgo.AddComponent<HDAdditionalLightData>(); shd.SetIntensity(4200f, LightUnit.Lumen);
            shd.affectsVolumetric = true; shd.volumetricDimmer = 14f; SL.shadows = LightShadows.Soft;
            // marine snow: boost + Simulate() — in editor batchmode (no play) the PS never emits otherwise
            foreach (var ps in UnityEngine.Object.FindObjectsByType<ParticleSystem>(FindObjectsSortMode.None))
            {
                var main = ps.main; main.maxParticles = 7000; main.startLifetime = 12f;
                main.startSize = new ParticleSystem.MinMaxCurve(0.015f, 0.045f);
                var em = ps.emission; em.rateOverTime = 900f;
                var shp = ps.shape; shp.shapeType = ParticleSystemShapeType.Box; shp.scale = new Vector3(10f, 8f, 10f);
                ps.transform.position = cam.transform.position + cam.transform.forward * 3.2f;
                ps.Clear(true); ps.Simulate(11f, true, true);
            }
            int W = 1920, H = 1080;
            var rt = new RenderTexture(W, H, 24) { name = "cap_rt" }; rt.antiAliasing = 1; rt.Create();
            cam.targetTexture = rt;
            for (int i = 0; i < 16; i++) cam.Render();
            RenderTexture.active = rt;
            var tex = new Texture2D(W, H, TextureFormat.RGB24, false);
            tex.ReadPixels(new Rect(0, 0, W, H), 0, 0); tex.Apply();
            RenderTexture.active = null; cam.targetTexture = null;
            var outp = RepoFile("Builds/smartfly_hdrp.png");
            System.IO.Directory.CreateDirectory(System.IO.Path.GetDirectoryName(outp));
            System.IO.File.WriteAllBytes(outp, tex.EncodeToPNG());
            Debug.Log("CAPTURE_DONE out=" + outp + " bytes=" + new System.IO.FileInfo(outp).Length);
            EditorApplication.Exit(0);
        }
        catch (Exception ex) { Debug.LogError("CAPTURE_EXCEPTION " + ex); EditorApplication.Exit(1); }
    }

    private static void CreateScene()
    {
        var scene = EditorSceneManager.OpenScene(BaseScene, OpenSceneMode.Single);
        QualitySettings.SetQualityLevel(QualitySettings.names.Length - 1, true);
        DisableUpstreamRuntimeClients();
        RovSimMjImportAssets.CleanForMjcf(Mjcf);

        var rov = new MjImporterWithAssets().ImportFile(Mjcf);
        rov.name = "rov_v1";
        rov.transform.position = new Vector3(0f, -3.0f, 0f);   // submerged in the water column
        rov.transform.rotation = Quaternion.identity;
        ForceMjMeshes(rov);
        StyleRobot(rov);
        if (Vehicle == "hercules") SwapInHerculesRov(rov); else if (Vehicle == "smartfly") SwapInSmartFlyRov(rov); else SwapInFuelRov(rov);
        var pilot = rov.AddComponent<RovPilotController>();
        pilot.autoQuitSeconds = 0f;    // interactive/demo build: runs until closed
        pilot.logMetrics = true;
        rov.AddComponent<RovManipulator>();   // v2: pilot-controlled 4-DOF + jaw manipulator arm

        var settings = UnityEngine.Object.FindObjectsByType<MjGlobalSettings>(FindObjectsSortMode.None).FirstOrDefault();
        if (settings == null) settings = new GameObject("MuJoCo Global Settings").AddComponent<MjGlobalSettings>();
        settings.UseRawGameObjectNames = true;
        settings.DebugFileName = "rov_sim_generated.xml";

        // Reuse MARUS's HDRP camera; disable any follow/agent controller on it, then frame the ROV.
        Camera frontCam = null; RenderTexture fpvRT = null;
        var cam = Camera.main ?? UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsSortMode.None).FirstOrDefault();
        if (cam != null)
        {
            foreach (var mb in cam.GetComponents<MonoBehaviour>())
            {
                if (mb == null) continue;
                var tn = mb.GetType().Name.ToLowerInvariant();
                if (tn.EndsWith("controller") || tn.Contains("follow") || tn.Contains("agent"))
                    mb.enabled = false;
            }
            cam.fieldOfView = 55f;
            var baseT = rov.GetComponentsInChildren<Transform>(true)
                .FirstOrDefault(t => t.name.IndexOf("base_link", StringComparison.OrdinalIgnoreCase) >= 0) ?? rov.transform;
            var follow = cam.gameObject.GetComponent<RovFollowCamera>() ?? cam.gameObject.AddComponent<RovFollowCamera>();
            follow.target = baseT;
            follow.offset = new Vector3(1.7f, 1.7f, -3.0f);   // higher + further back -> looks down onto ROV + seabed, surface out of frame
            cam.transform.position = baseT.position + follow.offset;
            cam.transform.LookAt(baseT.position);
            var hdCam = cam.GetComponent<HDAdditionalCameraData>();
            if (hdCam != null) hdCam.antialiasing = HDAdditionalCameraData.AntialiasingMode.TemporalAntialiasing;

            // onboard forward pilot camera (FPV). Renders to a RenderTexture and is ALWAYS enabled — never
            // toggled (toggling camera.enabled mid-run resets the org.mujoco sim). The controller positions it
            // at the ROV bow every frame and draws the RT full-screen during the onboard-camera phase.
            var fgo = new GameObject("Pilot Cam");
            frontCam = fgo.AddComponent<Camera>();
            frontCam.fieldOfView = 70f;
            fpvRT = new RenderTexture(1280, 720, 24) { name = "FPV_RT" }; fpvRT.Create();
            frontCam.targetTexture = fpvRT;
            if (fgo.GetComponent<HDAdditionalCameraData>() == null) fgo.AddComponent<HDAdditionalCameraData>();

            // (ROV work lights removed: in bright water their cones don't read while the bright sources
            // bloom into floating orbs. Authentic vehicle light-cones need a dark-water look — deferred.)
        }

        AddUnderwater();
        AddWater();
        AddEnvironment();
        // worksite waypoint in MuJoCo world (z up): x,y match the rendered wellhead; z = underwater hold depth
        if (pilot != null) { pilot.worksite = new Vector3(6.5f, -4.3f, -3.6f); pilot.chaseCamera = cam; pilot.frontCamera = frontCam; pilot.fpvRT = fpvRT; SetupPiP(pilot); HideClutter(); }

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene, OutScene);
        ScrubTrailingWhitespace(OutScene);
        AssetDatabase.SaveAssets();
        Debug.Log("ROVBUILD_SCENE_SAVED out=" + OutScene +
                  " rovBodies=" + rov.GetComponentsInChildren<MjBody>(true).Length +
                  " rovGeoms=" + rov.GetComponentsInChildren<MjGeom>(true).Length +
                  " cam=" + (cam != null ? cam.name : "none"));
    }

    private static void ScrubTrailingWhitespace(string assetPath)
    {
        var path = System.IO.Path.GetFullPath(System.IO.Path.Combine(Application.dataPath, "..", assetPath));
        if (!System.IO.File.Exists(path)) return;
        var lines = System.IO.File.ReadAllLines(path);
        for (int i = 0; i < lines.Length; i++) lines[i] = lines[i].TrimEnd(' ', '\t');
        System.IO.File.WriteAllText(path, string.Join("\n", lines) + "\n");
    }

    private static void DisableUpstreamRuntimeClients()
    {
        var remove = new System.Collections.Generic.List<GameObject>();
        foreach (var go in UnityEngine.Object.FindObjectsByType<GameObject>(FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            if (go == null) continue;
            var goName = go.name;
            if (goName.IndexOf("marus_boat", StringComparison.OrdinalIgnoreCase) >= 0)
                remove.Add(go);
            else if (goName.IndexOf("RosConnection", StringComparison.OrdinalIgnoreCase) >= 0)
                go.SetActive(false);
        }
        foreach (var go in remove)
            if (go != null) UnityEngine.Object.DestroyImmediate(go);
        foreach (var mb in UnityEngine.Object.FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            if (mb == null) continue;
            var t = mb.GetType();
            var name = t.FullName ?? t.Name;
            if (name.IndexOf(".ROS.", StringComparison.OrdinalIgnoreCase) >= 0 ||
                name.IndexOf("Ros", StringComparison.OrdinalIgnoreCase) >= 0 ||
                name.IndexOf("RosConnection", StringComparison.OrdinalIgnoreCase) >= 0 ||
                name.IndexOf("TfStreamerROS", StringComparison.OrdinalIgnoreCase) >= 0 ||
                name.IndexOf("SensorStreamer", StringComparison.OrdinalIgnoreCase) >= 0)
                mb.enabled = false;
        }
        Debug.Log("UPSTREAM_CLIENTS_DISABLED ros/grpc clients disabled and inherited MARUS vehicle removed for standalone ROV sim");
    }

    // MARUS ships sensor-visualization meshes (camera frusta / coordinate gizmos) that render as
    // small RGB-axis clutter in the scene. Hide them for a clean customer-facing demo.
    private static void HideClutter()
    {
        // MARUS ExampleScene includes a mountainous island Terrain that extends underwater as dark
        // peaks the ROV visually clips into. Hide it — our textured low-relief seabed is the ground.
        foreach (var ter in UnityEngine.Object.FindObjectsByType<Terrain>(FindObjectsSortMode.None))
            ter.gameObject.SetActive(false);
        foreach (var go in UnityEngine.Object.FindObjectsByType<GameObject>(FindObjectsSortMode.None))
            if (go.name.IndexOf("terrain", StringComparison.OrdinalIgnoreCase) >= 0 ||
                go.name.IndexOf("island", StringComparison.OrdinalIgnoreCase) >= 0) go.SetActive(false);
        foreach (var r in UnityEngine.Object.FindObjectsByType<Renderer>(FindObjectsSortMode.None))
        {
            var n = r.gameObject.name;
            var t = r.transform; bool inSensors = false;
            while (t != null) { if (t.name.IndexOf("sensor", StringComparison.OrdinalIgnoreCase) >= 0) { inSensors = true; break; } t = t.parent; }
            if (inSensors || n.IndexOf("Camera_mesh", StringComparison.OrdinalIgnoreCase) >= 0
                          || n.IndexOf("_mesh", StringComparison.OrdinalIgnoreCase) >= 0 && n.IndexOf("fuel", StringComparison.OrdinalIgnoreCase) < 0)
                r.enabled = false;
        }
        Debug.Log("CLUTTER_HIDDEN sensor-visualization meshes disabled");
    }

    private static void AddUnderwater()
    {
        var volGo = new GameObject("RovSim Underwater Volume");
        var vol = volGo.AddComponent<Volume>();
        vol.isGlobal = true;
        vol.priority = 100f;
        var profile = ScriptableObject.CreateInstance<VolumeProfile>();
        vol.sharedProfile = profile;

        // volumetric underwater fog (god-ray scattering + depth color absorption) — the water-volume
        // look: light attenuation + depth color shift + light shafts + particulate scattering.
        var fog = profile.Add<Fog>(true);
        fog.enabled.Override(true);
        fog.enableVolumetricFog.Override(true);
        fog.meanFreePath.Override(DeepWater ? 16f : 8.5f);   // deepwater = clearer (longer visibility)
        fog.albedo.Override(DeepWater ? new Color(0.07f, 0.28f, 0.55f) : new Color(0.09f, 0.40f, 0.50f));
        fog.tint.Override(new Color(0.04f, 0.18f, 0.26f));
        fog.anisotropy.Override(0.78f);                 // forward scatter -> visible light shafts
        fog.multipleScatteringIntensity.Override(0.55f);
        fog.depthExtent.Override(60f);
        fog.baseHeight.Override(-200f);
        fog.maximumHeight.Override(200f);

        // cinematic post (keep MARUS scene auto-exposure for brightness; just grade + tonemap)
        var tm = profile.Add<Tonemapping>(true);
        tm.mode.Override(TonemappingMode.ACES);
        var bloom = profile.Add<Bloom>(true);
        bloom.intensity.Override(0.28f); bloom.threshold.Override(0.9f); bloom.scatter.Override(0.78f);
        var ca = profile.Add<ColorAdjustments>(true);
        ca.postExposure.Override(0.15f); ca.contrast.Override(14f); ca.saturation.Override(8f);
        ca.colorFilter.Override(new Color(0.82f, 0.92f, 1.0f));

        // make the scene sun volumetric -> god-ray shafts
        var sun = UnityEngine.Object.FindObjectsByType<Light>(FindObjectsSortMode.None)
            .FirstOrDefault(l => l.type == LightType.Directional);
        if (sun != null)
        {
            var hd = sun.GetComponent<HDAdditionalLightData>();
            if (hd != null) { hd.affectsVolumetric = true; hd.volumetricDimmer = 8f; }
            sun.shadows = LightShadows.Soft;
        }

        AddMarineSnow();

        var seabed = GameObject.CreatePrimitive(PrimitiveType.Plane);
        seabed.name = "subsea floor";
        seabed.transform.position = new Vector3(0f, -4.4f, 0f);
        seabed.transform.localScale = new Vector3(8f, 1f, 8f);
        var shader = Shader.Find("HDRP/Lit");
        if (shader != null)
        {
            var m = new Material(shader);
            m.SetColor("_BaseColor", new Color(0.20f, 0.21f, 0.16f));
            if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", 0.08f);
            seabed.GetComponent<Renderer>().sharedMaterial = m;
        }
    }

    private static void AddWater()
    {
        var go = new GameObject("Ocean");
        go.transform.position = Vector3.zero;          // surface at y=0
        var w = go.AddComponent<WaterSurface>();
        w.surfaceType = WaterSurfaceType.OceanSeaLake;
        w.geometryType = WaterGeometryType.Infinite;
        w.underWater = true;                            // native underwater rendering
        w.underWaterRefraction = true;
        w.volumeDepth = 80f;                            // cover ROV (-3 m) + seabed (-4.4 m)
        w.caustics = true;
        w.causticsIntensity = 1.0f;
        w.causticsResolution = WaterSurface.WaterCausticsResolution.Caustics512;
        w.absorptionDistanceMultiplier = DeepWater ? 7.0f : 2.6f;   // deepwater: clearer; harbor: turbid
        w.underWaterScatteringColor = DeepWater ? new Color(0.03f, 0.10f, 0.26f) : new Color(0.04f, 0.13f, 0.20f); // bluer vs blue-green
        w.underWaterAmbientProbeContribution = 0.35f;   // dim ambient -> vehicle lights dominate (authentic ROV footage)
        Debug.Log("WATER_ADDED HDRP native WaterSurface (ocean + underwater + caustics)");
    }

    private static void SetupPiP(RovPilotController pilot)
    {
        var rt = new RenderTexture(384, 216, 16) { name = "PiP_RT" };
        rt.Create();
        var go = new GameObject("Inspection Cam");
        go.transform.position = new Vector3(8.6f, -2.7f, 3.3f);     // oblique 3/4 view, ~3 m from the wellhead (frames the whole structure, less occlusion)
        go.transform.LookAt(new Vector3(6.5f, -3.7f, 1.5f));        // wellhead mid-body
        var cam = go.AddComponent<Camera>();
        cam.targetTexture = rt;
        cam.fieldOfView = 50f;
        cam.depth = -5f;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = new Color(0.02f, 0.12f, 0.16f);
        if (go.GetComponent<HDAdditionalCameraData>() == null) go.AddComponent<HDAdditionalCameraData>();
        pilot.pipCamera = cam;
        pilot.pipRT = rt;
        Debug.Log("PIP_ADDED inspection camera -> 256x144 RT");
    }

    // Build Linux (works here) + attempt Windows + macOS; report per-target without failing the whole build.
    public static void BuildAllPlatforms()
    {
        try
        {
            CreateScene();
            var dir = System.IO.Path.GetDirectoryName(BuildPath);
            System.IO.Directory.CreateDirectory(dir);
            (BuildTarget, BuildTargetGroup, string)[] targets =
            {
                (BuildTarget.StandaloneLinux64, BuildTargetGroup.Standalone, dir + "/linux/rov_sim_v1.x86_64"),
                (BuildTarget.StandaloneWindows64, BuildTargetGroup.Standalone, dir + "/windows/rov_sim_v1.exe"),
                (BuildTarget.StandaloneOSX, BuildTargetGroup.Standalone, dir + "/macos/rov_sim_v1.app"),
            };
            foreach (var (t, g, path) in targets)
            {
                bool supported = BuildPipeline.IsBuildTargetSupported(g, t);
                if (!supported) { Debug.Log("BUILD_TARGET " + t + " = MODULE_MISSING (build-support not installed)"); continue; }
                try
                {
                    System.IO.Directory.CreateDirectory(System.IO.Path.GetDirectoryName(path));
                    var rep = BuildPipeline.BuildPlayer(new BuildPlayerOptions
                    { scenes = new[] { OutScene }, locationPathName = path, target = t, options = BuildOptions.None });
                    Debug.Log("BUILD_TARGET " + t + " = " + rep.summary.result + " size=" + rep.summary.totalSize);
                }
                catch (Exception ex) { Debug.Log("BUILD_TARGET " + t + " = ERROR " + ex.Message); }
            }
            EditorApplication.Exit(0);
        }
        catch (Exception ex) { Debug.LogError("BUILD_ALL_EXCEPTION " + ex); EditorApplication.Exit(1); }
    }

    private static void AddEnvironment()
    {
        const string dir = "Assets/RovSim/Models/env";
        float floorY = -4.3f;
        var sand = MakeTexMat(dir + "/ground_color.png", dir + "/ground_normal.png", new Vector2(11f, 11f), 0.10f, "M_seabed_sand");
        var rockMat = MakeTexMat(dir + "/rock_color.png", dir + "/rock_normal.png", new Vector2(1f, 1f), 0.18f, "M_rock");
        var steel = MakeMat(new Color(0.10f, 0.11f, 0.12f), 0.85f, 0.55f, "M_wet_steel");
        var pipe = MakeMat(new Color(0.07f, 0.08f, 0.09f), 0.7f, 0.45f, "M_pipe");
        var rust = MakeMat(new Color(0.32f, 0.18f, 0.09f), 0.3f, 0.3f, "M_rust");
        var yellow = MakeMat(new Color(0.9f, 0.7f, 0.08f), 0.1f, 0.4f, "M_valve_yellow");

        // dune seabed mesh
        var mesh = new Mesh { name = "seabed", indexFormat = IndexFormat.UInt32 };
        const int cells = 160; const float size = 44f;
        var verts = new Vector3[(cells + 1) * (cells + 1)]; var uvs = new Vector2[verts.Length]; var tris = new int[cells * cells * 6];
        for (int z = 0; z <= cells; z++)
            for (int x = 0; x <= cells; x++)
            {
                float xf = (x / (float)cells - 0.5f) * size, zf = (z / (float)cells - 0.5f) * size;
                float h = Mathf.PerlinNoise(xf * 0.09f + 11f, zf * 0.09f + 7f) * 0.34f
                        + Mathf.PerlinNoise(xf * 0.28f + 3f, zf * 0.28f + 9f) * 0.12f
                        + Mathf.PerlinNoise(xf * 0.9f, zf * 0.9f) * 0.04f;          // low relief: peaks stay >=0.5 m below the ROV
                verts[z * (cells + 1) + x] = new Vector3(xf, floorY - 0.25f + h, zf);
                uvs[z * (cells + 1) + x] = new Vector2(xf / 2f, zf / 2f);
            }
        int ti = 0;
        for (int z = 0; z < cells; z++)
            for (int x = 0; x < cells; x++)
            {
                int i = z * (cells + 1) + x;
                tris[ti++] = i; tris[ti++] = i + cells + 1; tris[ti++] = i + 1;
                tris[ti++] = i + 1; tris[ti++] = i + cells + 1; tris[ti++] = i + cells + 2;
            }
        mesh.vertices = verts; mesh.uv = uvs; mesh.triangles = tris; mesh.RecalculateNormals(); mesh.RecalculateTangents();
        var seabed = new GameObject("seabed");
        seabed.AddComponent<MeshFilter>().sharedMesh = mesh;
        var sr = seabed.AddComponent<MeshRenderer>(); sr.sharedMaterial = sand; sr.receiveShadows = true;
        seabed.AddComponent<MeshCollider>().sharedMesh = mesh;   // sonar/altimeter raycast target

        // offshore wind-turbine monopile foundation (the inspection target) at the worksite
        var monoMat = MakeMat(new Color(0.34f, 0.36f, 0.39f), 0.65f, 0.45f, "M_monopile");
        var collarMat = MakeMat(new Color(0.86f, 0.62f, 0.06f), 0.2f, 0.4f, "M_tp_collar");
        var worksite = new Vector3(6.5f, floorY, -3.6f);
        var pile = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        pile.name = "wt_monopile";
        pile.transform.position = worksite + new Vector3(2.6f, 4.0f, 0f);   // beside the worksite, spanning seabed->surface
        pile.transform.localScale = new Vector3(6f, 6f, 6f);                // ~6 m diameter, ~12 m tall
        pile.GetComponent<Renderer>().sharedMaterial = monoMat;
        pile.GetComponent<Renderer>().shadowCastingMode = ShadowCastingMode.On;
        var collar = GameObject.CreatePrimitive(PrimitiveType.Cylinder);   // transition-piece collar
        collar.name = "wt_transition_piece";
        collar.transform.SetParent(pile.transform, false);
        collar.transform.localScale = new Vector3(1.12f, 0.10f, 1.12f);
        collar.transform.localPosition = new Vector3(0f, 0.30f, 0f);
        collar.GetComponent<Renderer>().sharedMaterial = collarMat;

        // scattered rocks
        var rng = new System.Random(7); var rocks = new GameObject("boulders");
        for (int i = 0; i < 38; i++)
        {
            var rk = GameObject.CreatePrimitive(PrimitiveType.Sphere); rk.transform.SetParent(rocks.transform);
            float ang = (float)(rng.NextDouble() * Math.PI * 2), dist = 1.8f + (float)rng.NextDouble() * 11f;
            rk.transform.position = new Vector3(Mathf.Cos(ang) * dist, floorY - 0.2f - (float)rng.NextDouble() * 0.3f, Mathf.Sin(ang) * dist + 1f);
            float s = 0.3f + (float)Math.Pow(rng.NextDouble(), 1.7) * 1.8f;
            rk.transform.localScale = new Vector3(s, s * (0.5f + (float)rng.NextDouble() * 0.4f), s * (0.8f + (float)rng.NextDouble() * 0.5f));
            rk.transform.rotation = Quaternion.Euler((float)rng.NextDouble() * 40, (float)rng.NextDouble() * 360, (float)rng.NextDouble() * 40);
            rk.GetComponent<Renderer>().sharedMaterial = rockMat;
        }

        // subsea wellhead worksite (a few m from ROV start)
        var wh = new GameObject("subsea wellhead");
        wh.transform.position = new Vector3(6.5f, floorY, 1.5f);
        wh.transform.rotation = Quaternion.Euler(0, -24f, 0);
        var skid = GameObject.CreatePrimitive(PrimitiveType.Cube); skid.transform.SetParent(wh.transform, false);
        skid.transform.localPosition = new Vector3(0, 0.18f, 0); skid.transform.localScale = new Vector3(2.4f, 0.36f, 1.9f);
        skid.GetComponent<Renderer>().sharedMaterial = steel;
        var corners = new[] { new Vector3(-1.1f, 0.4f, -0.85f), new Vector3(1.1f, 0.4f, -0.85f), new Vector3(1.1f, 0.4f, 0.85f), new Vector3(-1.1f, 0.4f, 0.85f) };
        for (int i = 0; i < 4; i++) PipeSeg(wh.transform, corners[i], corners[(i + 1) % 4], 0.06f, pipe);
        foreach (var cn in corners) PipeSeg(wh.transform, cn, cn + Vector3.up * 1.0f, 0.06f, pipe);
        float sy = 0.4f;
        for (int i = 0; i < 4; i++)
        {
            var seg = GameObject.CreatePrimitive(PrimitiveType.Cylinder); seg.transform.SetParent(wh.transform, false);
            float h = 0.26f - i * 0.03f; seg.transform.localPosition = new Vector3(0, sy + h, 0); seg.transform.localScale = new Vector3(0.42f - i * 0.06f, h, 0.42f - i * 0.06f);
            seg.GetComponent<Renderer>().sharedMaterial = i % 2 == 0 ? steel : pipe;
            var fl = GameObject.CreatePrimitive(PrimitiveType.Cylinder); fl.transform.SetParent(wh.transform, false);
            fl.transform.localPosition = new Vector3(0, sy + h * 2, 0); fl.transform.localScale = new Vector3(0.52f - i * 0.06f, 0.025f, 0.52f - i * 0.06f);
            fl.GetComponent<Renderer>().sharedMaterial = steel; sy += h * 2;
        }
        for (int i = 0; i < 2; i++)
        {
            var v = GameObject.CreatePrimitive(PrimitiveType.Cylinder); v.transform.SetParent(wh.transform, false);
            v.transform.localPosition = new Vector3(-0.45f + i * 0.9f, 0.78f, -0.55f); v.transform.localRotation = Quaternion.Euler(90, 0, 0); v.transform.localScale = new Vector3(0.26f, 0.03f, 0.26f);
            v.GetComponent<Renderer>().sharedMaterial = i == 0 ? rust : yellow;
        }
        PipeSeg(wh.transform, new Vector3(-1.2f, 0.55f, 0), new Vector3(1.2f, 0.55f, 0), 0.09f, steel);

        // export pipeline running along the seabed to the wellhead (a survey/inspection target)
        var pl = new GameObject("pipeline");
        Vector3 pa = new Vector3(-4f, floorY + 0.22f, -1.5f), pb = new Vector3(6.2f, floorY + 0.22f, 1.3f);
        PipeSeg(pl.transform, pa, pb, 0.13f, pipe);
        for (int i = 0; i <= 5; i++)
        {
            var an = GameObject.CreatePrimitive(PrimitiveType.Cube); an.transform.SetParent(pl.transform, false);
            an.transform.localPosition = Vector3.Lerp(pa, pb, i / 5f) + Vector3.down * 0.16f;
            an.transform.localScale = new Vector3(0.46f, 0.26f, 0.46f);
            an.GetComponent<Renderer>().sharedMaterial = rust;
        }
        // secondary subsea manifold (additional infrastructure, off to the side)
        var mf = new GameObject("subsea manifold");
        mf.transform.position = new Vector3(-4.6f, floorY, 3.4f); mf.transform.rotation = Quaternion.Euler(0, 28f, 0);
        var msk = GameObject.CreatePrimitive(PrimitiveType.Cube); msk.transform.SetParent(mf.transform, false);
        msk.transform.localPosition = new Vector3(0, 0.2f, 0); msk.transform.localScale = new Vector3(1.7f, 0.4f, 1.2f);
        msk.GetComponent<Renderer>().sharedMaterial = steel;
        for (int i = 0; i < 3; i++)
        {
            var vv = GameObject.CreatePrimitive(PrimitiveType.Cylinder); vv.transform.SetParent(mf.transform, false);
            vv.transform.localPosition = new Vector3(-0.5f + i * 0.5f, 0.5f, 0); vv.transform.localScale = new Vector3(0.2f, 0.28f, 0.2f);
            vv.GetComponent<Renderer>().sharedMaterial = i == 1 ? yellow : steel;
        }
        Debug.Log("ENV_ADDED seabed+rocks+wellhead+pipeline+manifold floorY=" + floorY);
    }

    private static void PipeSeg(Transform parent, Vector3 a, Vector3 b, float r, Material m)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cylinder); go.transform.SetParent(parent, false);
        var mid = (a + b) * 0.5f; var dir = b - a; float len = dir.magnitude;
        go.transform.localPosition = mid; go.transform.up = dir.normalized; go.transform.localScale = new Vector3(r * 2, len * 0.5f, r * 2);
        go.GetComponent<Renderer>().sharedMaterial = m;
    }

    private static Material MakeMat(Color c, float metal, float smooth, string name)
    {
        var m = new Material(Shader.Find("HDRP/Lit")) { name = name };
        m.SetColor("_BaseColor", c);
        if (m.HasProperty("_Metallic")) m.SetFloat("_Metallic", metal);
        if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", smooth);
        return m;
    }

    private static Material MakeTexMat(string colorPath, string normalPath, Vector2 tiling, float smooth, string name)
    {
        var ni = AssetImporter.GetAtPath(normalPath) as TextureImporter;
        if (ni != null && ni.textureType != TextureImporterType.NormalMap) { ni.textureType = TextureImporterType.NormalMap; ni.SaveAndReimport(); }
        var m = new Material(Shader.Find("HDRP/Lit")) { name = name };
        var col = AssetDatabase.LoadAssetAtPath<Texture2D>(colorPath);
        var nrm = AssetDatabase.LoadAssetAtPath<Texture2D>(normalPath);
        if (col != null) { m.SetTexture("_BaseColorMap", col); m.SetTextureScale("_BaseColorMap", tiling); }
        if (nrm != null) { m.SetTexture("_NormalMap", nrm); m.SetTextureScale("_NormalMap", tiling); m.EnableKeyword("_NORMALMAP"); }
        if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", smooth);
        if (m.HasProperty("_Metallic")) m.SetFloat("_Metallic", 0f);
        return m;
    }

    private static void AddMarineSnow()
    {
        var go = new GameObject("marine snow");
        go.transform.position = new Vector3(0f, -3f, 0f);
        var ps = go.AddComponent<ParticleSystem>();
        var main = ps.main;
        main.startLifetime = 9f;
        main.startSpeed = 0.04f;
        main.startSize = new ParticleSystem.MinMaxCurve(0.01f, 0.03f);     // small -> reads as fine drift, not boxes
        main.maxParticles = 1500;
        main.simulationSpace = ParticleSystemSimulationSpace.World;
        main.startColor = new Color(0.88f, 0.96f, 1.0f, 0.5f);
        var em = ps.emission; em.rateOverTime = 170f;
        var shp = ps.shape; shp.shapeType = ParticleSystemShapeType.Box; shp.scale = new Vector3(16f, 7f, 16f);
        var vel = ps.velocityOverLifetime; vel.enabled = true;
        vel.x = new ParticleSystem.MinMaxCurve(0f, 0f);
        vel.y = new ParticleSystem.MinMaxCurve(-0.05f, -0.01f);
        vel.z = new ParticleSystem.MinMaxCurve(0f, 0f);
        var rend = ps.GetComponent<ParticleSystemRenderer>();
        var psh = Shader.Find("HDRP/Unlit");
        if (psh != null)
        {
            var pm = new Material(psh);
            var soft = MakeSoftDot();                       // radial-alpha sprite -> soft round specks, not hard squares
            pm.SetFloat("_SurfaceType", 1f);                // transparent
            pm.SetFloat("_BlendMode", 0f);                  // alpha blend
            pm.SetFloat("_AlphaCutoffEnable", 0f);
            pm.SetColor("_UnlitColor", new Color(0.9f, 0.97f, 1f, 0.5f));
            if (pm.HasProperty("_BaseColor")) pm.SetColor("_BaseColor", new Color(0.9f, 0.97f, 1f, 0.5f));
            pm.SetTexture("_UnlitColorMap", soft);
            if (pm.HasProperty("_BaseColorMap")) pm.SetTexture("_BaseColorMap", soft);
            pm.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
            pm.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
            try { HDMaterial.ValidateMaterial(pm); } catch { }
            rend.sharedMaterial = pm;
        }
    }

    private static Texture2D MakeSoftDot()
    {
        const int N = 64; var t = new Texture2D(N, N, TextureFormat.RGBA32, false);
        var c = new Vector2(N / 2f, N / 2f);
        for (int y = 0; y < N; y++)
            for (int x = 0; x < N; x++)
            {
                float d = Vector2.Distance(new Vector2(x + 0.5f, y + 0.5f), c) / (N / 2f);
                float a = Mathf.Clamp01(1f - d); a *= a;
                t.SetPixel(x, y, new Color(1f, 1f, 1f, a));
            }
        t.Apply(); return t;
    }

    private static void SwapInFuelRov(GameObject rov)
    {
        const string dae = "Assets/RovSim/Models/fuel_rov/fuel_rov.dae";
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(dae);
        if (prefab == null) { Debug.LogError("FUELROV_MISSING " + dae); return; }

        var baseT = rov.GetComponentsInChildren<Transform>(true)
            .FirstOrDefault(t => t.name.IndexOf("base_link", StringComparison.OrdinalIgnoreCase) >= 0) ?? rov.transform;

        // hide the primitive box visuals (keep MjGeom for physics/collision)
        foreach (var r in rov.GetComponentsInChildren<MeshRenderer>(true)) r.enabled = false;

        var vis = (GameObject)UnityEngine.Object.Instantiate(prefab);
        vis.name = "fuel_rov_visual";
        vis.transform.SetParent(baseT, false);
        vis.transform.localPosition = Vector3.zero;
        vis.transform.localRotation = Quaternion.Euler(0f, 36f, 0f);
        vis.transform.localScale = Vector3.one;

        var shader = Shader.Find("HDRP/Lit");
        if (shader != null)
        {
            const string dir = "Assets/RovSim/Models/fuel_rov";
            var col = AssetDatabase.LoadAssetAtPath<Texture2D>(dir + "/color.png");
            var nrm = AssetDatabase.LoadAssetAtPath<Texture2D>(dir + "/normal.png");
            var msk = AssetDatabase.LoadAssetAtPath<Texture2D>(dir + "/mask.png");
            var mat = new Material(shader);
            if (col != null) mat.SetTexture("_BaseColorMap", col);
            if (nrm != null) { mat.SetTexture("_NormalMap", nrm); mat.EnableKeyword("_NORMALMAP"); }
            if (msk != null) { mat.SetTexture("_MaskMap", msk); mat.EnableKeyword("_MASKMAP"); }
            if (mat.HasProperty("_Smoothness")) mat.SetFloat("_Smoothness", 0.5f);
            foreach (var r in vis.GetComponentsInChildren<MeshRenderer>(true)) r.sharedMaterial = mat;
        }

        // auto-scale to ~0.9 m longest dimension and recenter on base_link
        var b = CombinedBounds(vis);
        float maxDim = Mathf.Max(b.size.x, Mathf.Max(b.size.y, b.size.z));
        if (maxDim > 1e-3f)
        {
            vis.transform.localScale = Vector3.one * (0.9f / maxDim);
            var b2 = CombinedBounds(vis);
            vis.transform.position += baseT.position - b2.center;   // center the mesh on base_link
        }
        Debug.Log("FUELROV_SWAPPED maxDim=" + maxDim.ToString("F3") +
                  " renderers=" + vis.GetComponentsInChildren<MeshRenderer>(true).Length);
    }

    // EV Nautilus "ROV Hercules" (Sketchfab, CC-BY-4.0) — a real dual-arm work-class ROV that matches the
    // manipulator arm (v2). glTF->OBJ (382k faces, no UV/texture) -> single industrial material applied here.
    private static void SwapInHerculesRov(GameObject rov)
    {
        const string obj = "Assets/RovSim/Models/hercules/hercules.obj";
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(obj);
        if (prefab == null) { Debug.LogError("HERCULES_MISSING " + obj + " -> falling back to Fuel ROV"); SwapInFuelRov(rov); return; }

        var baseT = rov.GetComponentsInChildren<Transform>(true)
            .FirstOrDefault(t => t.name.IndexOf("base_link", StringComparison.OrdinalIgnoreCase) >= 0) ?? rov.transform;

        // Replace ONLY the base hull/thrusters/floats with the Hercules mesh, but KEEP the functional
        // manipulator arm visible — its MjGeom GameObjects (g_azi/g_upper/g_fore/g_wrist/g_jaw_*) articulate
        // with the physics, so the pilot sees the arm actually move. Style them as an industrial manipulator.
        var armShader = Shader.Find("HDRP/Lit");
        Material armMat = null, jawMat = null;
        if (armShader != null)
        {
            armMat = new Material(armShader) { name = "M_herc_arm" };
            armMat.SetColor("_BaseColor", new Color(0.92f, 0.46f, 0.04f));   // manipulator orange
            if (armMat.HasProperty("_Metallic")) armMat.SetFloat("_Metallic", 0.35f);
            if (armMat.HasProperty("_Smoothness")) armMat.SetFloat("_Smoothness", 0.42f);
            jawMat = new Material(armShader) { name = "M_herc_jaw" };
            jawMat.SetColor("_BaseColor", new Color(0.09f, 0.09f, 0.11f));
            if (jawMat.HasProperty("_Metallic")) jawMat.SetFloat("_Metallic", 0.85f);
            if (jawMat.HasProperty("_Smoothness")) jawMat.SetFloat("_Smoothness", 0.55f);
        }
        foreach (var r in rov.GetComponentsInChildren<MeshRenderer>(true))
        {
            var n = r.name.ToLowerInvariant();
            bool isJaw = n.Contains("jaw");
            bool isArm = n.Contains("azi") || n.Contains("upper") || n.Contains("fore") || n.Contains("wrist") || isJaw;
            if (isArm) { r.enabled = true; if (armMat != null) r.sharedMaterial = isJaw ? jawMat : armMat; r.shadowCastingMode = ShadowCastingMode.On; }
            else r.enabled = false;   // hull/deck/thrusters/floats -> replaced by the Hercules mesh
        }

        var vis = (GameObject)UnityEngine.Object.Instantiate(prefab);
        vis.name = "hercules_visual";
        vis.transform.SetParent(baseT, false);
        vis.transform.localRotation = Quaternion.Euler(-90f, 0f, 0f);   // STL is Z-up -> Unity Y-up (heading tuned after render)
        vis.transform.localScale = Vector3.one;

        // two-tone work-class palette (the OBJ is split into 2 submeshes: [0]=buoyancy top, [1]=frame/body)
        var shader = Shader.Find("HDRP/Lit");
        if (shader != null)
        {
            Material Mk(Color c, float metal, float smooth, string nm)
            {
                var m2 = new Material(shader) { name = nm };
                m2.SetColor("_BaseColor", c);
                if (m2.HasProperty("_Metallic")) m2.SetFloat("_Metallic", metal);
                if (m2.HasProperty("_Smoothness")) m2.SetFloat("_Smoothness", smooth);
                return m2;
            }
            var buoy = Mk(new Color(0.84f, 0.65f, 0.07f), 0.0f, 0.14f, "M_herc_buoyancy");   // syntactic-foam (matte, slightly dirty yellow)
            var frame = Mk(new Color(0.10f, 0.11f, 0.13f), 0.6f, 0.30f, "M_herc_frame");     // wet dark metal (matte, not plasticky)
            foreach (var r in vis.GetComponentsInChildren<MeshRenderer>(true))
            {
                r.enabled = true; r.shadowCastingMode = ShadowCastingMode.On;
                var mf = r.GetComponent<MeshFilter>();
                int subs = (mf != null && mf.sharedMesh != null) ? mf.sharedMesh.subMeshCount : 1;
                r.sharedMaterials = subs >= 2 ? new[] { buoy, frame } : new[] { frame };
            }
        }

        // forward ROV work-lights (two spots) — the vehicle reads as a working ROV and lights the worksite
        for (int i = 0; i < 2; i++)
        {
            var lgo = new GameObject("rov_worklight_" + i);
            lgo.transform.SetParent(baseT, false);
            lgo.transform.localPosition = new Vector3(i == 0 ? -0.20f : 0.20f, 0.06f, 0.30f);
            lgo.transform.localRotation = Quaternion.Euler(8f, 0f, 0f);   // forward + slightly down, toward the work area
            var L = lgo.AddComponent<Light>();
            L.type = LightType.Spot; L.spotAngle = 44f; L.innerSpotAngle = 22f; L.range = 6f; L.color = new Color(0.90f, 0.95f, 1f);
            var hd = lgo.AddComponent<HDAdditionalLightData>();
            hd.SetIntensity(2000f, LightUnit.Lumen);
            hd.affectsVolumetric = true; hd.volumetricDimmer = 0.5f; L.shadows = LightShadows.None;
        }

        var b = CombinedBounds(vis);
        float maxDim = Mathf.Max(b.size.x, Mathf.Max(b.size.y, b.size.z));
        if (maxDim > 1e-3f)
        {
            vis.transform.localScale = Vector3.one * (0.95f / maxDim);   // ~0.95 m longest dim, like the Fuel ROV
            var b2 = CombinedBounds(vis);
            vis.transform.position += baseT.position - b2.center;        // center on base_link
        }
        Debug.Log("HERCULES_SWAPPED maxDim=" + maxDim.ToString("F3") +
                  " renderers=" + vis.GetComponentsInChildren<MeshRenderer>(true).Length + " credit=EVNautilus(CC-BY-4.0)");
    }

    [System.Serializable] private class SfColor { public string name; public float r, g, b, metallic, smoothness; }
    [System.Serializable] private class SfColorTable { public SfColor[] items; }

    private static void SwapInSmartFlyRov(GameObject rov)
    {
        // Customer SmartFly/智海 inspection ROV — multi-material colored mesh from their own CAD (智海ROV.STEP),
        // 31 submeshes carrying the real per-part CAD colors + reconstructed ducted-thruster shrouds.
        const string obj = "Assets/RovSim/Models/smartfly/smartfly_real_v2.obj";
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(obj);
        if (prefab == null) { Debug.LogError("SMARTFLY_MISSING " + obj + " -> falling back to Fuel ROV"); SwapInFuelRov(rov); return; }

        var baseT = rov.GetComponentsInChildren<Transform>(true)
            .FirstOrDefault(t => t.name.IndexOf("base_link", StringComparison.OrdinalIgnoreCase) >= 0) ?? rov.transform;

        foreach (var r in rov.GetComponentsInChildren<MeshRenderer>(true))
        {
            var n = r.name.ToLowerInvariant();
            bool isArm = n.Contains("azi") || n.Contains("upper") || n.Contains("fore") || n.Contains("wrist") || n.Contains("jaw");
            r.enabled = false;                       // hide the generic rov_v2 hull/arm; SmartFly has no arm
            if (isArm) r.enabled = false;
        }

        var vis = (GameObject)UnityEngine.Object.Instantiate(prefab);
        vis.name = "smartfly_visual";
        vis.transform.SetParent(baseT, false);
        vis.transform.localRotation = Quaternion.identity;   // real-CAD OBJ is already Y-up (yellow floats on top)
        vis.transform.localScale = Vector3.one;

        var shader = Shader.Find("HDRP/Lit");
        // load the real per-part CAD color table (31 materials, by MTL name m0..mduct5)
        var ctab = new System.Collections.Generic.Dictionary<string, SfColor>();
        try {
            var js = System.IO.File.ReadAllText(RepoFile("Assets/RovSim/smartfly_real_v2_colors.json"));
            var tab = JsonUtility.FromJson<SfColorTable>(js);
            foreach (var it in tab.items) ctab[it.name] = it;
        } catch (Exception e) { Debug.LogError("SMARTFLY_COLORS_LOAD_FAIL " + e.Message); }
        if (shader != null)
        {
            var cache = new System.Collections.Generic.Dictionary<string, Material>();
            Material MatFor(string key) {
                if (cache.TryGetValue(key, out var mm)) return mm;
                var m2 = new Material(shader) { name = "M_sf_" + key };
                if (ctab.TryGetValue(key, out var e)) {
                    m2.SetColor("_BaseColor", new Color(e.r, e.g, e.b));
                    if (m2.HasProperty("_Metallic")) m2.SetFloat("_Metallic", e.metallic);
                    if (m2.HasProperty("_Smoothness")) m2.SetFloat("_Smoothness", e.smoothness);
                } else { m2.SetColor("_BaseColor", new Color(0.45f, 0.46f, 0.48f)); if (m2.HasProperty("_Metallic")) m2.SetFloat("_Metallic", 0.7f); }
                cache[key] = m2; return m2;
            }
            foreach (var r in vis.GetComponentsInChildren<MeshRenderer>(true))
            {
                r.enabled = true; r.shadowCastingMode = ShadowCastingMode.On;
                var src = r.sharedMaterials; var outm = new Material[src.Length];
                for (int i = 0; i < src.Length; i++) {
                    var nm = (src[i] != null ? src[i].name : ("m" + i)).Replace(" (Instance)", "").Trim();
                    outm[i] = MatFor(nm);
                }
                r.sharedMaterials = outm;
            }
        }

        for (int i = 0; i < 2; i++)
        {
            var lgo = new GameObject("rov_worklight_" + i);
            lgo.transform.SetParent(baseT, false);
            lgo.transform.localPosition = new Vector3(i == 0 ? -0.20f : 0.20f, 0.06f, 0.30f);
            lgo.transform.localRotation = Quaternion.Euler(8f, 0f, 0f);
            var L = lgo.AddComponent<Light>();
            L.type = LightType.Spot; L.spotAngle = 44f; L.innerSpotAngle = 22f; L.range = 6f; L.color = new Color(0.90f, 0.95f, 1f);
            var hd = lgo.AddComponent<HDAdditionalLightData>();
            hd.SetIntensity(2000f, LightUnit.Lumen);
            hd.affectsVolumetric = true; hd.volumetricDimmer = 0.5f; L.shadows = LightShadows.None;
        }

        // TRUE customer-CAD scale: mesh is in mm -> x0.001 = real size (~0.907 x 1.091 x 1.131 m). Do NOT
        // force-normalize to a fixed length (that imposed a wrong size); use the actual CAD dimensions.
        vis.transform.localScale = Vector3.one;   // colored OBJ already in meters
        var b = CombinedBounds(vis);
        float maxDim = Mathf.Max(b.size.x, Mathf.Max(b.size.y, b.size.z));
        vis.transform.position += baseT.position - b.center;
        Debug.Log("SMARTFLY_SWAPPED trueScale=0.001 maxDim_m=" + maxDim.ToString("F3") +
                  " renderers=" + vis.GetComponentsInChildren<MeshRenderer>(true).Length + " src=customerCAD(智海ROV.STEP)");
    }

    private static Bounds CombinedBounds(GameObject go)
    {
        var rs = go.GetComponentsInChildren<Renderer>(true);
        if (rs.Length == 0) return new Bounds(go.transform.position, Vector3.one * 0.5f);
        var b = rs[0].bounds;
        for (int i = 1; i < rs.Length; i++) b.Encapsulate(rs[i].bounds);
        return b;
    }

    private static void StyleRobot(GameObject root)
    {
        var shader = Shader.Find("HDRP/Lit");
        if (shader == null) return;
        foreach (var r in root.GetComponentsInChildren<MeshRenderer>(true))
        {
            var n = r.name.ToLowerInvariant();
            var m = new Material(shader);
            Color baseCol; float metal = 0f, smooth = 0.45f; Color emis = Color.black;
            if (n.Contains("thr")) { baseCol = new Color(0.05f, 0.05f, 0.06f); metal = 0.9f; smooth = 0.7f; }
            else if (n.Contains("float")) { baseCol = new Color(0.95f, 0.95f, 0.97f); }
            else if (n.Contains("cam")) { baseCol = new Color(0.02f, 0.02f, 0.03f); smooth = 0.8f; }
            else if (n.Contains("deck")) { baseCol = new Color(0.12f, 0.13f, 0.15f); metal = 0.6f; }
            else { baseCol = new Color(1.0f, 0.78f, 0.05f); emis = new Color(1.0f, 0.70f, 0.10f) * 1.6f; } // hull: bright yellow + glow
            m.SetColor("_BaseColor", baseCol);
            if (m.HasProperty("_Metallic")) m.SetFloat("_Metallic", metal);
            if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", smooth);
            if (emis != Color.black && m.HasProperty("_EmissiveColor"))
            {
                m.SetColor("_EmissiveColor", emis);
                m.EnableKeyword("_EMISSION");
                if (m.HasProperty("_UseEmissiveIntensity")) m.SetFloat("_UseEmissiveIntensity", 0f);
            }
            r.sharedMaterial = m;
            r.shadowCastingMode = ShadowCastingMode.On;
        }
    }

    private static void ForceMjMeshes(GameObject root)
    {
        var update = typeof(MjMeshFilter).GetMethod("Update",
            System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
        if (update == null) return;
        foreach (var f in root.GetComponentsInChildren<MjMeshFilter>(true)) update.Invoke(f, null);
    }
}
