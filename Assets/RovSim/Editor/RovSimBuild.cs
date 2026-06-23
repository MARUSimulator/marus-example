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
public static class RovSimBuild
{
    private const string Mjcf = "/mnt/storage/marus-rov-sim/staging/rov_v1.xml";
    private const string BaseScene = "Assets/Scenes/ExampleScene.unity";
    private const string OutScene = "Assets/RovSim/rov_sim_v1.unity";
    private const string BuildPath = "/mnt/storage/marus-rov-sim/build/rov_sim_v1.x86_64";

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
            PlayerSettings.productName = "OceanScale ROV Sim v1";
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

    private static void CreateScene()
    {
        var scene = EditorSceneManager.OpenScene(BaseScene, OpenSceneMode.Single);
        QualitySettings.SetQualityLevel(QualitySettings.names.Length - 1, true);

        var rov = new MjImporterWithAssets().ImportFile(Mjcf);
        rov.name = "rov_v1";
        rov.transform.position = new Vector3(0f, -3.0f, 0f);   // submerged in the water column
        rov.transform.rotation = Quaternion.identity;
        ForceMjMeshes(rov);
        StyleRobot(rov);
        SwapInFuelRov(rov);
        var pilot = rov.AddComponent<RovPilotController>();
        pilot.autoQuitSeconds = 0f;    // interactive/demo build: runs until closed
        pilot.logMetrics = true;

        var settings = UnityEngine.Object.FindObjectsByType<MjGlobalSettings>(FindObjectsSortMode.None).FirstOrDefault();
        if (settings == null) settings = new GameObject("MuJoCo Global Settings").AddComponent<MjGlobalSettings>();
        settings.UseRawGameObjectNames = true;
        settings.DebugFileName = "rov_sim_generated.xml";

        // Reuse MARUS's HDRP camera; disable any follow/agent controller on it, then frame the ROV.
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
            follow.offset = new Vector3(1.7f, 0.9f, -2.2f);
            cam.transform.position = baseT.position + follow.offset;
            cam.transform.LookAt(baseT.position);
            var hdCam = cam.GetComponent<HDAdditionalCameraData>();
            if (hdCam != null) hdCam.antialiasing = HDAdditionalCameraData.AntialiasingMode.TemporalAntialiasing;
        }

        AddUnderwater();

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene, OutScene);
        AssetDatabase.SaveAssets();
        Debug.Log("ROVBUILD_SCENE_SAVED out=" + OutScene +
                  " rovBodies=" + rov.GetComponentsInChildren<MjBody>(true).Length +
                  " rovGeoms=" + rov.GetComponentsInChildren<MjGeom>(true).Length +
                  " cam=" + (cam != null ? cam.name : "none"));
    }

    private static void AddUnderwater()
    {
        var volGo = new GameObject("RovSim Underwater Volume");
        var vol = volGo.AddComponent<Volume>();
        vol.isGlobal = true;
        vol.priority = 100f;
        var profile = ScriptableObject.CreateInstance<VolumeProfile>();
        vol.sharedProfile = profile;

        // volumetric underwater fog (god-ray scattering + depth color absorption)
        var fog = profile.Add<Fog>(true);
        fog.enabled.Override(true);
        fog.enableVolumetricFog.Override(true);
        fog.meanFreePath.Override(5f);                  // denser -> visible distance attenuation/murk
        fog.albedo.Override(new Color(0.10f, 0.46f, 0.52f));
        fog.tint.Override(new Color(0.05f, 0.22f, 0.28f));
        fog.anisotropy.Override(0.75f);                 // forward scatter -> visible light shafts
        fog.multipleScatteringIntensity.Override(0.4f);
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

    private static void AddMarineSnow()
    {
        var go = new GameObject("marine snow");
        go.transform.position = new Vector3(0f, -3f, 0f);
        var ps = go.AddComponent<ParticleSystem>();
        var main = ps.main;
        main.startLifetime = 9f;
        main.startSpeed = 0.04f;
        main.startSize = new ParticleSystem.MinMaxCurve(0.012f, 0.05f);
        main.maxParticles = 2600;
        main.simulationSpace = ParticleSystemSimulationSpace.World;
        main.startColor = new Color(0.88f, 0.96f, 1.0f, 0.8f);
        var em = ps.emission; em.rateOverTime = 300f;
        var shp = ps.shape; shp.shapeType = ParticleSystemShapeType.Box; shp.scale = new Vector3(16f, 7f, 16f);
        var vel = ps.velocityOverLifetime; vel.enabled = true;
        vel.y = new ParticleSystem.MinMaxCurve(-0.05f, -0.01f);
        var rend = ps.GetComponent<ParticleSystemRenderer>();
        var psh = Shader.Find("HDRP/Unlit");
        if (psh != null)
        {
            var pm = new Material(psh);
            if (pm.HasProperty("_BaseColor")) pm.SetColor("_BaseColor", new Color(0.9f, 0.97f, 1f, 0.55f));
            rend.sharedMaterial = pm;
        }
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
