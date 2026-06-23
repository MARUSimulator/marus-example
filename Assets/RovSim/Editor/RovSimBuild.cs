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

        var rov = new MjImporterWithAssets().ImportFile(Mjcf);
        rov.name = "rov_v1";
        rov.transform.position = new Vector3(0f, -3.0f, 0f);   // submerged in the water column
        rov.transform.rotation = Quaternion.identity;
        ForceMjMeshes(rov);
        StyleRobot(rov);
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
        var volGo = new GameObject("RovSim Underwater Fog");
        var vol = volGo.AddComponent<Volume>();
        vol.isGlobal = true;
        vol.priority = 100f;
        var profile = ScriptableObject.CreateInstance<VolumeProfile>();
        vol.sharedProfile = profile;
        var fog = profile.Add<Fog>(true);
        fog.enabled.Override(true);
        fog.meanFreePath.Override(3.5f);                          // dense murk: ~7 m visibility
        fog.albedo.Override(new Color(0.05f, 0.30f, 0.34f));
        fog.tint.Override(new Color(0.02f, 0.13f, 0.17f));        // dark blue-green, red absorbed
        fog.baseHeight.Override(-200f);
        fog.maximumHeight.Override(200f);

        var seabed = GameObject.CreatePrimitive(PrimitiveType.Plane);
        seabed.name = "subsea floor";
        seabed.transform.position = new Vector3(0f, -4.4f, 0f);
        seabed.transform.localScale = new Vector3(6f, 1f, 6f);
        var shader = Shader.Find("HDRP/Lit");
        if (shader != null)
        {
            var m = new Material(shader);
            m.SetColor("_BaseColor", new Color(0.16f, 0.18f, 0.14f));
            seabed.GetComponent<Renderer>().sharedMaterial = m;
        }
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
