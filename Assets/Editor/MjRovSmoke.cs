using System;
using System.Globalization;
using Mujoco;
using UnityEditor;
using UnityEngine;

// C4 smoke: prove org.mujoco compiles + steps in the MARUS Unity 6 project,
// and that xfrc_applied drives the base (pre-validates C5 = drive-by-force, not teleport).
public static class MjRovSmoke
{
    private const string Mjcf = "/mnt/storage/marus-rov-sim/staging/rov_v1.xml";

    public static void Run()
    {
        try { EditorApplication.Exit(RunInner() ? 0 : 1); }
        catch (Exception ex) { Debug.LogError("MJROV_EXCEPTION " + ex); EditorApplication.Exit(1); }
    }

    private static unsafe bool RunInner()
    {
        Debug.Log("MJROV_START mjcf=" + Mjcf);
        var root = new MjImporterWithAssets().ImportFile(Mjcf);
        int nb = root.GetComponentsInChildren<MjBody>(true).Length;
        int ng = root.GetComponentsInChildren<MjGeom>(true).Length;
        int nj = root.GetComponentsInChildren<MjBaseJoint>(true).Length;
        Debug.Log($"MJROV_IMPORTED root={root.name} bodies={nb} geoms={ng} joints={nj}");

        var scene = MjScene.Instance;
        scene.CreateScene();
        int nq = (int)scene.Model->nq, nv = (int)scene.Model->nv, nbody = (int)scene.Model->nbody;
        Debug.Log($"MJROV_SCENE nbody={nbody} nq={nq} nv={nv}");
        if (nq < 7) { Debug.LogError("MJROV_FAIL nq<7 (no free joint?)"); return false; }

        // base_link is body index 1 (0 = world); MJCF has a single free body.
        const int baseId = 1;
        double x0 = scene.Data->qpos[0], y0 = scene.Data->qpos[1], z0 = scene.Data->qpos[2];

        // Apply a constant world +X surge force on the base; zero-gravity → must translate by integration.
        for (int s = 0; s < 200; s++)
        {
            for (int i = 0; i < 6 * nbody; i++) scene.Data->xfrc_applied[i] = 0.0;
            scene.Data->xfrc_applied[6 * baseId + 0] = 40.0;
            scene.StepScene();
        }

        double x1 = scene.Data->qpos[0], y1 = scene.Data->qpos[1], z1 = scene.Data->qpos[2];
        double d = Math.Sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
        Debug.Log("MJROV_STEPPED start=("
            + x0.ToString("R", CultureInfo.InvariantCulture) + ","
            + y0.ToString("R", CultureInfo.InvariantCulture) + ","
            + z0.ToString("R", CultureInfo.InvariantCulture) + ") end=("
            + x1.ToString("R", CultureInfo.InvariantCulture) + ","
            + y1.ToString("R", CultureInfo.InvariantCulture) + ","
            + z1.ToString("R", CultureInfo.InvariantCulture) + ") translation_m="
            + d.ToString("R", CultureInfo.InvariantCulture));

        bool ok = d > 0.01;
        Debug.Log("MJROV_RESULT control_via_xfrc_advances=" + ok);
        scene.DestroyScene();
        UnityEngine.Object.DestroyImmediate(scene.gameObject);
        UnityEngine.Object.DestroyImmediate(root);
        return ok;
    }
}
