using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using Mujoco;
using UnityEditor;
using UnityEngine;

// Headless ROV sim test + full logging system.
// Drives deterministic scripted command scenarios through the SAME RovDynamics the player uses,
// steps real MuJoCo physics, writes a full per-step telemetry CSV per scenario, and asserts
// real physics oracles (surge->forward, yaw->heading, heave->depth, coast->decelerate,
// non-teleport continuity, attitude stability). Exit code 0 iff all oracles pass.
public static class RovSimTest
{
    private const string Mjcf = "/mnt/storage/marus-rov-sim/staging/rov_v1.xml";
    private const string LogDir = "/mnt/storage/marus-rov-sim/logs";
    private const int BaseBodyId = 1;
    private const int BaseDofAdr = 0;
    private const float Dt = 0.005f;

    private struct Phase { public string name; public float dur; public float su, sw, hv, yw; }

    public static void RunHeadless()
    {
        try { EditorApplication.Exit(Run() ? 0 : 1); }
        catch (Exception ex) { Debug.LogError("ROVTEST_EXCEPTION " + ex); EditorApplication.Exit(1); }
    }

    private static unsafe bool Run()
    {
        Directory.CreateDirectory(LogDir);
        Time.fixedDeltaTime = Dt;

        var root = new MjImporterWithAssets().ImportFile(Mjcf);
        var scene = MjScene.Instance;
        scene.CreateScene();
        int nq = (int)scene.Model->nq, nv = (int)scene.Model->nv;
        Debug.Log($"ROVTEST_SCENE nq={nq} nv={nv} dt={Dt}");
        if (nq < 7) { Debug.LogError("ROVTEST_FAIL nq<7"); return false; }

        var p = RovDynamics.Params.Default;
        var phases = new[]
        {
            new Phase { name = "idle",      dur = 1f, su = 0,  sw = 0, hv = 0,  yw = 0 },
            new Phase { name = "surge",     dur = 3f, su = 1,  sw = 0, hv = 0,  yw = 0 },
            new Phase { name = "coast",     dur = 2f, su = 0,  sw = 0, hv = 0,  yw = 0 },
            new Phase { name = "yaw",       dur = 3f, su = 0,  sw = 0, hv = 0,  yw = 1 },
            new Phase { name = "heave_dn",  dur = 2f, su = 0,  sw = 0, hv = -1, yw = 0 },
            new Phase { name = "settle",    dur = 1f, su = 0,  sw = 0, hv = 0,  yw = 0 },
        };

        var oracles = new List<(string name, bool pass, string detail)>();
        float globalMaxStepDelta = 0f, globalMaxTilt = 0f;
        var summary = new StringBuilder();
        summary.AppendLine("# RovSim headless test — " + DateTime.UtcNow.ToString("o"));
        summary.AppendLine($"scene nq={nq} nv={nv} dt={Dt}");

        double simTime = 0;
        foreach (var ph in phases)
        {
            int steps = Mathf.RoundToInt(ph.dur / Dt);
            var csv = new StreamWriter(Path.Combine(LogDir, $"test_{ph.name}.csv"), false);
            csv.WriteLine("step,sim_time,pos_x,pos_y,pos_z,depth_m,heading_deg,pitch_deg,roll_deg," +
                          "vel_x,vel_y,vel_z,speed_mps,vspeed_mps,cmd_su,cmd_sw,cmd_hv,cmd_yw,fx,fy,fz,tx,ty,tz");

            Vector3 startPos = ReadVec(scene.Data->xpos, BaseBodyId);
            Vector3 startFwd = Frame(scene, out _, out _);
            float startHeading = Heading(scene), startSpeed = Speed(scene), startDepth = -startPos.z;
            float maxTilt = 0f, maxStepDelta = 0f; Vector3 prev = startPos;
            float headingAccum = 0f, lastHeading = startHeading;

            for (int s = 0; s < steps; s++)
            {
                Vector3 fwd = Frame(scene, out Vector3 stbd, out Vector3 up);
                Vector3 lin = ReadVel(scene, 0), ang = ReadVel(scene, 3);
                RovDynamics.ComputeWrench(fwd, stbd, up, lin, ang, ph.su, ph.sw, ph.hv, ph.yw, p,
                    out Vector3 force, out Vector3 torque);
                int off = BaseBodyId * 6;
                scene.Data->xfrc_applied[off + 0] = force.x; scene.Data->xfrc_applied[off + 1] = force.y;
                scene.Data->xfrc_applied[off + 2] = force.z; scene.Data->xfrc_applied[off + 3] = torque.x;
                scene.Data->xfrc_applied[off + 4] = torque.y; scene.Data->xfrc_applied[off + 5] = torque.z;
                scene.StepScene();
                simTime += Dt;

                Vector3 pos = ReadVec(scene.Data->xpos, BaseBodyId);
                float depth = -pos.z, heading = Heading(scene), speed = Speed(scene);
                float pitch, roll; Attitude(scene, out pitch, out roll);
                float tilt = Mathf.Max(Mathf.Abs(pitch), Mathf.Abs(roll));
                maxTilt = Mathf.Max(maxTilt, tilt);
                float stepDelta = (pos - prev).magnitude; maxStepDelta = Mathf.Max(maxStepDelta, stepDelta); prev = pos;
                float dh = Mathf.DeltaAngle(lastHeading, heading); headingAccum += dh; lastHeading = heading;

                var c = CultureInfo.InvariantCulture;
                csv.WriteLine(string.Join(",", new[] {
                    s.ToString(c), simTime.ToString("F3", c),
                    pos.x.ToString("F4", c), pos.y.ToString("F4", c), pos.z.ToString("F4", c),
                    depth.ToString("F4", c), heading.ToString("F2", c), pitch.ToString("F2", c), roll.ToString("F2", c),
                    lin.x.ToString("F4", c), lin.y.ToString("F4", c), lin.z.ToString("F4", c),
                    speed.ToString("F4", c), (lin.z).ToString("F4", c),
                    ph.su.ToString("F1", c), ph.sw.ToString("F1", c), ph.hv.ToString("F1", c), ph.yw.ToString("F1", c),
                    force.x.ToString("F2", c), force.y.ToString("F2", c), force.z.ToString("F2", c),
                    torque.x.ToString("F2", c), torque.y.ToString("F2", c), torque.z.ToString("F2", c)
                }));
            }
            csv.Flush(); csv.Close();

            Vector3 endPos = ReadVec(scene.Data->xpos, BaseBodyId);
            Vector3 disp = endPos - startPos;
            float fwdComp = Vector3.Dot(disp, startFwd);
            float endSpeed = Speed(scene), endDepth = -endPos.z;
            globalMaxStepDelta = Mathf.Max(globalMaxStepDelta, maxStepDelta);
            globalMaxTilt = Mathf.Max(globalMaxTilt, maxTilt);

            summary.AppendLine($"[{ph.name}] cmd=({ph.su},{ph.sw},{ph.hv},{ph.yw}) disp={disp.magnitude:F3}m fwdComp={fwdComp:F3}m " +
                               $"dHeading={headingAccum:F1}deg dDepth={(endDepth - startDepth):F3}m startSpd={startSpeed:F3} endSpd={endSpeed:F3} maxTilt={maxTilt:F1}deg maxStep={maxStepDelta:F4}m");

            if (ph.name == "surge")
            {
                bool ok = fwdComp > 0.3f && fwdComp > 0.7f * disp.magnitude;
                oracles.Add(("surge_forward", ok, $"fwdComp={fwdComp:F3}m (need >0.3 & aligned)"));
            }
            else if (ph.name == "coast")
            {
                bool ok = endSpeed < startSpeed - 0.02f;
                oracles.Add(("coast_decelerates", ok, $"{startSpeed:F3}->{endSpeed:F3} m/s (drag)"));
            }
            else if (ph.name == "yaw")
            {
                bool ok = Mathf.Abs(headingAccum) > 15f;
                oracles.Add(("yaw_turns", ok, $"dHeading={headingAccum:F1}deg (need >15)"));
            }
            else if (ph.name == "heave_dn")
            {
                bool ok = (endDepth - startDepth) > 0.2f;
                oracles.Add(("heave_descends", ok, $"dDepth={(endDepth - startDepth):F3}m (need >0.2)"));
            }
        }

        oracles.Add(("non_teleport", globalMaxStepDelta < 0.1f, $"maxStepDelta={globalMaxStepDelta:F4}m (need <0.1; teleport would jump)"));
        oracles.Add(("attitude_stable", globalMaxTilt < 25f, $"maxTilt={globalMaxTilt:F1}deg (need <25; self-righting)"));

        bool all = true;
        summary.AppendLine("\n# ORACLES");
        foreach (var o in oracles)
        {
            all &= o.pass;
            string line = $"  [{(o.pass ? "PASS" : "FAIL")}] {o.name}: {o.detail}";
            summary.AppendLine(line);
            Debug.Log("ROVTEST_ORACLE " + (o.pass ? "PASS" : "FAIL") + " " + o.name + " :: " + o.detail);
        }
        summary.AppendLine($"\nRESULT {(all ? "ALL_PASS" : "FAIL")}  ({oracles.FindAll(o => o.pass).Count}/{oracles.Count})");
        File.WriteAllText(Path.Combine(LogDir, "test_summary.txt"), summary.ToString());
        Debug.Log("ROVTEST_RESULT " + (all ? "ALL_PASS" : "FAIL") + " " + oracles.FindAll(o => o.pass).Count + "/" + oracles.Count);

        scene.DestroyScene();
        UnityEngine.Object.DestroyImmediate(scene.gameObject);
        UnityEngine.Object.DestroyImmediate(root);
        return all;
    }

    private static unsafe Vector3 ReadVec(double* arr, int bodyId)
    {
        double* a = arr + bodyId * 3;
        return new Vector3((float)a[0], (float)a[1], (float)a[2]);
    }

    private static unsafe Vector3 ReadVel(MjScene scene, int o)
    {
        return new Vector3((float)scene.Data->qvel[BaseDofAdr + o + 0],
                           (float)scene.Data->qvel[BaseDofAdr + o + 1],
                           (float)scene.Data->qvel[BaseDofAdr + o + 2]);
    }

    private static unsafe Vector3 Frame(MjScene scene, out Vector3 stbd, out Vector3 up)
    {
        double* m = scene.Data->xmat + BaseBodyId * 9;
        var fwd = new Vector3((float)m[0], (float)m[3], (float)m[6]).normalized;
        stbd = new Vector3((float)m[1], (float)m[4], (float)m[7]).normalized;
        up = new Vector3((float)m[2], (float)m[5], (float)m[8]).normalized;
        return fwd;
    }

    private static unsafe float Speed(MjScene scene) => ReadVel(scene, 0).magnitude;

    private static unsafe float Heading(MjScene scene)
    {
        double* q = scene.Data->xquat + BaseBodyId * 4;
        double w = q[0], x = q[1], y = q[2], z = q[3];
        float h = (float)(Math.Atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * 180.0 / Math.PI);
        h %= 360f; if (h < 0) h += 360f; return h;
    }

    private static unsafe void Attitude(MjScene scene, out float pitch, out float roll)
    {
        double* q = scene.Data->xquat + BaseBodyId * 4;
        double w = q[0], x = q[1], y = q[2], z = q[3];
        roll = (float)(Math.Atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * 180.0 / Math.PI);
        double sinp = 2.0 * (w * y - z * x);
        pitch = (float)(Math.Abs(sinp) >= 1 ? Math.Sign(sinp) * 90.0 : Math.Asin(sinp) * 180.0 / Math.PI);
    }
}
