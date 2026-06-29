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
// real physics oracles. Exit code 0 iff all oracles pass.
public static class RovSimTest
{
    private static string Mjcf {
        get {
            return Path.GetFullPath(Path.Combine(Application.dataPath, "..", "staging", "rov_v2.xml"));
        }
    }
    private static string LogDir => Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Logs"));
    private const int BaseBodyId = 1;
    private const int BaseDofAdr = 0;
    private const float Dt = 0.0025f;

    private struct Phase { public string name; public float dur; public float su, sw, hv, yw; }

    private class Res
    {
        public Vector3 disp;
        public float fwdComp, lateralComp, dHeading, dDepth, startSpeed, endSpeed, maxTilt, maxStepDelta, maxSpeed;
        public bool anyNaN;
    }

    public static void RunHeadless()
    {
        try { EditorApplication.Exit(Run() ? 0 : 1); }
        catch (Exception ex) { Debug.LogError("ROVTEST_EXCEPTION " + ex); EditorApplication.Exit(1); }
    }

    private static unsafe bool Run()
    {
        Directory.CreateDirectory(LogDir);
        Time.fixedDeltaTime = Dt;

        RovSimMjImportAssets.CleanForMjcf(Mjcf);
        var root = new MjImporterWithAssets().ImportFile(Mjcf);
        var scene = MjScene.Instance;
        scene.CreateScene();
        int nq = (int)scene.Model->nq, nv = (int)scene.Model->nv;
        Debug.Log($"ROVTEST_SCENE nq={nq} nv={nv} dt={Dt}");
        if (nq < 7) { Debug.LogError("ROVTEST_FAIL nq<7"); return false; }

        var p = RovDynamics.Params.Default;
        var phases = new[]
        {
            new Phase { name = "idle",     dur = 1f, su = 0,  sw = 0,  hv = 0,  yw = 0 },
            new Phase { name = "surge",    dur = 3f, su = 1,  sw = 0,  hv = 0,  yw = 0 },
            new Phase { name = "coast",    dur = 2f, su = 0,  sw = 0,  hv = 0,  yw = 0 },
            new Phase { name = "reverse",  dur = 3f, su = -1, sw = 0,  hv = 0,  yw = 0 },
            new Phase { name = "coast2",   dur = 2f, su = 0,  sw = 0,  hv = 0,  yw = 0 },
            new Phase { name = "sway",     dur = 3f, su = 0,  sw = 1,  hv = 0,  yw = 0 },
            new Phase { name = "coast3",   dur = 2f, su = 0,  sw = 0,  hv = 0,  yw = 0 },
            new Phase { name = "heave_dn", dur = 2f, su = 0,  sw = 0,  hv = -1, yw = 0 },
            new Phase { name = "heave_up", dur = 2f, su = 0,  sw = 0,  hv = 1,  yw = 0 },
            new Phase { name = "coast4",   dur = 2f, su = 0,  sw = 0,  hv = 0,  yw = 0 },
            new Phase { name = "yaw",      dur = 3f, su = 0,  sw = 0,  hv = 0,  yw = 1 },
            new Phase { name = "surgeyaw", dur = 3f, su = 1,  sw = 0,  hv = 0,  yw = 1 },
            new Phase { name = "settle",   dur = 1f, su = 0,  sw = 0,  hv = 0,  yw = 0 },
        };

        var R = new Dictionary<string, Res>();
        float gMaxStep = 0f, gMaxTilt = 0f, gMaxSpeed = 0f; bool gNaN = false;
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

            var r = new Res();
            Vector3 startPos = ReadVec(scene.Data->xpos, BaseBodyId);
            Vector3 startFwd = Frame(scene, out Vector3 startStbd, out _);
            float startHeading = Heading(scene); r.startSpeed = Speed(scene);
            float startDepth = -startPos.z; float lastHeading = startHeading; Vector3 prev = startPos;

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
                ClampBaseSpin(scene);   // mirror the player's angular-rate governor (stable arm-laden base)
                simTime += Dt;

                Vector3 pos = ReadVec(scene.Data->xpos, BaseBodyId);
                float depth = -pos.z, heading = Heading(scene), speed = Speed(scene);
                Attitude(scene, out float pitch, out float roll);
                if (Bad(pos) || Bad(lin) || float.IsNaN(speed)) { r.anyNaN = true; gNaN = true; }
                r.maxTilt = Mathf.Max(r.maxTilt, Mathf.Max(Mathf.Abs(pitch), Mathf.Abs(roll)));
                r.maxSpeed = Mathf.Max(r.maxSpeed, speed);
                float sd = (pos - prev).magnitude; r.maxStepDelta = Mathf.Max(r.maxStepDelta, sd); prev = pos;
                r.dHeading += Mathf.DeltaAngle(lastHeading, heading); lastHeading = heading;

                var c = CultureInfo.InvariantCulture;
                csv.WriteLine(string.Join(",", new[] {
                    s.ToString(c), simTime.ToString("F3", c),
                    pos.x.ToString("F4", c), pos.y.ToString("F4", c), pos.z.ToString("F4", c),
                    depth.ToString("F4", c), heading.ToString("F2", c), pitch.ToString("F2", c), roll.ToString("F2", c),
                    lin.x.ToString("F4", c), lin.y.ToString("F4", c), lin.z.ToString("F4", c),
                    speed.ToString("F4", c), lin.z.ToString("F4", c),
                    ph.su.ToString("F1", c), ph.sw.ToString("F1", c), ph.hv.ToString("F1", c), ph.yw.ToString("F1", c),
                    force.x.ToString("F2", c), force.y.ToString("F2", c), force.z.ToString("F2", c),
                    torque.x.ToString("F2", c), torque.y.ToString("F2", c), torque.z.ToString("F2", c)
                }));
            }
            csv.Flush(); csv.Close();

            Vector3 endPos = ReadVec(scene.Data->xpos, BaseBodyId);
            r.disp = endPos - startPos;
            r.fwdComp = Vector3.Dot(r.disp, startFwd);
            r.lateralComp = (r.disp - r.fwdComp * startFwd).magnitude;
            r.endSpeed = Speed(scene); r.dDepth = (-endPos.z) - startDepth;
            R[ph.name] = r;
            gMaxStep = Mathf.Max(gMaxStep, r.maxStepDelta); gMaxTilt = Mathf.Max(gMaxTilt, r.maxTilt);
            gMaxSpeed = Mathf.Max(gMaxSpeed, r.maxSpeed);
            summary.AppendLine($"[{ph.name}] cmd=({ph.su},{ph.sw},{ph.hv},{ph.yw}) disp={r.disp.magnitude:F3} fwd={r.fwdComp:F3} lat={r.lateralComp:F3} " +
                               $"dHead={r.dHeading:F1} dDepth={r.dDepth:F3} spd {r.startSpeed:F3}->{r.endSpeed:F3} maxTilt={r.maxTilt:F2} maxStep={r.maxStepDelta:F4}");
        }

        // ---- simulated joystick: drive via the exact mapping a real Logitech would use ----
        var joyCmd = RovInput.MapFlightStick(0f, -1f, 0f, 0f); // simulate: flight stick pushed forward
        Vector3 jStart = ReadVec(scene.Data->xpos, BaseBodyId);
        Vector3 jFwd0 = Frame(scene, out _, out _);
        for (int s = 0; s < Mathf.RoundToInt(2.5f / Dt); s++)
        {
            Vector3 fwd = Frame(scene, out Vector3 stbd, out Vector3 up);
            Vector3 lin = ReadVel(scene, 0), ang = ReadVel(scene, 3);
            RovDynamics.ComputeWrench(fwd, stbd, up, lin, ang, joyCmd.surge, joyCmd.sway, joyCmd.heave, joyCmd.yaw, p,
                out Vector3 force, out Vector3 torque);
            int off = BaseBodyId * 6;
            scene.Data->xfrc_applied[off + 0] = force.x; scene.Data->xfrc_applied[off + 1] = force.y;
            scene.Data->xfrc_applied[off + 2] = force.z; scene.Data->xfrc_applied[off + 3] = torque.x;
            scene.Data->xfrc_applied[off + 4] = torque.y; scene.Data->xfrc_applied[off + 5] = torque.z;
            scene.StepScene();
            ClampBaseSpin(scene);
        }
        float joyFwd = Vector3.Dot(ReadVec(scene.Data->xpos, BaseBodyId) - jStart, jFwd0);
        Debug.Log("ROVTEST_STAGE base+joystick phases complete OK");

        // ---- manipulator arm (v2): command poses with the base station-kept (thrusters hold the ROV) ----
        ArmTest(scene, out float armReachErr, out float jawOpenGap, out float jawCloseGap,
                out float eeMove, out float armAzi, out float armSh, out bool armNan);
        Debug.Log("ROVTEST_STAGE arm test complete OK reachErr=" + armReachErr + " eeMove=" + eeMove);

        var oracles = new List<(string, bool, string)>();
        void O(string n, bool pass, string d) => oracles.Add((n, pass, d));

        O("idle_stationary", R["idle"].disp.magnitude < 1e-3f, $"disp={R["idle"].disp.magnitude:F5}m (rest, deterministic)");
        O("surge_forward", R["surge"].fwdComp > 0.3f && R["surge"].fwdComp > 0.7f * R["surge"].disp.magnitude, $"fwd={R["surge"].fwdComp:F3}m");
        O("reverse_backward", R["reverse"].fwdComp < -0.3f, $"fwd={R["reverse"].fwdComp:F3}m (backwards)");
        O("sway_lateral", R["sway"].lateralComp > 0.15f, $"lat={R["sway"].lateralComp:F3}m");
        O("coast_decelerates", R["coast"].endSpeed < R["coast"].startSpeed - 0.02f, $"{R["coast"].startSpeed:F3}->{R["coast"].endSpeed:F3} m/s");
        O("heave_descends", R["heave_dn"].dDepth > 0.2f, $"dDepth={R["heave_dn"].dDepth:F3}m");
        O("heave_ascends", R["heave_up"].dDepth < -0.2f, $"dDepth={R["heave_up"].dDepth:F3}m");
        O("yaw_turns", Mathf.Abs(R["yaw"].dHeading) > 15f, $"dHead={R["yaw"].dHeading:F1}deg");
        O("surgeyaw_curves", Mathf.Abs(R["surgeyaw"].dHeading) > 15f && R["surgeyaw"].lateralComp > 0.1f,
            $"dHead={R["surgeyaw"].dHeading:F1}deg lat={R["surgeyaw"].lateralComp:F3}m (surge follows heading => curved)");
        // anisotropy: forward is the least-damped axis -> highest terminal speed
        float vSurge = R["surge"].endSpeed, vSway = R["sway"].endSpeed, vHeave = R["heave_dn"].endSpeed;
        O("anisotropy_surge_fastest", vSurge > vSway + 0.1f && vSurge > vHeave + 0.1f,
            $"terminal surge={vSurge:F3} > sway={vSway:F3}, heave={vHeave:F3} m/s (ROV-like)");
        O("cross_coupling_low", R["surge"].lateralComp < 0.05f * Mathf.Max(R["surge"].disp.magnitude, 1e-3f) && R["surge"].maxTilt < 2f,
            $"surge lat={R["surge"].lateralComp:F4}m tilt={R["surge"].maxTilt:F2}deg");
        O("non_teleport", gMaxStep < 0.1f, $"maxStep={gMaxStep:F4}m (<0.1; teleport would jump)");
        O("attitude_stable", gMaxTilt < 25f, $"maxTilt={gMaxTilt:F2}deg");
        O("no_nan_bounded", !gNaN && gMaxSpeed < 5f, $"nan={gNaN} maxSpeed={gMaxSpeed:F3} m/s");

        // simulated-joystick (Logitech) mapping oracles
        var jf = RovInput.MapFlightStick(0f, -1f, 0f, 0f);
        O("joy_stick_fwd_surge", jf.surge > 0.9f && Mathf.Abs(jf.sway) < 0.01f, $"stick fwd -> surge={jf.surge:F2}");
        var jt = RovInput.MapFlightStick(0f, 0f, 1f, 0f);
        O("joy_stick_twist_yaw", jt.yaw > 0.9f, $"twist -> yaw={jt.yaw:F2}");
        var jh = RovInput.MapFlightStick(0f, 0f, 0f, 1f);
        O("joy_stick_throttle_heave", jh.heave > 0.9f, $"throttle -> heave={jh.heave:F2}");
        var jg = RovInput.MapGamepad(new Vector2(0f, 1f), Vector2.zero, 0f, 0f);
        O("joy_gamepad_surge", jg.surge > 0.9f, $"L-stick up -> surge={jg.surge:F2}");
        var jgy = RovInput.MapGamepad(Vector2.zero, new Vector2(1f, 0f), 0f, 0f);
        O("joy_gamepad_yaw", jgy.yaw > 0.9f, $"R-stick X -> yaw={jgy.yaw:F2}");
        O("joy_drive_forward", joyFwd > 0.3f, $"sim-joystick stick-fwd -> {joyFwd:F3}m vehicle motion");

        // ---- manipulator arm oracles (v2) ----
        O("arm_reaches_pose", armReachErr < 0.06f, $"max joint err={armReachErr:F4} rad (position servo converges)");
        O("arm_jaw_opens_closes", jawOpenGap > 0.8f && jawCloseGap < 0.15f, $"jaw gap open={jawOpenGap:F3} -> close={jawCloseGap:F3} rad");
        O("arm_ee_moves", eeMove > 0.1f, $"end-effector moved {eeMove:F3}m between commanded poses");
        O("arm_joint_limits", armAzi <= 1.41f && armSh <= 1.21f, $"over-command clamps: azi={armAzi:F3}(<=1.4) shoulder={armSh:F3}(<=1.2)");
        O("arm_no_nan", !armNan, $"arm finite (nan={armNan})");

        bool all = true;
        summary.AppendLine("\n# ORACLES");
        foreach (var (n, pass, d) in oracles)
        {
            all &= pass;
            summary.AppendLine($"  [{(pass ? "PASS" : "FAIL")}] {n}: {d}");
            Debug.Log("ROVTEST_ORACLE " + (pass ? "PASS" : "FAIL") + " " + n + " :: " + d);
        }
        int np = oracles.FindAll(o => o.Item2).Count;
        summary.AppendLine($"\nRESULT {(all ? "ALL_PASS" : "FAIL")}  ({np}/{oracles.Count})");
        File.WriteAllText(Path.Combine(LogDir, "test_summary.txt"), summary.ToString());
        Debug.Log("ROVTEST_RESULT " + (all ? "ALL_PASS" : "FAIL") + " " + np + "/" + oracles.Count);

        scene.DestroyScene();
        UnityEngine.Object.DestroyImmediate(scene.gameObject);
        UnityEngine.Object.DestroyImmediate(root);
        return all;
    }

    private static bool Bad(Vector3 v) => float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z) || float.IsInfinity(v.x);

    // mirror RovPilotController's angular-rate governor (roll/pitch 0.2, yaw 0.8 rad/s) so the
    // oracle runs the SAME control the vehicle does -> the arm-laden free base stays stable.
    private static unsafe void ClampBaseSpin(MjScene scene)
    {
        int r = BaseDofAdr + 3, p = BaseDofAdr + 4, y = BaseDofAdr + 5;
        if (scene.Data->qvel[r] > 0.2) scene.Data->qvel[r] = 0.2; else if (scene.Data->qvel[r] < -0.2) scene.Data->qvel[r] = -0.2;
        if (scene.Data->qvel[p] > 0.2) scene.Data->qvel[p] = 0.2; else if (scene.Data->qvel[p] < -0.2) scene.Data->qvel[p] = -0.2;
        if (scene.Data->qvel[y] > 0.8) scene.Data->qvel[y] = 0.8; else if (scene.Data->qvel[y] < -0.8) scene.Data->qvel[y] = -0.8;
    }

    // Manipulator-arm oracle scenario. Commands arm poses via the position actuators (ctrl[0..5]) while
    // the base is station-kept (thrusters hold the ROV during intervention). Bodies: 1=base_link,
    // 5=arm_wrist (end-effector proxy). qpos: [7]azimuth [8]shoulder [9]elbow [10]wrist [11]jawL [12]jawR.
    private static unsafe void ArmTest(MjScene scene, out float reachErr, out float jawOpen, out float jawClose,
                                       out float eeMove, out float azi, out float sh, out bool nan)
    {
        const int WristBody = 5;
        // Drive the arm to a target ctrl while the base station-keeps. ctrl is RE-APPLIED every step
        // (org.mujoco's per-step SyncUnityToMjState resyncs MjActuator.Control -> Data->ctrl, which would
        // otherwise clobber a one-time write), and the base angular rate is governed every step.
        void Drive(double[] ctrl, int n)
        {
            int o = BaseBodyId * 6;
            for (int s = 0; s < n; s++)
            {
                for (int j = 0; j < 6; j++) scene.Data->ctrl[j] = ctrl[j];
                for (int k = 0; k < 3; k++) scene.Data->xfrc_applied[o + k] = -40.0 * scene.Data->qvel[BaseDofAdr + k];
                for (int k = 3; k < 6; k++) scene.Data->xfrc_applied[o + k] = -26.0 * scene.Data->qvel[BaseDofAdr + k];
                scene.StepScene();
                ClampBaseSpin(scene);
            }
        }
        // vehicle comes to rest and station-keeps before intervention (clean precondition for arm kinematics)
        for (int k = 0; k < 12; k++) scene.Data->qvel[k] = 0.0;
        Drive(new double[] { 0, 0, 0, 0, 0, 0 }, 200);
        double[] tgt = { 0.6, 0.5, -1.2, 0.4, 0.6, -0.6 };
        Drive(tgt, 2400);
        reachErr = 0f; for (int i = 0; i < 6; i++) reachErr = Mathf.Max(reachErr, Mathf.Abs((float)scene.Data->qpos[7 + i] - (float)tgt[i]));
        jawOpen = Mathf.Abs((float)scene.Data->qpos[11] - (float)scene.Data->qpos[12]);
        Vector3 ee0 = ReadVec(scene.Data->xpos, WristBody);
        Drive(new double[] { 0.6, 0.5, -1.2, 0.4, 0.0, 0.0 }, 1400);
        jawClose = Mathf.Abs((float)scene.Data->qpos[11] - (float)scene.Data->qpos[12]);
        Drive(new double[] { -0.6, -0.5, -0.3, -0.4, 0.0, 0.0 }, 2400);
        eeMove = (ReadVec(scene.Data->xpos, WristBody) - ee0).magnitude;
        Drive(new double[] { 5, 5, 5, 5, 5, 5 }, 1400);
        azi = (float)scene.Data->qpos[7]; sh = (float)scene.Data->qpos[8];
        nan = float.IsNaN(azi) || float.IsNaN(reachErr) || float.IsNaN(eeMove);
        for (int k = 0; k < 6; k++) scene.Data->ctrl[k] = 0.0;
    }

    private static unsafe Vector3 ReadVec(double* arr, int bodyId)
    { double* a = arr + bodyId * 3; return new Vector3((float)a[0], (float)a[1], (float)a[2]); }

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
