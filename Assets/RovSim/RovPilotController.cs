using System;
using System.Globalization;
using System.IO;
using Mujoco;
using UnityEngine;

// v1 ROV/AUV pilot controller — 6-DOF vehicle control via MuJoCo xfrc_applied (force-integrated, NOT teleport).
// No manipulator arm (v1). Uses shared RovDynamics so runtime == headless-test math.
// Emits a full per-step telemetry CSV for the logging/test system.
public class RovPilotController : MonoBehaviour
{
    [Header("Thrust  (N / N·m)")]
    public float forceSurge = 72f;
    public float forceSway = 50f;
    public float forceHeave = 58f;
    public float torqueYaw = 11f;

    [Header("Hydrodynamics (anisotropic body-frame drag)")]
    public float dragSurgeLin = 30f, dragSurgeQuad = 40f;
    public float dragSwayLin = 120f, dragSwayQuad = 160f;
    public float dragHeaveLin = 120f, dragHeaveQuad = 160f;
    public float angularDrag = 26f;
    public float attitudeGain = 34f;

    [Header("HUD / logging")]
    public float surfaceZ = 0f;
    public bool drawHud = true;
    public bool logCsv = true;
    public string csvPath = "/mnt/storage/marus-rov-sim/logs/rov_session.csv";

    [Header("Headless metrics")]
    public bool logMetrics = false;
    public string metricsPath = "/mnt/storage/marus-rov-sim/rov_motion_log.txt";
    public float autoQuitSeconds = 0f;

    private MjScene _scene;
    private const int BaseBodyId = 1;
    private const int BaseDofAdr = 0;
    private bool _ready;

    private float _cSurge, _cSway, _cHeave, _cYaw;
    private Vector3 _startPos, _pos, _linVel, _angVel, _lastForce, _lastTorque;
    private float _headingDeg, _speed, _vSpeed, _depth, _rollDeg, _pitchDeg, _maxTranslation, _maxStepDelta;
    private int _step;
    private double _simTime;
    private readonly System.Diagnostics.Stopwatch _clock = new System.Diagnostics.Stopwatch();
    private StreamWriter _csv;
    private GUIStyle _box, _title, _val;

    private RovDynamics.Params P => new RovDynamics.Params
    {
        forceSurge = forceSurge, forceSway = forceSway, forceHeave = forceHeave, torqueYaw = torqueYaw,
        dragSurgeLin = dragSurgeLin, dragSurgeQuad = dragSurgeQuad,
        dragSwayLin = dragSwayLin, dragSwayQuad = dragSwayQuad,
        dragHeaveLin = dragHeaveLin, dragHeaveQuad = dragHeaveQuad,
        angularDrag = angularDrag, attitudeGain = attitudeGain
    };

    private void Start()
    {
        Time.fixedDeltaTime = 0.005f;
        _clock.Start();
        TryInit();
    }

    private void Update()
    {
        if (!_ready) { TryInit(); return; }
        ReadKeyboard();
        if (autoQuitSeconds > 0f && _clock.Elapsed.TotalSeconds >= autoQuitSeconds)
        {
            WriteMetrics("auto_quit");
            CloseCsv();
            Application.Quit();
        }
    }

    private unsafe void TryInit()
    {
        _scene = MjScene.Instance;
        if (_scene == null || _scene.Model == null || _scene.Data == null) return;
        if ((int)_scene.Model->nq < 7) return;
        _startPos = ReadPos();
        _ready = true;
        OpenCsv();
        Debug.Log("ROVPILOT_READY nq=" + (int)_scene.Model->nq + " nv=" + (int)_scene.Model->nv);
    }

    private void ReadKeyboard()
    {
        _cSurge = Axis(Input.GetKey(KeyCode.W), Input.GetKey(KeyCode.S));
        _cSway = Axis(Input.GetKey(KeyCode.D), Input.GetKey(KeyCode.A));
        _cHeave = Axis(Input.GetKey(KeyCode.R), Input.GetKey(KeyCode.F));
        _cYaw = Axis(Input.GetKey(KeyCode.E), Input.GetKey(KeyCode.Q));
    }

    private static float Axis(bool pos, bool neg) => (pos ? 1f : 0f) - (neg ? 1f : 0f);

    private unsafe void FixedUpdate()
    {
        if (!_ready) return;

        double* m = _scene.Data->xmat + BaseBodyId * 9;
        var fwd = new Vector3((float)m[0], (float)m[3], (float)m[6]).normalized;
        var stbd = new Vector3((float)m[1], (float)m[4], (float)m[7]).normalized;
        var bodyUp = new Vector3((float)m[2], (float)m[5], (float)m[8]).normalized;

        _linVel = new Vector3((float)_scene.Data->qvel[BaseDofAdr + 0],
                              (float)_scene.Data->qvel[BaseDofAdr + 1],
                              (float)_scene.Data->qvel[BaseDofAdr + 2]);
        _angVel = new Vector3((float)_scene.Data->qvel[BaseDofAdr + 3],
                              (float)_scene.Data->qvel[BaseDofAdr + 4],
                              (float)_scene.Data->qvel[BaseDofAdr + 5]);

        RovDynamics.ComputeWrench(fwd, stbd, bodyUp, _linVel, _angVel,
            _cSurge, _cSway, _cHeave, _cYaw, P, out var force, out var torque);
        _lastForce = force; _lastTorque = torque;

        int off = BaseBodyId * 6;
        _scene.Data->xfrc_applied[off + 0] = force.x;
        _scene.Data->xfrc_applied[off + 1] = force.y;
        _scene.Data->xfrc_applied[off + 2] = force.z;
        _scene.Data->xfrc_applied[off + 3] = torque.x;
        _scene.Data->xfrc_applied[off + 4] = torque.y;
        _scene.Data->xfrc_applied[off + 5] = torque.z;
        _simTime += Time.fixedDeltaTime;
    }

    private unsafe Vector3 ReadPos()
    {
        double* p = _scene.Data->xpos + BaseBodyId * 3;
        return new Vector3((float)p[0], (float)p[1], (float)p[2]);
    }

    private unsafe void LateUpdate()
    {
        if (!_ready) return;
        var prev = _pos;
        _pos = ReadPos();
        if (_step > 0) _maxStepDelta = Mathf.Max(_maxStepDelta, (_pos - prev).magnitude);
        _depth = surfaceZ - _pos.z;
        _speed = _linVel.magnitude;
        _vSpeed = _linVel.z;
        _maxTranslation = Mathf.Max(_maxTranslation, (_pos - _startPos).magnitude);

        double* q = _scene.Data->xquat + BaseBodyId * 4;
        double w = q[0], x = q[1], y = q[2], z = q[3];
        _headingDeg = NormalizeHeading((float)(Math.Atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * 180.0 / Math.PI));
        _rollDeg = (float)(Math.Atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * 180.0 / Math.PI);
        double sinp = 2.0 * (w * y - z * x);
        _pitchDeg = (float)(Math.Abs(sinp) >= 1 ? Math.Sign(sinp) * 90.0 : Math.Asin(sinp) * 180.0 / Math.PI);

        WriteCsvRow();
        _step++;
    }

    private static float NormalizeHeading(float h) { h %= 360f; if (h < 0f) h += 360f; return h; }

    private void OpenCsv()
    {
        if (!logCsv) return;
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(csvPath));
            _csv = new StreamWriter(csvPath, false);
            _csv.WriteLine("step,sim_time,fps,pos_x,pos_y,pos_z,depth_m,heading_deg,pitch_deg,roll_deg," +
                           "vel_x,vel_y,vel_z,speed_mps,vspeed_mps,angvel_x,angvel_y,angvel_z," +
                           "cmd_surge,cmd_sway,cmd_heave,cmd_yaw,force_x,force_y,force_z,torque_x,torque_y,torque_z");
        }
        catch (Exception ex) { Debug.LogError("ROVPILOT_CSV_OPEN_FAIL " + ex); _csv = null; }
    }

    private void WriteCsvRow()
    {
        if (_csv == null) return;
        var c = CultureInfo.InvariantCulture;
        float fps = Time.unscaledDeltaTime > 0f ? 1f / Time.unscaledDeltaTime : 0f;
        _csv.WriteLine(string.Join(",", new[]
        {
            _step.ToString(c), _simTime.ToString("F3", c), fps.ToString("F1", c),
            _pos.x.ToString("F4", c), _pos.y.ToString("F4", c), _pos.z.ToString("F4", c),
            _depth.ToString("F4", c), _headingDeg.ToString("F2", c), _pitchDeg.ToString("F2", c), _rollDeg.ToString("F2", c),
            _linVel.x.ToString("F4", c), _linVel.y.ToString("F4", c), _linVel.z.ToString("F4", c),
            _speed.ToString("F4", c), _vSpeed.ToString("F4", c),
            _angVel.x.ToString("F4", c), _angVel.y.ToString("F4", c), _angVel.z.ToString("F4", c),
            _cSurge.ToString("F2", c), _cSway.ToString("F2", c), _cHeave.ToString("F2", c), _cYaw.ToString("F2", c),
            _lastForce.x.ToString("F2", c), _lastForce.y.ToString("F2", c), _lastForce.z.ToString("F2", c),
            _lastTorque.x.ToString("F2", c), _lastTorque.y.ToString("F2", c), _lastTorque.z.ToString("F2", c)
        }));
        if ((_step & 31) == 0) _csv.Flush();
    }

    private void CloseCsv()
    {
        if (_csv == null) return;
        try { _csv.Flush(); _csv.Close(); } catch { }
        _csv = null;
        Debug.Log("ROVPILOT_CSV_CLOSED rows=" + _step + " path=" + csvPath);
    }

    private void OnApplicationQuit() { if (_ready) { WriteMetrics("app_quit"); CloseCsv(); } }
    private void OnDisable() { CloseCsv(); }

    private void OnGUI()
    {
        if (!drawHud || !_ready) return;
        if (_box == null)
        {
            _box = new GUIStyle(GUI.skin.box) { };
            _box.normal.background = Texture2D.grayTexture;
            _title = new GUIStyle(GUI.skin.label) { fontSize = 16, fontStyle = FontStyle.Bold };
            _title.normal.textColor = new Color(1f, 0.84f, 0.38f);
            _val = new GUIStyle(GUI.skin.label) { fontSize = 14 };
            _val.normal.textColor = new Color(0.82f, 0.98f, 0.90f);
        }
        GUI.Box(new Rect(14, 14, 250, 196), GUIContent.none, _box);
        GUI.Label(new Rect(26, 20, 230, 22), "ROV OPERATOR — v1", _title);
        int y = 48;
        Row(ref y, "DEPTH", _depth.ToString("F2") + " m");
        Row(ref y, "HEADING", _headingDeg.ToString("F0") + "°");
        Row(ref y, "PITCH / ROLL", _pitchDeg.ToString("F0") + "° / " + _rollDeg.ToString("F0") + "°");
        Row(ref y, "SPEED", _speed.ToString("F2") + " m/s");
        Row(ref y, "VERT SPEED", _vSpeed.ToString("F2") + " m/s");
        Row(ref y, "CMD S/Y", _cSurge.ToString("F1") + " / " + _cYaw.ToString("F1"));
        Row(ref y, "CMD SW/HV", _cSway.ToString("F1") + " / " + _cHeave.ToString("F1"));
        GUI.Box(new Rect(14, 220, 250, 24), GUIContent.none, _box);
        GUI.Label(new Rect(26, 222, 234, 22), "W/S surge  A/D sway  R/F heave  Q/E yaw", _val);
    }

    private void Row(ref int y, string k, string v)
    {
        GUI.Label(new Rect(26, y, 120, 20), k, _val);
        GUI.Label(new Rect(150, y, 100, 20), v, _val);
        y += 21;
    }

    private void WriteMetrics(string reason)
    {
        try
        {
            var c = CultureInfo.InvariantCulture;
            File.WriteAllText(metricsPath,
                "reason=" + reason + "\n" +
                "control=keyboard surge/sway/heave/yaw -> xfrc_applied[base]; base pose NEVER written directly\n" +
                "steps=" + _step + "\n" +
                "base_translation_m=" + _maxTranslation.ToString("R", c) + "\n" +
                "max_step_delta_m=" + _maxStepDelta.ToString("R", c) + "\n" +
                "final_depth_m=" + _depth.ToString("R", c) + "\n" +
                "final_heading_deg=" + _headingDeg.ToString("R", c) + "\n" +
                "control_works=" + (_maxTranslation > 0.05f ? "yes" : "no") + "\n" +
                "csv=" + csvPath + "\n");
            Debug.Log("ROVPILOT_METRICS base_translation_m=" + _maxTranslation.ToString("R", c) +
                      " control_works=" + (_maxTranslation > 0.05f ? "yes" : "no"));
        }
        catch (Exception ex) { Debug.LogError("ROVPILOT_METRICS_FAIL " + ex); }
    }
}
