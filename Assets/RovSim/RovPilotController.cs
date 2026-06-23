using System;
using System.Globalization;
using System.IO;
using System.Linq;
using Mujoco;
using UnityEngine;

// v1 ROV/AUV pilot controller — 6-DOF vehicle control via MuJoCo xfrc_applied (force-integrated, NOT teleport).
// Operator-sim features: forward sonar, altimeter, guided mission + scoring, PiP camera. Full telemetry CSV.
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

    [Header("Sensors / mission")]
    public Vector3 worksite = new Vector3(5f, 2.5f, -3.6f);   // worksite waypoint in MuJoCo world (z up)
    public Camera pipCamera;
    public RenderTexture pipRT;

    [Header("Headless metrics")]
    public bool logMetrics = false;
    public string metricsPath = "/mnt/storage/marus-rov-sim/rov_motion_log.txt";
    public float autoQuitSeconds = 0f;

    private MjScene _scene;
    private const int BaseBodyId = 1;
    private const int BaseDofAdr = 0;
    private bool _ready;
    private Transform _baseLink;

    private float _cSurge, _cSway, _cHeave, _cYaw;
    private Vector3 _startPos, _pos, _linVel, _angVel, _lastForce, _lastTorque, _fwd, _stbd;
    private float _headingDeg, _speed, _vSpeed, _depth, _rollDeg, _pitchDeg, _maxTranslation, _maxStepDelta;
    private int _step;
    private double _simTime;
    private readonly System.Diagnostics.Stopwatch _clock = new System.Diagnostics.Stopwatch();
    private StreamWriter _csv;
    private GUIStyle _box, _title, _val;
    private RovInput.Source _inputSrc;
    private string _inputDevice = "keyboard";

    // sensors
    private const int SonarRays = 64;
    private const float SonarRange = 18f;
    private readonly float[] _sonar = new float[SonarRays];
    private int _sonarHits;
    private float _altitude = -1f;
    private readonly System.Random _noise = new System.Random(20260623);
    private Texture2D _px;

    // mission
    private enum MState { Descend, Transit, Hold, Complete }
    private MState _mstate = MState.Descend;
    private float _holdTimer, _missionTime, _stationErrAccum; private int _stationN;
    private bool _logDescend, _logTransit, _logHold, _logComplete;
    private float _distH, _score;

    private RovDynamics.Params P => new RovDynamics.Params
    {
        forceSurge = forceSurge, forceSway = forceSway, forceHeave = forceHeave, torqueYaw = torqueYaw,
        dragSurgeLin = dragSurgeLin, dragSurgeQuad = dragSurgeQuad,
        dragSwayLin = dragSwayLin, dragSwayQuad = dragSwayQuad,
        dragHeaveLin = dragHeaveLin, dragHeaveQuad = dragHeaveQuad,
        angularDrag = angularDrag, attitudeGain = attitudeGain
    };

    // offline recording (player framebuffer, no desktop overlays)
    private bool _recording;
    private string _recordDir;
    private int _recordFrame, _recordTotal;

    private void Start()
    {
        Time.fixedDeltaTime = 0.005f;
        _clock.Start();
        _px = Texture2D.whiteTexture;
        _recordDir = Environment.GetEnvironmentVariable("ROVSIM_RECORD");
        if (!string.IsNullOrEmpty(_recordDir))
        {
            _recording = true;
            Directory.CreateDirectory(_recordDir);
            Time.captureFramerate = 30;
            float secs = 38f;
            if (float.TryParse(Environment.GetEnvironmentVariable("ROVSIM_RECORD_SECONDS"), out var s) && s > 1f) secs = s;
            _recordTotal = Mathf.RoundToInt(secs * 30f);
        }
        TryInit();
    }

    private void Update()
    {
        if (!_ready) { TryInit(); return; }
        ReadInput();
        if (autoQuitSeconds > 0f && _clock.Elapsed.TotalSeconds >= autoQuitSeconds)
        {
            WriteMetrics("auto_quit"); CloseCsv(); Application.Quit();
        }
    }

    private unsafe void TryInit()
    {
        _scene = MjScene.Instance;
        if (_scene == null || _scene.Model == null || _scene.Data == null) return;
        if ((int)_scene.Model->nq < 7) return;
        _baseLink = GetComponentsInChildren<Transform>(true)
            .FirstOrDefault(t => t.name.IndexOf("base_link", StringComparison.OrdinalIgnoreCase) >= 0) ?? transform;
        _startPos = ReadPos();
        _ready = true;
        OpenCsv();
        Debug.Log("ROVPILOT_READY nq=" + (int)_scene.Model->nq + " nv=" + (int)_scene.Model->nv +
                  " baseLink=" + _baseLink.name + " worksite=" + worksite);
        Debug.Log("FRAMECHECK unityPos=" + _baseLink.position.ToString("F2") + " mjPos=" + ReadPos().ToString("F2") +
                  " unityFwd=" + _baseLink.forward.ToString("F2"));
    }

    private void ReadInput()
    {
        if (_recording) { _inputSrc = RovInput.Source.Keyboard; _inputDevice = "autopilot"; Guidance(); return; }
        var c = RovInput.Read(out _inputSrc, out _inputDevice);
        _cSurge = c.surge; _cSway = c.sway; _cHeave = c.heave; _cYaw = c.yaw;
    }

    // scripted mission autopilot for the demo recording: descend, transit to the worksite, hold.
    // Closed-loop holonomic homing in the MuJoCo world frame (z up) using the same body axes the
    // thrust is applied along (_fwd/_stbd from xmat), so surge+sway drive straight at the waypoint
    // regardless of heading. Working in one frame avoids the Unity<->MuJoCo axis mismatch.
    private void Guidance()
    {
        _cSurge = _cSway = _cHeave = _cYaw = 0f;
        if (!_ready) return;
        float depth = -_pos.z;                       // MuJoCo z up: deeper = more negative
        float targetDepth = -worksite.z;             // 3.6 m
        if (depth < targetDepth - 0.15f) _cHeave = -1f;        // go deeper
        else if (depth > targetDepth + 0.15f) _cHeave = 0.5f;  // rise

        if (_mstate == MState.Complete) return;
        Vector3 e = worksite - _pos; e.z = 0f;       // horizontal error in MuJoCo XY
        float dist = e.magnitude;
        if (dist < 0.05f) return;
        Vector3 fwdH = _fwd; fwdH.z = 0f; fwdH.Normalize();
        Vector3 stbdH = _stbd; stbdH.z = 0f; stbdH.Normalize();
        float drive = Mathf.Clamp01(dist / 2.5f);    // full thrust far out, taper within 2.5 m
        _cSurge = Mathf.Clamp(Vector3.Dot(e.normalized, fwdH) * drive, -1f, 1f);
        _cSway = Mathf.Clamp(Vector3.Dot(e.normalized, stbdH) * drive, -1f, 1f);
    }

    private unsafe void FixedUpdate()
    {
        if (!_ready) return;
        double* m = _scene.Data->xmat + BaseBodyId * 9;
        var fwd = new Vector3((float)m[0], (float)m[3], (float)m[6]).normalized;
        var stbd = new Vector3((float)m[1], (float)m[4], (float)m[7]).normalized;
        var bodyUp = new Vector3((float)m[2], (float)m[5], (float)m[8]).normalized;
        _fwd = fwd; _stbd = stbd;
        _linVel = new Vector3((float)_scene.Data->qvel[BaseDofAdr + 0], (float)_scene.Data->qvel[BaseDofAdr + 1], (float)_scene.Data->qvel[BaseDofAdr + 2]);
        _angVel = new Vector3((float)_scene.Data->qvel[BaseDofAdr + 3], (float)_scene.Data->qvel[BaseDofAdr + 4], (float)_scene.Data->qvel[BaseDofAdr + 5]);
        RovDynamics.ComputeWrench(fwd, stbd, bodyUp, _linVel, _angVel, _cSurge, _cSway, _cHeave, _cYaw, P, out var force, out var torque);
        _lastForce = force; _lastTorque = torque;
        int off = BaseBodyId * 6;
        _scene.Data->xfrc_applied[off + 0] = force.x; _scene.Data->xfrc_applied[off + 1] = force.y; _scene.Data->xfrc_applied[off + 2] = force.z;
        _scene.Data->xfrc_applied[off + 3] = torque.x; _scene.Data->xfrc_applied[off + 4] = torque.y; _scene.Data->xfrc_applied[off + 5] = torque.z;
        _simTime += Time.fixedDeltaTime;
    }

    private unsafe Vector3 ReadPos() { double* p = _scene.Data->xpos + BaseBodyId * 3; return new Vector3((float)p[0], (float)p[1], (float)p[2]); }

    private unsafe void LateUpdate()
    {
        if (!_ready) return;
        var prev = _pos;
        _pos = ReadPos();
        if (_step > 0) _maxStepDelta = Mathf.Max(_maxStepDelta, (_pos - prev).magnitude);
        _depth = surfaceZ - _pos.z;
        _speed = _linVel.magnitude; _vSpeed = _linVel.z;
        _maxTranslation = Mathf.Max(_maxTranslation, (_pos - _startPos).magnitude);
        double* q = _scene.Data->xquat + BaseBodyId * 4;
        double w = q[0], x = q[1], y = q[2], z = q[3];
        _headingDeg = NormalizeHeading((float)(Math.Atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * 180.0 / Math.PI));
        _rollDeg = (float)(Math.Atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * 180.0 / Math.PI);
        double sinp = 2.0 * (w * y - z * x);
        _pitchDeg = (float)(Math.Abs(sinp) >= 1 ? Math.Sign(sinp) * 90.0 : Math.Asin(sinp) * 180.0 / Math.PI);

        UpdateSensors();
        UpdateMission();
        WriteCsvRow();
        _step++;

        if (_recording)
        {
            ScreenCapture.CaptureScreenshot($"{_recordDir}/frame_{_recordFrame:00000}.png");
            _recordFrame++;
            if (_recordFrame >= _recordTotal) { WriteMetrics("record_done"); CloseCsv(); Application.Quit(); }
        }
    }

    private void UpdateSensors()
    {
        if (_baseLink == null) return;
        Vector3 org = _baseLink.position;
        // altimeter (downward to seabed)
        _altitude = Physics.Raycast(org, Vector3.down, out var ha, 60f) ? ha.distance : -1f;
        // forward sonar fan (horizontal sweep around heading)
        Vector3 hf = Vector3.ProjectOnPlane(_baseLink.forward, Vector3.up);
        if (hf.sqrMagnitude < 0.01f) hf = Vector3.ProjectOnPlane(_baseLink.right, Vector3.up);
        if (hf.sqrMagnitude < 0.01f) hf = Vector3.forward;
        hf.Normalize();
        _sonarHits = 0;
        for (int i = 0; i < SonarRays; i++)
        {
            float a = Mathf.Lerp(-55f, 55f, i / (float)(SonarRays - 1));
            Vector3 d = Quaternion.AngleAxis(a, Vector3.up) * hf;
            if (Physics.Raycast(org, d, out var h, SonarRange))
            { _sonar[i] = Mathf.Max(0.2f, h.distance + (float)(_noise.NextDouble() - 0.5) * 0.18f); _sonarHits++; }
            else _sonar[i] = 0f;
        }
    }

    private void UpdateMission()
    {
        if (!_ready) return;
        float depth = -_pos.z;                                  // MuJoCo z up
        Vector3 toWp = worksite - _pos; toWp.z = 0f; _distH = toWp.magnitude;
        _missionTime += Time.deltaTime;
        switch (_mstate)
        {
            case MState.Descend:
                if (!_logDescend) { Debug.Log("MISSION step=DESCEND start depth=" + depth.ToString("F2")); _logDescend = true; }
                if (depth >= 3.0f) { _mstate = MState.Transit; if (!_logTransit) { Debug.Log("MISSION step=DESCEND->TRANSIT t=" + _missionTime.ToString("F1") + " depth=" + depth.ToString("F2")); _logTransit = true; } }
                break;
            case MState.Transit:
                if (_distH <= 3.0f) { _mstate = MState.Hold; _holdTimer = 0f; if (!_logHold) { Debug.Log("MISSION step=TRANSIT->HOLD t=" + _missionTime.ToString("F1") + " dist=" + _distH.ToString("F2")); _logHold = true; } }
                break;
            case MState.Hold:
                _stationErrAccum += _distH; _stationN++;
                _holdTimer += _distH <= 3.5f ? Time.deltaTime : -Time.deltaTime;
                _holdTimer = Mathf.Max(0f, _holdTimer);
                if (_holdTimer >= 8f) { _mstate = MState.Complete; }
                break;
        }
        float avgErr = _stationN > 0 ? _stationErrAccum / _stationN : 0f;
        if (_mstate != MState.Complete)
            _score = Mathf.Clamp(100f - _missionTime * 1.2f - avgErr * 12f, 0f, 100f);  // freeze at completion
        if (_mstate == MState.Complete && !_logComplete)
        { Debug.Log("MISSION step=HOLD->COMPLETE time=" + _missionTime.ToString("F1") + " avgStationErr=" + avgErr.ToString("F2") + " score=" + _score.ToString("F0")); _logComplete = true; }
    }

    private static float NormalizeHeading(float h) { h %= 360f; if (h < 0f) h += 360f; return h; }

    // ---------- HUD ----------
    private void OnGUI()
    {
        if (!drawHud || !_ready) return;
        if (_box == null)
        {
            _box = new GUIStyle(GUI.skin.box); _box.normal.background = Texture2D.grayTexture;
            _title = new GUIStyle(GUI.skin.label) { fontSize = 16, fontStyle = FontStyle.Bold }; _title.normal.textColor = new Color(1f, 0.84f, 0.38f);
            _val = new GUIStyle(GUI.skin.label) { fontSize = 14 }; _val.normal.textColor = new Color(0.82f, 0.98f, 0.90f);
        }
        // telemetry panel
        GUI.Box(new Rect(14, 14, 254, 242), GUIContent.none, _box);
        GUI.Label(new Rect(26, 20, 234, 22), "ROV OPERATOR — v1", _title);
        int y = 48;
        Row(ref y, "INPUT", _inputSrc + " (" + _inputDevice + ")");
        Row(ref y, "DEPTH", _depth.ToString("F2") + " m");
        Row(ref y, "ALTITUDE", _altitude >= 0f ? _altitude.ToString("F2") + " m" : "--");
        Row(ref y, "HEADING", _headingDeg.ToString("F0") + "°");
        Row(ref y, "PITCH / ROLL", _pitchDeg.ToString("F0") + "° / " + _rollDeg.ToString("F0") + "°");
        Row(ref y, "SPEED", _speed.ToString("F2") + " m/s");
        Row(ref y, "VERT SPEED", _vSpeed.ToString("F2") + " m/s");
        Row(ref y, "CMD S/Y", _cSurge.ToString("F1") + " / " + _cYaw.ToString("F1"));
        Row(ref y, "CMD SW/HV", _cSway.ToString("F1") + " / " + _cHeave.ToString("F1"));
        GUI.Box(new Rect(14, 264, 254, 24), GUIContent.none, _box);
        GUI.Label(new Rect(26, 266, 238, 22), "Joystick  or  W/S A/D R/F Q/E", _val);

        DrawMission();
        DrawSonar();
        DrawPip();
    }

    private void Row(ref int y, string k, string v)
    {
        GUI.Label(new Rect(26, y, 120, 20), k, _val);
        GUI.Label(new Rect(150, y, 110, 20), v, _val);
        y += 21;
    }

    private void DrawMission()
    {
        float W = 256, H = 150; float X = Screen.width - W - 14, Y = 14;
        GUI.Box(new Rect(X, Y, W, H), GUIContent.none, _box);
        GUI.Label(new Rect(X + 12, Y + 6, W - 20, 22), "MISSION — inspect wellhead", _title);
        int my = (int)Y + 34;
        Chk(X + 14, ref my, _mstate > MState.Descend, "1. Descend below 3 m");
        Chk(X + 14, ref my, _mstate > MState.Transit, "2. Transit to worksite (<3 m)");
        Chk(X + 14, ref my, _mstate == MState.Complete, "3. Hold station 8 s");
        GUI.Label(new Rect(X + 14, my + 4, W - 20, 20), "RANGE  " + _distH.ToString("F1") + " m    STATE  " + _mstate, _val);
        var sc = new GUIStyle(_title) { fontSize = 18 };
        sc.normal.textColor = _mstate == MState.Complete ? new Color(0.5f, 1f, 0.6f) : new Color(1f, 0.84f, 0.38f);
        GUI.Label(new Rect(X + 14, my + 26, W - 20, 24), "SCORE  " + _score.ToString("F0") + " / 100" + (_mstate == MState.Complete ? "  ✓ COMPLETE" : ""), sc);
    }

    private void Chk(float x, ref int y, bool done, string label)
    {
        var c = done ? new Color(0.5f, 1f, 0.6f) : new Color(0.6f, 0.7f, 0.7f);
        var st = new GUIStyle(_val); st.normal.textColor = c;
        GUI.Label(new Rect(x, y, 240, 20), (done ? "☑ " : "☐ ") + label, st);
        y += 20;
    }

    private void DrawSonar()
    {
        float S = 200f; float cx = Screen.width - S / 2f - 24f; float cy = Screen.height - S / 2f - 24f;
        GUI.Box(new Rect(cx - S / 2f, cy - S / 2f, S, S), GUIContent.none, _box);
        GUI.Label(new Rect(cx - S / 2f + 8, cy - S / 2f + 4, S, 20), "FLS SONAR", _val);
        Vector2 apex = new Vector2(cx, cy + S / 2f - 14f);
        // range rings
        for (int r = 1; r <= 3; r++) { } // (kept simple; bars below convey range)
        for (int i = 0; i < SonarRays; i++)
        {
            float frac = _sonar[i] <= 0f ? 1f : Mathf.Clamp01(_sonar[i] / SonarRange);
            float ang = Mathf.Lerp(-55f, 55f, i / (float)(SonarRays - 1));
            float len = (S - 30f) * frac;
            var dir = new Vector2(Mathf.Sin(ang * Mathf.Deg2Rad), -Mathf.Cos(ang * Mathf.Deg2Rad));
            var col = _sonar[i] <= 0f ? new Color(0.1f, 0.3f, 0.3f, 0.25f) : Color.Lerp(new Color(0.4f, 1f, 0.5f), new Color(1f, 0.9f, 0.3f), 1f - frac);
            DrawLine(apex, apex + dir * len, col, _sonar[i] <= 0f ? 1f : 2.2f);
        }
    }

    private void DrawPip()
    {
        if (pipRT == null) return;
        float W = 256f, H = 144f; float X = Screen.width - W - 14f, Y = 172f;
        GUI.Box(new Rect(X - 2, Y - 2, W + 4, H + 22), GUIContent.none, _box);
        GUI.Label(new Rect(X, Y - 2, W, 18), "INSPECTION CAM", _val);
        GUI.DrawTexture(new Rect(X, Y + 16, W, H), pipRT, ScaleMode.ScaleToFit, false);
    }

    private void DrawLine(Vector2 a, Vector2 b, Color col, float width)
    {
        var d = b - a; float len = d.magnitude; if (len < 0.5f) return;
        float ang = Mathf.Atan2(d.y, d.x) * Mathf.Rad2Deg;
        var prev = GUI.color; var m = GUI.matrix;
        GUI.color = col;
        GUIUtility.RotateAroundPivot(ang, a);
        GUI.DrawTexture(new Rect(a.x, a.y - width / 2f, len, width), _px);
        GUI.matrix = m; GUI.color = prev;
    }

    private void OpenCsv()
    {
        if (!logCsv) return;
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(csvPath));
            _csv = new StreamWriter(csvPath, false);
            _csv.WriteLine("step,sim_time,fps,pos_x,pos_y,pos_z,depth_m,altitude_m,heading_deg,pitch_deg,roll_deg," +
                           "vel_x,vel_y,vel_z,speed_mps,vspeed_mps,cmd_surge,cmd_sway,cmd_heave,cmd_yaw," +
                           "sonar_hits,mission_state,range_m,score");
        }
        catch (Exception ex) { Debug.LogError("ROVPILOT_CSV_OPEN_FAIL " + ex); _csv = null; }
    }

    private void WriteCsvRow()
    {
        if (_csv == null) return;
        var c = CultureInfo.InvariantCulture;
        _csv.WriteLine(string.Join(",", new[]
        {
            _step.ToString(c), _simTime.ToString("F3", c), (Time.unscaledDeltaTime > 0 ? 1f/Time.unscaledDeltaTime : 0f).ToString("F1", c),
            _pos.x.ToString("F4", c), _pos.y.ToString("F4", c), _pos.z.ToString("F4", c),
            _depth.ToString("F3", c), _altitude.ToString("F3", c), _headingDeg.ToString("F2", c), _pitchDeg.ToString("F2", c), _rollDeg.ToString("F2", c),
            _linVel.x.ToString("F4", c), _linVel.y.ToString("F4", c), _linVel.z.ToString("F4", c), _speed.ToString("F4", c), _vSpeed.ToString("F4", c),
            _cSurge.ToString("F2", c), _cSway.ToString("F2", c), _cHeave.ToString("F2", c), _cYaw.ToString("F2", c),
            _sonarHits.ToString(c), _mstate.ToString(), _distH.ToString("F3", c), _score.ToString("F1", c)
        }));
        if ((_step & 31) == 0) _csv.Flush();
    }

    private void CloseCsv() { if (_csv == null) return; try { _csv.Flush(); _csv.Close(); } catch { } _csv = null; Debug.Log("ROVPILOT_CSV_CLOSED rows=" + _step); }
    private void OnApplicationQuit() { if (_ready) { WriteMetrics("app_quit"); CloseCsv(); } }
    private void OnDisable() { CloseCsv(); }

    private void WriteMetrics(string reason)
    {
        try
        {
            var c = CultureInfo.InvariantCulture;
            File.WriteAllText(metricsPath,
                "reason=" + reason + "\nsteps=" + _step + "\nbase_translation_m=" + _maxTranslation.ToString("R", c) +
                "\nfinal_depth_m=" + _depth.ToString("R", c) + "\nfinal_altitude_m=" + _altitude.ToString("R", c) +
                "\nsonar_hits=" + _sonarHits + "\nmission_state=" + _mstate + "\nmission_score=" + _score.ToString("F0", c) +
                "\nmission_complete=" + (_mstate == MState.Complete ? "yes" : "no") +
                "\ncontrol_works=" + (_maxTranslation > 0.05f ? "yes" : "no") + "\n");
            Debug.Log("ROVPILOT_METRICS translation=" + _maxTranslation.ToString("F2", c) + " sonarHits=" + _sonarHits +
                      " mission=" + _mstate + " score=" + _score.ToString("F0") + " complete=" + (_mstate == MState.Complete ? "yes" : "no"));
        }
        catch (Exception ex) { Debug.LogError("ROVPILOT_METRICS_FAIL " + ex); }
    }
}
