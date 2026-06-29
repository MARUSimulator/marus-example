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
    public float angularDrag = 26f;     // explicit damping stable at dt=0.005 (c*dt/I < 1 on all axes)
    public float attitudeGain = 34f;
    public float maxYawRate = 0.8f;     // rad/s governor — caps yaw spin
    public float maxTiltRate = 0.2f;    // rad/s governor — keeps roll/pitch smooth + level

    [Header("HUD / logging")]
    public float surfaceZ = 0f;
    public bool drawHud = true;
    public bool logCsv = true;
    public string csvPath = "";

    [Header("Sensors / mission")]
    public Vector3 worksite = new Vector3(6.5f, -4.3f, -3.6f);   // worksite waypoint in MuJoCo world (z up); set by build
    public Camera pipCamera;
    public RenderTexture pipRT;
    public Camera chaseCamera, frontCamera;
    public RenderTexture fpvRT;

    [Header("Headless metrics")]
    public bool logMetrics = false;
    public string metricsPath = "";
    public float autoQuitSeconds = 0f;

    private MjScene _scene;
    private const int BaseBodyId = 1;
    private const int BaseDofAdr = 0;
    private bool _ready;
    private Transform _baseLink;

    private float _cSurge, _cSway, _cHeave, _cYaw;       // raw command
    private float _sSurge, _sSway, _sHeave, _sYaw;       // smoothed (actually-applied) thrust fraction
    private Vector3 _startPos, _pos, _linVel, _angVel, _lastForce, _lastTorque, _fwd, _stbd;
    private float _headingDeg, _speed, _vSpeed, _depth, _rollDeg, _pitchDeg, _maxTranslation, _maxStepDelta;
    private int _step;
    private double _simTime;
    private readonly System.Diagnostics.Stopwatch _clock = new System.Diagnostics.Stopwatch();
    private StreamWriter _csv;
    private GUIStyle _box, _title, _val, _small;
    private Texture2D _panelTex;
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
    private bool _missionArmed;

    // scripted pitch tour (recording mode)
    private float _tourT;
    private string _tourCap = "", _tourSub = "";
    private GUIStyle _capStyle, _subStyle;

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
        Time.fixedDeltaTime = 0.0025f;   // match staging/rov_v2.xml timestep (arm stability)
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
        if (_recording) { _inputSrc = RovInput.Source.Keyboard; _inputDevice = "scripted tour"; Tour(); return; }
        var c = RovInput.Read(out _inputSrc, out _inputDevice);
        _cSurge = c.surge; _cSway = c.sway; _cHeave = c.heave; _cYaw = c.yaw;
    }

    // Closed-loop holonomic homing in the MuJoCo world frame (z up) using the same body axes the
    // thrust is applied along (_fwd/_stbd from xmat), so surge+sway drive straight at the waypoint
    // regardless of heading. minDist > 0 lets the sonar phase approach but stop short of the worksite.
    private void HomeToWorksite(float minDist) => HomeTo(worksite, minDist);

    private void HomeTo(Vector3 target, float minDist)
    {
        float depth = -_pos.z;
        float targetDepth = -target.z;
        if (depth < targetDepth - 0.15f) _cHeave = -1f;
        else if (depth > targetDepth + 0.15f) _cHeave = 0.5f;
        Vector3 e = target - _pos; e.z = 0f;
        float dist = e.magnitude;
        if (dist <= minDist + 0.05f) return;
        Vector3 fwdH = _fwd; fwdH.z = 0f; fwdH.Normalize();
        Vector3 stbdH = _stbd; stbdH.z = 0f; stbdH.Normalize();
        float drive = Mathf.Clamp01((dist - minDist) / 2.5f);
        _cSurge = Mathf.Clamp(Vector3.Dot(e.normalized, fwdH) * drive, -1f, 1f);
        _cSway = Mathf.Clamp(Vector3.Dot(e.normalized, stbdH) * drive, -1f, 1f);
    }

    private void HoldDepth(float targetDepth)
    {
        float depth = -_pos.z;
        if (depth < targetDepth - 0.15f) _cHeave = -1f;
        else if (depth > targetDepth + 0.15f) _cHeave = 0.5f;
    }

    // scripted pitch/teaching tour: title -> 6-DOF control -> sonar -> guided mission -> inspection.
    private void Tour()
    {
        _cSurge = _cSway = _cHeave = _cYaw = 0f;
        if (!_ready) return;
        _tourT += Time.deltaTime;
        float t = _tourT;
        if (t < 4f)                                  // intro: stabilize at depth
        { _tourCap = "OceanScale ROV Simulator"; _tourSub = "Subsea ROV pilot training"; HoldDepth(3.0f); }
        else if (t < 8f)                             // surge
        { _tourCap = "Pilot Thruster Control · Surge"; _tourSub = "Forward / reverse"; HoldDepth(3.0f); _cSurge = (t < 6f) ? 1f : -1f; }
        else if (t < 12f)                            // sway
        { _tourCap = "Pilot Thruster Control · Sway"; _tourSub = "Lateral translation"; HoldDepth(3.0f); _cSway = (t < 10f) ? 1f : -1f; }
        else if (t < 16f)                            // heave (bounded: above the seabed ~4.3 m, below the surface)
        { _tourCap = "Pilot Thruster Control · Heave"; _tourSub = "Dive & ascend (depth hold)"; HoldDepth(t < 14f ? 3.9f : 1.8f); }
        else if (t < 21f)                            // yaw
        { _tourCap = "Pilot Thruster Control · Yaw"; _tourSub = "Heading / turn rate"; HoldDepth(3.2f); _cYaw = 0.6f; }
        else if (t < 27f)                            // sonar approach (stop short at 4.5 m)
        { _tourCap = "Forward Sonar + Altimeter"; _tourSub = "64-beam FLS · height above seabed"; HomeToWorksite(4.5f); }
        else if (t < 34f)                            // onboard pilot camera (FPV) — hold off the worksite
        { _tourCap = "Onboard Pilot Camera (FPV)"; _tourSub = "First-person view from the ROV bow"; HomeToWorksite(4.5f); }
        else if (t < 46f)                            // Task 1: autonomous guided mission (3rd-person)
        { _tourCap = "Task 1 · Wellhead Inspection"; _tourSub = "Auto: Descend → Transit → Hold → Complete"; if (!_missionArmed) { _missionArmed = true; _missionTime = 0f; } HomeToWorksite(0f); }
        else if (t < 58f)                            // Task 2: survey the export pipeline
        { _tourCap = "Task 2 · Pipeline Survey"; _tourSub = "Track the export pipeline along the seabed"; HomeTo(new Vector3(0f, 0f, -3.6f), 0f); }
        else if (t < 78f)                            // Task 3: autonomous lawnmower grid survey
        {
            _tourCap = "Task 3 · Sonar Grid Survey"; _tourSub = "Autonomous lawnmower survey of the seabed";
            Vector3[] grid = { new Vector3(4f, 0f, -3.6f), new Vector3(4f, 3.5f, -3.6f), new Vector3(0f, 3.5f, -3.6f), new Vector3(0f, 6f, -3.6f) };
            HomeTo(grid[Mathf.Clamp((int)((t - 58f) / 5f), 0, grid.Length - 1)], 0f);
        }
        else if (t < 86f)                            // Task 4: ascent / recovery
        { _tourCap = "Task 4 · Ascent & Recovery"; _tourSub = "Controlled ascent toward the surface"; HoldDepth(1.4f); }
        else                                          // wrap: station-keeping
        { _tourCap = "OceanScale ROV Simulator"; _tourSub = "Pilot training · 4-task autonomous demo"; HoldDepth(1.6f); }
    }

    private unsafe void FixedUpdate()
    {
        if (!_ready) return;
        // safety governor: cap angular rates so the explicit damping can't go numerically unstable.
        // Roll/pitch get a TIGHT cap -> attitude stays smoothly level (no jitter, no fast-spin tumble);
        // yaw gets a looser cap so turns are visible but bounded.
        ClampVel(BaseDofAdr + 3, maxTiltRate); ClampVel(BaseDofAdr + 4, maxTiltRate);
        ClampVel(BaseDofAdr + 5, maxYawRate);
        double* m = _scene.Data->xmat + BaseBodyId * 9;
        var fwd = new Vector3((float)m[0], (float)m[3], (float)m[6]).normalized;
        var stbd = new Vector3((float)m[1], (float)m[4], (float)m[7]).normalized;
        var bodyUp = new Vector3((float)m[2], (float)m[5], (float)m[8]).normalized;
        _fwd = fwd; _stbd = stbd;
        _linVel = new Vector3((float)_scene.Data->qvel[BaseDofAdr + 0], (float)_scene.Data->qvel[BaseDofAdr + 1], (float)_scene.Data->qvel[BaseDofAdr + 2]);
        _angVel = new Vector3((float)_scene.Data->qvel[BaseDofAdr + 3], (float)_scene.Data->qvel[BaseDofAdr + 4], (float)_scene.Data->qvel[BaseDofAdr + 5]);
        // low-pass the thrust commands so phase transitions ramp smoothly (thruster spin-up) instead of jerking
        float k = 1f - Mathf.Exp(-Time.fixedDeltaTime / 0.35f);
        _sSurge = Mathf.Lerp(_sSurge, _cSurge, k); _sSway = Mathf.Lerp(_sSway, _cSway, k);
        _sHeave = Mathf.Lerp(_sHeave, _cHeave, k); _sYaw = Mathf.Lerp(_sYaw, _cYaw, k);
        RovDynamics.ComputeWrench(fwd, stbd, bodyUp, _linVel, _angVel, _sSurge, _sSway, _sHeave, _sYaw, P, out var force, out var torque);
        _lastForce = force; _lastTorque = torque;
        int off = BaseBodyId * 6;
        _scene.Data->xfrc_applied[off + 0] = force.x; _scene.Data->xfrc_applied[off + 1] = force.y; _scene.Data->xfrc_applied[off + 2] = force.z;
        _scene.Data->xfrc_applied[off + 3] = torque.x; _scene.Data->xfrc_applied[off + 4] = torque.y; _scene.Data->xfrc_applied[off + 5] = torque.z;
        _simTime += Time.fixedDeltaTime;
    }

    private unsafe Vector3 ReadPos() { double* p = _scene.Data->xpos + BaseBodyId * 3; return new Vector3((float)p[0], (float)p[1], (float)p[2]); }

    private unsafe void ClampVel(int i, float cap)
    {
        double v = _scene.Data->qvel[i];
        if (v > cap) _scene.Data->qvel[i] = cap;
        else if (v < -cap) _scene.Data->qvel[i] = -cap;
    }

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

        if (frontCamera != null && _baseLink != null)        // onboard cam tracks the ROV bow (standalone object)
        {
            frontCamera.transform.position = _baseLink.TransformPoint(new Vector3(0f, 0.3f, 0.55f));
            frontCamera.transform.rotation = _baseLink.rotation;
        }

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
        if (_recording && !_missionArmed) return;               // tour: mission stays on STANDBY until its phase
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
            var dark = new Texture2D(1, 1); dark.SetPixel(0, 0, new Color(0.03f, 0.06f, 0.09f, 0.82f)); dark.Apply();
            _panelTex = dark;
            _box = new GUIStyle(GUI.skin.box); _box.normal.background = dark; _box.border = new RectOffset(2, 2, 2, 2);
            _title = new GUIStyle(GUI.skin.label) { fontSize = 15, fontStyle = FontStyle.Bold }; _title.normal.textColor = new Color(1f, 0.84f, 0.38f);
            _val = new GUIStyle(GUI.skin.label) { fontSize = 13 }; _val.normal.textColor = new Color(0.82f, 0.98f, 0.90f);
            _small = new GUIStyle(GUI.skin.label) { fontSize = 11 }; _small.normal.textColor = new Color(0.7f, 0.85f, 0.9f);
            _capStyle = new GUIStyle(GUI.skin.label) { fontSize = 30, fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleCenter }; _capStyle.normal.textColor = new Color(1f, 0.86f, 0.42f);
            _subStyle = new GUIStyle(GUI.skin.label) { fontSize = 18, alignment = TextAnchor.MiddleCenter }; _subStyle.normal.textColor = new Color(0.85f, 0.97f, 1f);
        }
        // LEFT: telemetry panel (numeric ground truth)
        GUI.Box(new Rect(14, 14, 230, 176), GUIContent.none, _box);
        GUI.Label(new Rect(26, 19, 210, 22), "OceanScale ROV", _title);
        int y = 46;
        Row(ref y, "DEPTH", _depth.ToString("F2") + " m");
        Row(ref y, "ALTITUDE", _altitude >= 0f ? _altitude.ToString("F2") + " m" : "--");
        Row(ref y, "HEADING", _headingDeg.ToString("F0") + "°");
        Row(ref y, "PITCH / ROLL", _pitchDeg.ToString("F0") + "° / " + _rollDeg.ToString("F0") + "°");
        Row(ref y, "SPEED", _speed.ToString("F2") + " m/s");
        Row(ref y, "VERT SPEED", _vSpeed.ToString("F2") + " m/s");

        DrawThrusters(14, 198);                 // thruster command bars (real cmd_*)
        DrawDepthGauge(254, 14, 176);           // vertical depth+altitude gauge
        DrawCompass(Screen.width / 2f, 14, 460); // heading tape (real heading)
        DrawAttitude(Screen.width / 2f, 74, 116); // artificial horizon (real roll/pitch)
        DrawMission();
        DrawCameras();                          // onboard FPV + inspection PiPs (multi-camera)
        DrawSonar();

        if (!_recording)                        // keybind help only for interactive use, not the pitch video
        {
            GUI.Box(new Rect(14, Screen.height - 32, 360, 24), GUIContent.none, _box);
            GUI.Label(new Rect(24, Screen.height - 30, 350, 22), "Joystick  ·  W/S A/D R/F Q/E", _val);
        }
        if (_recording && !string.IsNullOrEmpty(_tourCap))
        {
            float bw = 760f, bx = (Screen.width - bw) / 2f, by = Screen.height - 96f;
            GUI.Box(new Rect(bx, by, bw, 70), GUIContent.none, _box);
            GUI.Label(new Rect(bx, by + 6, bw, 36), _tourCap, _capStyle);
            GUI.Label(new Rect(bx, by + 42, bw, 24), _tourSub, _subStyle);
        }
    }

    // ---- real-data gauges (all read live sim fields) ----
    private void DrawThrusters(float X, float Y)
    {
        GUI.Box(new Rect(X, Y, 230, 92), GUIContent.none, _box);
        GUI.Label(new Rect(X + 12, Y + 4, 210, 18), "THRUSTER CMD", _title);
        DrawBar(X + 14, Y + 28, "SURGE", _sSurge);
        DrawBar(X + 14, Y + 44, "SWAY", _sSway);
        DrawBar(X + 14, Y + 60, "HEAVE", _sHeave);
        DrawBar(X + 14, Y + 76, "YAW", _sYaw);
    }

    private void DrawBar(float x, float y, string label, float v)
    {
        GUI.Label(new Rect(x, y - 1, 52, 16), label, _small);
        float bx = x + 54, bw = 150, cx = bx + bw / 2f;
        GUI.color = new Color(0.2f, 0.3f, 0.34f, 0.9f); GUI.DrawTexture(new Rect(bx, y + 2, bw, 9), _px); GUI.color = Color.white;
        DrawLine(new Vector2(cx, y), new Vector2(cx, y + 13), new Color(0.6f, 0.7f, 0.7f, 0.8f), 1f);
        float f = Mathf.Clamp(v, -1f, 1f) * (bw / 2f);
        var col = new Color(0.35f, 0.95f, 0.75f, 0.95f);
        GUI.color = col; GUI.DrawTexture(new Rect(f >= 0 ? cx : cx + f, y + 2, Mathf.Abs(f), 9), _px); GUI.color = Color.white;
    }

    private void DrawDepthGauge(float X, float Y, float H)
    {
        GUI.Box(new Rect(X, Y, 64, H), GUIContent.none, _box);
        GUI.Label(new Rect(X + 6, Y + 4, 56, 16), "DEPTH", _small);
        float top = Y + 24, bot = Y + H - 8, span = bot - top;
        const float maxD = 5f;                          // 0 m (surface) .. 5 m
        for (int d = 0; d <= 5; d++)
        {
            float yy = top + span * (d / maxD);
            DrawLine(new Vector2(X + 8, yy), new Vector2(X + 20, yy), new Color(0.5f, 0.6f, 0.65f, 0.7f), 1f);
            GUI.Label(new Rect(X + 22, yy - 8, 24, 16), d.ToString(), _small);
        }
        float sb = top + span * Mathf.Clamp01((_depth + (_altitude >= 0 ? _altitude : 0.8f)) / maxD);  // seabed
        DrawLine(new Vector2(X + 6, sb), new Vector2(X + 58, sb), new Color(0.7f, 0.6f, 0.35f, 0.9f), 2f);
        float ry = top + span * Mathf.Clamp01(_depth / maxD);                                          // ROV marker
        GUI.color = new Color(1f, 0.85f, 0.3f); GUI.DrawTexture(new Rect(X + 6, ry - 3, 18, 6), _px); GUI.color = Color.white;
    }

    private void DrawCompass(float cx, float Y, float W)
    {
        float H = 52f;
        GUI.Box(new Rect(cx - W / 2f, Y, W, H), GUIContent.none, _box);
        var lab = new GUIStyle(_val) { alignment = TextAnchor.MiddleCenter, fontStyle = FontStyle.Bold };
        float scale = W / 150f;                          // show +-75 deg
        int start = (int)(Mathf.Floor((_headingDeg - 75f) / 10f) * 10f);
        for (int hd = start; hd <= _headingDeg + 75; hd += 10)         // ticks + labels at ABSOLUTE headings
        {
            float x = cx + (hd - _headingDeg) * scale;
            if (x < cx - W / 2f + 6 || x > cx + W / 2f - 6) continue;
            int h = ((hd % 360) + 360) % 360;
            bool card = h % 90 == 0;
            DrawLine(new Vector2(x, Y + 26), new Vector2(x, Y + (card ? 36 : 32)), new Color(0.6f, 0.78f, 0.82f, 0.85f), card ? 2f : 1f);
            if (h % 30 == 0)
            {
                string s = h == 0 ? "N" : h == 90 ? "E" : h == 180 ? "S" : h == 270 ? "W" : h.ToString();
                lab.normal.textColor = card ? new Color(1f, 0.86f, 0.42f) : new Color(0.8f, 0.92f, 0.95f);
                GUI.Label(new Rect(x - 16, Y + 4, 32, 18), s, lab);
            }
        }
        DrawLine(new Vector2(cx, Y + 24), new Vector2(cx, Y + 40), new Color(1f, 0.5f, 0.3f), 2f);   // lubber line
        var hs = new GUIStyle(_val) { alignment = TextAnchor.MiddleCenter, fontStyle = FontStyle.Bold }; hs.normal.textColor = new Color(1f, 0.86f, 0.42f);
        GUI.Label(new Rect(cx - 40, Y + H - 18, 80, 16), _headingDeg.ToString("F0") + "°", hs);
    }

    private void DrawAttitude(float cx, float Y, float S)
    {
        float x0 = cx - S / 2f;
        GUI.Box(new Rect(x0, Y, S, S), GUIContent.none, _box);
        Vector2 c = new Vector2(cx, Y + S / 2f);
        // horizon line: rotate by roll, shift by pitch
        float pitchOff = Mathf.Clamp(_pitchDeg, -45f, 45f) / 45f * (S / 2.4f);
        float roll = _rollDeg * Mathf.Deg2Rad;
        Vector2 dir = new Vector2(Mathf.Cos(roll), Mathf.Sin(roll)) * (S / 2.4f);
        Vector2 mid = c + new Vector2(Mathf.Sin(roll), -Mathf.Cos(roll)) * (-pitchOff);
        DrawLine(mid - dir, mid + dir, new Color(0.4f, 0.9f, 1f, 0.95f), 2f);
        // fixed boresight
        DrawLine(c + new Vector2(-10, 0), c + new Vector2(-3, 0), new Color(1f, 0.85f, 0.3f), 2f);
        DrawLine(c + new Vector2(3, 0), c + new Vector2(10, 0), new Color(1f, 0.85f, 0.3f), 2f);
        GUI.Label(new Rect(x0 + 4, Y + 2, S, 14), "ATTITUDE", _small);
    }

    private void Row(ref int y, string k, string v)
    {
        GUI.Label(new Rect(26, y, 120, 20), k, _val);
        GUI.Label(new Rect(150, y, 110, 20), v, _val);
        y += 21;
    }

    private void DrawMission()
    {
        float W = 304, H = 150; float X = Screen.width - W - 14, Y = 14;
        bool standby = _recording && !_missionArmed;
        GUI.Box(new Rect(X, Y, W, H), GUIContent.none, _box);
        GUI.Label(new Rect(X + 12, Y + 6, W - 20, 22), standby ? "MISSION · Wellhead (standby)" : "MISSION · Wellhead Inspection", _title);
        int my = (int)Y + 34;
        Chk(X + 14, ref my, _mstate > MState.Descend, "1. Descend below 3 m");
        Chk(X + 14, ref my, _mstate > MState.Transit, "2. Transit to worksite (< 3 m)");
        Chk(X + 14, ref my, _mstate == MState.Complete, "3. Hold station 8 s");
        string state = standby ? "STANDBY" : _mstate.ToString();
        GUI.Label(new Rect(X + 14, my + 4, W - 20, 20), "RANGE  " + _distH.ToString("F1") + " m    STATE  " + state, _val);
        var sc = new GUIStyle(_title) { fontSize = 18 };
        sc.normal.textColor = _mstate == MState.Complete ? new Color(0.5f, 1f, 0.6f) : new Color(1f, 0.84f, 0.38f);
        GUI.Label(new Rect(X + 14, my + 26, W - 20, 24), "SCORE  " + _score.ToString("F0") + " / 100" + (_mstate == MState.Complete ? "  ✓ COMPLETE" : ""), sc);
    }

    private void Chk(float x, ref int y, bool done, string label)
    {
        var c = done ? new Color(0.5f, 1f, 0.6f) : new Color(0.6f, 0.7f, 0.7f);
        var st = new GUIStyle(_val); st.normal.textColor = c;
        GUI.Label(new Rect(x, y, 282, 20), (done ? "☑ " : "☐ ") + label, st);
        y += 20;
    }

    private void DrawSonar()
    {
        float S = 200f; float cx = Screen.width - S / 2f - 24f; float cy = Screen.height - S / 2f - 24f;
        GUI.Box(new Rect(cx - S / 2f, cy - S / 2f, S, S), GUIContent.none, _box);
        GUI.Label(new Rect(cx - S / 2f + 8, cy - S / 2f + 4, S, 20), "FLS SONAR", _val);
        Vector2 apex = new Vector2(cx, cy + S / 2f - 14f);
        float maxLen = (S / 2f - 12f) / Mathf.Sin(55f * Mathf.Deg2Rad);   // fit the ±55° fan inside the panel
        // range-ring arcs (always drawn -> reads as a live sonar scope, not an empty box)
        var ring = new Color(0.25f, 0.75f, 0.75f, 0.5f);
        for (int r = 1; r <= 3; r++)
        {
            float rad = maxLen * r / 3f;
            Vector2 prev = Vector2.zero; bool has = false;
            for (float a = -55f; a <= 55f; a += 5.5f)
            {
                var p = apex + new Vector2(Mathf.Sin(a * Mathf.Deg2Rad), -Mathf.Cos(a * Mathf.Deg2Rad)) * rad;
                if (has) DrawLine(prev, p, ring, 1f);
                prev = p; has = true;
            }
        }
        // edge bearings of the fan
        DrawLine(apex, apex + new Vector2(Mathf.Sin(-55f * Mathf.Deg2Rad), -Mathf.Cos(-55f * Mathf.Deg2Rad)) * maxLen, ring, 1f);
        DrawLine(apex, apex + new Vector2(Mathf.Sin(55f * Mathf.Deg2Rad), -Mathf.Cos(55f * Mathf.Deg2Rad)) * maxLen, ring, 1f);
        for (int i = 0; i < SonarRays; i++)
        {
            bool hit = _sonar[i] > 0f;
            float frac = hit ? Mathf.Clamp01(_sonar[i] / SonarRange) : 1f;
            float ang = Mathf.Lerp(-55f, 55f, i / (float)(SonarRays - 1));
            var dir = new Vector2(Mathf.Sin(ang * Mathf.Deg2Rad), -Mathf.Cos(ang * Mathf.Deg2Rad));
            // faint sweep spoke always visible; bright return + blob where the beam hits
            DrawLine(apex, apex + dir * maxLen, new Color(0.2f, 0.6f, 0.6f, 0.18f), 1f);
            if (hit)
            {
                var col = Color.Lerp(new Color(0.4f, 1f, 0.5f), new Color(1f, 0.85f, 0.3f), 1f - frac);
                Vector2 p = apex + dir * (maxLen * frac);
                DrawLine(apex, p, col, 2.4f);
                GUI.color = col; GUI.DrawTexture(new Rect(p.x - 2, p.y - 2, 4, 4), _px); GUI.color = Color.white;
            }
        }
        GUI.Label(new Rect(cx - S / 2f + 8, cy + S / 2f - 20, S, 18), "contacts: " + _sonarHits + "/" + SonarRays, _val);
    }

    private void DrawCameras()
    {
        float W = 256f, H = 144f, X = Screen.width - W - 14f;
        // onboard pilot camera (FPV)
        float y1 = 176f;
        GUI.Box(new Rect(X - 2, y1 - 2, W + 4, H + 22), GUIContent.none, _box);
        GUI.Label(new Rect(X + 4, y1, W, 18), "ONBOARD PILOT CAM", _val);
        if (fpvRT != null) GUI.DrawTexture(new Rect(X, y1 + 18, W, H), fpvRT, ScaleMode.ScaleAndCrop, false);
        // inspection camera (oblique, frames the wellhead)
        float y2 = y1 + H + 24f;
        GUI.Box(new Rect(X - 2, y2 - 2, W + 4, H + 22), GUIContent.none, _box);
        GUI.Label(new Rect(X + 4, y2, W, 18), "INSPECTION CAM", _val);
        if (pipRT != null) GUI.DrawTexture(new Rect(X, y2 + 18, W, H), pipRT, ScaleMode.ScaleAndCrop, false);
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
            csvPath = ResolveOutputPath("ROVSIM_CSV", csvPath, "rov_session.csv");
            EnsureParentDirectory(csvPath);
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
            metricsPath = ResolveOutputPath("ROVSIM_METRICS", metricsPath, "rov_motion_log.txt");
            EnsureParentDirectory(metricsPath);
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

    private static string ResolveOutputPath(string envName, string configuredPath, string fileName)
    {
        var env = Environment.GetEnvironmentVariable(envName);
        if (!string.IsNullOrEmpty(env)) return env;
        if (!string.IsNullOrEmpty(configuredPath)) return configuredPath;
        var root = Application.persistentDataPath;
        if (string.IsNullOrEmpty(root)) root = Directory.GetCurrentDirectory();
        return Path.Combine(root, fileName);
    }

    private static void EnsureParentDirectory(string path)
    {
        var dir = Path.GetDirectoryName(Path.GetFullPath(path));
        if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);
    }
}
