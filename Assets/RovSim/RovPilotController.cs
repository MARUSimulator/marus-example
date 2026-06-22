using System;
using System.Globalization;
using Mujoco;
using UnityEngine;

// v1 ROV/AUV pilot controller — 6-DOF vehicle control via MuJoCo xfrc_applied (force-integrated, NOT teleport).
// No manipulator arm (v1 scope). MuJoCo is z-up; we read/write raw MuJoCo arrays in MuJoCo global frame.
// Attach to the imported base_link GameObject. Requires an MjScene in the scene (org.mujoco).
public class RovPilotController : MonoBehaviour
{
    [Header("Thrust  (N / N·m)")]
    public float forceSurge = 72f;
    public float forceSway = 50f;
    public float forceHeave = 58f;
    public float torqueYaw = 11f;

    [Header("Hydrodynamics")]
    public float linearDrag = 60f;      // F = -linearDrag * v
    public float quadraticDrag = 70f;   // F = -quadraticDrag * |v| * v
    public float angularDrag = 26f;
    public float attitudeGain = 34f;    // passive self-righting (roll/pitch -> upright)

    [Header("HUD")]
    public float surfaceZ = 0f;         // MuJoCo z of water surface; depth = surfaceZ - z
    public bool drawHud = true;

    [Header("Headless metrics (batchmode test)")]
    public bool logMetrics = false;
    public string metricsPath = "/mnt/storage/marus-rov-sim/rov_motion_log.txt";
    public float autoQuitSeconds = 0f;

    private MjScene _scene;
    private const int BaseBodyId = 1;   // body 0 = world; single free body = 1
    private const int BaseDofAdr = 0;   // free joint qvel base
    private bool _ready;

    private float _cSurge, _cSway, _cHeave, _cYaw;
    private Vector3 _startPos, _pos, _linVel, _angVel;
    private float _headingDeg, _speed, _vSpeed, _depth, _rollDeg, _pitchDeg, _maxTranslation;
    private readonly System.Diagnostics.Stopwatch _clock = new System.Diagnostics.Stopwatch();
    private GUIStyle _box, _title, _val;

    private void Start()
    {
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
        Debug.Log("ROVPILOT_READY nq=" + (int)_scene.Model->nq + " nv=" + (int)_scene.Model->nv +
                  " nbody=" + (int)_scene.Model->nbody);
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

        // body frame (columns of xmat) and linear/angular velocity, MuJoCo global frame, z-up
        double* m = _scene.Data->xmat + BaseBodyId * 9;
        var fwd = new Vector3((float)m[0], (float)m[3], (float)m[6]).normalized;       // body +x
        var stbd = new Vector3((float)m[1], (float)m[4], (float)m[7]).normalized;      // body +y
        var bodyUp = new Vector3((float)m[2], (float)m[5], (float)m[8]).normalized;    // body +z
        var worldUp = new Vector3(0f, 0f, 1f);

        _linVel = new Vector3((float)_scene.Data->qvel[BaseDofAdr + 0],
                              (float)_scene.Data->qvel[BaseDofAdr + 1],
                              (float)_scene.Data->qvel[BaseDofAdr + 2]);
        _angVel = new Vector3((float)_scene.Data->qvel[BaseDofAdr + 3],
                              (float)_scene.Data->qvel[BaseDofAdr + 4],
                              (float)_scene.Data->qvel[BaseDofAdr + 5]);

        // thrust: surge follows heading (body fwd), sway along starboard, heave along world vertical
        var force = fwd * (_cSurge * forceSurge)
                  + stbd * (_cSway * forceSway)
                  + worldUp * (_cHeave * forceHeave);

        // quadratic + linear hydro drag opposing translation
        float speed = _linVel.magnitude;
        force += -_linVel * linearDrag - _linVel * (speed * quadraticDrag);

        // passive self-righting torque + commanded yaw + angular drag
        var restoring = Vector3.Cross(bodyUp, worldUp) * attitudeGain;
        var yaw = worldUp * (_cYaw * torqueYaw);
        var torque = restoring + yaw - _angVel * angularDrag;

        int off = BaseBodyId * 6;
        _scene.Data->xfrc_applied[off + 0] = force.x;
        _scene.Data->xfrc_applied[off + 1] = force.y;
        _scene.Data->xfrc_applied[off + 2] = force.z;
        _scene.Data->xfrc_applied[off + 3] = torque.x;
        _scene.Data->xfrc_applied[off + 4] = torque.y;
        _scene.Data->xfrc_applied[off + 5] = torque.z;
    }

    private unsafe Vector3 ReadPos()
    {
        double* p = _scene.Data->xpos + BaseBodyId * 3;
        return new Vector3((float)p[0], (float)p[1], (float)p[2]);
    }

    private unsafe void LateUpdate()
    {
        if (!_ready) return;
        _pos = ReadPos();
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
    }

    private static float NormalizeHeading(float h) { h %= 360f; if (h < 0f) h += 360f; return h; }

    private void OnGUI()
    {
        if (!drawHud || !_ready) return;
        if (_box == null)
        {
            _box = new GUIStyle(GUI.skin.box);
            _box.normal.background = Texture2D.grayTexture;
            _title = new GUIStyle(GUI.skin.label) { fontSize = 16, fontStyle = FontStyle.Bold };
            _title.normal.textColor = new Color(1f, 0.84f, 0.38f);
            _val = new GUIStyle(GUI.skin.label) { fontSize = 14 };
            _val.normal.textColor = new Color(0.82f, 0.98f, 0.90f);
        }
        GUI.Box(new Rect(14, 14, 246, 196), GUIContent.none, _box);
        GUI.Label(new Rect(26, 20, 230, 22), "ROV OPERATOR — v1", _title);
        int y = 48;
        Row(ref y, "DEPTH", _depth.ToString("F2") + " m");
        Row(ref y, "HEADING", _headingDeg.ToString("F0") + "°");
        Row(ref y, "PITCH / ROLL", _pitchDeg.ToString("F0") + "° / " + _rollDeg.ToString("F0") + "°");
        Row(ref y, "SPEED", _speed.ToString("F2") + " m/s");
        Row(ref y, "VERT SPEED", _vSpeed.ToString("F2") + " m/s");
        Row(ref y, "CMD S/Y", _cSurge.ToString("F1") + " / " + _cYaw.ToString("F1"));
        Row(ref y, "CMD SW/HV", _cSway.ToString("F1") + " / " + _cHeave.ToString("F1"));

        GUI.Box(new Rect(14, 220, 246, 24), GUIContent.none, _box);
        GUI.Label(new Rect(26, 222, 230, 22), "W/S surge  A/D sway  R/F heave  Q/E yaw", _val);
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
            System.IO.File.WriteAllText(metricsPath,
                "reason=" + reason + "\n" +
                "control=keyboard surge/sway/heave/yaw -> xfrc_applied[base]; base pose NEVER written directly\n" +
                "base_translation_m=" + _maxTranslation.ToString("R", c) + "\n" +
                "final_depth_m=" + _depth.ToString("R", c) + "\n" +
                "final_heading_deg=" + _headingDeg.ToString("R", c) + "\n" +
                "control_works=" + (_maxTranslation > 0.05f ? "yes" : "no") + "\n");
            Debug.Log("ROVPILOT_METRICS base_translation_m=" + _maxTranslation.ToString("R", c) +
                      " control_works=" + (_maxTranslation > 0.05f ? "yes" : "no"));
        }
        catch (Exception ex) { Debug.LogError("ROVPILOT_METRICS_FAIL " + ex); }
    }
}
