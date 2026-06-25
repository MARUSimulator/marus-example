using System;
using System.Linq;
using Mujoco;
using UnityEngine;

// v2 manipulator-arm pilot control. Drives the 6 position actuators via MjActuator.Control
// (org.mujoco resyncs Control -> Data->ctrl every physics step, so we set Control, never Data->ctrl).
// Rate-control from the keyboard; a small HUD shows joint angles + jaw state. The vehicle base is
// held by RovPilotController's station-keeping (angular drag + rate governor) during intervention.
public class RovManipulator : MonoBehaviour
{
    [Header("Arm rate (rad/s of target change while a key is held)")]
    public float jointRate = 0.7f;
    public float jawRate = 1.2f;
    public bool drawHud = true;

    // joint ranges (match staging/rov_v2.xml actuator ctrlrange)
    private const float AziR = 1.4f, ShR = 1.2f, ElLo = -2.3f, ElHi = 0.2f, WrR = 1.5f, JawMax = 0.85f;

    private MjActuator _azi, _sh, _el, _wr, _jl, _jr;
    private float _tAzi, _tSh, _tEl = -0.6f, _tWr, _tJaw = 0.4f;   // targets; jaw 0=closed .. 0.85=open
    private bool _ready;
    private GUIStyle _box, _title, _val;
    private Texture2D _px;

    private void Start() { _px = Texture2D.whiteTexture; TryBind(); }

    private void TryBind()
    {
        var acts = GetComponentsInChildren<MjActuator>(true);
        if (acts == null || acts.Length == 0)
            acts = UnityEngine.Object.FindObjectsByType<MjActuator>(FindObjectsSortMode.None);
        MjActuator F(string n) => acts.FirstOrDefault(a => a != null && a.name.IndexOf(n, StringComparison.OrdinalIgnoreCase) >= 0);
        _azi = F("azimuth"); _sh = F("shoulder"); _el = F("elbow"); _wr = F("wrist"); _jl = F("jaw_l"); _jr = F("jaw_r");
        _ready = _azi && _sh && _el && _wr && _jl && _jr;
        Debug.Log(_ready ? "ROVMANIP_READY bound 6 arm actuators" : "ROVMANIP_BIND_FAIL actuators=" + (acts?.Length ?? 0));
    }

    private void Update()
    {
        if (!_ready) { TryBind(); return; }
        float dt = Time.deltaTime;
        _tAzi = Mathf.Clamp(_tAzi + Axis(KeyCode.L, KeyCode.J) * jointRate * dt, -AziR, AziR);   // J/L azimuth
        _tSh = Mathf.Clamp(_tSh + Axis(KeyCode.I, KeyCode.K) * jointRate * dt, -ShR, ShR);        // I/K shoulder
        _tEl = Mathf.Clamp(_tEl + Axis(KeyCode.O, KeyCode.U) * jointRate * dt, ElLo, ElHi);        // U/O elbow
        _tWr = Mathf.Clamp(_tWr + Axis(KeyCode.Y, KeyCode.H) * jointRate * dt, -WrR, WrR);         // Y/H wrist
        _tJaw = Mathf.Clamp(_tJaw + Axis(KeyCode.B, KeyCode.G) * jawRate * dt, 0f, JawMax);        // B open / G close
        _azi.Control = _tAzi; _sh.Control = _tSh; _el.Control = _tEl; _wr.Control = _tWr;
        _jl.Control = _tJaw; _jr.Control = -_tJaw;   // mirrored parallel-jaw fingers
    }

    private static float Axis(KeyCode plus, KeyCode minus)
        => (Input.GetKey(plus) ? 1f : 0f) - (Input.GetKey(minus) ? 1f : 0f);

    private void OnGUI()
    {
        if (!drawHud || !_ready) return;
        if (_box == null)
        {
            var dark = new Texture2D(1, 1); dark.SetPixel(0, 0, new Color(0.03f, 0.06f, 0.09f, 0.82f)); dark.Apply();
            _box = new GUIStyle(GUI.skin.box) { border = new RectOffset(2, 2, 2, 2) }; _box.normal.background = dark;
            _title = new GUIStyle(GUI.skin.label) { fontSize = 13, fontStyle = FontStyle.Bold }; _title.normal.textColor = new Color(1f, 0.84f, 0.38f);
            _val = new GUIStyle(GUI.skin.label) { fontSize = 12 }; _val.normal.textColor = new Color(0.82f, 0.98f, 0.90f);
        }
        float X = 14f, Y = 200f, W = 230f, H = 118f;
        GUI.Box(new Rect(X, Y, W, H), GUIContent.none, _box);
        GUI.Label(new Rect(X + 12, Y + 4, W, 18), "MANIPULATOR (7-fn)", _title);
        int y = (int)Y + 26;
        Row(ref y, "AZIMUTH", Deg(_tAzi)); Row(ref y, "SHOULDER", Deg(_tSh));
        Row(ref y, "ELBOW", Deg(_tEl)); Row(ref y, "WRIST", Deg(_tWr));
        Row(ref y, "JAW", Mathf.RoundToInt(_tJaw / JawMax * 100f) + "% open");
    }

    private void Row(ref int y, string k, string v)
    { GUI.Label(new Rect(26, y, 120, 18), k, _val); GUI.Label(new Rect(150, y, 90, 18), v, _val); y += 18; }
    private static string Deg(float rad) => Mathf.RoundToInt(rad * Mathf.Rad2Deg) + "°";
}
