using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

// Pilot input: Logitech-style joystick (gamepad OR flight stick) with keyboard fallback.
// The MAP functions are pure (no hardware) so the joystick behaviour is testable headlessly;
// Read() does the live device read (Unity Input System) and is what a real Logitech drives.
public static class RovInput
{
    public enum Source { Keyboard, Gamepad, FlightStick }

    public struct Cmd
    {
        public float surge, sway, heave, yaw;
        public void Clamp()
        {
            surge = Mathf.Clamp(surge, -1f, 1f); sway = Mathf.Clamp(sway, -1f, 1f);
            heave = Mathf.Clamp(heave, -1f, 1f); yaw = Mathf.Clamp(yaw, -1f, 1f);
        }
        public float Magnitude => Mathf.Abs(surge) + Mathf.Abs(sway) + Mathf.Abs(heave) + Mathf.Abs(yaw);
    }

    // ---- pure mappings (unit-testable: this is the "simulated joystick") ----

    // Logitech flight stick (e.g. Extreme 3D Pro): push-forward stickY=-1 -> surge+, stickX=sway, twist=yaw, throttle=heave
    public static Cmd MapFlightStick(float stickX, float stickY, float twist, float throttle)
    {
        var c = new Cmd { surge = -stickY, sway = stickX, yaw = twist, heave = throttle };
        c.Clamp(); return c;
    }

    // Logitech gamepad (F310/F710): left stick surge/sway, right stick X yaw, triggers heave (RT up / LT down)
    public static Cmd MapGamepad(Vector2 leftStick, Vector2 rightStick, float lt, float rt)
    {
        var c = new Cmd { surge = leftStick.y, sway = leftStick.x, yaw = rightStick.x, heave = rt - lt };
        c.Clamp(); return c;
    }

    public static Cmd MapKeyboard(bool w, bool s, bool a, bool d, bool r, bool f, bool q, bool e)
    {
        var c = new Cmd
        {
            surge = (w ? 1f : 0f) - (s ? 1f : 0f),
            sway = (d ? 1f : 0f) - (a ? 1f : 0f),
            heave = (r ? 1f : 0f) - (f ? 1f : 0f),
            yaw = (e ? 1f : 0f) - (q ? 1f : 0f)
        };
        c.Clamp(); return c;
    }

    private static float Dz(float v, float d) => Mathf.Abs(v) < d ? 0f : v;
    private static Vector2 Dz(Vector2 v, float d) => new Vector2(Dz(v.x, d), Dz(v.y, d));
    private static bool Active(Cmd c) => c.Magnitude > 0.02f;

    // ---- live device read (real Logitech via Unity Input System; keyboard fallback) ----
    public static Cmd Read(out Source src, out string device, float deadzone = 0.08f)
    {
        src = Source.Keyboard; device = "keyboard";
#if ENABLE_INPUT_SYSTEM
        var gp = Gamepad.current;
        if (gp != null)
        {
            var c = MapGamepad(Dz(gp.leftStick.ReadValue(), deadzone), Dz(gp.rightStick.ReadValue(), deadzone),
                               gp.leftTrigger.ReadValue(), gp.rightTrigger.ReadValue());
            if (Active(c)) { src = Source.Gamepad; device = gp.displayName; return c; }
            src = Source.Gamepad; device = gp.displayName; // connected but idle
        }
        var js = Joystick.current;
        if (js != null)
        {
            float sx = js.stick != null ? js.stick.ReadValue().x : 0f;
            float sy = js.stick != null ? js.stick.ReadValue().y : 0f;
            float tw = js.twist != null ? js.twist.ReadValue() : 0f;
            float thr = js.trigger != null && js.trigger.isPressed ? 1f : 0f; // best-effort heave on trigger
            var c = MapFlightStick(Dz(sx, deadzone), Dz(sy, deadzone), Dz(tw, deadzone), thr);
            if (Active(c)) { src = Source.FlightStick; device = js.displayName; return c; }
            if (src == Source.Keyboard) { src = Source.FlightStick; device = js.displayName; }
        }
        var kb = Keyboard.current;
        if (kb != null)
        {
            var c = MapKeyboard(kb.wKey.isPressed, kb.sKey.isPressed, kb.aKey.isPressed, kb.dKey.isPressed,
                                kb.rKey.isPressed, kb.fKey.isPressed, kb.qKey.isPressed, kb.eKey.isPressed);
            if (Active(c)) { src = Source.Keyboard; device = "keyboard"; return c; }
        }
#endif
        // legacy Input Manager fallback (works under activeInputHandler=Both)
        return MapKeyboard(Input.GetKey(KeyCode.W), Input.GetKey(KeyCode.S), Input.GetKey(KeyCode.A), Input.GetKey(KeyCode.D),
                           Input.GetKey(KeyCode.R), Input.GetKey(KeyCode.F), Input.GetKey(KeyCode.Q), Input.GetKey(KeyCode.E));
    }
}
