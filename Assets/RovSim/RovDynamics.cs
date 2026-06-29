using UnityEngine;

// Shared ROV dynamics — used BOTH by RovPilotController (runtime) and RovSimTest (headless test),
// so the test exercises exactly the math the player runs. Pure function, MuJoCo global frame (z-up).
// Drag is ANISOTROPIC in the body frame (Fossen-style cross-flow): low along surge (streamlined),
// high along sway/heave — so the vehicle behaves like a real ROV (fast forward, resists sideways).
public static class RovDynamics
{
    public struct Params
    {
        public float forceSurge, forceSway, forceHeave, torqueYaw;
        public float dragSurgeLin, dragSurgeQuad;
        public float dragSwayLin, dragSwayQuad;
        public float dragHeaveLin, dragHeaveQuad;
        public float angularDrag, attitudeGain;

        public static Params Default => new Params
        {
            forceSurge = 72f, forceSway = 50f, forceHeave = 58f, torqueYaw = 11f,
            dragSurgeLin = 30f, dragSurgeQuad = 40f,      // terminal surge ~1.0 m/s
            dragSwayLin = 120f, dragSwayQuad = 160f,      // terminal sway  ~0.30 m/s
            dragHeaveLin = 120f, dragHeaveQuad = 160f,    // terminal heave ~0.33 m/s
            angularDrag = 26f, attitudeGain = 34f         // explicit damping stable at dt=0.005 (c*dt/I < 1 on all axes)
        };
    }

    // fwd/stbd/bodyUp = body axes expressed in MuJoCo world frame (columns of xmat).
    public static void ComputeWrench(
        Vector3 fwd, Vector3 stbd, Vector3 bodyUp,
        Vector3 linVel, Vector3 angVel,
        float surge, float sway, float heave, float yaw,
        Params p, out Vector3 force, out Vector3 torque)
    {
        var worldUp = new Vector3(0f, 0f, 1f);

        // thrust: surge follows heading, sway along starboard, heave along world vertical
        force = fwd * (surge * p.forceSurge)
              + stbd * (sway * p.forceSway)
              + worldUp * (heave * p.forceHeave);

        // anisotropic hydro drag, resolved on body axes (linear + quadratic per axis)
        float vF = Vector3.Dot(linVel, fwd);
        float vS = Vector3.Dot(linVel, stbd);
        float vU = Vector3.Dot(linVel, bodyUp);
        force += fwd * (-p.dragSurgeLin * vF - p.dragSurgeQuad * Mathf.Abs(vF) * vF)
              + stbd * (-p.dragSwayLin * vS - p.dragSwayQuad * Mathf.Abs(vS) * vS)
              + bodyUp * (-p.dragHeaveLin * vU - p.dragHeaveQuad * Mathf.Abs(vU) * vU);

        // passive self-righting + commanded yaw + angular drag
        var restoring = Vector3.Cross(bodyUp, worldUp) * p.attitudeGain;
        var yawT = worldUp * (yaw * p.torqueYaw);
        torque = restoring + yawT - angVel * p.angularDrag;
    }
}
