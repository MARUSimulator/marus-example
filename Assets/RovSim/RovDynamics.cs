using UnityEngine;

// Shared ROV dynamics — used BOTH by RovPilotController (runtime) and RovSimTest (headless test),
// so the test exercises exactly the math the player runs. Pure function, MuJoCo global frame (z-up).
public static class RovDynamics
{
    public struct Params
    {
        public float forceSurge, forceSway, forceHeave, torqueYaw;
        public float linearDrag, quadraticDrag, angularDrag, attitudeGain;

        public static Params Default => new Params
        {
            forceSurge = 72f, forceSway = 50f, forceHeave = 58f, torqueYaw = 11f,
            linearDrag = 60f, quadraticDrag = 70f, angularDrag = 26f, attitudeGain = 34f
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
        force = fwd * (surge * p.forceSurge)
              + stbd * (sway * p.forceSway)
              + worldUp * (heave * p.forceHeave);

        float speed = linVel.magnitude;
        force += -linVel * p.linearDrag - linVel * (speed * p.quadraticDrag);

        var restoring = Vector3.Cross(bodyUp, worldUp) * p.attitudeGain;
        var yawT = worldUp * (yaw * p.torqueYaw);
        torque = restoring + yawT - angVel * p.angularDrag;
    }
}
