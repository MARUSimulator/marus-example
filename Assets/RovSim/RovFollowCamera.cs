using UnityEngine;

// Chase camera: keeps the ROV centered and close in frame so the robot is always clearly visible.
public class RovFollowCamera : MonoBehaviour
{
    public Transform target;
    public Vector3 offset = new Vector3(1.7f, 1.0f, -2.2f);
    public float smooth = 7f;

    private void LateUpdate()
    {
        if (target == null) return;
        Vector3 desired = target.position + offset;
        float k = 1f - Mathf.Exp(-smooth * Mathf.Max(Time.deltaTime, 1e-4f));
        transform.position = Vector3.Lerp(transform.position, desired, k);
        transform.LookAt(target.position + Vector3.down * 0.7f);   // bias view downward to seabed -> reads underwater, not surface
    }
}
