using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using M2MqttUnity;
using uPLibrary.Networking.M2Mqtt.Messages;

/// <summary>
/// Production IK-based pose retargeting for L&T Digital Twin.
/// 
/// Uses Unity's OnAnimatorIK with proper body grounding.
/// All positions are computed relative to the hip center, then
/// scaled to match the avatar's actual proportions.
/// 
/// REQUIRES: AnimatorController with IK Pass enabled.
/// </summary>
[RequireComponent(typeof(Animator))]
public class PoseLandmarkReceiver : M2MqttUnityClient
{
    [Header("MQTT")]
    public string poseTopic = "phone/pose";

    [Header("IK Weights")]
    [Range(0f, 1f)] public float handWeight = 1f;
    [Range(0f, 1f)] public float footWeight = 1f;
    [Range(0f, 1f)] public float bodyWeight = 0.6f;
    [Range(0f, 1f)] public float headWeight = 0.7f;

    [Header("Smoothing")]
    [Range(0.05f, 0.6f)] public float emaAlpha = 0.25f;
    [Range(0f, 0.02f)] public float jitterThreshold = 0.005f;

    [Header("Visibility")]
    [Range(0f, 1f)] public float minVisibility = 0.3f;

    [Header("Debug")]
    public bool showDebugLogs = false;
    public bool drawDebugSkeleton = true;

    // ── MediaPipe indices ────────────────────────────────────────
    const int NOSE = 0;
    const int L_EAR = 7, R_EAR = 8;
    const int L_SHOULDER = 11, R_SHOULDER = 12;
    const int L_ELBOW = 13, R_ELBOW = 14;
    const int L_WRIST = 15, R_WRIST = 16;
    const int L_HIP = 23, R_HIP = 24;
    const int L_KNEE = 25, R_KNEE = 26;
    const int L_ANKLE = 27, R_ANKLE = 28;
    const int L_FOOT = 31, R_FOOT = 32;

    // ── Animator & avatar measurements ───────────────────────────
    private Animator anim;
    private float avatarHeight = 1.7f;
    private float avatarLegLength = 0.85f;
    private float avatarArmLength = 0.55f;
    private float avatarShoulderWidth = 0.35f;
    private Vector3 avatarRootPos; // where the avatar stands (ground level)
    private float avatarHipHeight; // hip height from ground

    // ── IK targets ───────────────────────────────────────────────
    private Vector3 ikLeftHand, ikRightHand;
    private Vector3 ikLeftFoot, ikRightFoot;
    private Vector3 ikLeftElbow, ikRightElbow;
    private Vector3 ikLeftKnee, ikRightKnee;
    private Vector3 ikBodyPos;
    private Quaternion ikBodyRot = Quaternion.identity;
    private Vector3 ikLookTarget;
    private bool hasData = false;

    // ── EMA smoothing ───────────────────────────────────────────
    private float[] smoothX, smoothY, smoothZ;
    private bool emaInit = false;

    // ── MediaPipe body measurements (learned from first frame) ───
    private float mpShoulderWidth = 0.1f;
    private float mpTorsoHeight = 0.1f;
    private bool mpCalibrated = false;

    // ── MQTT ─────────────────────────────────────────────────────
    private LandmarkFrame latestFrame;
    private bool newFrameReceived;
    private readonly object frameLock = new object();

    // ── Debug ────────────────────────────────────────────────────
    private Vector3[] debugWorldPts;

    // ─────────────────────────────────────────────────────────────
    protected override void Start()
    {
        if (string.IsNullOrEmpty(brokerAddress)) brokerAddress = "192.168.1.100";
        if (brokerPort == 0) brokerPort = 1883;
        isEncrypted = false;
        autoConnect = true;
        base.Start();

        anim = GetComponent<Animator>();
        if (!anim || !anim.isHuman)
        {
            Debug.LogError("[Pose] Animator missing or not Humanoid!");
            return;
        }
        if (anim.runtimeAnimatorController == null)
        {
            Debug.LogError("[Pose] No AnimatorController! IK won't work.");
            return;
        }

        avatarRootPos = transform.position;

        // Measure the avatar's actual proportions from the rig
        Transform head = anim.GetBoneTransform(HumanBodyBones.Head);
        Transform hips = anim.GetBoneTransform(HumanBodyBones.Hips);
        Transform lFoot = anim.GetBoneTransform(HumanBodyBones.LeftFoot);
        Transform lHand = anim.GetBoneTransform(HumanBodyBones.LeftHand);
        Transform lShoulder = anim.GetBoneTransform(HumanBodyBones.LeftUpperArm);
        Transform rShoulder = anim.GetBoneTransform(HumanBodyBones.RightUpperArm);
        Transform lElbow = anim.GetBoneTransform(HumanBodyBones.LeftLowerArm);

        if (head && lFoot)
            avatarHeight = head.position.y - lFoot.position.y;
        if (hips && lFoot)
            avatarHipHeight = hips.position.y - lFoot.position.y;
        if (hips && lFoot)
            avatarLegLength = hips.position.y - lFoot.position.y;
        if (lShoulder && lHand)
            avatarArmLength = Vector3.Distance(lShoulder.position, lHand.position);
        if (lShoulder && rShoulder)
            avatarShoulderWidth = Vector3.Distance(lShoulder.position, rShoulder.position);

        // Initialize IK targets to current avatar pose
        ikBodyPos = hips ? hips.position : avatarRootPos + Vector3.up;
        ikLeftHand = lHand ? lHand.position : ikBodyPos;
        ikRightHand = anim.GetBoneTransform(HumanBodyBones.RightHand)?.position ?? ikBodyPos;
        ikLeftFoot = lFoot ? lFoot.position : avatarRootPos;
        ikRightFoot = anim.GetBoneTransform(HumanBodyBones.RightFoot)?.position ?? avatarRootPos;
        ikLeftElbow = lElbow ? lElbow.position : ikBodyPos;
        ikRightElbow = anim.GetBoneTransform(HumanBodyBones.RightLowerArm)?.position ?? ikBodyPos;
        ikLeftKnee = anim.GetBoneTransform(HumanBodyBones.LeftLowerLeg)?.position ?? avatarRootPos + Vector3.up * 0.4f;
        ikRightKnee = anim.GetBoneTransform(HumanBodyBones.RightLowerLeg)?.position ?? avatarRootPos + Vector3.up * 0.4f;
        ikLookTarget = avatarRootPos + Vector3.forward * 3f + Vector3.up * avatarHeight;

        Debug.Log($"[Pose] Avatar: height={avatarHeight:F2}, hipH={avatarHipHeight:F2}, " +
                  $"arm={avatarArmLength:F2}, shoulder={avatarShoulderWidth:F2}");
    }

    // ── MQTT ───────────────────────────────────────────────────────
    protected override void SubscribeTopics()
    {
        client.Subscribe(new[] { poseTopic }, new[] { MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE });
        Debug.Log("[Pose] Subscribed: " + poseTopic);
    }
    protected override void UnsubscribeTopics() { client.Unsubscribe(new[] { poseTopic }); }

    protected override void DecodeMessage(string topic, byte[] message)
    {
        if (topic != poseTopic) return;
        try
        {
            var f = JsonUtility.FromJson<LandmarkFrame>(Encoding.UTF8.GetString(message));
            if (f?.landmarks != null && f.landmarks.Length >= 29)
            {
                lock (frameLock) { latestFrame = f; newFrameReceived = true; }
                if (showDebugLogs) Debug.Log("[Pose] " + f.landmarks.Length + " landmarks");
            }
        }
        catch (Exception e) { Debug.LogWarning("[Pose] Parse: " + e.Message); }
    }

    // ── Update ─────────────────────────────────────────────────────
    protected override void Update()
    {
        base.Update();

        LandmarkFrame f = null;
        lock (frameLock)
        {
            if (newFrameReceived) { f = latestFrame; newFrameReceived = false; }
        }
        if (f == null || !anim) return;

        var lm = f.landmarks;
        int n = lm.Length;

        // ── EMA smooth raw landmarks (in MediaPipe space) ─────────
        if (!emaInit)
        {
            smoothX = new float[n]; smoothY = new float[n]; smoothZ = new float[n];
            for (int i = 0; i < n; i++)
            { smoothX[i] = lm[i].x; smoothY[i] = lm[i].y; smoothZ[i] = lm[i].z; }
            emaInit = true;
        }
        else
        {
            for (int i = 0; i < n; i++)
            {
                float dx = Mathf.Abs(lm[i].x - smoothX[i]);
                float dy = Mathf.Abs(lm[i].y - smoothY[i]);
                float dz = Mathf.Abs(lm[i].z - smoothZ[i]);
                float delta = dx + dy + dz;

                if (delta > jitterThreshold)
                {
                    float a = Mathf.Lerp(emaAlpha * 0.4f, Mathf.Min(emaAlpha * 2f, 0.65f),
                        Mathf.Clamp01(delta / 0.15f));
                    smoothX[i] = Mathf.Lerp(smoothX[i], lm[i].x, a);
                    smoothY[i] = Mathf.Lerp(smoothY[i], lm[i].y, a);
                    smoothZ[i] = Mathf.Lerp(smoothZ[i], lm[i].z, a);
                }
            }
        }

        // ── Calibrate body proportions from MediaPipe data ────────
        if (!mpCalibrated && V(lm, L_SHOULDER) && V(lm, R_SHOULDER) &&
            V(lm, L_HIP) && V(lm, R_HIP))
        {
            mpShoulderWidth = Mathf.Abs(smoothX[L_SHOULDER] - smoothX[R_SHOULDER]);
            float shoulderY = (smoothY[L_SHOULDER] + smoothY[R_SHOULDER]) * 0.5f;
            float hipY = (smoothY[L_HIP] + smoothY[R_HIP]) * 0.5f;
            mpTorsoHeight = Mathf.Abs(hipY - shoulderY);

            if (mpShoulderWidth > 0.01f && mpTorsoHeight > 0.01f)
            {
                mpCalibrated = true;
                Debug.Log($"[Pose] Calibrated: MP shoulder={mpShoulderWidth:F3}, torso={mpTorsoHeight:F3}");
            }
        }

        if (!mpCalibrated) return;

        // ── Compute scale factor: map MediaPipe proportions to avatar ──
        // Use shoulder width as the reference measurement
        float scaleFactor = (avatarShoulderWidth / mpShoulderWidth) * 3f;

        // ── Convert landmarks to avatar-relative positions ────────
        // All positions are relative to hip center, then scaled to avatar size
        float hipCX = (smoothX[L_HIP] + smoothX[R_HIP]) * 0.5f;
        float hipCY = (smoothY[L_HIP] + smoothY[R_HIP]) * 0.5f;
        float hipCZ = (smoothZ[L_HIP] + smoothZ[R_HIP]) * 0.5f;

        // Convert a MediaPipe landmark to avatar world position
        // relative to hip center, scaled to avatar proportions
        Vector3 MP(int idx)
        {
            float rx = -(smoothX[idx] - hipCX) * scaleFactor; // negate X (mirror fix)
            float ry = -(smoothY[idx] - hipCY) * scaleFactor; // negate Y (flip up)
            float rz = -(smoothZ[idx] - hipCZ) * scaleFactor * 0.3f; // reduce Z depth influence

            // Place relative to avatar's hip position
            return avatarRootPos + new Vector3(rx, ry + avatarHipHeight * 0.45f, rz);
        }

        float dt = Time.deltaTime * 10f;

        // Store debug points
        debugWorldPts = new Vector3[n];
        for (int i = 0; i < n; i++) debugWorldPts[i] = MP(i);

        // ── Body position (hips) ──────────────────────────────────
        if (V(lm, L_HIP) && V(lm, R_HIP))
        {
            // Hip lateral movement only (keep grounded)
            float hipLateralX = -((smoothX[L_HIP] + smoothX[R_HIP]) * 0.5f - 0.5f) * scaleFactor;
            float hipLateralZ = -((smoothZ[L_HIP] + smoothZ[R_HIP]) * 0.5f) * scaleFactor;

            Vector3 bodyTarget = avatarRootPos + new Vector3(hipLateralX, avatarHipHeight, hipLateralZ);
            ikBodyPos = Vector3.Lerp(ikBodyPos, bodyTarget, dt);
        }

        // ── Body rotation ─────────────────────────────────────────
        if (V(lm, L_SHOULDER) && V(lm, R_SHOULDER) && V(lm, L_HIP) && V(lm, R_HIP))
        {
            Vector3 lSho = MP(L_SHOULDER), rSho = MP(R_SHOULDER);
            Vector3 lHip = MP(L_HIP), rHip = MP(R_HIP);
            Vector3 hipMid = (lHip + rHip) * 0.5f;
            Vector3 shoMid = (lSho + rSho) * 0.5f;

            Vector3 up = (shoMid - hipMid).normalized;
            Vector3 right = (rSho - lSho).normalized;
            Vector3 forward = Vector3.Cross(up, right).normalized;

            if (forward.sqrMagnitude > 0.001f)
                ikBodyRot = Quaternion.Slerp(ikBodyRot, Quaternion.LookRotation(forward, up), dt);
        }

        // ── Hands ─────────────────────────────────────────────────
        if (V(lm, L_WRIST))
            ikLeftHand = Vector3.Lerp(ikLeftHand, MP(L_WRIST), dt * 2f);
        if (V(lm, R_WRIST))
            ikRightHand = Vector3.Lerp(ikRightHand, MP(R_WRIST), dt * 2f);

        // ── Elbows (hints) ────────────────────────────────────────
        if (V(lm, L_ELBOW))
            ikLeftElbow = Vector3.Lerp(ikLeftElbow, MP(L_ELBOW), dt);
        if (V(lm, R_ELBOW))
            ikRightElbow = Vector3.Lerp(ikRightElbow, MP(R_ELBOW), dt);

        // ── Feet ──────────────────────────────────────────────────
        if (V(lm, L_ANKLE))
        {
            Vector3 ft = MP(L_ANKLE);
            ft.y = avatarRootPos.y; // always on ground
            ikLeftFoot = Vector3.Lerp(ikLeftFoot, ft, dt);
        }
        if (V(lm, R_ANKLE))
        {
            Vector3 ft = MP(R_ANKLE);
            ft.y = avatarRootPos.y; // always on ground
            ikRightFoot = Vector3.Lerp(ikRightFoot, ft, dt);
        }

        // ── Knees (hints) ─────────────────────────────────────────
        if (V(lm, L_KNEE))
            ikLeftKnee = Vector3.Lerp(ikLeftKnee, MP(L_KNEE), dt);
        if (V(lm, R_KNEE))
            ikRightKnee = Vector3.Lerp(ikRightKnee, MP(R_KNEE), dt);

        // ── Head look ─────────────────────────────────────────────
        if (V(lm, NOSE))
        {
            Vector3 noseW = MP(NOSE);
            Vector3 lookDir = Vector3.forward;
            if (V(lm, L_EAR) && V(lm, R_EAR))
            {
                Vector3 earMid = (MP(L_EAR) + MP(R_EAR)) * 0.5f;
                lookDir = (noseW - earMid).normalized;
            }
            ikLookTarget = Vector3.Lerp(ikLookTarget, noseW + lookDir * 3f, dt);
        }

        hasData = true;
    }

    // ── OnAnimatorIK ──────────────────────────────────────────────
    private void OnAnimatorIK(int layerIndex)
    {
        if (!anim || !hasData) return;

        // Body
        Vector3 fixedBody = anim.bodyPosition;
        fixedBody.y = transform.position.y + avatarHipHeight;
        anim.bodyPosition = fixedBody;
        //anim.bodyRotation = Quaternion.Slerp(anim.bodyRotation, ikBodyRot, bodyWeight);

        // Left hand
        anim.SetIKPositionWeight(AvatarIKGoal.LeftHand, handWeight);
        anim.SetIKPosition(AvatarIKGoal.LeftHand, ikLeftHand);
        anim.SetIKHintPositionWeight(AvatarIKHint.LeftElbow, 0.7f);
        anim.SetIKHintPosition(AvatarIKHint.LeftElbow, ikLeftElbow);

        // Right hand
        anim.SetIKPositionWeight(AvatarIKGoal.RightHand, handWeight);
        anim.SetIKPosition(AvatarIKGoal.RightHand, ikRightHand);
        anim.SetIKHintPositionWeight(AvatarIKHint.RightElbow, 0.7f);
        anim.SetIKHintPosition(AvatarIKHint.RightElbow, ikRightElbow);

        // Left foot
        anim.SetIKPositionWeight(AvatarIKGoal.LeftFoot, footWeight);
        anim.SetIKPosition(AvatarIKGoal.LeftFoot, ikLeftFoot);
        anim.SetIKHintPositionWeight(AvatarIKHint.LeftKnee, 0.5f);
        anim.SetIKHintPosition(AvatarIKHint.LeftKnee, ikLeftKnee);

        // Right foot
        anim.SetIKPositionWeight(AvatarIKGoal.RightFoot, footWeight);
        anim.SetIKPosition(AvatarIKGoal.RightFoot, ikRightFoot);
        anim.SetIKHintPositionWeight(AvatarIKHint.RightKnee, 0.5f);
        anim.SetIKHintPosition(AvatarIKHint.RightKnee, ikRightKnee);

        // Head
        anim.SetLookAtWeight(headWeight, 0.3f, 0.6f, 0.4f, 0.5f);
        anim.SetLookAtPosition(ikLookTarget);
    }

    bool V(LandmarkData[] lm, int i) => i < lm.Length && lm[i].visibility >= minVisibility;

    // ── Debug ────────────────────────────────────────────────────
    void OnDrawGizmos()
    {
        if (!drawDebugSkeleton || debugWorldPts == null || debugWorldPts.Length < 29) return;

        int[] keys = { NOSE, L_SHOULDER, R_SHOULDER, L_ELBOW, R_ELBOW, L_WRIST, R_WRIST,
                      L_HIP, R_HIP, L_KNEE, R_KNEE, L_ANKLE, R_ANKLE };
        Gizmos.color = Color.green;
        foreach (int i in keys) if (i < debugWorldPts.Length) Gizmos.DrawSphere(debugWorldPts[i], 0.03f);

        Gizmos.color = Color.cyan;
        GL(L_SHOULDER, R_SHOULDER); GL(L_HIP, R_HIP); GL(L_SHOULDER, L_HIP); GL(R_SHOULDER, R_HIP);
        Gizmos.color = Color.yellow;
        GL(L_SHOULDER, L_ELBOW); GL(L_ELBOW, L_WRIST);
        Gizmos.color = Color.red;
        GL(R_SHOULDER, R_ELBOW); GL(R_ELBOW, R_WRIST);
        Gizmos.color = Color.magenta;
        GL(L_HIP, L_KNEE); GL(L_KNEE, L_ANKLE);
        Gizmos.color = Color.blue;
        GL(R_HIP, R_KNEE); GL(R_KNEE, R_ANKLE);

        // IK targets
        Gizmos.color = new Color(1, 0.5f, 0, 0.7f);
        Gizmos.DrawWireSphere(ikLeftHand, 0.05f);
        Gizmos.DrawWireSphere(ikRightHand, 0.05f);
        Gizmos.color = new Color(0, 0.5f, 1, 0.7f);
        Gizmos.DrawWireSphere(ikLeftFoot, 0.05f);
        Gizmos.DrawWireSphere(ikRightFoot, 0.05f);

        // Ground line
        Gizmos.color = Color.gray;
        Gizmos.DrawLine(avatarRootPos + Vector3.left * 2, avatarRootPos + Vector3.right * 2);
    }

    void GL(int a, int b)
    {
        if (debugWorldPts != null && a < debugWorldPts.Length && b < debugWorldPts.Length)
            Gizmos.DrawLine(debugWorldPts[a], debugWorldPts[b]);
    }
}

// ── JSON ───────────────────────────────────────────────────────────
[Serializable] public class LandmarkFrame { public string type; public long timestamp; public LandmarkData[] landmarks; }
[Serializable] public class LandmarkData { public float x, y, z, visibility; }