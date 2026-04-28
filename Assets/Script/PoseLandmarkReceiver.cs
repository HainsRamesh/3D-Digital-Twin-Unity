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
/// Handles occlusion gracefully: when a body part loses visibility,
/// it holds for a short delay, then drifts toward a natural rest pose
/// instead of freezing or jittering.
/// 
/// REQUIRES: AnimatorController with IK Pass enabled.
/// </summary>
[RequireComponent(typeof(Animator))]
public class PoseLandmarkReceiver : M2MqttUnityClient
{
    [Header("Fine Tuning")]
    [Range(0.5f, 1.5f)] public float handReachScale = 1f;

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

    [Header("Occlusion Handling")]
    [Tooltip("Seconds to hold last good position before drifting to rest")]
    [Range(0f, 2f)] public float missingHoldDelay = 0.5f;
    [Tooltip("How quickly missing limbs relax to rest pose (higher = faster)")]
    [Range(0f, 2f)] public float missingDecayRate = 0.3f;

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
    const float MAX_JUMP_PER_FRAME = 0.4f;   // meters — reject teleports

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

    // ── Occlusion: time since each limb was last seen ────────────
    private float leftHandMissing = 0f, rightHandMissing = 0f;
    private float leftElbowMissing = 0f, rightElbowMissing = 0f;
    private float leftFootMissing = 0f, rightFootMissing = 0f;
    private float leftKneeMissing = 0f, rightKneeMissing = 0f;

    // ── EMA smoothing ───────────────────────────────────────────
    private float[] smoothX, smoothY, smoothZ;
    private bool emaInit = false;

    // ── MediaPipe body measurements (learned from first frame) ───
    private float mpHipAnkleDist = 0.3f;
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

    Vector3 SafeLerp(Vector3 current, Vector3 target, float t)
    {
        Vector3 delta = target - current;
        if (delta.magnitude > MAX_JUMP_PER_FRAME)
            target = current + delta.normalized * MAX_JUMP_PER_FRAME;
        return Vector3.Lerp(current, target, t);
    }

    /// <summary>
    /// Updates a missing-time counter and drifts the IK target toward a rest position
    /// once the limb has been invisible for longer than missingHoldDelay.
    /// Pure decay logic — call this only when the landmark is NOT visible.
    /// </summary>
    Vector3 DriftToRest(Vector3 current, Vector3 restPos, ref float missingTimer, float dt)
    {
        missingTimer += Time.deltaTime;
        if (missingTimer > missingHoldDelay)
        {
            return Vector3.Lerp(current, restPos, dt * missingDecayRate);
        }
        return current; // hold last good position during the delay window
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
            V(lm, L_HIP) && V(lm, R_HIP) &&
            V(lm, L_ANKLE) && V(lm, R_ANKLE))
        {
            mpShoulderWidth = Mathf.Abs(smoothX[L_SHOULDER] - smoothX[R_SHOULDER]);
            float shoulderY = (smoothY[L_SHOULDER] + smoothY[R_SHOULDER]) * 0.5f;
            float hipY = (smoothY[L_HIP] + smoothY[R_HIP]) * 0.5f;
            float ankleY = (smoothY[L_ANKLE] + smoothY[R_ANKLE]) * 0.5f;
            mpTorsoHeight = Mathf.Abs(hipY - shoulderY);
            mpHipAnkleDist = Mathf.Abs(ankleY - hipY);

            if (mpShoulderWidth > 0.01f && mpTorsoHeight > 0.01f && mpHipAnkleDist > 0.01f)
            {
                mpCalibrated = true;
                Debug.Log($"[Pose] Calibrated: shoulder={mpShoulderWidth:F3}, " +
                          $"torso={mpTorsoHeight:F3}, hipAnkle={mpHipAnkleDist:F3}");
            }
        }

        if (!mpCalibrated) return;

        // ── Compute scale factors ──────────────────────────────────
        float scaleFactor = (avatarShoulderWidth / mpShoulderWidth);
        float yScaleFactor = avatarHipHeight / mpHipAnkleDist;

        // ── Hip center ─────────────────────────────────────────────
        float hipCX = (smoothX[L_HIP] + smoothX[R_HIP]) * 0.5f;
        float hipCY = (smoothY[L_HIP] + smoothY[R_HIP]) * 0.5f;
        float hipCZ = (smoothZ[L_HIP] + smoothZ[R_HIP]) * 0.5f;

        // Convert a MediaPipe landmark to avatar world position
        Vector3 MP(int idx)
        {
            float rx = -(smoothX[idx] - hipCX) * scaleFactor;
            float ry = -(smoothY[idx] - hipCY) * yScaleFactor;
            float rz = -(smoothZ[idx] - hipCZ) * scaleFactor * 0.3f;

            return avatarRootPos + new Vector3(rx, ry + avatarHipHeight, rz);
        }

        float dt = Time.deltaTime * 18f;

        // Store debug points
        debugWorldPts = new Vector3[n];
        for (int i = 0; i < n; i++) debugWorldPts[i] = MP(i);

        // ── Body position (hips) ──────────────────────────────────
        if (V(lm, L_HIP) && V(lm, R_HIP))
        {
            float hipLateralX = -((smoothX[L_HIP] + smoothX[R_HIP]) * 0.5f - 0.5f) * scaleFactor;
            float hipLateralZ = -((smoothZ[L_HIP] + smoothZ[R_HIP]) * 0.5f) * scaleFactor;

            Vector3 bodyTarget = avatarRootPos + new Vector3(hipLateralX, avatarHipHeight, hipLateralZ);
            ikBodyPos = Vector3.Lerp(ikBodyPos, bodyTarget, dt);
        }

        // ── Body rotation (Y-axis twist only, horizontal plane) ──
        if (V(lm, L_SHOULDER) && V(lm, R_SHOULDER))
        {
            Vector3 lSho = MP(L_SHOULDER), rSho = MP(R_SHOULDER);

            Vector3 shoulderLine = rSho - lSho;
            shoulderLine.y = 0f;

            if (shoulderLine.sqrMagnitude > 0.001f)
            {
                Vector3 right = shoulderLine.normalized;
                Vector3 forward = Vector3.Cross(right, Vector3.up).normalized;

                Quaternion targetRot = Quaternion.LookRotation(forward, Vector3.up);
                ikBodyRot = Quaternion.Slerp(ikBodyRot, targetRot, dt);
            }
        }

        // ── Hands (with occlusion handling) ───────────────────────
        if (V(lm, L_WRIST))
        {
            leftHandMissing = 0f;
            Vector3 shoulderPos = MP(L_SHOULDER);
            Vector3 target = shoulderPos + (MP(L_WRIST) - shoulderPos) * handReachScale;
            ikLeftHand = SafeLerp(ikLeftHand, target, dt * 2f);
        }
        else
        {
            // Rest pose: hand hangs naturally at side, below shoulder
            Vector3 shoulderPos = V(lm, L_SHOULDER) ? MP(L_SHOULDER)
                : avatarRootPos + new Vector3(-avatarShoulderWidth * 0.5f, avatarHipHeight + 0.5f, 0);
            Vector3 restPos = shoulderPos + Vector3.down * avatarArmLength * 0.9f;
            ikLeftHand = DriftToRest(ikLeftHand, restPos, ref leftHandMissing, dt);
        }

        if (V(lm, R_WRIST))
        {
            rightHandMissing = 0f;
            Vector3 shoulderPos = MP(R_SHOULDER);
            Vector3 target = shoulderPos + (MP(R_WRIST) - shoulderPos) * handReachScale;
            ikRightHand = SafeLerp(ikRightHand, target, dt * 2f);
        }
        else
        {
            Vector3 shoulderPos = V(lm, R_SHOULDER) ? MP(R_SHOULDER)
                : avatarRootPos + new Vector3(avatarShoulderWidth * 0.5f, avatarHipHeight + 0.5f, 0);
            Vector3 restPos = shoulderPos + Vector3.down * avatarArmLength * 0.9f;
            ikRightHand = DriftToRest(ikRightHand, restPos, ref rightHandMissing, dt);
        }

        // ── Elbows (with occlusion handling) ──────────────────────
        if (V(lm, L_ELBOW))
        {
            leftElbowMissing = 0f;
            ikLeftElbow = Vector3.Lerp(ikLeftElbow, MP(L_ELBOW), dt * 1.5f);
        }
        else
        {
            // Rest: elbow halfway between shoulder and resting hand position
            Vector3 restPos = (ikLeftHand + (V(lm, L_SHOULDER) ? MP(L_SHOULDER) : ikLeftHand)) * 0.5f;
            ikLeftElbow = DriftToRest(ikLeftElbow, restPos, ref leftElbowMissing, dt);
        }

        if (V(lm, R_ELBOW))
        {
            rightElbowMissing = 0f;
            ikRightElbow = Vector3.Lerp(ikRightElbow, MP(R_ELBOW), dt * 1.5f);
        }
        else
        {
            Vector3 restPos = (ikRightHand + (V(lm, R_SHOULDER) ? MP(R_SHOULDER) : ikRightHand)) * 0.5f;
            ikRightElbow = DriftToRest(ikRightElbow, restPos, ref rightElbowMissing, dt);
        }

        // ── Feet (with occlusion handling) ────────────────────────
        if (V(lm, L_ANKLE))
        {
            leftFootMissing = 0f;
            Vector3 ft = MP(L_ANKLE);
            float groundY = transform.position.y;
            if (ft.y < groundY + 0.05f)
                ft.y = groundY;
            ikLeftFoot = SafeLerp(ikLeftFoot, ft, dt * 1.5f);
        }
        else
        {
            // Rest: foot on ground beneath the hip
            Vector3 restPos = avatarRootPos + new Vector3(-avatarShoulderWidth * 0.4f, 0, 0);
            ikLeftFoot = DriftToRest(ikLeftFoot, restPos, ref leftFootMissing, dt);
        }

        if (V(lm, R_ANKLE))
        {
            rightFootMissing = 0f;
            Vector3 ft = MP(R_ANKLE);
            float groundY = transform.position.y;
            if (ft.y < groundY + 0.05f)
                ft.y = groundY;
            ikRightFoot = SafeLerp(ikRightFoot, ft, dt * 1.5f);
        }
        else
        {
            Vector3 restPos = avatarRootPos + new Vector3(avatarShoulderWidth * 0.4f, 0, 0);
            ikRightFoot = DriftToRest(ikRightFoot, restPos, ref rightFootMissing, dt);
        }

        // ── Knees (with occlusion handling) ───────────────────────
        if (V(lm, L_KNEE))
        {
            leftKneeMissing = 0f;
            ikLeftKnee = Vector3.Lerp(ikLeftKnee, MP(L_KNEE), dt * 1.5f);
        }
        else
        {
            // Rest: knee halfway between hip and resting foot
            Vector3 hipPos = avatarRootPos + new Vector3(-avatarShoulderWidth * 0.4f, avatarHipHeight, 0);
            Vector3 restPos = (hipPos + ikLeftFoot) * 0.5f;
            ikLeftKnee = DriftToRest(ikLeftKnee, restPos, ref leftKneeMissing, dt);
        }

        if (V(lm, R_KNEE))
        {
            rightKneeMissing = 0f;
            ikRightKnee = Vector3.Lerp(ikRightKnee, MP(R_KNEE), dt * 1.5f);
        }
        else
        {
            Vector3 hipPos = avatarRootPos + new Vector3(avatarShoulderWidth * 0.4f, avatarHipHeight, 0);
            Vector3 restPos = (hipPos + ikRightFoot) * 0.5f;
            ikRightKnee = DriftToRest(ikRightKnee, restPos, ref rightKneeMissing, dt);
        }

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
        anim.bodyRotation = Quaternion.Slerp(anim.bodyRotation, ikBodyRot, bodyWeight);

        // Left hand
        anim.SetIKPositionWeight(AvatarIKGoal.LeftHand, handWeight);
        anim.SetIKPosition(AvatarIKGoal.LeftHand, ikLeftHand);
        anim.SetIKHintPositionWeight(AvatarIKHint.LeftElbow, 1f);
        anim.SetIKHintPosition(AvatarIKHint.LeftElbow, ikLeftElbow);

        // Right hand
        anim.SetIKPositionWeight(AvatarIKGoal.RightHand, handWeight);
        anim.SetIKPosition(AvatarIKGoal.RightHand, ikRightHand);
        anim.SetIKHintPositionWeight(AvatarIKHint.RightElbow, 1f);
        anim.SetIKHintPosition(AvatarIKHint.RightElbow, ikRightElbow);

        // Left foot
        anim.SetIKPositionWeight(AvatarIKGoal.LeftFoot, footWeight);
        anim.SetIKPosition(AvatarIKGoal.LeftFoot, ikLeftFoot);
        anim.SetIKHintPositionWeight(AvatarIKHint.LeftKnee, 1f);
        anim.SetIKHintPosition(AvatarIKHint.LeftKnee, ikLeftKnee);

        // Right foot
        anim.SetIKPositionWeight(AvatarIKGoal.RightFoot, footWeight);
        anim.SetIKPosition(AvatarIKGoal.RightFoot, ikRightFoot);
        anim.SetIKHintPositionWeight(AvatarIKHint.RightKnee, 1f);
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