using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using M2MqttUnity;
using uPLibrary.Networking.M2Mqtt.Messages;

/// <summary>
/// Production IK-based pose retargeting for L&T Digital Twin.
///
/// Hand and elbow IK targets are anchored on the avatar's actual bones,
/// using MediaPipe only for direction. The elbow hint is hinge-constrained
/// so the joint can never bend backward. Palm orientation is computed
/// dynamically in LateUpdate so palms always face the body's centerline
/// regardless of arm pose.
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

    [Header("Smoothing (One-Euro Filter)")]
    [Tooltip("Lower = smoother when still (more lag). Typical 0.5–2.0")]
    [Range(0.1f, 5f)] public float oneEuroMinCutoff = 1.0f;
    [Tooltip("Higher = less lag when moving fast. Typical 0.001–0.05")]
    [Range(0f, 0.1f)] public float oneEuroBeta = 0.007f;

    [Header("Visibility")]
    [Range(0f, 1f)] public float minVisibility = 0.3f;
    [Tooltip("Stricter threshold for wrists, which often misdetect when off-camera or close to body")]
    [Range(0f, 1f)] public float minWristVisibility = 0.55f;

    [Header("Stability")]
    [Tooltip("Body twist deadband ratio. Suppresses depth-noise twist when facing camera.")]
    [Range(0f, 1f)] public float bodyTwistDeadband = 0.4f;
    [Tooltip("Damping factor on body-rotation Slerp. Lower = smoother but laggier on real turns.")]
    [Range(0.1f, 2f)] public float bodyRotationDamping = 0.5f;
    [Tooltip("How much the head can turn independently of the body.")]
    [Range(0f, 1f)] public float headTurnAmount = 0.5f;

    [Header("Hand Twist (post-IK)")]
    [Tooltip("How strongly to apply the post-IK hand twist. 0 = no twist (use IK default), 1 = full twist.")]
    [Range(0f, 1f)] public float handTwistWeight = 1f;
    [Tooltip("Extra wrist roll in degrees around the arm's own axis, mirrored between hands. " +
             "Leave at 0 for a palms-down T-pose rig. Set to 180 for a palms-up rig.")]
    [Range(-180f, 180f)] public float handTwistOffset = 0f;
    [Tooltip("Forward wrist flex in degrees. Positive bends hands slightly forward (natural idle pose).")]
    [Range(-45f, 45f)] public float handFlexAngle = -15f;

    [Header("Occlusion Handling")]
    [Tooltip("Seconds to hold last good position before drifting to rest")]
    [Range(0f, 2f)] public float missingHoldDelay = 0.5f;
    [Tooltip("How quickly missing limbs relax to rest pose (higher = faster)")]
    [Range(0f, 2f)] public float missingDecayRate = 0.3f;

    [Header("Anatomical Constraints")]
    [Range(0f, 1f)] public float elbowForwardBias = 0.4f;
    [Range(0f, 1f)] public float kneeForwardBias = 0.3f;
    [Range(0.5f, 1.5f)] public float maxArmReach = 0.98f;
    [Range(0.5f, 1.5f)] public float maxLegReach = 0.98f;
    [Range(0f, 0.5f)] public float groundSnapDistance = 0.2f;
    [Tooltip("Vertical offset to align foot bone with floor.")]
    [Range(-0.2f, 0.2f)] public float footOffset = 0f;

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
    const float MAX_JUMP_PER_FRAME = 0.4f;

    // ── Animator & avatar measurements ───────────────────────────
    private Animator anim;
    private float avatarHeight = 1.7f;
    private float avatarLegLength = 0.85f;
    private float avatarArmLength = 0.55f;
    private float avatarShoulderWidth = 0.35f;
    private Vector3 avatarRootPos;
    private float avatarHipHeight;

    // ── IK targets ───────────────────────────────────────────────
    private Vector3 ikLeftHand, ikRightHand;
    private Vector3 ikLeftFoot, ikRightFoot;
    private Vector3 ikLeftElbow, ikRightElbow;
    private Vector3 ikLeftKnee, ikRightKnee;
    private Vector3 ikBodyPos;
    private Quaternion ikBodyRot = Quaternion.identity;
    private Vector3 ikLookTarget;
    private bool hasData = false;

    // ── Palm-direction local axes captured from bind pose ────────
    private Vector3 leftPalmAxisLocal = Vector3.down;
    private Vector3 rightPalmAxisLocal = Vector3.down;
    private bool palmAxesCaptured = false;

    // ── Occlusion timers ─────────────────────────────────────────
    private float leftHandMissing = 0f, rightHandMissing = 0f;
    private float leftElbowMissing = 0f, rightElbowMissing = 0f;
    private float leftFootMissing = 0f, rightFootMissing = 0f;
    private float leftKneeMissing = 0f, rightKneeMissing = 0f;

    // ── One-Euro filters ─────────────────────────────────────────
    private OneEuroFilter[] filtersX, filtersY, filtersZ;
    private float[] smoothX, smoothY, smoothZ;
    private bool filtersInit = false;

    // ── MediaPipe measurements ───────────────────────────────────
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

        Transform head = anim.GetBoneTransform(HumanBodyBones.Head);
        Transform hips = anim.GetBoneTransform(HumanBodyBones.Hips);
        Transform lFoot = anim.GetBoneTransform(HumanBodyBones.LeftFoot);
        Transform lHand = anim.GetBoneTransform(HumanBodyBones.LeftHand);
        Transform lShoulder = anim.GetBoneTransform(HumanBodyBones.LeftUpperArm);
        Transform rShoulder = anim.GetBoneTransform(HumanBodyBones.RightUpperArm);
        Transform lElbow = anim.GetBoneTransform(HumanBodyBones.LeftLowerArm);

        if (head && lFoot) avatarHeight = head.position.y - lFoot.position.y;
        if (hips && lFoot) avatarHipHeight = hips.position.y - lFoot.position.y;
        if (hips && lFoot) avatarLegLength = hips.position.y - lFoot.position.y;
        if (lShoulder && lHand) avatarArmLength = Vector3.Distance(lShoulder.position, lHand.position);
        if (lShoulder && rShoulder) avatarShoulderWidth = Vector3.Distance(lShoulder.position, rShoulder.position);

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

        // Capture each hand's "palm-direction" local axis from the bind pose.
        // Assumes a T-pose palms-down rig (Unity Mecanim convention). For a palms-up
        // rig, set handTwistOffset = 180 in the inspector.
        Transform rHandBone = anim.GetBoneTransform(HumanBodyBones.RightHand);
        if (lHand != null) leftPalmAxisLocal = Quaternion.Inverse(lHand.rotation) * Vector3.down;
        if (rHandBone != null) rightPalmAxisLocal = Quaternion.Inverse(rHandBone.rotation) * Vector3.down;
        palmAxesCaptured = true;

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

    Vector3 DriftToRest(Vector3 current, Vector3 restPos, ref float missingTimer, float dt)
    {
        missingTimer += Time.deltaTime;
        if (missingTimer > missingHoldDelay)
            return Vector3.Lerp(current, restPos, dt * missingDecayRate);
        return current;
    }

    Vector3 ComputeSmartHint(Vector3 jointStart, Vector3 jointEnd, Vector3 rawHint, float forwardBias)
    {
        if (forwardBias < 0.01f) return rawHint;
        Vector3 forward = ikBodyRot * Vector3.forward;
        forward.y = 0f;
        if (forward.sqrMagnitude < 0.001f) forward = Vector3.forward;
        forward.Normalize();
        return rawHint + forward * forwardBias;
    }

    /// <summary>
    /// Anatomically-safe elbow hint. Continuous across all extension values
    /// (no discontinuity that flips bend direction). The perpendicular offset
    /// from the shoulder→hand line is forced to have a non-negative forward
    /// component, with a ramped minimum near full extension so noise can't
    /// flip the bend direction during the bent → straight transition.
    /// </summary>
    Vector3 ComputeElbowHint(Vector3 shoulder, Vector3 hand, Vector3 rawElbow, float extension)
    {
        Vector3 forward = ikBodyRot * Vector3.forward;
        forward.y = 0f;
        if (forward.sqrMagnitude < 0.001f) forward = Vector3.forward;
        forward.Normalize();

        // Continuous base: midpoint blended with raw elbow by bend factor.
        float bendFactor = Mathf.Clamp01(1f - extension);
        Vector3 midpoint = (shoulder + hand) * 0.5f;
        Vector3 baseHint = Vector3.Lerp(midpoint, rawElbow, bendFactor)
                         + forward * (elbowForwardBias * bendFactor);

        // Decompose into "along shoulder→hand line" + "perpendicular to it".
        Vector3 armLine = hand - shoulder;
        float armLen = armLine.magnitude;
        if (armLen < 0.001f) return baseHint;
        Vector3 armDir = armLine / armLen;

        Vector3 offsetFromShoulder = baseHint - shoulder;
        Vector3 alongArm = Vector3.Project(offsetFromShoulder, armDir);
        Vector3 perpendicular = offsetFromShoulder - alongArm;

        // Reflect any backward portion of the perpendicular component to forward.
        float fwdAmount = Vector3.Dot(perpendicular, forward);
        if (fwdAmount < 0f)
        {
            perpendicular -= 2f * fwdAmount * forward;
            fwdAmount = -fwdAmount;
        }

        // Ramp a minimum forward offset as the arm straightens.
        float ramp = Mathf.Clamp01((extension - 0.8f) * 5f);
        float minFwd = armLen * 0.08f * ramp;
        if (fwdAmount < minFwd)
            perpendicular += (minFwd - fwdAmount) * forward;

        return shoulder + alongArm + perpendicular;
    }

    Vector3 ClampReach(Vector3 anchor, Vector3 target, float maxReach)
    {
        Vector3 delta = target - anchor;
        float distance = delta.magnitude;
        if (distance > maxReach && distance > 0.001f)
            return anchor + delta * (maxReach / distance);
        return target;
    }

    /// <summary>
    /// Reflects the hand IK target's backward component to forward, so the elbow
    /// (a hinge joint) never has to bend backward to reach the wrist. The lateral
    /// and vertical components are preserved.
    /// </summary>
    Vector3 ClampInFrontOfBody(Vector3 handTarget, Vector3 shoulderRef)
    {
        Vector3 bodyFwd = ikBodyRot * Vector3.forward;
        bodyFwd.y = 0f;
        if (bodyFwd.sqrMagnitude < 0.001f) return handTarget;
        bodyFwd.Normalize();

        Vector3 offset = handTarget - shoulderRef;
        float fwdComponent = Vector3.Dot(offset, bodyFwd);
        if (fwdComponent < 0f)
        {
            offset -= 2f * fwdComponent * bodyFwd;
            return shoulderRef + offset;
        }
        return handTarget;
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

        // ── One-Euro smoothing ────────────────────────────────────
        float dtSmooth = Mathf.Max(Time.deltaTime, 1e-4f);
        if (!filtersInit)
        {
            filtersX = new OneEuroFilter[n];
            filtersY = new OneEuroFilter[n];
            filtersZ = new OneEuroFilter[n];
            smoothX = new float[n]; smoothY = new float[n]; smoothZ = new float[n];
            for (int i = 0; i < n; i++)
            {
                filtersX[i] = new OneEuroFilter(oneEuroMinCutoff, oneEuroBeta);
                filtersY[i] = new OneEuroFilter(oneEuroMinCutoff, oneEuroBeta);
                filtersZ[i] = new OneEuroFilter(oneEuroMinCutoff, oneEuroBeta);
                smoothX[i] = lm[i].x; smoothY[i] = lm[i].y; smoothZ[i] = lm[i].z;
            }
            filtersInit = true;
        }
        else
        {
            for (int i = 0; i < n; i++)
            {
                filtersX[i].minCutoff = oneEuroMinCutoff; filtersX[i].beta = oneEuroBeta;
                filtersY[i].minCutoff = oneEuroMinCutoff; filtersY[i].beta = oneEuroBeta;
                filtersZ[i].minCutoff = oneEuroMinCutoff; filtersZ[i].beta = oneEuroBeta;
                smoothX[i] = filtersX[i].Filter(lm[i].x, dtSmooth);
                smoothY[i] = filtersY[i].Filter(lm[i].y, dtSmooth);
                smoothZ[i] = filtersZ[i].Filter(lm[i].z, dtSmooth);
            }
        }

        // ── Dynamic calibration ────────────────────────────────────
        if (V(lm, L_SHOULDER) && V(lm, R_SHOULDER) &&
            V(lm, L_HIP) && V(lm, R_HIP) &&
            V(lm, L_ANKLE) && V(lm, R_ANKLE))
        {
            float currentShoulderWidth = Mathf.Abs(smoothX[L_SHOULDER] - smoothX[R_SHOULDER]);
            float shoulderY = (smoothY[L_SHOULDER] + smoothY[R_SHOULDER]) * 0.5f;
            float hipY = (smoothY[L_HIP] + smoothY[R_HIP]) * 0.5f;
            float ankleY = (smoothY[L_ANKLE] + smoothY[R_ANKLE]) * 0.5f;
            float currentTorsoHeight = Mathf.Abs(hipY - shoulderY);
            float currentHipAnkleDist = Mathf.Abs(ankleY - hipY);

            if (currentShoulderWidth > 0.01f && currentTorsoHeight > 0.01f && currentHipAnkleDist > 0.01f)
            {
                if (!mpCalibrated)
                {
                    mpShoulderWidth = currentShoulderWidth;
                    mpTorsoHeight = currentTorsoHeight;
                    mpHipAnkleDist = currentHipAnkleDist;
                    mpCalibrated = true;
                    Debug.Log($"[Pose] Initial calibration: shoulder={mpShoulderWidth:F3}, " +
                              $"torso={mpTorsoHeight:F3}, hipAnkle={mpHipAnkleDist:F3}");
                }
                else
                {
                    const float SCALE_SMOOTHING = 0.05f;
                    mpShoulderWidth = Mathf.Lerp(mpShoulderWidth, currentShoulderWidth, SCALE_SMOOTHING);
                    mpTorsoHeight = Mathf.Lerp(mpTorsoHeight, currentTorsoHeight, SCALE_SMOOTHING);
                    mpHipAnkleDist = Mathf.Lerp(mpHipAnkleDist, currentHipAnkleDist, SCALE_SMOOTHING);
                }
            }
        }
        if (!mpCalibrated) return;

        // ── World landmarks check ─────────────────────────────────
        bool hasWorldLandmarks = f.worldLandmarks != null && f.worldLandmarks.Length >= 33;

        Vector3 wHipCenter = Vector3.zero;
        float worldScale = 1f;
        if (hasWorldLandmarks)
        {
            Vector3 wHipL = new Vector3(-f.worldLandmarks[L_HIP].x, -f.worldLandmarks[L_HIP].y, -f.worldLandmarks[L_HIP].z);
            Vector3 wHipR = new Vector3(-f.worldLandmarks[R_HIP].x, -f.worldLandmarks[R_HIP].y, -f.worldLandmarks[R_HIP].z);
            wHipCenter = (wHipL + wHipR) * 0.5f;

            Vector3 wShoL = new Vector3(-f.worldLandmarks[L_SHOULDER].x, -f.worldLandmarks[L_SHOULDER].y, -f.worldLandmarks[L_SHOULDER].z);
            Vector3 wShoR = new Vector3(-f.worldLandmarks[R_SHOULDER].x, -f.worldLandmarks[R_SHOULDER].y, -f.worldLandmarks[R_SHOULDER].z);
            float wShoulderWidth = Vector3.Distance(wShoL, wShoR);
            if (wShoulderWidth > 0.01f) worldScale = avatarShoulderWidth / wShoulderWidth;
        }

        Vector3 MPW(int idx)
        {
            if (!hasWorldLandmarks) return MP(idx);
            var wl = f.worldLandmarks[idx];
            Vector3 raw = new Vector3(-wl.x, -wl.y, -wl.z);
            Vector3 relative = (raw - wHipCenter) * worldScale;
            return avatarRootPos + new Vector3(0, avatarHipHeight, 0) + relative;
        }

        float scaleFactor = avatarShoulderWidth / mpShoulderWidth;
        float yScaleFactor = avatarHipHeight / mpHipAnkleDist;

        float hipCX = (smoothX[L_HIP] + smoothX[R_HIP]) * 0.5f;
        float hipCY = (smoothY[L_HIP] + smoothY[R_HIP]) * 0.5f;
        float hipCZ = (smoothZ[L_HIP] + smoothZ[R_HIP]) * 0.5f;

        Vector3 MP(int idx)
        {
            float rx = -(smoothX[idx] - hipCX) * scaleFactor;
            float ry = -(smoothY[idx] - hipCY) * yScaleFactor;
            float rz = -(smoothZ[idx] - hipCZ) * scaleFactor * 0.3f;
            return avatarRootPos + new Vector3(rx, ry + avatarHipHeight, rz);
        }

        float dt = Time.deltaTime * 18f;

        debugWorldPts = new Vector3[n];
        for (int i = 0; i < n; i++) debugWorldPts[i] = MP(i);

        // ── Body position ─────────────────────────────────────────
        if (V(lm, L_HIP) && V(lm, R_HIP))
        {
            float hipLateralX = -((smoothX[L_HIP] + smoothX[R_HIP]) * 0.5f - 0.5f) * scaleFactor;
            float hipLateralZ = -((smoothZ[L_HIP] + smoothZ[R_HIP]) * 0.5f) * scaleFactor;
            Vector3 bodyTarget = avatarRootPos + new Vector3(hipLateralX, avatarHipHeight, hipLateralZ);
            ikBodyPos = Vector3.Lerp(ikBodyPos, bodyTarget, dt);
        }

        // ── Body rotation with z-deadband ─────────────────────────
        if (V(lm, L_SHOULDER) && V(lm, R_SHOULDER))
        {
            Vector3 lSho = MP(L_SHOULDER), rSho = MP(R_SHOULDER);
            Vector3 shoulderLine = rSho - lSho;
            shoulderLine.y = 0f;

            if (shoulderLine.sqrMagnitude > 0.001f)
            {
                float absX = Mathf.Abs(shoulderLine.x);
                float absZ = Mathf.Abs(shoulderLine.z);
                if (absZ < absX * bodyTwistDeadband)
                    shoulderLine.z = 0f;
                else
                {
                    float t = Mathf.InverseLerp(absX * bodyTwistDeadband, absX, absZ);
                    shoulderLine.z *= t;
                }

                Vector3 right = shoulderLine.normalized;
                Vector3 forward = Vector3.Cross(right, Vector3.up).normalized;
                Quaternion targetRot = Quaternion.LookRotation(forward, Vector3.up);
                ikBodyRot = Quaternion.Slerp(ikBodyRot, targetRot, dt * bodyRotationDamping);
            }
        }

        // ── Left hand: anchored on the upper-arm bone ─────────────
        if (VWrist(lm, L_WRIST, L_SHOULDER, L_HIP) && V(lm, L_SHOULDER) && V(lm, L_ELBOW))
        {
            leftHandMissing = 0f;

            Vector2 sho2D = new Vector2(smoothX[L_SHOULDER], smoothY[L_SHOULDER]);
            Vector2 elb2D = new Vector2(smoothX[L_ELBOW], smoothY[L_ELBOW]);
            Vector2 wri2D = new Vector2(smoothX[L_WRIST], smoothY[L_WRIST]);
            float mpChain = Vector2.Distance(sho2D, elb2D) + Vector2.Distance(elb2D, wri2D);
            float mpChord = Vector2.Distance(sho2D, wri2D);
            float extension = (mpChain > 0.01f) ? Mathf.Clamp01(mpChord / mpChain) : 1f;
            if (extension > 0.95f) extension = 1.0f;

            // Anchor on actual bone. MediaPipe gives direction only.
            Transform lUpperArmBone = anim.GetBoneTransform(HumanBodyBones.LeftUpperArm);
            Vector3 anchor = (lUpperArmBone != null) ? lUpperArmBone.position : MPW(L_SHOULDER);
            Vector3 dir = (MPW(L_WRIST) - MPW(L_SHOULDER)).normalized;
            Vector3 target = anchor + dir * (extension * avatarArmLength * handReachScale);
            target = ClampReach(anchor, target, avatarArmLength * maxArmReach);

            ikLeftHand = SafeLerp(ikLeftHand, target, dt * 2f);
        }
        else
        {
            Vector3 shoulderPos = V(lm, L_SHOULDER) ? MP(L_SHOULDER)
                : avatarRootPos + new Vector3(-avatarShoulderWidth * 0.5f, avatarHipHeight + 0.5f, 0);
            Vector3 restPos = shoulderPos + Vector3.down * avatarArmLength * 0.9f;
            ikLeftHand = DriftToRest(ikLeftHand, restPos, ref leftHandMissing, dt);
        }

        if (VWrist(lm, R_WRIST, R_SHOULDER, R_HIP) && V(lm, R_SHOULDER) && V(lm, R_ELBOW))
        {
            rightHandMissing = 0f;

            Vector2 sho2D = new Vector2(smoothX[R_SHOULDER], smoothY[R_SHOULDER]);
            Vector2 elb2D = new Vector2(smoothX[R_ELBOW], smoothY[R_ELBOW]);
            Vector2 wri2D = new Vector2(smoothX[R_WRIST], smoothY[R_WRIST]);
            float mpChain = Vector2.Distance(sho2D, elb2D) + Vector2.Distance(elb2D, wri2D);
            float mpChord = Vector2.Distance(sho2D, wri2D);
            float extension = (mpChain > 0.01f) ? Mathf.Clamp01(mpChord / mpChain) : 1f;

            Transform rUpperArmBone = anim.GetBoneTransform(HumanBodyBones.RightUpperArm);
            Vector3 anchor = (rUpperArmBone != null) ? rUpperArmBone.position : MPW(R_SHOULDER);
            Vector3 dir = (MPW(R_WRIST) - MPW(R_SHOULDER)).normalized;
            Vector3 target = anchor + dir * (extension * avatarArmLength * handReachScale);
            target = ClampReach(anchor, target, avatarArmLength * maxArmReach);

            ikRightHand = SafeLerp(ikRightHand, target, dt * 2f);
        }
        else
        {
            Vector3 shoulderPos = V(lm, R_SHOULDER) ? MP(R_SHOULDER)
                : avatarRootPos + new Vector3(avatarShoulderWidth * 0.5f, avatarHipHeight + 0.5f, 0);
            Vector3 restPos = shoulderPos + Vector3.down * avatarArmLength * 0.9f;
            ikRightHand = DriftToRest(ikRightHand, restPos, ref rightHandMissing, dt);
        }

        // ── Elbows: anchored on bone, MediaPipe offset applied ────
        if (V(lm, L_ELBOW))
        {
            leftElbowMissing = 0f;
            Transform lUpperArmBone = anim.GetBoneTransform(HumanBodyBones.LeftUpperArm);
            Vector3 boneShoulder = (lUpperArmBone != null) ? lUpperArmBone.position : ikBodyPos;

            Vector3 rawElbowAnchored = V(lm, L_SHOULDER)
                ? boneShoulder + (MPW(L_ELBOW) - MPW(L_SHOULDER))
                : MPW(L_ELBOW);

            float extension = 1f;
            if (V(lm, L_SHOULDER) && V(lm, L_WRIST))
            {
                Vector2 sho2D = new Vector2(smoothX[L_SHOULDER], smoothY[L_SHOULDER]);
                Vector2 elb2D = new Vector2(smoothX[L_ELBOW], smoothY[L_ELBOW]);
                Vector2 wri2D = new Vector2(smoothX[L_WRIST], smoothY[L_WRIST]);
                float mpChain = Vector2.Distance(sho2D, elb2D) + Vector2.Distance(elb2D, wri2D);
                float mpChord = Vector2.Distance(sho2D, wri2D);
                extension = (mpChain > 0.01f) ? Mathf.Clamp01(mpChord / mpChain) : 1f;
            }

            Vector3 smartElbow = ComputeElbowHint(boneShoulder, ikLeftHand, rawElbowAnchored, extension);
            ikLeftElbow = Vector3.Lerp(ikLeftElbow, smartElbow, dt * 1.5f);
        }
        else
        {
            Vector3 restPos = (ikLeftHand + (V(lm, L_SHOULDER) ? MPW(L_SHOULDER) : ikLeftHand)) * 0.5f;
            ikLeftElbow = DriftToRest(ikLeftElbow, restPos, ref leftElbowMissing, dt);
        }

        if (V(lm, R_ELBOW))
        {
            rightElbowMissing = 0f;
            Transform rUpperArmBone = anim.GetBoneTransform(HumanBodyBones.RightUpperArm);
            Vector3 boneShoulder = (rUpperArmBone != null) ? rUpperArmBone.position : ikBodyPos;

            Vector3 rawElbowAnchored = V(lm, R_SHOULDER)
                ? boneShoulder + (MPW(R_ELBOW) - MPW(R_SHOULDER))
                : MPW(R_ELBOW);

            float extension = 1f;
            if (V(lm, R_SHOULDER) && V(lm, R_WRIST))
            {
                Vector2 sho2D = new Vector2(smoothX[R_SHOULDER], smoothY[R_SHOULDER]);
                Vector2 elb2D = new Vector2(smoothX[R_ELBOW], smoothY[R_ELBOW]);
                Vector2 wri2D = new Vector2(smoothX[R_WRIST], smoothY[R_WRIST]);
                float mpChain = Vector2.Distance(sho2D, elb2D) + Vector2.Distance(elb2D, wri2D);
                float mpChord = Vector2.Distance(sho2D, wri2D);
                extension = (mpChain > 0.01f) ? Mathf.Clamp01(mpChord / mpChain) : 1f;
            }

            Vector3 smartElbow = ComputeElbowHint(boneShoulder, ikRightHand, rawElbowAnchored, extension);
            ikRightElbow = Vector3.Lerp(ikRightElbow, smartElbow, dt * 1.5f);
        }
        else
        {
            Vector3 restPos = (ikRightHand + (V(lm, R_SHOULDER) ? MPW(R_SHOULDER) : ikRightHand)) * 0.5f;
            ikRightElbow = DriftToRest(ikRightElbow, restPos, ref rightElbowMissing, dt);
        }

        // ── Feet ──────────────────────────────────────────────────
        if (V(lm, L_ANKLE))
        {
            leftFootMissing = 0f;
            Vector3 ft = MPW(L_ANKLE);
            Vector3 hipPos = avatarRootPos + new Vector3(-avatarShoulderWidth * 0.4f, avatarHipHeight, 0);
            ft = ClampReach(hipPos, ft, avatarLegLength * maxLegReach);
            float groundY = transform.position.y + footOffset;

            if (ft.y < groundY) ft.y = groundY;
            else if (ft.y < groundY + groundSnapDistance)
            {
                float t = (ft.y - groundY) / groundSnapDistance;
                ft.y = Mathf.Lerp(groundY, ft.y, t * t);
            }
            ikLeftFoot = SafeLerp(ikLeftFoot, ft, dt * 1.5f);
        }
        else
        {
            Vector3 restPos = avatarRootPos + new Vector3(-avatarShoulderWidth * 0.4f, 0, 0);
            ikLeftFoot = DriftToRest(ikLeftFoot, restPos, ref leftFootMissing, dt);
        }

        if (V(lm, R_ANKLE))
        {
            rightFootMissing = 0f;
            Vector3 ft = MPW(R_ANKLE);
            Vector3 hipPos = avatarRootPos + new Vector3(avatarShoulderWidth * 0.4f, avatarHipHeight, 0);
            ft = ClampReach(hipPos, ft, avatarLegLength * maxLegReach);
            float groundY = transform.position.y + footOffset;

            if (ft.y < groundY) ft.y = groundY;
            else if (ft.y < groundY + groundSnapDistance)
            {
                float t = (ft.y - groundY) / groundSnapDistance;
                ft.y = Mathf.Lerp(groundY, ft.y, t * t);
            }
            ikRightFoot = SafeLerp(ikRightFoot, ft, dt * 1.5f);
        }
        else
        {
            Vector3 restPos = avatarRootPos + new Vector3(avatarShoulderWidth * 0.4f, 0, 0);
            ikRightFoot = DriftToRest(ikRightFoot, restPos, ref rightFootMissing, dt);
        }

        // ── Knees ─────────────────────────────────────────────────
        if (V(lm, L_KNEE))
        {
            leftKneeMissing = 0f;
            Vector3 rawKnee = MPW(L_KNEE);
            Vector3 hip = avatarRootPos + new Vector3(-avatarShoulderWidth * 0.4f, avatarHipHeight, 0);
            Vector3 smartKnee = ComputeSmartHint(hip, ikLeftFoot, rawKnee, kneeForwardBias);
            ikLeftKnee = Vector3.Lerp(ikLeftKnee, smartKnee, dt * 1.5f);
        }
        else
        {
            Vector3 hipPos = avatarRootPos + new Vector3(-avatarShoulderWidth * 0.4f, avatarHipHeight, 0);
            Vector3 restPos = (hipPos + ikLeftFoot) * 0.5f;
            ikLeftKnee = DriftToRest(ikLeftKnee, restPos, ref leftKneeMissing, dt);
        }

        if (V(lm, R_KNEE))
        {
            rightKneeMissing = 0f;
            Vector3 rawKnee = MPW(R_KNEE);
            Vector3 hip = avatarRootPos + new Vector3(avatarShoulderWidth * 0.4f, avatarHipHeight, 0);
            Vector3 smartKnee = ComputeSmartHint(hip, ikRightFoot, rawKnee, kneeForwardBias);
            ikRightKnee = Vector3.Lerp(ikRightKnee, smartKnee, dt * 1.5f);
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
            Vector3 noseW = MPW(NOSE);
            Vector3 bodyFwd = ikBodyRot * Vector3.forward;
            Vector3 lookDir = bodyFwd;

            if (V(lm, L_EAR) && V(lm, R_EAR))
            {
                Vector3 earMid = (MPW(L_EAR) + MPW(R_EAR)) * 0.5f;
                Vector3 rawLookDir = (noseW - earMid).normalized;
                lookDir = Vector3.Slerp(bodyFwd, rawLookDir, headTurnAmount).normalized;
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

        // Hand IK targets clamped to front half-space, just in case noise leaks past.
        Transform lUA = anim.GetBoneTransform(HumanBodyBones.LeftUpperArm);
        Transform rUA = anim.GetBoneTransform(HumanBodyBones.RightUpperArm);
        if (lUA != null) ikLeftHand = ClampInFrontOfBody(ikLeftHand, lUA.position);
        if (rUA != null) ikRightHand = ClampInFrontOfBody(ikRightHand, rUA.position);

        anim.SetIKPositionWeight(AvatarIKGoal.LeftHand, handWeight);
        anim.SetIKPosition(AvatarIKGoal.LeftHand, ikLeftHand);
        anim.SetIKHintPositionWeight(AvatarIKHint.LeftElbow, 1f);
        anim.SetIKHintPosition(AvatarIKHint.LeftElbow, ikLeftElbow);

        anim.SetIKPositionWeight(AvatarIKGoal.RightHand, handWeight);
        anim.SetIKPosition(AvatarIKGoal.RightHand, ikRightHand);
        anim.SetIKHintPositionWeight(AvatarIKHint.RightElbow, 1f);
        anim.SetIKHintPosition(AvatarIKHint.RightElbow, ikRightElbow);

        anim.SetIKPositionWeight(AvatarIKGoal.LeftFoot, footWeight);
        anim.SetIKPosition(AvatarIKGoal.LeftFoot, ikLeftFoot);
        anim.SetIKHintPositionWeight(AvatarIKHint.LeftKnee, 1f);
        anim.SetIKHintPosition(AvatarIKHint.LeftKnee, ikLeftKnee);

        anim.SetIKPositionWeight(AvatarIKGoal.RightFoot, footWeight);
        anim.SetIKPosition(AvatarIKGoal.RightFoot, ikRightFoot);
        anim.SetIKHintPositionWeight(AvatarIKHint.RightKnee, 1f);
        anim.SetIKHintPosition(AvatarIKHint.RightKnee, ikRightKnee);

        anim.SetLookAtWeight(headWeight, 0.3f, 0.6f, 0.4f, 0.5f);
        anim.SetLookAtPosition(ikLookTarget);
    }

    // ── LateUpdate: dynamic palm alignment + wrist flex ───────────
    void LateUpdate()
    {
        if (!anim || !hasData || !palmAxesCaptured) return;

        Transform lUpperArm = anim.GetBoneTransform(HumanBodyBones.LeftUpperArm);
        Transform lHand = anim.GetBoneTransform(HumanBodyBones.LeftHand);
        Transform rUpperArm = anim.GetBoneTransform(HumanBodyBones.RightUpperArm);
        Transform rHand = anim.GetBoneTransform(HumanBodyBones.RightHand);

        Vector3 bodyRight = ikBodyRot * Vector3.right;
        Vector3 bodyFwd = ikBodyRot * Vector3.forward;
        Quaternion flex = Quaternion.AngleAxis(handFlexAngle, bodyRight);

        // LEFT  arm: palm faces body's RIGHT (+bodyRight) — medial for left arm.
        if (lUpperArm != null && lHand != null)
            AlignPalmToward(lUpperArm, lHand, leftPalmAxisLocal, bodyRight, bodyFwd, handTwistOffset, flex);

        // RIGHT arm: palm faces body's LEFT (-bodyRight) — medial for right arm.
        if (rUpperArm != null && rHand != null)
            AlignPalmToward(rUpperArm, rHand, rightPalmAxisLocal, -bodyRight, bodyFwd, -handTwistOffset, flex);
    }

    /// <summary>
    /// Rotates the hand around the arm axis so its palm faces the desired direction,
    /// then stacks the user's twist offset and the wrist flex on top. Pure
    /// post-IK adjustment — a rotation around the arm axis cannot change the
    /// arm geometry, so this can't bend the elbow.
    /// </summary>
    void AlignPalmToward(Transform upperArm, Transform hand, Vector3 palmAxisLocal,
                         Vector3 desiredPalmDir, Vector3 fallbackDir,
                         float extraTwistDeg, Quaternion flex)
    {
        Vector3 armDir = (hand.position - upperArm.position).normalized;
        if (armDir.sqrMagnitude < 0.001f) return;

        Vector3 currentPalm = hand.rotation * palmAxisLocal;
        Vector3 currentPerp = currentPalm - Vector3.Project(currentPalm, armDir);
        if (currentPerp.sqrMagnitude < 0.001f) return;
        currentPerp.Normalize();

        Vector3 desiredPerp = desiredPalmDir - Vector3.Project(desiredPalmDir, armDir);
        if (desiredPerp.sqrMagnitude < 0.01f)
        {
            desiredPerp = fallbackDir - Vector3.Project(fallbackDir, armDir);
            if (desiredPerp.sqrMagnitude < 0.001f) return;
        }
        desiredPerp.Normalize();

        Quaternion alignRot = Quaternion.FromToRotation(currentPerp, desiredPerp);
        Quaternion userTwist = Quaternion.AngleAxis(extraTwistDeg, armDir);
        Quaternion target = flex * userTwist * alignRot * hand.rotation;

        hand.rotation = Quaternion.Slerp(hand.rotation, target, handTwistWeight);
    }

    bool V(LandmarkData[] lm, int i) => i < lm.Length && lm[i].visibility >= minVisibility;

    bool VWrist(LandmarkData[] lm, int wristIdx, int shoulderIdx, int hipIdx)
    {
        if (wristIdx >= lm.Length) return false;
        if (lm[wristIdx].visibility < minWristVisibility) return false;
        if (lm[wristIdx].visibility >= 0.85f) return true;

        if (shoulderIdx >= lm.Length || hipIdx >= lm.Length) return true;
        if (lm[shoulderIdx].visibility < minVisibility || lm[hipIdx].visibility < minVisibility) return true;

        float wristY = smoothY[wristIdx];
        float shoulderY = smoothY[shoulderIdx];
        float hipY = smoothY[hipIdx];
        bool inSuspectBand = wristY > shoulderY - 0.02f && wristY < hipY + 0.04f;
        return !inSuspectBand;
    }

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

        Gizmos.color = new Color(1, 0.5f, 0, 0.7f);
        Gizmos.DrawWireSphere(ikLeftHand, 0.05f);
        Gizmos.DrawWireSphere(ikRightHand, 0.05f);
        Gizmos.color = new Color(0, 0.5f, 1, 0.7f);
        Gizmos.DrawWireSphere(ikLeftFoot, 0.05f);
        Gizmos.DrawWireSphere(ikRightFoot, 0.05f);

        Gizmos.color = Color.gray;
        Gizmos.DrawLine(avatarRootPos + Vector3.left * 2, avatarRootPos + Vector3.right * 2);
    }

    void GL(int a, int b)
    {
        if (debugWorldPts != null && a < debugWorldPts.Length && b < debugWorldPts.Length)
            Gizmos.DrawLine(debugWorldPts[a], debugWorldPts[b]);
    }
}

[Serializable] public class LandmarkFrame { public string type; public long timestamp; public LandmarkData[] landmarks; public LandmarkData[] worldLandmarks; }
[Serializable] public class LandmarkData { public float x, y, z, visibility; }

[Serializable]
public class OneEuroFilter
{
    public float minCutoff = 1.0f;
    public float beta = 0.007f;
    public float dCutoff = 1.0f;

    private float xPrev;
    private float dxPrev;
    private bool initialized;

    public OneEuroFilter(float minCutoff = 1.0f, float beta = 0.007f, float dCutoff = 1.0f)
    {
        this.minCutoff = minCutoff;
        this.beta = beta;
        this.dCutoff = dCutoff;
    }

    public float Filter(float x, float dt)
    {
        if (!initialized) { xPrev = x; dxPrev = 0f; initialized = true; return x; }
        if (dt <= 0f) return xPrev;

        float dx = (x - xPrev) / dt;
        float aD = SmoothingFactor(dt, dCutoff);
        float dxHat = aD * dx + (1f - aD) * dxPrev;

        float cutoff = minCutoff + beta * Mathf.Abs(dxHat);
        float a = SmoothingFactor(dt, cutoff);
        float xHat = a * x + (1f - a) * xPrev;

        xPrev = xHat;
        dxPrev = dxHat;
        return xHat;
    }

    public void Reset() { initialized = false; }

    private static float SmoothingFactor(float dt, float cutoff)
    {
        float r = 2f * Mathf.PI * cutoff * dt;
        return r / (r + 1f);
    }
}