using System.Text;
using UnityEngine;
using M2MqttUnity;
using uPLibrary.Networking.M2Mqtt.Messages;

public class PhoneController : M2MqttUnityClient
{
    [Header("Topics")]
    public string sensorTopic = "phone/sensors";

    [Header("Target")]
    public GameObject targetCharacter;

    [Header("Rotation Settings")]
    public float rotationSmoothSpeed = 5f;

    [Header("Position Settings")]
    public float moveSpeed = 1f;
    public float deadzone = 0.1f;
    public float damping = 0.95f;
    public bool resetPosition = false;

    // Rotation
    private Quaternion targetRotation;
    private bool newRotationReceived = false;

    // Position
    private Vector3 pendingAccel = Vector3.zero;
    private Vector3 velocity = Vector3.zero;
    private Vector3 characterPosition = Vector3.zero;

    // -------------------------------------------------------
    // MQTT
    // -------------------------------------------------------

    // one single topic for everything

   

    protected override void SubscribeTopics()
    {
        client.Subscribe(
            new string[] { sensorTopic },
            new byte[] { MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE }
        );
        Debug.Log("Subscribed to: " + sensorTopic);
    }

    protected override void UnsubscribeTopics()
    {
        client.Unsubscribe(new string[] { sensorTopic });
    }

    protected override void DecodeMessage(string topic, byte[] message)
    {
        string payload = Encoding.UTF8.GetString(message);
        Debug.Log("TOPIC: " + topic + " | PAYLOAD: " + payload); // add this

        if (payload.Contains("rotation_vector"))
        {
            DecodeRotation(payload);
        }
        else if (payload.Contains("accelerometer"))
        {
            DecodePosition(payload);
        }
    }

    // -------------------------------------------------------
    // Rotation decoder
    // -------------------------------------------------------

    private void DecodeRotation(string payload)
    {
        try
        {
            if (!payload.Contains("rotation_vector")) return;

            int start = payload.IndexOf("[") + 1;
            int end = payload.IndexOf("]");
            string[] parts = payload.Substring(start, end - start).Split(',');

            if (parts.Length < 4) return;

            float x = float.Parse(parts[0], System.Globalization.CultureInfo.InvariantCulture);
            float y = float.Parse(parts[1], System.Globalization.CultureInfo.InvariantCulture);
            float z = float.Parse(parts[2], System.Globalization.CultureInfo.InvariantCulture);
            float w = float.Parse(parts[3], System.Globalization.CultureInfo.InvariantCulture);

            float yaw = new Quaternion(x, y, z, w).eulerAngles.y;
            Debug.Log("YAW: " + yaw);  // add this line
            targetRotation = Quaternion.Euler(0f, yaw, 0f);
            newRotationReceived = true;
        }
        catch (System.Exception e)
        {
            Debug.LogWarning("Rotation parse error: " + e.Message);
        }
    }

    // -------------------------------------------------------
    // Position decoder
    // -------------------------------------------------------

    private void DecodePosition(string payload)
    {
        try
        {
            if (!payload.Contains("accelerometer")) return;

            int start = payload.IndexOf("[") + 1;
            int end = payload.IndexOf("]");
            string[] parts = payload.Substring(start, end - start).Split(',');

            if (parts.Length < 3) return;

            float ax = float.Parse(parts[0], System.Globalization.CultureInfo.InvariantCulture);
            float ay = float.Parse(parts[1], System.Globalization.CultureInfo.InvariantCulture);
            float az = float.Parse(parts[2], System.Globalization.CultureInfo.InvariantCulture);

            az -= 9.8f;

            if (Mathf.Abs(ax) < deadzone) ax = 0f;
            if (Mathf.Abs(ay) < deadzone) ay = 0f;
            if (Mathf.Abs(az) < deadzone) az = 0f;

            pendingAccel = new Vector3(ax, 0f, ay);

            // If phone is still — kill velocity immediately
            if (ax == 0f && ay == 0f)
            {
                velocity = Vector3.zero;
            }

        }
        catch (System.Exception e)
        {
            Debug.LogWarning("Position parse error: " + e.Message);
        }
    }

    // -------------------------------------------------------
    // Update — apply both rotation and position
    // -------------------------------------------------------

    protected override void Update()
    {
        base.Update();

        if (targetCharacter == null) return;

        // Reset
        if (resetPosition)
        {
            velocity = Vector3.zero;
            characterPosition = Vector3.zero;
            targetCharacter.transform.position = Vector3.zero;
            resetPosition = false;
            Debug.Log("Position reset");
        }

        // Apply rotation
        if (newRotationReceived)
        {
            Debug.Log("Applying rotation: " + targetRotation.eulerAngles);  // add this
            targetCharacter.transform.rotation = Quaternion.Slerp(
                targetCharacter.transform.rotation,
                targetRotation,
                Time.deltaTime * rotationSmoothSpeed
            );
        }

        // Apply position
        velocity += pendingAccel * moveSpeed * Time.deltaTime;

        // Hard stop when no input, soft damp when moving
        if (pendingAccel == Vector3.zero)
            velocity = Vector3.Lerp(velocity, Vector3.zero, Time.deltaTime * 10f);
        else
            velocity *= damping;

        characterPosition += velocity * Time.deltaTime;
        targetCharacter.transform.position = characterPosition;
    }
}