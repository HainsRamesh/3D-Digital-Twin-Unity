using System.Text;
using UnityEngine;
using M2MqttUnity;
using uPLibrary.Networking.M2Mqtt.Messages;

public class PhoneRotationReceiver : M2MqttUnityClient
{
    [Header("Phone Rotation Settings")]
    public string rotationTopic = "phone/rotation";  // must match SensorSpot topic
    public GameObject targetObject;                  // drag your 3D object here in Inspector
    public float smoothSpeed = 5f;                   // smoothing (lower = smoother)

    private Quaternion targetRotation;
    private bool newDataReceived = false;

    // Called when connected — subscribe to the topic
    protected override void SubscribeTopics()
    {
        client.Subscribe(
            new string[] { rotationTopic },
            new byte[] { MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE }
        );
        Debug.Log("Subscribed to: " + rotationTopic);
    }

    // Called when disconnecting — unsubscribe
    protected override void DecodeMessage(string topic, byte[] message)
    {
        string payload = Encoding.UTF8.GetString(message);

        try
        {
            // Skip non-rotation messages silently
            if (!payload.Contains("rotation_vector")) return;

            // Manually extract the values array
            int start = payload.IndexOf("[") + 1;
            int end = payload.IndexOf("]");
            string valuesStr = payload.Substring(start, end - start);
            string[] parts = valuesStr.Split(',');

            if (parts.Length < 4) return;

            float x = float.Parse(parts[0], System.Globalization.CultureInfo.InvariantCulture);
            float y = float.Parse(parts[1], System.Globalization.CultureInfo.InvariantCulture);
            float z = float.Parse(parts[2], System.Globalization.CultureInfo.InvariantCulture);
            float w = float.Parse(parts[3], System.Globalization.CultureInfo.InvariantCulture);

            Quaternion rawRotation = new Quaternion(x, y, z, w);
            float yaw = rawRotation.eulerAngles.y;  // already 0-360 when phone is flat
            targetRotation = Quaternion.Euler(0f, yaw, 0f);
            newDataReceived = true;
        }
        catch (System.Exception e)
        {
            Debug.LogWarning("Parse error: " + e.Message);
        }
    }

    // Smoothly apply rotation on the main thread
    protected override void Update()
    {
        base.Update(); // always call this — processes the MQTT message queue

        if (newDataReceived && targetObject != null)
        {
            targetObject.transform.rotation = Quaternion.Slerp(
                targetObject.transform.rotation,
                targetRotation,
                Time.deltaTime * smoothSpeed
            );
        }
    }
}

// Must match the JSON keys SensorSpot sends
[System.Serializable]
public class SensorData
{
    public string type;
    public float[] values;
    public long timestamp;
}