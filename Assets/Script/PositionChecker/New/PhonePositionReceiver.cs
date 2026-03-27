using M2MqttUnity;
using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;
using uPLibrary.Networking.M2Mqtt.Messages;

public class PhonePositionReceiver : M2MqttUnityClient
{
    [Header("Phone Position Settings")]
    public string positionTopic = "phone/position";  
    public GameObject targetCharacter;              
    public float moveSpeed = 1f;                    
    public float deadzone = 0.1f;                    
    public bool resetPosition = false;
    public float damping = 0.95f;

    private Vector3 velocity = Vector3.zero;
    private Vector3 characterPosition = Vector3.zero;
    private Vector3 pendingAccel = Vector3.zero;
    private bool newDataReceived = false;

    protected override void SubscribeTopics()
    {
        client.Subscribe(
            new string[] { positionTopic },
            new byte[] { MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE }
        );
        Debug.Log("Position subscribed to: " + positionTopic);
    }

    protected override void UnsubscribeTopics()
    {
        client.Unsubscribe(new string[] { positionTopic });
    }

    protected override void DecodeMessage(string topic, byte[] message)
    {
        string payload = Encoding.UTF8.GetString(message);

        try
        {
            // Only process accelerometer messages
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

    protected override void Update()
    {
        base.Update();

        if (resetPosition)
        {
            velocity = Vector3.zero;
            characterPosition = Vector3.zero;
            if (targetCharacter != null)
                targetCharacter.transform.position = Vector3.zero;
            resetPosition = false;
            Debug.Log("Position reset");
        }

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
