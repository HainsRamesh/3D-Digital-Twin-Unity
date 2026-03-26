using UnityEngine;
using M2MqttUnity;
using uPLibrary.Networking.M2Mqtt.Messages;

public class GPSSubscriber : M2MqttUnityClient
{
    public Transform worker;

    // Tunnel origin GPS (Unity 0,0,0)
    public double originLat = 10.016663;
    public double originLon = 76.365781;

    float posX;
    float posZ;
    bool newData = false;

    public float movementThreshold = 0.5f; // meters
    public float smoothSpeed = 5f;

    Vector3 targetPosition;

    //protected override void Awake()
    //{
    //    base.Awake();
    //    Debug.Log("awake has been called");
    //}
    protected override void Start()
    {
        brokerAddress = "d342b011d9c945b4a7c5a2efed018881.s1.eu.hivemq.cloud";
        brokerPort = 8883;
        mqttUserName = "hivemq.webclient.1774432445087";
        mqttPassword = ".u#WtRML?A01w$Hne87d";
        isEncrypted = true;

        base.Start();
        //Connect();
    }

    protected override void SubscribeTopics()
    {
        Debug.Log("Subscribing to topic tunnel/workers/gps");

        client.Subscribe(
            new string[] { "tunnel/workers/gps" },
            new byte[] { MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE });
    }

    protected override void DecodeMessage(string topic, byte[] message)
    {

        Debug.Log("Raw MQTT: " + topic);
        string msg = System.Text.Encoding.UTF8.GetString(message);

        Debug.Log("Raw MQTT: " + msg);

        GPSData gps = JsonUtility.FromJson<GPSData>(msg);

        double lat = gps.latitude;
        double lon = gps.longitude;

        Debug.Log("GPS: " + lat + "," + lon);

        double deltaLat = lat - originLat;
        double deltaLon = lon - originLon;

        posX = (float)(deltaLon * 111320 * Mathf.Cos((float)lat * Mathf.Deg2Rad));
        posZ = (float)(deltaLat * 111320);

        Vector3 newPos = new Vector3(posX, 0, posZ);

        if (Vector3.Distance(worker.position, newPos) > movementThreshold)
        {
            targetPosition = newPos;
            newData = true;
        }
    }

    protected override void Update()
    {
        base.Update();

        if (newData)
        {
            worker.position = Vector3.Lerp(
                worker.position,
                targetPosition,
                Time.deltaTime * smoothSpeed
            );

            // Stop updating when close enough
            if (Vector3.Distance(worker.position, targetPosition) < 0.05f)
            {
                newData = false;
            }
        }
    }
}