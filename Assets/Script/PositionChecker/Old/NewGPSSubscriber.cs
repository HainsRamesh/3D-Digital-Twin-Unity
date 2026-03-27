using UnityEngine;
using M2MqttUnity;
using uPLibrary.Networking.M2Mqtt.Messages;


public class NewGPSSubscriber : M2MqttUnityClient
{
    [Header("Worker")]
    public Transform worker;
    public Animator workerAnimator;         // drag your Animator here

    [Header("Tunnel Origin GPS  →  Unity (0,0,0)")]
    public double originLat = 10.016663;
    public double originLon = 76.365781;

    [Header("Movement")]
    public float movementThreshold = 0.5f;  // ignore jitter below this (meters)
    public float smoothSpeed = 5f;    // lerp speed toward target

    [Header("EMA Filter  (0=frozen, 1=raw)")]
    [Range(0.05f, 1f)]
    public float emaAlpha = 0.25f;          // tweak this in Inspector at runtime

    // --- internals ---
    private double _smoothLat;
    private double _smoothLon;
    private bool _filterInitialised = false;

    private Vector3 _targetPosition;
    private Vector3 _prevPosition;
    private bool _newData = false;

    //public float maxAccuracy = 15f;   // reject readings worse than this (metres)
    //public float minSpeedToMove = 0.3f;  // phone must report this m/s to accept movement
    //public float minDeltaVsAccuracy = 0.5f;

    // ---------------------------------------------------------------
    protected override void Start()
    {
        brokerAddress = "d342b011d9c945b4a7c5a2efed018881.s1.eu.hivemq.cloud";
        brokerPort = 8883;
        mqttUserName = "hivemq.webclient.1774432445087";
        mqttPassword = ".u#WtRML?A01w$Hne87d";
        isEncrypted = true;
        base.Start();
    }

    protected override void SubscribeTopics()
    {
        Debug.Log("Subscribing to tunnel/workers/gps");
        client.Subscribe(
            new string[] { "tunnel/workers/gps" },
            new byte[] { MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE }
        );
    }

    // ---------------------------------------------------------------
    protected override void DecodeMessage(string topic, byte[] message)
    {

        //string msg = System.Text.Encoding.UTF8.GetString(message);
        //GPSData gps = JsonUtility.FromJson<GPSData>(msg);
        //if (gps == null) { Debug.LogWarning("Failed to parse GPS JSON"); return; }

        //// --- Gate 1: reject truly bad fixes only ---
        //if (gps.accuracy > maxAccuracy)
        //{
        //    Debug.Log($"Skipped: poor accuracy {gps.accuracy:F1}m");
        //    return;
        //}

        //// --- Always feed EMA regardless of speed ---
        //if (!_filterInitialised)
        //{
        //    _smoothLat = gps.latitude;
        //    _smoothLon = gps.longitude;
        //    _filterInitialised = true;
        //    return;
        //}

        //_smoothLat = _smoothLat + emaAlpha * (gps.latitude - _smoothLat);
        //_smoothLon = _smoothLon + emaAlpha * (gps.longitude - _smoothLon);

        //// --- Convert smoothed GPS → Unity coords ---
        //double deltaLat = _smoothLat - originLat;
        //double deltaLon = _smoothLon - originLon;
        //double cosLat = System.Math.Cos(originLat * System.Math.PI / 180.0);
        //float posX = (float)(deltaLon * 111320.0 * cosLat);
        //float posZ = (float)(deltaLat * 111320.0);
        //Vector3 newPos = new Vector3(posX, 0f, posZ);

        //// --- Gate 2: delta must clearly exceed the GPS noise floor ---
        //float delta = Vector3.Distance(worker.position, newPos);
        //float noiseFloor = gps.accuracy * minDeltaVsAccuracy; // e.g. 10m * 0.8 = 8m threshold

        //if (delta < noiseFloor)
        //{
        //    Debug.Log($"Skipped: delta {delta:F1}m < noise floor {noiseFloor:F1}m");
        //    return;
        //}

        //// --- Gate 3: speed as a soft confirmation only ---
        //if (gps.speed < minSpeedToMove)
        //{
        //    Debug.Log($"Skipped: speed {gps.speed:F2} m/s too low");
        //    return;
        //}

        //_targetPosition = newPos;
        //_newData = true;
        //Debug.Log($"Accepted: delta {delta:F1}m, speed {gps.speed:F2}m/s, acc {gps.accuracy:F1}m");


        string msg = System.Text.Encoding.UTF8.GetString(message);
        GPSData gps = JsonUtility.FromJson<GPSData>(msg);

        if (gps == null) { Debug.LogWarning("Failed to parse GPS JSON"); return; }

        // --- 1. EMA filter on raw GPS before anything else ---
        if (!_filterInitialised)
        {
            _smoothLat = gps.latitude;
            _smoothLon = gps.longitude;
            _filterInitialised = true;
        }
        else
        {
            _smoothLat = _smoothLat + emaAlpha * (gps.latitude - _smoothLat);
            _smoothLon = _smoothLon + emaAlpha * (gps.longitude - _smoothLon);
        }

        // --- 2. Convert smoothed GPS → Unity coords  (keep double until cast) ---
        double deltaLat = _smoothLat - originLat;
        double deltaLon = _smoothLon - originLon;

        double cosLat = System.Math.Cos(originLat * System.Math.PI / 180.0); // use origin, not live reading
        float posX = (float)(deltaLon * 111320.0 * cosLat);
        float posZ = (float)(deltaLat * 111320.0);

        Vector3 newPos = new Vector3(posX, 0f, posZ);

        // --- 3. Threshold gate — your good idea, kept ---
        if (Vector3.Distance(worker.position, newPos) > movementThreshold)
        {
            _targetPosition = newPos;
            _newData = true;
        }
    }

    // ---------------------------------------------------------------
    protected override void Update()
    {
        base.Update();

        if (!_newData) return;

        _prevPosition = worker.position;
        worker.position = Vector3.Lerp(
            worker.position,
            _targetPosition,
            Time.deltaTime * smoothSpeed
        );
        if (_newData)
        {
            Debug.Log(worker.position);
        }

        // --- 4. Drive animator from actual frame displacement ---
        if (workerAnimator != null)
        {
            float frameDist = Vector3.Distance(_prevPosition, worker.position);
            float speedMPS = frameDist / Time.deltaTime;          // meters per second
            float blendSpeed = Mathf.Clamp(speedMPS, 0f, 6f);      // cap at running speed
            workerAnimator.SetFloat("Speed", blendSpeed, 0.1f, Time.deltaTime); // damped set
        }

        // --- 5. Face the direction of travel ---
        Vector3 moveDir = worker.position - _prevPosition;
        if (moveDir.sqrMagnitude > 0.0001f)
            worker.rotation = Quaternion.Slerp(
                worker.rotation,
                Quaternion.LookRotation(moveDir),
                Time.deltaTime * smoothSpeed * 2f
            );

        if (Vector3.Distance(worker.position, _targetPosition) < 0.05f)
        {
            _newData = false;
            if (workerAnimator != null)
                workerAnimator.SetFloat("Speed", 0f, 0.15f, Time.deltaTime); // glide to idle
        }
    }
}
