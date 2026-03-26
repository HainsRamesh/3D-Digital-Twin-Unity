using UnityEngine;

public class GPSTest : MonoBehaviour
{
    public double latitude = 9.931233;
    public double longitude = 76.267303;

    public double originLatitude = 9.931233;
    public double originLongitude = 76.267303;

    void Start()
    {
        double latDiff = latitude - originLatitude;
        double lonDiff = longitude - originLongitude;

        float x = (float)(lonDiff * 111320);
        float z = (float)(latDiff * 110540);

        Vector3 pos = new Vector3(x, 0, z);

        transform.position = pos;

        Debug.Log("Unity Position: " + pos);
    }
}