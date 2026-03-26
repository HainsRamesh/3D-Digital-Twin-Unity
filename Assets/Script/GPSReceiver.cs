using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class GPSReceiver : MonoBehaviour
{
    UdpClient client;
    public int port = 5052;

    public Transform worker;

    double originLat = 9.931233;
    double originLon = 76.267303;

    void Start()
    {
        client = new UdpClient(port);
        client.BeginReceive(ReceiveData, null);
    }

    void ReceiveData(IAsyncResult result)
    {
        IPEndPoint ip = new IPEndPoint(IPAddress.Any, port);
        byte[] data = client.EndReceive(result, ref ip);

        string text = Encoding.UTF8.GetString(data);

        string[] gps = text.Split(',');

        double lat = double.Parse(gps[0]);
        double lon = double.Parse(gps[1]);

        double deltaLat = lat - originLat;
        double deltaLon = lon - originLon;

        double x = deltaLon * 111320 * Mathf.Cos((float)lat * Mathf.Deg2Rad);
        double z = deltaLat * 111320;

        Vector3 pos = new Vector3((float)x, 0, (float)z);

        worker.position = pos;

        client.BeginReceive(ReceiveData, null);
    }
}