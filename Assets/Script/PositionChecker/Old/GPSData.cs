//[System.Serializable]
//public class GPSData
//{
//    public string type;
//    public double latitude;
//    public double longitude;
//    public double altitude;
//    public double accuracy;
//    public double speed;
//    public long time;
//    public double bearing;
//}


// GPSData.cs
using UnityEngine;

[System.Serializable]
public class GPSData
{
    public double latitude;
    public double longitude;
    public double altitude;
    public float accuracy;   // metres — IMPORTANT
    public float speed;      // m/s from phone — IMPORTANT
    public long time;
    public float bearing;
    public string type;
}