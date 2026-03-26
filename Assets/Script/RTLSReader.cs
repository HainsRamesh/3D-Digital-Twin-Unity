using System.Collections;
using UnityEngine;
using UnityEngine.Networking;

public class RTLSReader : MonoBehaviour
{

    string url = "http://localhost:3000/workers";

    void Start()
    {
        StartCoroutine(GetWorkers());
    }

    IEnumerator GetWorkers()
    {

        UnityWebRequest request = UnityWebRequest.Get(url);

        yield return request.SendWebRequest();

        if (request.result == UnityWebRequest.Result.Success)
        {
            Debug.Log(request.downloadHandler.text);
        }
        else
        {
            Debug.Log(request.error);
        }

    }
}