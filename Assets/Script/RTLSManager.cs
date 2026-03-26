using System.Collections;
using UnityEngine;
using UnityEngine.Networking;
using System.Collections.Generic;

public class RTLSManager : MonoBehaviour
{

    public string apiURL = "http://localhost:3000/workers";

    public GameObject workerPrefab;

    Dictionary<string, GameObject> workerObjects = new Dictionary<string, GameObject>();


    void Start()
    {
        StartCoroutine(UpdateWorkers());
    }


    IEnumerator UpdateWorkers()
    {

        while (true)
        {

            UnityWebRequest request = UnityWebRequest.Get(apiURL);

            yield return request.SendWebRequest();

            if (request.result == UnityWebRequest.Result.Success)
            {

                string json = request.downloadHandler.text;

                WorkerList workers = JsonUtility.FromJson<WorkerList>(json);

                foreach (WorkerData worker in workers.workers)
                {

                    if (!workerObjects.ContainsKey(worker.tag_id))
                    {
                        GameObject obj = Instantiate(workerPrefab);
                        workerObjects.Add(worker.tag_id, obj);
                    }

                    GameObject workerObj = workerObjects[worker.tag_id];

                    workerObj.transform.position = new Vector3(
                        worker.pos_x,
                        worker.pos_y,
                        worker.pos_z
                    );

                }

            }

            yield return new WaitForSeconds(1f);
        }
    }
}