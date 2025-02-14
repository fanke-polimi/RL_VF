using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ConsDetect : MonoBehaviour
{
    // [HideInInspector]
    public continuousAgent agent;

    void OnTriggerEnter(Collider col)
    {
        // Touched goal.
        if (col.gameObject.CompareTag("cons"))
        {
            agent.ScoredACons();
            Debug.Log("碰撞了幺幺");
        }
    }

    void OnTriggerStay(Collider col)
    {
        if (col.gameObject.CompareTag("cons"))
        {
            agent.ScoredACons();
            Debug.Log("在碰撞中");
        }
    }

    void OnTriggerExit(Collider col)
    {
        if (col.gameObject.CompareTag("cons"))
        {
            agent.ExitCons();
            Debug.Log("碰撞结束");
        }
    }
}
