using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class getcost_2d : MonoBehaviour
{
    public GameObject c0;
    public GameObject c1;
    public GameObject c2;
    public GameObject c3;
    // public GameObject c4;
    // public GameObject c5;
    // public GameObject c4;

    public float CostList1;
    public float CostList0;
    public float CostList2;
    public float CostList3;
    // public float CostList4;
    // public float CostList5;

    
    // Start is called before the first frame update
    void Start()
    {
        // CostList = new List<float>{0,0,0,0};

    }

    // Update is called once per frame
    void Update()
    {
        CostList0 = c0.GetComponent<twodagent>().cost;
        CostList1 = c1.GetComponent<twodagent>().cost;
        CostList2 = c2.GetComponent<twodagent>().cost;
        CostList3 = c3.GetComponent<twodagent>().cost;
        // CostList3 = c4.GetComponent<psmagent>().cost;
        // CostList3 = c5.GetComponent<psmagent>().cost;
    }
}
