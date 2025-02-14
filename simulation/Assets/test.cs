using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class test : MonoBehaviour
{
    public GameObject eet;
    public Vector3 ee_con;
    public float dis;
    public float divx;
    public float divy;
    public float divz;
    public Vector3 eet_pos;
    public Vector3 con_pos;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        ee_con = transform.InverseTransformPoint(eet.transform.position);
        dis = Vector3.Distance(eet.transform.position, transform.position);
        divy = eet.transform.position.y - transform.position.y;
        divx = eet.transform.position.x - transform.position.x;
        divz = eet.transform.position.z - transform.position.z;
        eet_pos = eet.transform.position;
        con_pos = transform.position;

        
    }
}
