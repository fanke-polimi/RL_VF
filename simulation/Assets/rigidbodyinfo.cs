using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class rigidbodyinfo : MonoBehaviour
{
    public GameObject rb;
    Rigidbody rb1;
    public Vector3 vol;
    public Vector3 pos;
    // public Vector3 rot;
    // Start is called before the first frame update
    void Start()
    {
         rb1 = this.GetComponentInChildren<Rigidbody>();
    }

    // Update is called once per frame
     void FixedUpdate()
    {
        
            
            // rb.velocity = new Vector3(0, 10, 0);
            vol = rb1.velocity;
            pos = rb.transform.position;
            // rot = rb.rotation;
        

        
            
        
    }
}
