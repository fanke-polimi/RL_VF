using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class grasp_targ : MonoBehaviour
{
    public Transform eet, jaw;
    Vector3 init_pos;
    bool first;
    // Start is called before the first frame update
    void Start()
    {
        init_pos=transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        if( !first && jaw.localRotation == Quaternion.Euler(0,-20,0)){
          this.transform.position = eet.transform.position;
        }
        if (Vector3.Distance(init_pos,transform.position)>0.001f){
            first = true;
        }
        
    }
}
