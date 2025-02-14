using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotPosBase : MonoBehaviour
{
    // Start is called before the first frame update
    public static Matrix4x4 baseMat;
    public static Matrix4x4 base_to_world;

    public static Matrix4x4 ground_to_world;
    public static Matrix4x4 baseMatInv;

    public Transform ee;

    

    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // Vector3 position1 = transform.position;
        // Quaternion rotation1 = transform.rotation;
        // Vector3 postion2 = ee.transform.InverseTransformPoint(position1);
        
        ground_to_world = Matrix4x4.TRS(ee.transform.position, ee.transform.rotation, new Vector3(1,1,1));
        base_to_world = Matrix4x4.TRS(transform.position, transform.rotation, new Vector3(1, 1, 1));
        baseMat = ground_to_world.inverse *  base_to_world;
        // Debug.Log("base"+baseMat);


        baseMatInv = baseMat.inverse;

        //Debug.Log("Base: \n"+ baseMat);
        

    }
}
