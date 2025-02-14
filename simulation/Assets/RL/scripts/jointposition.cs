using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class jointposition : MonoBehaviour
{
    public ArticulationBody outer_yaw;
    public float position;
    public string JointName;
    // Start is called before the first frame update
    void Start()
    {
         outer_yaw = GetComponent<ArticulationBody>();
        //   Debug.Log( "jointposition"+outer_yaw.jointPosition[0]);
        
        
    }

    // Update is called once per frame
    void Update()
    {
        position = outer_yaw.jointPosition[0];
    }
    public void Read(out string name, out float position, out float velocity, out float effort)
    {
        name = JointName;
        position = outer_yaw.jointPosition[0];;
        velocity = outer_yaw.jointVelocity[0];;
        effort = outer_yaw.jointForce[0];
    }
}
