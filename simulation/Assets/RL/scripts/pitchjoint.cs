using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class pitchjoint : MonoBehaviour
{
    private ArticulationBody articulation;
    public float primaryAxisRotation;
    public float damping;
    public float stiff;
    public float maxforce;
    // Start is called before the first frame update
    void Start()
    {
        articulation = GetComponent<ArticulationBody>();
    }

    // Update is called once per frame
    void Update()
    {
         RotateTo(primaryAxisRotation);
         

        //  damping = drive.damping;
        //  stiffness = drive.stiffness;
    }

     void RotateTo(float primaryAxisRotation)
    {
        var drive = articulation.xDrive;
        // drive.target = primaryAxisRotation;
         drive.damping=damping;
         drive.stiffness=stiff;
         drive.forceLimit=maxforce;
        // Debug.Log("zai 中"+ primaryAxisRotation);
        
        drive.target = primaryAxisRotation;
        articulation.xDrive = drive;
    }

    public void Reset(){
        var drive = articulation.xDrive;
        drive.target = 0f;
        drive.damping=0f;
        drive.stiffness=1000000f;
        articulation.xDrive = drive;
        primaryAxisRotation =  0f;
        // damping = drive.damping;
        // stiffness = drive.stiffness;



    }

    // public void run(float primaryAxis){
    //     var drive = articulation.xDrive;
    //     drive.target = primaryAxisRotation;
    //     drive.damping=2000f;
    //     drive.stiffness=10000f;
    //     articulation.xDrive = drive;

    //     primaryAxisRotation =  primaryAxis;
    //     damping = drive.damping;
    //     stiffness = drive.stiffness;

    // }
}
