using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };

public class JointController : MonoBehaviour
{
    // public RotationDirection rotationState = RotationDirection.None;
    // public float speed = 300.0f;

    private ArticulationBody articulation;
    public float primaryAxisRotation;
    public float damping;
    public float stiff;
    public float maxforce;



    // LIFE CYCLE

    void Start()
    {
        articulation = GetComponent<ArticulationBody>();
    }

    void FixedUpdate() 
    {
        // if (rotationState != RotationDirection.None) {
        //     float rotationChange = (float)rotationState * speed * Time.fixedDeltaTime;
        //     float rotationGoal = CurrentPrimaryAxisRotation() + rotationChange;
            RotateTo(primaryAxisRotation);
        //     var drive = articulation.xDrive;
        // // drive.target = primaryAxisRotation;
        //     drive.damping=2000f;
        //     drive.stiffness=10000f;
        //     articulation.xDrive = drive;
        // }


    }


    // MOVEMENT HELPERS

    // float CurrentPrimaryAxisRotation()
    // {
    //     float currentRotationRads = articulation.jointPosition[0];
    //     float currentRotation = Mathf.Rad2Deg * currentRotationRads;
    //     return currentRotation;
    // }

    void RotateTo(float primaryAxisRotation)
    {
        // Debug.Log("zai 中111"+ primaryAxisRotation);
        var drive = articulation.xDrive;
        drive.target = primaryAxisRotation;
        drive.damping=damping;
        drive.stiffness=stiff;
        drive.forceLimit=maxforce;
        
        articulation.xDrive = drive;
    }
    public void Reset(){
        var drive = articulation.xDrive;
        drive.target = 0f;
        drive.damping=0f;
        drive.stiffness=1000000f;
        articulation.xDrive = drive;
        print("reset");

    }

  



}
