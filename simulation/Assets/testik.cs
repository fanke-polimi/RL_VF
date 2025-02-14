using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DVRK;
using System.ComponentModel;
using System;
//using System.ComponentModel.DataAnnotations.Schema;
using System.Diagnostics.Contracts;
using System.Security.Cryptography.X509Certificates;
//using System;
using System.Collections.Specialized;
//using System.Numerics;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;
using UnityEngine.Serialization;
using System.Reflection;
//using System.Diagnostics;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;


public class testik : Agent
{
    public List<URDFJoint> independentJoints = new List<URDFJoint>();
    // public psm_ik_semi ground;

    public GameObject ee;
    public Rigidbody eerb;
    public GameObject eet;
    public GameObject eet1;
    public GameObject target;
    public GameObject Cons1;
    private Transform eePositionRotation;
    private Vector3 startingPos;
    private Quaternion startingRot;

    // public ArticulationBody outer_roll;
    // public ArticulationBody wrist_pitch;
    // public ArticulationBody wrist_yaw;

    int pos_x = 0;
    int pos_y = 0;
    int pos_z = 0;
    int pos_4 = 0;
    int pos_5 = 0;
    int pos_6 = 0;

    public float deltamove;

    public float control;
    public float distance;
    public float disv;
    public float disv1;

    public double divx;
    public double dis;

   public  float fx;
    public float fy;
    public float fz;

    public float cost;
    public Vector3 force;
    // Start is called before the first frame update
    void Start()
    {
        startingPos = ee.transform.localPosition;
        eerb = ee.GetComponent<Rigidbody>();
        // outer_roll = GetComponent<ArticulationBody>();
        // wrist_pitch = GetComponent<ArticulationBody>();
        // wrist_yaw = GetComponent<ArticulationBody>();
        // ground = GetComponent<psm_ik_semi>();
       independentJoints[0].SetJointValue(45);
        independentJoints[1].SetJointValue(0);
        independentJoints[2].SetJointValue(0);
        // independentJoints[1].SetJointValue(45);
        // independentJoints[2].SetJointValue(45);
        control = independentJoints[0].currentJointValue;
        
        
    }

    public override void OnEpisodeBegin()
    {
        independentJoints[0].SetJointValue(45);
        independentJoints[1].SetJointValue(0);
        independentJoints[2].SetJointValue(0);
        // ground.joint4_roll = 45f;
        // JointController joint4 = outer_roll.GetComponentInChildren<JointController>();
        // joint4.primaryAxisRotation = 0f;
        // // joint4.reset();
        // JointController joint5 = wrist_pitch.GetComponentInChildren<JointController>();
        // joint5.primaryAxisRotation = 0f;
        // // joint5.reset();
        // JointController joint6 = wrist_pitch.GetComponentInChildren<JointController>();
        // joint6.primaryAxisRotation = 0f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(target.transform.localPosition);
        sensor.AddObservation(target.transform.InverseTransformPoint(eet.transform.position));
        sensor.AddObservation(eet.transform.InverseTransformPoint(ee.transform.position));

        sensor.AddObservation(disv);
        sensor.AddObservation(disv1);
        sensor.AddObservation(distance);

        sensor.AddObservation(independentJoints[0].currentJointValue);
        sensor.AddObservation(independentJoints[1].currentJointValue);
        sensor.AddObservation(independentJoints[2].currentJointValue);

        // sensor.AddObservation(outer_roll.GetComponentInChildren<JointController>().primaryAxisRotation);
        // sensor.AddObservation(wrist_pitch.GetComponentInChildren<JointController>().primaryAxisRotation);
        // sensor.AddObservation(wrist_pitch.GetComponentInChildren<JointController>().primaryAxisRotation);
    }


    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var continuousActions = actionBuffers.ContinuousActions;
        var i = -1;
        
        float J1 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
        float J2 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
        float J3 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
        float J4 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
        float J5 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
        float J6 = Mathf.Clamp(continuousActions[++i], -1f, 1f);

        // J1 = (J1 + 1f) * 0.5f;
         fx = J1;
         fy = J2;
         fz = J3;//Mathf.Lerp(-90f, 90f, J1);

        force = new Vector3(fx, fy, fz);
        eerb.AddForce(force, ForceMode.VelocityChange);

        J4 = (J4 + 1f) * 0.5f;
        J4 = Mathf.Lerp(-130f, 130f, J4);
        independentJoints[0].SetJointValue(J4);
        // JointController joint4 = outer_roll.GetComponentInChildren<JointController>();
        // joint4.primaryAxisRotation = Mathf.Lerp(-130f, 130f, J4);
        // // joint4.run(Mathf.Lerp(-130f, 130f, J4));
       
        J5 = (J5 + 1f) * 0.5f;
        J5 = Mathf.Lerp(-90f, 90f, J5);
        independentJoints[1].SetJointValue(J5);
        // JointController joint5 = wrist_pitch.GetComponentInChildren<JointController>();
        // joint5.primaryAxisRotation = Mathf.Lerp(-90f, 90f, J5);
        // joint5.run(Mathf.Lerp(-90f, 90f, J5));
        
        J6 = (J6 + 1f) * 0.5f;
        J6 = Mathf.Lerp(-80f, 80f, J6);
        independentJoints[2].SetJointValue(J6);
        // JointController joint6 = wrist_pitch.GetComponentInChildren<JointController>();
        // joint6.primaryAxisRotation = Mathf.Lerp(-80f, 80f, J6);



    }

    // Update is called once per frame
    void FixedUpdate()
    {
        dis = (float)Vector3.Distance(eet.transform.position, Cons1.transform.position );
        divx = (float)eet.transform.position.x - Cons1.transform.position.x;
        double disvs = Math.Pow(dis, 2) - Math.Pow(divx, 2);
        disv = (float)Math.Sqrt(disvs);

        float dis1 = (float)Vector3.Distance(eet1.transform.position, Cons1.transform.position );
        float divx1 = (float)eet1.transform.position.x - Cons1.transform.position.x;
        double disvs1 = Math.Pow(dis1, 2) - Math.Pow(divx1, 2);
        disv1 = (float)Math.Sqrt(disvs1);
        if (disv1 < 0.9f){
            cost = 30f;
        }
        if (disv < 1f){
            cost = 30f;
        }
        
        //  sin = (float)(divx/dis);

        //  ang = Mathf.Rad2Deg*(Mathf.Asin(sin)); 



        // Vector3 eet_right =  right.position-eet.position;
        // Vector3 eet_left =  left.position-eet.position;
        // Vector3 eet_cons = Cons1.transform.position - eet.transform.position;

        // Vector3 nn = Vector3.Cross(eet_left, eet_right);

        // Vector3 eetconsv = Quaternion.AngleAxis(ang, nn) * eet_cons;


        // Vector3 vol = et.velocity;
        // // Vector3 vol_local = workspace.InverseTransformPoint(vol);
        // // Vector3 vol_locall = vol_local- eet.transform.position;
        // // Vector3 cons_local = eet.InverseTransformPoint(eet.transform.position);
        // constovol =90f - Vector3.Angle(nn, vol);
        // if (dis <0.15f){
        //     AddReward(constovol/90f);
        // }
        
        // float joint5 = wrist_pitch.GetComponentInChildren<jointposition>().position;
        // if (dis <0.15f){
        //     AddReward(-(1.6f - joint5));
        // }
        distance = Vector3.Distance(target.transform.position, eet.transform.position);
        float reward = 1-distance/15f;
        SetReward(reward);

        if (Vector3.Distance(target.transform.position, ee.transform.position) < 1.79f) {
                 AddReward(10f);
                //  EndEpisode();
                //  Debug.Log("end1");
                //  Debug.Log("end1"+distance);
                 }
        if (Vector3.Distance(target.transform.position, eet.transform.position) < 0.1f) {
                 AddReward(50f);
                 EndEpisode();
                //  Debug.Log("end1");
                //  Debug.Log("end1"+distance);
                 }
        
        
    }
}
