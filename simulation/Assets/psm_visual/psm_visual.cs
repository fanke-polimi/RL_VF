using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.ComponentModel;
//using System.ComponentModel.DataAnnotations.Schema;
using System.Diagnostics.Contracts;
using System.Security.Cryptography.X509Certificates;
//using System;
using System.Collections.Specialized;
//using System.Numerics;
using UnityEngine.Serialization;
using System.Reflection;
//using System.Diagnostics;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using RosSharp.RosBridgeClient;

public class psm_visual : Agent
{
    public jaw_state_pub jaw_state;
    public JointState_Pub js_pub;
    public ArticulationBody[] joints;
    public Transform[] jaw;
    public Transform[] targs;
    public Transform ground,eet,ECM;
    public float dampingCoff, stiffCoff, maxforce;
    // Start is called before the first frame update
    public int step, target_turn;
    public float distance;
    JointController joint1;
       
    pitchjoint joint2 ;
    
    articulation_joint_prismatic joint3 ;

    JointController joint4;
    
    JointController joint5 ;
   
    JointController joint6 ;
    jointposition jpos1, jpos2, jpos3, jpos4, jpos5, jpos6;
    float J1, J2, J3, J4,J5,J6;
    Vector3 Target_init_pos, ECM_init_pos, Target;
   
    
    public override void Initialize()
    {
        // enabled = false;
         joint1 = joints[0].GetComponentInChildren<JointController>();
       
         joint2 = joints[1].GetComponentInChildren<pitchjoint>();
    
         joint3 = joints[2].GetComponentInChildren<articulation_joint_prismatic>();

         joint4 = joints[3].GetComponentInChildren<JointController>();
    
         joint5 = joints[4].GetComponentInChildren<JointController>();
   
         joint6 = joints[5].GetComponentInChildren<JointController>();
         jpos1 = joints[0].GetComponentInChildren<jointposition>();
         jpos2 = joints[1].GetComponentInChildren<jointposition>();
         jpos3 = joints[2].GetComponentInChildren<jointposition>();
         jpos4 = joints[3].GetComponentInChildren<jointposition>();
         jpos5 = joints[4].GetComponentInChildren<jointposition>();
         jpos6 = joints[5].GetComponentInChildren<jointposition>();
        //  print("jpos"+jpos1);

        //  Target_init_pos = Target.transform.localPosition;
     

    }

    public override void OnEpisodeBegin()
    {
        step=0;
        jaw[0].localRotation = Quaternion.Euler(0,-45,0);
        jaw[1].localRotation = Quaternion.Euler(0,45,0);
        jaw_state.jaw_state = 1.200078539f;
        // js_pub.enabled = true;
        // // joint_T[0].transform.localRotation=Quaternion.Euler(0f,0f,0f);
        // print("reset"+joint_T[0].transform.localRotation);
        // joint1.damping=0;
        // joint1.stiff=10000000000000000000000f;
        // joint1.maxforce= 1000000000000000000000f;
        // joint2.damping=0;
        // joint2.stiff=10000000000000000000000f;
        // joint2.maxforce= 100000000000000;
        // joint3.damping=0;
        // joint3.stiff=10000000000000000000000f;
        // joint3.maxforce= 1000000000000000000000f;
        // joint4.damping=0;
        // joint4.stiff=10000000000000000000000f;
        // joint4.maxforce= 1000000000000000000000f;
        // joint5.damping=0;
        // joint5.stiff=10000000000000000000000f;
        // joint5.maxforce= 100000000000000;
        // joint6.damping=0;
        // joint6.stiff=10000000000000000000000f;
        // joint6.maxforce= 1000000000000000000000f;
        // joint1.primaryAxisRotation = -Random.Range(-0.8f,1.39f)/3.1415926f*180f;//
        // joint2.primaryAxisRotation = -Random.Range(-0.349f,0.349f)/3.1415926f*180f;
        // joint3.primaryAxisRotation = Random.Range(0f,0.1f)*10f;
        // joint4.primaryAxisRotation = -Random.Range(-1f,1f)/3.1415926f*180f;
        // joint5.primaryAxisRotation = -Random.Range(-0.26f,0.26f)/3.1415926f*180f;
        // joint6.primaryAxisRotation = -Random.Range(-0.26f,0.26f)/3.1415926f*180f;
        // //////////////////////////////////////////////////////////////////////////////////////////////////
    
        // ///////////////////////////////////////////////////////////////////////////////////////////////////////

        // Target.transform.localPosition = new Vector3(Target_init_pos.x+Random.Range(-0.10f, 0.10f),Target_init_pos.y+Random.Range(-0.05f,0.05f),Target_init_pos.z+Random.Range(-0.1f,0.1f));
        if (target_turn==1){
            print("!"+target_turn);
            Target = targs[0].position;
        }
        if (target_turn==2){
            print("!!"+target_turn);
            Target = targs[1].position;
        }
        if (target_turn==3){
            Target = targs[2].position;
        }
        // if (target_turn==1){
        //     Target.localPosition = targs[0].localPosition;
        // }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
         sensor.AddObservation(jpos1.position);
         sensor.AddObservation(jpos2.position);
         sensor.AddObservation(jpos3.position);
         sensor.AddObservation(jpos4.position);
         sensor.AddObservation(jpos5.position);
         sensor.AddObservation(jpos6.position);
         sensor.AddObservation(J1);
         sensor.AddObservation(J2);
         sensor.AddObservation(J3);
         sensor.AddObservation(J4);
         sensor.AddObservation(J5);
         sensor.AddObservation(J6);
        //  print("j_pos1"+jpos1.position);
        //  print("j_pos2"+jpos2.position);
        //  print("j_pos3"+jpos3.position);
        //  print("j_pos4"+jpos4.position);
        //  print("j_pos5"+jpos5.position);
        //  print("j_pos6"+jpos6.position);

         

         sensor.AddObservation(ECM.transform.InverseTransformPoint(Target));
         sensor.AddObservation(ECM.transform.InverseTransformPoint(eet.transform.position));
        // sensor.AddObservation(Cons1.transform.InverseTransformPoint(eet.transform.position));
        // sensor.AddObservation(disv);
        // sensor.AddObservation(constovol);
        sensor.AddObservation(Vector3.Distance(ECM.transform.InverseTransformPoint(Target), ECM.transform.InverseTransformPoint(eet.transform.position)));

    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        step++;
        var continuousActions = actionBuffers.ContinuousActions;
        var i = -1;
        
         J1 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
         J2 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
         J3 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
         J4 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
         J5 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
         J6 = Mathf.Clamp(continuousActions[++i], -1f, 1f);

        J1 = (J1 + 1f) * 0.5f;
        J2 = (J2 + 1f) * 0.5f;
        J3 = (J3 + 1f) * 0.5f;
        J4 = (J4 + 1f) * 0.5f;
        J5 = (J5 + 1f) * 0.5f;
        J6 = (J6 + 1f) * 0.5f;
    
        joint1.damping=dampingCoff;
        joint1.stiff=stiffCoff;
        joint1.maxforce= maxforce;
        joint2.damping=dampingCoff;
        joint2.stiff=stiffCoff;
        joint2.maxforce= maxforce;
        joint3.damping=dampingCoff;
        joint3.stiff=stiffCoff;
        joint3.maxforce= maxforce;
        joint4.damping=dampingCoff;
        joint4.stiff=stiffCoff;
        joint4.maxforce= maxforce;
        joint5.damping=dampingCoff;
        joint5.stiff=stiffCoff;
        joint5.maxforce= maxforce;
        joint6.damping=dampingCoff;
        joint6.stiff=stiffCoff;
        joint6.maxforce= maxforce;
        joint1.primaryAxisRotation = Mathf.Lerp(-90f, 90f, J1);//
        joint2.primaryAxisRotation = Mathf.Lerp(-45f, 45f, J2);
        joint3.primaryAxisRotation = Mathf.Lerp(0f, 2.4f, J3);
        joint4.primaryAxisRotation = Mathf.Lerp(-130f, 130f, J4);
        joint5.primaryAxisRotation = Mathf.Lerp(-90f, 90f, J5);
        joint6.primaryAxisRotation = Mathf.Lerp(-80f, 80f, J6);
        
        // print("action1"+J1);
        // print("action"+J2);
        // print("action"+J3);
        // print("action"+J4);
        // print("action"+J5);
        // print("action6"+J6);







    }


    // Update is called once per frame
    void Update()
    {
        distance = Vector3.Distance(ECM.transform.InverseTransformPoint(Target), ECM.transform.InverseTransformPoint(eet.transform.position));

        if (Vector3.Distance(ECM.transform.InverseTransformPoint(Target), ECM.transform.InverseTransformPoint(eet.transform.position)) > 0.4f) {
                 AddReward(-1f);
                //  EndEpisode();
                //  Debug.Log("end1");
                //  Debug.Log("end1"+distance);
                 }

       
        AddReward(-distance);
       
        if (Vector3.Distance(ECM.transform.InverseTransformPoint(Target), ECM.transform.InverseTransformPoint(eet.transform.position)) < 0.003f) {
            
                AddReward(1f);
                
                jaw_state.jaw_state= -0.0510f;
                jaw[0].localRotation = Quaternion.Euler(0,-20,0);
                jaw[1].localRotation = Quaternion.Euler(0,20,0);

                joint3.primaryAxisRotation = jpos3.position;
                joint4.primaryAxisRotation = jpos4.position/3.1415926f*180f;
                joint5.primaryAxisRotation = jpos5.position/3.1415926f*180f;
                joint6.primaryAxisRotation = jpos6.position/3.1415926f*180f;
                joint1.damping=10000000;
                joint2.damping=10000000;
                joint3.damping=10000000;
           
                enabled = false;
                // EndEpisode();
                 }  
                 
    }
}
