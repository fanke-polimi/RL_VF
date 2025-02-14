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
using DVRK;




public class psmagent : Agent
{

    public List<URDFJoint> independentJoints = new List<URDFJoint>();
   
    public Transform Target1;
    public GameObject Cons1;
   
    public Transform ee;
    public Transform eet;
    public Transform tar;
    public Vector3 eeposition;
    public float deltamove = 0.02f;
    public Transform Min;
    public Transform Max;


    public float reward;
    public float maxdistance;
    public float ee_tarDis;
    public float cost;
    public float joint3;
    public float joint4;
    public float joint5;
    public int step = 0;

    public float movedistance;
    public Vector3 controlSignal;
    public int num_move_steps;

    void Start()
    {
        maxdistance = Vector3.Distance(Min.transform.localPosition, Max.transform.localPosition);
    }
    //public Transform Target;
    public override void OnEpisodeBegin()
    {
        step =0;

        independentJoints[0].SetJointValue(0);

        independentJoints[1].SetJointValue(0);
        independentJoints[2].SetJointValue(0);

        independentJoints[3].SetJointValue(0);
        independentJoints[4].SetJointValue(0);
        independentJoints[5].SetJointValue(0);
        independentJoints[6].SetJointValue(60);




        // }
        // Target.localPosition = new Vector3(Random.value * 8 - 4, 0.5f, Random.value *8 - 4);
        float a = UnityEngine.Random.value * 0.06f - UnityEngine.Random.value*0.06f;
        float b = (float)Math.Sqrt(0.04f*0.04f-a*a);
        float c = UnityEngine.Random.value * b - UnityEngine.Random.value*b;
        Target1.localPosition = new Vector3(a, -0.14f, -0.02f);
       /*   a = UnityEngine.Random.value * 0.04f - UnityEngine.Random.value*0.04f;
         b = (float)Math.Sqrt(0.04f*0.04f-a*a);
         c = UnityEngine.Random.value * b - UnityEngine.Random.value*b;
        Target2.localPosition = new Vector3(a, 0.01f, c);
         a = UnityEngine.Random.value * 0.04f - UnityEngine.Random.value*0.04f;
         b = (float)Math.Sqrt(0.04f*0.04f-a*a);
         c = UnityEngine.Random.value * b - UnityEngine.Random.value*b;
        Target3.localPosition = new Vector3(a, 0.01f, c); */ //定义圆形随机生成物体
    }

    public override void CollectObservations(VectorSensor sensor)
    {

        // Debug.Log("OBSERVATION");
        // sensor.AddObservation(Target1.transform.InverseTransformPoint(ee.transform.position));
        sensor.AddObservation(Target1.localPosition);
        sensor.AddObservation(Target1.transform.InverseTransformPoint(eet.transform.position));
        sensor.AddObservation(eet.transform.InverseTransformPoint(ee.transform.position));
       
        
    
        sensor.AddObservation(Vector3.Distance(Target1.transform.position, eet.transform.position));
        sensor.AddObservation(-(independentJoints[3].currentJointValue));
        sensor.AddObservation(-(independentJoints[4].currentJointValue));
        sensor.AddObservation(-(independentJoints[5].currentJointValue));
       
    }

    //public float forceMultiplier = 10;
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var i = -1;

        controlSignal = Vector3.zero;
        // var continuousActions = actionBuffers.DiscreteActions

         controlSignal.x = Mathf.Clamp(actionBuffers.ContinuousActions[++i],0f, 1f);
         controlSignal.z = Mathf.Clamp(actionBuffers.ContinuousActions[++i],0f, 1f);
         controlSignal.y = Mathf.Clamp(actionBuffers.ContinuousActions[++i],0f, 1f);
         Debug.Log(controlSignal.x + controlSignal.y +  controlSignal.z);

         float j3 = Mathf.Clamp(actionBuffers.ContinuousActions[++i], 0f, 1f);
         float j4 = Mathf.Clamp(actionBuffers.ContinuousActions[++i], 0f, 1f);
         float j5 = Mathf.Clamp(actionBuffers.ContinuousActions[++i], 0f, 1f);


        controlSignal.x = Mathf.Lerp(Min.transform.localPosition.x, Max.transform.localPosition.x, controlSignal.x);
        controlSignal.y = Mathf.Lerp(Min.transform.localPosition.y, Max.transform.localPosition.y, controlSignal.y);
        controlSignal.z = Mathf.Lerp(Min.transform.localPosition.z, Max.transform.localPosition.z, controlSignal.z);

        joint3 =  Mathf.Lerp(-130f, 130f, j3);
        joint4 =  Mathf.Lerp(-90f, 90f, j4);
        joint5 =  Mathf.Lerp(-80f, 80f, j5);

        tar.transform.localPosition = controlSignal;

    //    Vector3 tar =  ee.transform.InverseTransformPoint(position);//获取目标点相对于ee的坐标

        movedistance = Vector3.Distance(ee.transform.localPosition, tar.transform.localPosition);//求模

        num_move_steps = (int)Math.Floor(movedistance/deltamove);
        Debug.Log("步数"+num_move_steps);

        for(int j = 0; j <= num_move_steps; j++){
             Debug.Log("移动"+ i);
               ee.transform.localPosition = Vector3.MoveTowards(ee.transform.localPosition, tar.transform.localPosition, deltamove);
            //    float r3 = Mathf.Lerp(-independentJoints[3].currentJointValue, joint3, i/(num_move_steps+1));
            //    float r4 = Mathf.Lerp(-independentJoints[4].currentJointValue, joint3, i/(num_move_steps+1));
            //    float r5 = Mathf.Lerp(-independentJoints[5].currentJointValue, joint3, i/(num_move_steps+1));
               

        }
               independentJoints[3].SetJointValue(joint3);
               independentJoints[4].SetJointValue(joint4);
               independentJoints[5].SetJointValue(joint5);

        // MoveTo(tar.transform.localPosition, joint3, joint4, joint5, num_move_steps);
        Debug.Log("onactionreceive");




        
    }
    // private void MoveTo(Vector3 position, float joint3, float joint4, float joint5, int num_move_steps)
    // {

    //     // independentJoints[3].SetJointValue((independentJoints[3].currentJointValue)-joint3);
    //     // independentJoints[4].SetJointValue((independentJoints[4].currentJointValue)-joint4);
    //     // independentJoints[5].SetJointValue((independentJoints[5].currentJointValue)-joint5);

    //     // Vector3 tar = ground.transform.InverseTransformPoint
        

    //     for(int i = 0; i <= num_move_steps; i++){
    //          Debug.Log("移动"+ i);
    //            ee.transform.localPosition = Vector3.MoveTowards(ee.transform.localPosition, position, deltamove*Time.deltaTime);
    //            float r3 = Mathf.Lerp(-independentJoints[3].currentJointValue, joint3, i/num_move_steps);
    //            float r4 = Mathf.Lerp(-independentJoints[4].currentJointValue, joint3, i/num_move_steps);
    //            float r5 = Mathf.Lerp(-independentJoints[5].currentJointValue, joint3, i/num_move_steps);
    //            independentJoints[3].SetJointValue(-r3);
    //            independentJoints[4].SetJointValue(-r4);
    //            independentJoints[5].SetJointValue(-r5);

    //     }

    // }

    void FixedUpdate()
    {
        // float reward;
        // cost = 2.5f;

        // if (Vector3.Distance(Target1.transform.localPosition, ee.transform.localPosition) > 0.03f){
        float reward = 0.015f- ((Vector3.Distance(Target1.transform.localPosition, ee.transform.localPosition)) );
        // SetReward(reward);
        // cost = 0.2f - ((Vector3.Distance(Cons1.transform.position, ee.transform.position)) );
        if (Vector3.Distance(Target1.transform.position, ee.transform.position) > 2f){
             reward=-1f;
            //  AddReward(reward);
            EndEpisode();
            Debug.Log("end1");
        }

        if(ee.transform.localPosition.y < -0.16f){
            reward =-1f;
            EndEpisode();
            Debug.Log("end2");

        }
        
        
        
        // }else if(Vector3.Distance(Target1.transform.localPosition, ee.transform.localPosition) < 0.0006f)
        // {  reward = 1f ; }
        if (Vector3.Distance(Target1.transform.position, eet.transform.position) < 0.01f) {
                reward = 1f;
                independentJoints[6].SetJointValue(0);
                //  cost = 3f;
                EndEpisode();
                 } 
        ee_tarDis = Vector3.Distance(Target1.transform.position, ee.transform.position);
        eeposition = ee.transform.localPosition;
        // Debug.Log("reward"+reward);    
        SetReward(reward);
    }



    public void ScoredACons()
    {
        // We use a reward of 5.
        // AddReward(5f);
        
        // cost = 1f;
        // By marking an agent as done AgentReset() will be called automatically.
        // EndEpisode();
        // Debug.Log("碰撞了噢噢噢噢噢噢噢噢");

        // Swap ground material for a bit to indicate we scored.
        // StartCoroutine(GoalScoredSwapGroundMaterial(m_PushBlockSettings.goalScoredMaterial, 0.5f));
    }   
       

    // Update is called once per frame
    
}


