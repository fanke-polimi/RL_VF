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




public class RollerAgent : Agent
{

    public List<URDFJoint> independentJoints = new List<URDFJoint>();
    // public List<URDFJoint> mimicJoints = new List<URDFJoint>();

    // public Transform TargetPrefab; //Target prefab to use in Dynamic envs
    public Transform Target1;
    public GameObject Cons1;
    // public Transform Target2;
    // public Transform Target3;
    //private Transform m_Target; //Target the agent will walk towards during training.
    // public Transform griperL;
    // public Transform griperR;
    public Transform ee;
    public Transform eet;
    public float deltamove;
    public Transform Min;
    public Transform Max;
    // public Transform griperL;
    // public Transform griperR;


    //public Transform limitmax;
    //public Transform limitmin; 
    float xmin = -1.5f;
    float xmax = 1.5f;
    float ymin = -0.9f;
    float ymax = -1.4f;
    float zmin = -1f;
    float zmax = 0.9f;
    int pos_x = 0;
    int pos_y = 0;
    int pos_z = 0;
    int pos_4 = 0;
    int pos_5 = 0;
    int pos_6 = 0;

    float p1;
    float p2;

    float joint4_roll;
    float joint5_pitch;
    float joint6_yaw;


    public float reward;
    public float maxdistance;
    public float ee_tarDis;

    //public TargetContact targetContact;
    private Transform eePositionRotation;
    private Vector3 startingPos;
    private Quaternion startingRot;
    public float cost;

    public int step = 0;
    // public int MaxStep = 4000;

    // public float joint1

    //public List<URDFJoint> independentJoints = new List<URDFJoint>();
    // public DVRK.URDFJoint jaw = null;

    //URDFRobot m_JdController;
    // Start is called before the first frame update
    //Rigidbody rBody;
    void Start()
    {
        //rBody = GetComponent<Rigidbody>();
        startingPos = ee.transform.localPosition;
        // Debug.Log(startingPos);
        startingRot = ee.transform.localRotation;
        eePositionRotation = ee.transform;
        maxdistance = Vector3.Distance(Min.transform.localPosition, Max.transform.localPosition);
        

    }
    //public Transform Target;
    public override void OnEpisodeBegin()
    {
        // Debug.Log("Episode Begin");
        ee.transform.localPosition = startingPos;
        // Debug.Log("ee位置"+ee.transform.localPosition);
        ee.transform.localRotation = startingRot;
        step =0;

        // if ((ee.transform.localPosition.x < xmin) || (ee.transform.localPosition.x > xmax) || (ee.transform.localPosition.y < ymin) || (ee.transform.localPosition.y > ymax) || (ee.transform.localPosition.z < zmin) || (ee.transform.localPosition.z > zmax))
        // {
        //     ee.transform.localPosition = startingPos;
        //     ee.transform.localRotation = startingRot;
            
        // }
        //  Debug.Log("用法一");
        // joint4_roll = 0f;
        // // Random.Range(-129f, 129f);

        // independentJoints[0].SetJointValue(0);

        // // joint5_pitch = 0f;
        // // // Random.Range(-89f, 89f);

        independentJoints[3].SetJointValue(0);
        independentJoints[4].SetJointValue(0);

        independentJoints[5].SetJointValue(0);
        independentJoints[6].SetJointValue(0);
        // independentJoints[4].SetJointValue(0);

        // joint6_yaw = 0f;
        // // Random.Range(-79f, 79f);


        // independentJoints[2].SetJointValue(joint6_yaw);

        // float griper1 = 30f;
        // float griper2 = -30f;

        // mimicJoints[0].SetJointValue(30f);
        // mimicJoints[1].SetJointValue(-30f);
        


        // if (this.transform.localPosition.y < 0) //if the agent fell, zero its momentum
        // {
        //     this.rBody.angularVelocity = Vector3.zero;
        //     this.rBody.velocity = Vector3.zero;
        //     this.transform.localPosition = new Vector3(0, 0.5f, 0);

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
        // sensor.AddObservation(Target.localPosition);
        // sensor.AddObservation(this.transform.localPosition);

        // sensor.AddObservation(rBody.velocity.x);
        // sensor.AddObservation(rBody.velocity.z);

        // Debug.Log("OBSERVATION");
        sensor.AddObservation(Target1.localPosition);
        sensor.AddObservation(Target1.transform.InverseTransformPoint(eet.transform.position));
        sensor.AddObservation(eet.transform.InverseTransformPoint(ee.transform.position));
        sensor.AddObservation(-independentJoints[3].currentJointValue);
        sensor.AddObservation(-independentJoints[4].currentJointValue);
        sensor.AddObservation(-independentJoints[5].currentJointValue);
        // sensor.AddObservation(Cons1.transform.localPosition);
        // sensor.AddObservation(Cons1.transform.InverseTransformPoint(ee.transform.position));
            //sensor.AddObservation(ee.transform.rotation);

        
            //添加目标和末端的相对位置
        // sensor.AddObservation(ee.transform.InverseTransformPoint(Target1.localPosition));
        sensor.AddObservation(Vector3.Distance(Target1.transform.position, eet.transform.position));
        
        
        // sensor.AddObservation(ee.transform.InverseTransformPoint(Target2.localPosition));
        // sensor.AddObservation(ee.transform.InverseTransformPoint(Target3.localPosition));
        // sensor.AddObservation(Vector3.Distance(Target1.transform.localPosition,ee.transform.localPosition));
        // sensor.AddObservation(Vector3.Distance(Target2.transform.localPosition,ee.transform.localPosition));
        // sensor.AddObservation(Vector3.Distance(Target3.transform.localPosition,ee.transform.localPosition));
    }

    //public float forceMultiplier = 10;
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Actions
        // Debug.Log("ACTIONRECEIVED");
        // Vector3 controlSignal = Vector3.zero;
        // var i = -1;
        // int Pos_y = Mathf.FloorToInt(actionBuffers.DiscreteActions[0]);
        // int Pos_z = Mathf.FloorToInt(actionBuffers.DiscreteActions[1]);
        // int Pos_x = Mathf.FloorToInt(actionBuffers.DiscreteActions[2]);

        // if (Pos_y == 0) {  pos_y = -1; }
        // if (Pos_y == 1) {  pos_y = 0; }
        // if (Pos_y == 2) {  pos_y = 1; }

        // if (Pos_z == 0) {  pos_z = -1; }
        // if (Pos_z == 1) {  pos_z = 0; }
        // if (Pos_z == 2) {  pos_z = 1; }

        // if (Pos_x == 0) {  pos_x = -1; }
        // if (Pos_x == 1) {  pos_x = 0; }
        // if (Pos_x == 2) {  pos_x = 1; }
        
        step++;
        
        //  Vector3 position = new Vector3(pos_x*deltamove,pos_y*deltamove,pos_z*deltamove);

        //  /     Vector3 controlSignal = Vector3.zero;
    //     var i = -1;
        int Pos_y = Mathf.FloorToInt(actionBuffers.DiscreteActions[0]);
        int Pos_z = Mathf.FloorToInt(actionBuffers.DiscreteActions[1]);
        int Pos_x = Mathf.FloorToInt(actionBuffers.DiscreteActions[2]);
        int Pos_4 = Mathf.FloorToInt(actionBuffers.DiscreteActions[3]);
        int Pos_5 = Mathf.FloorToInt(actionBuffers.DiscreteActions[4]);
        int Pos_6 = Mathf.FloorToInt(actionBuffers.DiscreteActions[5]);
        // int Pos_7 = Mathf.FloorToInt(actionBuffers.DiscreteActions[6]);

        if (Pos_y == 0) {  pos_y = -1; }
        if (Pos_y == 1) {  pos_y = 0; }
        if (Pos_y == 2) {  pos_y = 1; }

        if (Pos_z == 0) {  pos_z = -1; }
        if (Pos_z == 1) {  pos_z = 0; }
        if (Pos_z == 2) {  pos_z = 1; }

        if (Pos_x == 0) {  pos_x = -1; }
        if (Pos_x == 1) {  pos_x = 0; }
        if (Pos_x == 2) {  pos_x = 1; }

        Vector3 position = new Vector3(pos_x*deltamove, pos_y*deltamove, pos_z*deltamove);

        if (Pos_4 == 0) {  pos_4 = -1; }
        if (Pos_4 == 1) {  pos_4 = 0; }
        if (Pos_4 == 2) {  pos_4 = 1; }

        if (Pos_5 == 0) {  pos_5 = -1; }
        if (Pos_5 == 1) {  pos_5 = 0; }
        if (Pos_5 == 2) {  pos_5 = 1; }

        if (Pos_6 == 0) {  pos_6 = -1; }
        if (Pos_6 == 1) {  pos_6 = 0; }
        if (Pos_6 == 2) {  pos_6 = 1; }

        // if (Pos_6 == 0) {  pos_6 = 1; }
        // if (Pos_6 == 1) {  pos_6 = 0; }
        

    //    float joint0 = independentJoints[0].currentJointValue + pos_y*deltamove;
    //    float joint1 = independentJoints[1].currentJointValue + pos_z*deltamove;
    //    float joint2 = independentJoints[2].currentJointValue + pos_x*deltamove;
    //    joint2 = joint2/100;
       float joint3 = -independentJoints[3].currentJointValue + pos_4*deltamove*1000f;
       float joint4 = -independentJoints[4].currentJointValue + pos_5*deltamove*1000f;
       float joint5 = -independentJoints[5].currentJointValue + pos_6*deltamove*1000f;

        // independentJoints[0].SetJointValue(joint0);
        // independentJoints[1].SetJointValue(joint1);
        // independentJoints[2].SetJointValue(joint2);
        independentJoints[3].SetJointValue(joint3);
        independentJoints[4].SetJointValue(joint4);
        independentJoints[5].SetJointValue(joint5);
        // var i = -1;

        // Vector3 controlSignal = Vector3.zero;
        // // var continuousActions = actionBuffers.DiscreteActions

        //  controlSignal.x = Mathf.Clamp(actionBuffers.ContinuousActions[++i], -0.01f, 0.01f);
        //  controlSignal.z = Mathf.Clamp(actionBuffers.ContinuousActions[++i], -0.01f, 0.01f);
        //  controlSignal.y = Mathf.Clamp(actionBuffers.ContinuousActions[++i],-0.01f, 0.01f);

        // float pos1=  actionBuffers.ContinuousActions[0];
        // float pos2= actionBuffers.ContinuousActions[1];
        // float pos3 = actionBuffers.ContinuousActions[2];

        
        // Vector3 position = (pos1, pos2, pos3);
        Vector3 rotation = new Vector3(0, 0, 0);

        //ee.transform.localPosition = controlSignal;

        SetAgentPosition(position, rotation);
        // Debug.Log("control"+position*1000);

        // float reward;
        // cost = 2.5f;

        // if (Vector3.Distance(Target1.transform.localPosition, ee.transform.localPosition) > 0.03f){
        reward = 0.5f - ((Vector3.Distance(Target1.transform.position, eet.transform.position))/maxdistance );
        AddReward(reward);
        
        if (Vector3.Distance(Target1.transform.position, eet.transform.position) > 1.5f) {
                //  reward = -1f;
                //  cost = 3f;
                 AddReward(-1f);
                 EndEpisode();
                 
                 Debug.Log("end1");
                 }

        if (eet.transform.localPosition.y < -0.15f) {
                //  reward = -1f;
                //  cost = 3f; 
                 AddReward(reward);
                 EndEpisode();
                
                 Debug.Log("end2");
                 }

                
        if (step > MaxStep) {
                //  reward = -1f;
                //  cost = 3f;
                 AddReward(-1f);
                 EndEpisode();
                 
                 Debug.Log("end3");
                 }
        
        
        // }else if(Vector3.Distance(Target1.transform.localPosition, ee.transform.localPosition) < 0.0006f)
        // {  reward = 1f ; }
        if (Vector3.Distance(Target1.transform.position, eet.transform.position) < 0.05f) {
                //  reward = 1f;
                AddReward(1f);
                //  cost = 3f;
                EndEpisode();
                 } 
        ee_tarDis = Vector3.Distance(Target1.transform.position, eet.transform.position);
        // Debug.Log("reward"+reward);    
        // SetReward(reward);
        // if (Vector3.Distance(Target2.transform.position, ee.transform.position) < 0.007f) {
        //         reward = 1;
        //         EndEpisode();
        //     } else {
        //         reward = ((Vector3.Distance(Target2.transform.position, ee.transform.position)) * -0.1f)-0.25f;
        //     }

        // if (Vector3.Distance(Target3.transform.position, ee.transform.position) < 0.007f) {
        //         reward = 1;
        //         EndEpisode();
        //     } else {
        //         reward = ((Vector3.Distance(Target3.transform.position, ee.transform.position)) * -0.1f)-0.25f;
        //     }
        


        // if (Vector3.Distance(Target1.transform.localPosition, ee.transform.localPosition) > 0.001f)
        // {
        //     SetReward(-0.7f);
        //     Debug.Log("距离"+ Vector3.Distance(Target1.transform.localPosition, griperL.transform.localPosition));
        //     // EndEpisode();
        // } 

        // if (Vector3.Distance(Target1.transform.localPosition, ee.transform.localPosition) < 0.001f)
        // {
        //     SetReward(0.3f);
        //     Debug.Log("距离"+ Vector3.Distance(Target1.transform.localPosition, griperL.transform.localPosition));
        //     // EndEpisode();
        // }


        // if (Vector3.Distance(Target1.transform.localPosition, griperL.transform.localPosition) > 1.3f || Vector3.Distance(Target1.transform.localPosition, griperR.transform.localPosition) > 1.3f)
        // {
        //     SetReward(-0.7f);
        //     Debug.Log("距离"+ Vector3.Distance(Target1.transform.localPosition, griperL.transform.localPosition));
        //     // EndEpisode();
        // } 

        // if(Vector3.Distance(griperL.transform.localPosition, Target1.localPosition)<0.32f ||Vector3.Distance(griperR.transform.localPosition, Target1.localPosition)<0.32f)
        // {
        //     SetReward(0.5f);
        //     EndEpisode();
        // }



        // if ((ee.localPosition.x < -1.5f) || (ee.localPosition.x > 1.5f))
        // {
        //     SetReward(-1f);
        //     Debug.Log("x"+ ee.localPosition.x);
        //     EndEpisode();
        //     //System.Diagnostics.Console.WriteLine("jieshuhueshuieshu结束结束\n");
        //     //Debug.Log(ee.localPosition + "用法一");
            

        // }
        // // if((ee.localPosition.y < -1.4f) || (ee.localPosition.y > -0.9f))
        // // {
        // //      EndEpisode();
        // //      Debug.Log(ee.localPosition + "用法一");
        // // }
        // if((ee.localPosition.z < -1f) || (ee.localPosition.z > 0.9f))
        // {
        //      SetReward(-1f);
        //      Debug.Log("z"+ ee.localPosition.z);
        //      EndEpisode();
        // }

        // controlSignal.x = p1;
        // controlSignal.z = p2;
         //rBody.AddForce(ee.transform.localPosition * 10f);

        // //REwards
        // float distanceToTarget = Vector3.Distance(this.transform.localPosition, Target.localPosition);

        // if (distanceToTarget < 1.42f)
        // {
        //     SetReward(1.0f);
        //     EndEpisode();
        // }

        // if (this.transform.localPosition.y < 0)
        // {
        //     EndEpisode();
        // }

        
    }
    private void SetAgentPosition(Vector3 pos, Vector3 rotation)
    {
        var tmp = eePositionRotation.transform.transform.localEulerAngles.x + rotation.x;
        rotation.x = tmp > 360 ? tmp - 360 : tmp;
        tmp = eePositionRotation.transform.transform.localEulerAngles.y + rotation.y;
        rotation.y = tmp > 360 ? tmp - 360 : tmp;
        tmp = eePositionRotation.transform.transform.localEulerAngles.z + rotation.z;
        rotation.z = tmp > 360 ? tmp - 360 : tmp;
        eePositionRotation.transform.transform.localEulerAngles = rotation;

        ee.transform.localPosition += pos;
        // if ((ee.transform.position.x < xmin) || (ee.transform.position.x > xmax || (ee.transform.position.y < ymin) || (ee.transform.position.y > ymax) || (ee.transform.position.z < zmin) || (ee.transform.position.z > zmax))
        // {
            
        // }
        // if (desired.x > limitmax.transform.position.x || desired.x < limitmin.transform.position.x)
        //     pos.x = 0f;
        // if (desired.y > limitmax.transform.position.y || desired.y < limitmin.transform.position.y)
        //     pos.y = 0f;
        // if (desired.z > limitmax.transform.position.z || desired.z < limitmin.transform.position.z)
        //     pos.z = 0f;

        
        // float p1 = ee.transform.localPosition.x;
        // float p2 = ee.transform.localPosition.y;
       

    }

    public override void Heuristic(in ActionBuffers actionsOut)
        {
            var ContinuousActionsOut = actionsOut.ContinuousActions;
            // ContinuousActionsOut[0] = Input.GetAxis("Horizontal");
            // ContinuousActionsOut[1] = Input.GetAxis("Vertical");
            
        
        if (Input.GetKey(KeyCode.S))
        {
        ContinuousActionsOut[0] = 0.1f;
        }
        if (Input.GetKey(KeyCode.A))
        {
        ContinuousActionsOut[0] = -0.1f;
        }
        if (Input.GetKey(KeyCode.D))
        {
        ContinuousActionsOut[2] = -0.1f;}
        if (Input.GetKey(KeyCode.W))
        {
        ContinuousActionsOut[2] =0.1f;}
      
        

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


