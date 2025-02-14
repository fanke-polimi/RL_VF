using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.ComponentModel;
using System;
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

public class continuousAgent : Agent
{
    public Transform Target1;
    public GameObject Cons1;
    public Transform eet;
    public Transform eet1;
    public Transform eet2;
    public Transform workspace;
    public Transform right;
    public Transform left;
    public Rigidbody et;
    public Transform Min;
    public Transform Max;

    public ArticulationBody outer_yaw;
    public ArticulationBody outer_pitch;
    public ArticulationBody insertion;
    public ArticulationBody outer_roll;
    public ArticulationBody wrist_pitch;
    public ArticulationBody wrist_yaw;

    public Transform outer_yaw_joint;
    public Transform outer_pitch_joint;
    public Transform insertion_joint;
    public Transform outer_roll_joint;
    public Transform wrist_pitch_joint;
    public Transform wrist_yaw_joint;

    public ArticulationBody outer_pitch1;
    public ArticulationBody outer_pitch3;
    public ArticulationBody outer_pitch5;
    public ArticulationBody outer_pitch4;
    public ArticulationBody outer_pitch2;

    Quaternion outer_yaw_start;
    public Quaternion outer_pitch_start;
    public Vector3 insertion_start;
    Quaternion outer_roll_start;
    Quaternion wrist_pitch_start;
    Quaternion wrist_yaw_start;

    Vector3 outer_pitch_start1;
    public Vector3 outer_pitch_start3;
    Vector3 outer_pitch_start5;
    Vector3 outer_pitch_start4;
    Vector3 outer_pitch_start2;
    
    



    public int step = 0;
    // public int MaxStep;
    public float reward;
    public float cost;
    public float maxdistance;

    public float control;
    public float distance;
    public float disv;
    public float disv1;
    public float disv2;

    public double divx;
    public double dis;

   public  float sin;
    public float ang;
    public float constovol;
    public bool collision;
    // public  gamejointposition1;




    // Start is called before the first frame update
    public override void Initialize()
    {
        maxdistance = Vector3.Distance(Min.transform.position, Max.transform.position);
        // SpawnTarget(TargetPrefab, transform.position); //spawn target

        outer_yaw = GetComponent<ArticulationBody>();
        outer_yaw_start = outer_yaw.GetComponentInChildren<Transform>().rotation;
        outer_pitch = GetComponent<ArticulationBody>();
        outer_pitch_start = outer_pitch.GetComponentInChildren<Transform>().rotation;
        insertion = GetComponent<ArticulationBody>();
        insertion_start = insertion.GetComponentInChildren<Transform>().position;
        outer_roll = GetComponent<ArticulationBody>();
        outer_roll_start = outer_roll.GetComponentInChildren<Transform>().rotation;
        wrist_pitch = GetComponent<ArticulationBody>();
        wrist_pitch_start = wrist_pitch.GetComponentInChildren<Transform>().rotation;
        wrist_yaw = GetComponent<ArticulationBody>();
        wrist_yaw_start = wrist_yaw.GetComponentInChildren<Transform>().rotation;

        outer_pitch1 = GetComponent<ArticulationBody>();
        outer_pitch_start1 = outer_pitch1.GetComponentInChildren<Transform>().position;
        outer_pitch3 = GetComponent<ArticulationBody>();
        outer_pitch_start3 = outer_pitch3.GetComponentInChildren<Transform>().position;
        outer_pitch5 = GetComponent<ArticulationBody>();
        outer_pitch_start5 = outer_pitch5.GetComponentInChildren<Transform>().position;
        outer_pitch4 = GetComponent<ArticulationBody>();
        outer_pitch_start4 = outer_pitch4.GetComponentInChildren<Transform>().position;
        outer_pitch2 = GetComponent<ArticulationBody>();
        outer_pitch_start2 = outer_pitch2.GetComponentInChildren<Transform>().position;

        outer_yaw_joint = GetComponent<Transform>();
        outer_pitch_joint = GetComponent<Transform>();
        insertion_joint = GetComponent<Transform>();
        insertion_joint.position = new Vector3(0,0.2f,0);
       outer_roll_joint = GetComponent<Transform>();
        wrist_pitch_joint = GetComponent<Transform>();
        wrist_yaw_joint = GetComponent<Transform>();
       
       cost=0f;

        et = et.GetComponent<Rigidbody>();

        //Setup each body part
    }

     public override void OnEpisodeBegin()
    {
        step = 0;
        Debug.Log("kaishi");
        collision = false;
        // cost = 0f;

        // outer_yaw_joint.rotation = outer_yaw_start;
        // outer_pitch_joint.rotation = outer_pitch_start;
        // insertion_joint.position = new Vector3(0,0.1f,0);
        // outer_roll_joint.rotation = outer_roll_start;
        // wrist_pitch_joint.rotation = wrist_pitch_start;
        // wrist_yaw_joint.rotation = wrist_yaw_start;
        // var drive1 = outer_yaw.xDrive;
        // drive1.damping = 0f;
        // drive1.stiffness = 1000000f;
        // outer_yaw.xDrive = drive1;

        // var drive2 = outer_pitch.xDrive;
        // drive2.damping = 0f;
        // drive2.stiffness = 1000000f;
        // outer_pitch.xDrive = drive2;

        // var drive3 = insertion.xDrive;
        // drive3.damping = 0f;
        // drive3.stiffness = 1000000f;
        // insertion.xDrive = drive3;

        // var drive4 = outer_roll.xDrive;
        // drive4.damping = 0f;
        // drive4.stiffness = 1000000f;
        // outer_roll.xDrive = drive4;

        // var drive5 = wrist_pitch.xDrive;
        // drive5.damping = 0f;
        // drive5.stiffness = 1000000f;
        // wrist_pitch.xDrive = drive5;

        // var drive6 = wrist_yaw.xDrive;
        // drive6.damping = 0f;
        // drive6.stiffness = 1000000f;
        // wrist_yaw.xDrive = drive6;
        

        JointController joint1 = outer_yaw.GetComponentInChildren<JointController>();
        joint1.primaryAxisRotation = 0f;
        // joint1.reset();
         pitchjoint joint2 = outer_pitch.GetComponentInChildren<pitchjoint>();
        joint2.primaryAxisRotation = 0f;
        // joint2.reset();
        articulation_joint_prismatic joint3 = insertion.GetComponentInChildren<articulation_joint_prismatic>();
        joint3.primaryAxisRotation = 1f;
        // joint3.reset();
        Debug.Log("reset");

        JointController joint4 = outer_roll.GetComponentInChildren<JointController>();
        joint4.primaryAxisRotation = 0f;
        // joint4.reset();
        JointController joint5 = wrist_pitch.GetComponentInChildren<JointController>();
        joint5.primaryAxisRotation = 0f;
        // joint5.reset();
        JointController joint6 = wrist_pitch.GetComponentInChildren<JointController>();
        joint6.primaryAxisRotation = 0f; 
        // joint6.reset();

        //Random start rotation to help generalize

        //Set our goal walking speed
        float a = UnityEngine.Random.value * 0.06f - UnityEngine.Random.value*0.06f;
        float b = (float)Math.Sqrt(0.04f*0.04f-a*a);
        float c = UnityEngine.Random.value * b - UnityEngine.Random.value*b;
        Target1.localPosition = new Vector3(a, -0.14f, -0.02f);
        
        // float jointposition1 = outer_yaw.GetComponentInChildren<jointposition>().position;
        // Debug.Log( "jointposition"+jointposition1);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // collision = ScoredACons.collision;
        
        


        
        sensor.AddObservation(Target1.localPosition);
        sensor.AddObservation(Target1.transform.InverseTransformPoint(eet.transform.position));
        // sensor.AddObservation(Cons1.transform.InverseTransformPoint(eet.transform.position));
        // sensor.AddObservation(disv);
        // sensor.AddObservation(constovol);
        sensor.AddObservation(Vector3.Distance(Target1.transform.position, eet.transform.position));
        // sensor.AddObservation(collision);

        sensor.AddObservation(outer_yaw.GetComponentInChildren<jointposition>().position);
        sensor.AddObservation(outer_pitch.GetComponentInChildren<jointposition>().position);
        sensor.AddObservation(insertion.GetComponentInChildren<jointposition>().position);
        sensor.AddObservation(outer_roll.GetComponentInChildren<jointposition>().position);
        sensor.AddObservation(wrist_pitch.GetComponentInChildren<jointposition>().position);
        sensor.AddObservation(wrist_yaw.GetComponentInChildren<jointposition>().position);

        sensor.AddObservation(outer_yaw.GetComponentInChildren<JointController>().primaryAxisRotation);
        sensor.AddObservation(outer_pitch.GetComponentInChildren<JointController>().primaryAxisRotation);
        sensor.AddObservation(insertion.GetComponentInChildren<articulation_joint_prismatic>().primaryAxisRotation);
        sensor.AddObservation(outer_roll.GetComponentInChildren<JointController>().primaryAxisRotation);
        sensor.AddObservation(wrist_pitch.GetComponentInChildren<JointController>().primaryAxisRotation);
        sensor.AddObservation(wrist_pitch.GetComponentInChildren<JointController>().primaryAxisRotation);

        
        
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        //  var drive1 = outer_yaw.xDrive;
        // drive1.damping = 2000f;
        // drive1.stiffness = 10000f;

        // var drive2 = outer_pitch.xDrive;
        // drive2.damping = 2000f;
        // drive2.stiffness = 10000f;
        // var drive3 = insertion.xDrive;
        // drive3.damping = 2000f;
        // drive3.stiffness = 10000f;
        // var drive4 = outer_roll.xDrive;
        // drive4.damping = 2000f;
        // drive4.stiffness = 10000f;

        // var drive5 = wrist_pitch.xDrive;
        // drive5.damping = 2000f;
        // drive5.stiffness = 10000f;
        // var drive6 = wrist_yaw.xDrive;
        // drive6.damping = 2000f;
        // drive6.stiffness = 10000f;
        step++;
        var continuousActions = actionBuffers.ContinuousActions;
        var i = -1;
        
        float J1 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
        float J2 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
        float J3 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
        float J4 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
        float J5 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
        float J6 = Mathf.Clamp(continuousActions[++i], -1f, 1f);
        
        J1 = (J1 + 1f) * 0.5f;
        JointController joint1 = outer_yaw.GetComponentInChildren<JointController>();
        joint1.primaryAxisRotation = Mathf.Lerp(-90f, 90f, J1);//
        // joint1.run(Mathf.Lerp(-90f, 90f, J1));
        
        J2 = (J2 + 1f) * 0.5f;
        pitchjoint joint2 = outer_pitch.GetComponentInChildren<pitchjoint>();
        joint2.primaryAxisRotation = Mathf.Lerp(-45f, 45f, J2);
        // joint2.run(Mathf.Lerp(-45f, 45f, J2));
        // Debug.Log("control  "+joint2.primaryAxisRotation);
       
        J3 = (J3 + 1f) * 0.5f;
        articulation_joint_prismatic joint3 = insertion.GetComponentInChildren<articulation_joint_prismatic>();
        joint3.primaryAxisRotation = Mathf.Lerp(0f, 2.4f, J3);
        // joint3.run(Mathf.Lerp(0f, 2.4f, J3));
        // Debug.Log("control333  "+joint3.primaryAxisRotation);
       
        J4 = (J4 + 1f) * 0.5f;
        JointController joint4 = outer_roll.GetComponentInChildren<JointController>();
        joint4.primaryAxisRotation = Mathf.Lerp(-130f, 130f, J4);
        // joint4.run(Mathf.Lerp(-130f, 130f, J4));
       
        J5 = (J5 + 1f) * 0.5f;
        JointController joint5 = wrist_pitch.GetComponentInChildren<JointController>();
        joint5.primaryAxisRotation = Mathf.Lerp(-90f, 90f, J5);
        // joint5.run(Mathf.Lerp(-90f, 90f, J5));
        
        J6 = (J6 + 1f) * 0.5f;
        JointController joint6 = wrist_pitch.GetComponentInChildren<JointController>();
        joint6.primaryAxisRotation = Mathf.Lerp(-80f, 80f, J6);
        // joint6.run(Mathf.Lerp(-80f, 80f, J6)); 

        // pitchjoint joint21 = outer_pitch.GetComponentInChildren<pitchjoint>();
        // joint21.primaryAxisRotation = joint2.primaryAxisRotation;

        // pitchjoint joint23 = outer_pitch.GetComponentInChildren<pitchjoint>();
        // joint23.primaryAxisRotation = -joint2.primaryAxisRotation;

        // pitchjoint joint25 = outer_pitch.GetComponentInChildren<pitchjoint>();
        // joint25.primaryAxisRotation = joint2.primaryAxisRotation;

        // pitchjoint joint24 = outer_pitch.GetComponentInChildren<pitchjoint>();
        // joint24.primaryAxisRotation = -joint2.primaryAxisRotation;

        // pitchjoint joint22 = outer_pitch.GetComponentInChildren<pitchjoint>();
        // joint22.primaryAxisRotation = joint2.primaryAxisRotation;

        

    }

     void FixedUpdate()
    {
        //  dis = (float)Vector3.Distance(eet.transform.position, Cons1.transform.position );
        //  divx = (float)eet.transform.position.x - Cons1.transform.position.x;
        // double disvs = Math.Pow(dis, 2) - Math.Pow(divx, 2);
        // disv = (float)Math.Sqrt(disvs)/10;
        
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
        

        // Debug.DrawRay(eet.transform.position, vol*10000, Color.green);
        // Debug.DrawRay(eet.transform.position, eetconsv, Color.red);
        // Debug.DrawRay(eet.transform.position,nn,Color.yellow);
        // Debug.DrawRay(eet.transform.position,eet_cons, Color.cyan);
        dis = (float)Vector3.Distance(eet.transform.position, Cons1.transform.position );
        divx = (float)eet.transform.position.x - Cons1.transform.position.x;
        double disvs = Math.Pow(dis, 2) - Math.Pow(divx, 2);
        disv = (float)Math.Sqrt(disvs);

        float dis1 = (float)Vector3.Distance(eet1.transform.position, Cons1.transform.position );
        float divx1 = (float)eet1.transform.position.x - Cons1.transform.position.x;
        double disvs1 = Math.Pow(dis1, 2) - Math.Pow(divx1, 2);
        disv1 = (float)Math.Sqrt(disvs1);

        float dis2 = (float)Vector3.Distance(eet2.transform.position, Cons1.transform.position );
        float divx2 = (float)eet2.transform.position.x - Cons1.transform.position.x;
        double disvs2 = Math.Pow(dis2, 2) - Math.Pow(divx2, 2);
        disv2 = (float)Math.Sqrt(disvs2);
        if (disv1 < 1f){
            cost = 30f;
        }
        if (disv < 0.85f){
            cost = 30f;
        }
        if (disv2 < 1f){
            cost = 30f;
        }

        if (disv1 >= 1f && disv >= 0.85f &&disv2 >= 1f){
            cost = 0f;
        }
        
       distance = Vector3.Distance(Target1.transform.position, eet.transform.position);

        if (Vector3.Distance(Target1.transform.position, eet.transform.position) > 2.3f) {
                 AddReward(-1f);
                 EndEpisode();
                 Debug.Log("end1");
                 Debug.Log("end1"+distance);
                 }

        if (step >= MaxStep/5) {
                 AddReward(-1f);
                 Debug.Log("end3");
                 EndEpisode();
                 
                 
                 }

        // if (eet.transform.localPosition.y < -0.15f) {
                
        //          SetReward(reward);
        //          EndEpisode();
                
        //          Debug.Log("end2");
        //          }
        // distance = Vector3.Distance(Target1.transform.position, eet.transform.position);
        if (Vector3.Distance(Target1.transform.position, eet.transform.position) < 0.0005f) {
            
                AddReward(1f);
                
                // EndEpisode();
                 }  
                 
        // reward = - ((10*Vector3.Distance(Target1.transform.position, eet.transform.position))/maxdistance);
        // AddReward(reward);

    }

    public void ScoredACons()
    {
        // We use a reward of 5.
        // AddReward(5f);
        
        // AddReward(-1f);
        // collision = true;
        // By marking an agent as done AgentReset() will be called automatically.
        // EndEpisode();
        // Debug.Log("碰撞了噢噢噢噢噢噢噢噢");

        // Swap ground material for a bit to indicate we scored.
        // StartCoroutine(GoalScoredSwapGroundMaterial(m_PushBlockSettings.goalScoredMaterial, 0.5f));
    }  

    public void ExitCons()
    {
        // We use a reward of 5.
        // AddReward(5f);
        
        // cost = 0f;
        // collision = true;
        // By marking an agent as done AgentReset() will be called automatically.
        // EndEpisode();
        // Debug.Log("碰撞了噢噢噢噢噢噢噢噢");

        // Swap ground material for a bit to indicate we scored.
        // StartCoroutine(GoalScoredSwapGroundMaterial(m_PushBlockSettings.goalScoredMaterial, 0.5f));
    } 

   
}
