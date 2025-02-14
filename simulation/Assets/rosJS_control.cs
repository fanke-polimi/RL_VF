using System.Collections;
using System.Collections.Generic;
using RosSharp.RosBridgeClient;
using UnityEngine;

public class rosJS_control : MonoBehaviour
{
    public ArticulationBody[] joints;
    public joint_state_sub js_sub;
    // public joint_state_sub_conti js_sub;
    public jaw_state_pub jaw;
    public procedure_control js;
    JointController joint1;
       
    pitchjoint joint2 ;
    
    articulation_joint_prismatic joint3 ;

    JointController joint4;
    
    JointController joint5 ;
   
    JointController joint6 ;
    public float pos1,pos2,pos3,pos4,pos5,pos6;
    public bool conti;
    // Start is called before the first frame update
    void Start()
    {
        // enabled=false;
         joint1 = joints[0].GetComponentInChildren<JointController>();
       
         joint2 = joints[1].GetComponentInChildren<pitchjoint>();
    
         joint3 = joints[2].GetComponentInChildren<articulation_joint_prismatic>();

         joint4 = joints[3].GetComponentInChildren<JointController>();
    
         joint5 = joints[4].GetComponentInChildren<JointController>();
   
         joint6 = joints[5].GetComponentInChildren<JointController>();
         
        joint1.damping=0;
        joint1.stiff=10000000000000000000000f;
        joint1.maxforce= 1000000000000000000000f;
        joint2.damping=0;
        joint2.stiff=10000000000000000000000f;
        joint2.maxforce= 100000000000000;
        joint3.damping=0;
        joint3.stiff=10000000000000000000000f;
        joint3.maxforce= 1000000000000000000000f;
        joint4.damping=0;
        joint4.stiff=10000000000000000000000f;
        joint4.maxforce= 1000000000000000000000f;
        joint5.damping=0;
        joint5.stiff=10000000000000000000000f;
        joint5.maxforce= 100000000000000;
        joint6.damping=0;
        joint6.stiff=10000000000000000000000f;
        joint6.maxforce= 1000000000000000000000f;
     
        
        
    }

    // Update is called once per frame
    void Update()
    {
        jaw.jaw_state = 1.2f;
        
        joint1.primaryAxisRotation = -js_sub.JointState[0]/3.1415926f*180f;//
        // print("agent!!"+enabled+ js.JS[0]);
        joint2.primaryAxisRotation = -js_sub.JointState[1]/3.1415926f*180f;
        joint3.primaryAxisRotation = js_sub.JointState[2]*10f;
        joint4.primaryAxisRotation = -js_sub.JointState[3]/3.1415926f*180f;
        joint5.primaryAxisRotation = -js_sub.JointState[4]/3.1415926f*180f;
        joint6.primaryAxisRotation = -js_sub.JointState[5]/3.1415926f*180f;
        pos1= -js_sub.JointState[0];
        pos2= -js_sub.JointState[1];
        pos3= js_sub.JointState[2];
        pos4= -js_sub.JointState[3];
        pos5= -js_sub.JointState[4];
        pos6= -js_sub.JointState[5];
        // if (conti){
        //     joint1.primaryAxisRotation = -js_sub_conti.JointState[0]/3.1415926f*180f;//
        // print("agent!!"+enabled+ js.JS[0]+js_sub_conti.JointState[0]);
        // joint2.primaryAxisRotation = -js_sub_conti.JointState[1]/3.1415926f*180f;
        // joint3.primaryAxisRotation = js_sub_conti.JointState[2]*10f;
        // joint4.primaryAxisRotation = -js_sub_conti.JointState[3]/3.1415926f*180f;
        // joint5.primaryAxisRotation = -js_sub_conti.JointState[4]/3.1415926f*180f;
        // joint6.primaryAxisRotation = -js_sub_conti.JointState[5]/3.1415926f*180f;
        // }
        
        if (joints[0].jointPosition[0]!=0f){
            // print("ros"+joints[0].jointPosition[0]);
            // print("ros"+joints[1].jointPosition[0]);
            print("enabled"+enabled);
            enabled = false;
        }
    }
}
