using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class after_reach_pcd : MonoBehaviour
{
    public jaw_state_pub jaw_state;
    public JointState_Pub js_pub;
    public ArticulationBody[] joints;
    public Transform[] jaw;
    public Transform[] targs;
    public Transform eet;
    public float dampingCoff, stiffCoff, maxforce,target_turn;

    JointController joint1;
       
    pitchjoint joint2 ;
    
    articulation_joint_prismatic joint3 ;

    JointController joint4;
    
    JointController joint5 ;
   
    JointController joint6 ;
    jointposition jpos1, jpos2, jpos3, jpos4, jpos5, jpos6;

     public bool first;

    public float place_increment, step;
    // Start is called before the first frame update
    void Start()
    {
        enabled = false;
        
        first = false;
        step = 0f;
    }

    // Update is called once per frame
    void Update()
    {   joint1 = joints[0].GetComponentInChildren<JointController>();
    
        joint2 = joints[1].GetComponentInChildren<pitchjoint>();

        joint3 = joints[2].GetComponentInChildren<articulation_joint_prismatic>();

        joint4 = joints[3].GetComponentInChildren<JointController>();

        joint5 = joints[4].GetComponentInChildren<JointController>();

        joint6 = joints[5].GetComponentInChildren<JointController>();

        jpos1 = joints[0].GetComponentInChildren<jointposition>();
        jpos2 = joints[1].GetComponentInChildren<jointposition>();
        jpos3 = joints[2].GetComponentInChildren<jointposition>();
          
        joint1.damping=dampingCoff;
        joint1.stiff=stiffCoff;
        joint1.maxforce= maxforce;
        joint2.damping=dampingCoff;
        joint2.stiff=stiffCoff;
        joint2.maxforce= maxforce;
        joint3.damping=dampingCoff;
        joint3.stiff=stiffCoff;
        joint3.maxforce= maxforce;
        // jpos4 = joints[3].GetComponentInChildren<jointposition>();
        // jpos5 = joints[4].GetComponentInChildren<jointposition>();
        // jpos6 = joints[5].GetComponentInChildren<jointposition>();
        if (!first){ ///if not use this, the joint3 will allways be set at 0.3               ////agent reach target, then joint3 back
            jaw_state.jaw_state= -0.0510f;
            jaw[0].localRotation = Quaternion.Euler(0,-20,0);
            jaw[1].localRotation = Quaternion.Euler(0,20,0);
            
            // joint3.primaryAxisRotation = 0.1f;
            joint2.primaryAxisRotation = 10f;
            step=0f;

            print("after"+first);
        }
        if (target_turn==1 && jaw[0].localRotation == Quaternion.Euler(0,-20,0)){           
            print("!"+target_turn);
            targs[0].position = eet.position;
            place_increment = 0;
        }
        if (target_turn==2 && jaw[0].localRotation == Quaternion.Euler(0,-20,0)){
            // print("!!"+target_turn);
            targs[1].position = eet.position;
            place_increment = 2f;
        }
        if (target_turn==3 && jaw[0].localRotation == Quaternion.Euler(0,-20,0)){
            targs[2].position = eet.position;
            place_increment = 4f;
        }
        if (target_turn==4 && jaw[0].localRotation == Quaternion.Euler(0,-20,0)){
            targs[3].position = eet.position;
            place_increment = 4f;
        }
        
        // joint4.primaryAxisRotation = jpos4.position/3.1415926f*180f;
        // joint5.primaryAxisRotation = jpos5.position/3.1415926f*180f;
        // joint6.primaryAxisRotation = jpos6.position/3.1415926f*180f;
        // print("!"+jpos3.position);
        if (jpos2.position >0.1745f && !first){  
            joint5.damping=dampingCoff-30000f;
            joint5.stiff=stiffCoff+30000f;                                                      ///joint3 back, then joint1&2 back
           joint3.primaryAxisRotation = 0.1f;  
        joint1.primaryAxisRotation = -48f+place_increment;//
        joint2.primaryAxisRotation = 20f+place_increment;
        joint5.primaryAxisRotation = 0f;
        
        
        }
        if (jpos3.position < 0.2f && jpos1.position < -0.837f+(place_increment/180f*3.1415926f) && jpos2.position > 0.34f+(place_increment/180f*3.1415926f)){  /// joint1&2&3 all back then joint3 go place
            print("!!!");
            first = true;
            // 
            joint3.primaryAxisRotation = 1.2f;
           
            
        }
        if (jpos3.position>1.15f && jpos2.position > 0.34f+(place_increment/180f*3.1415926f)){  
            step+=1f;
            print("!!!!");
            jaw_state.jaw_state= 1f;
            if(step>200f){
            joint3.primaryAxisRotation = 0.05f;                          // place, then back                   
            joint1.primaryAxisRotation = 0f;//
            joint2.primaryAxisRotation = 0f;
            jaw[0].localRotation = Quaternion.Euler(0,-60,0);
            jaw[1].localRotation = Quaternion.Euler(0,60,0);
            enabled=false;}
        }
         
    }
}
