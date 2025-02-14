using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DVRK;

public class psm_ik_semi : MonoBehaviour
{
     public List<URDFJoint> independentJoints = new List<URDFJoint>();
   // public List<JointController> independentJoints = new List<JointController>();
    public List<URDFJoint> mimicJoints = new List<URDFJoint>();
    // public List<articulation_joint_prismatic> prismaticJoints = new List<articulation_joint_prismatic>();

   [SerializeField] GameObject EE;
    Matrix4x4 tipToWorldMat;
    [SerializeField] bool activeIK;
    public Transform ground;
    // public float joint4_roll;
    // public Transform insert;


    Matrix4x4 baseMat_To_tipToWorld;    //Base to Tip(C) 
    Matrix4x4 tipToGroundMat;
    Matrix4x4 groundTOworld;
    Vector3 nC;                         //unit vector of C
    Vector3 nEE;                        //unit vector of EE
    Vector3 t_Tc;                       //t of BaseToTip
    Vector3 p;                          //t of BaseToTip normalized
    Vector3 nB;                         //nC x p
    Vector3 vB;                         //unit vector from B to C   
    Vector3 pC;                         //traslation vector of C
    Vector3 pB;                         //traslation vector of B
    Vector3 pO;                         //traslation vector of Base
    Vector3 pEE;

    Vector3 nX;
    Vector3 nY;
    Vector3 nZ;

    Vector3 C_To_B;
    Vector3 B_To_Base;
    Vector3 Base_To_C;
    Vector3 B_To_C;
    Vector3 C_To_EE;
    Vector3 Base_To_X;
    Vector3 Base_To_B;

   public float joint6_yaw;
    public float joint5_pitch;
    public float joint4_roll;
    float joint3_prismatic;
    float joint2_pitch;
    float joint1_yaw;

    float offsetBC = 0;//0.009099992f;             //offset B to C
    float offsetCEE = 0;//0.01059f;
    float offsetPrismatic = 0.01560001f;
    // Start is called before the first frame update
    void Start()
    {
        nX = Vector3.right;
        nY = Vector3.up;
        nZ = Vector3.forward;
        
    }

    // Update is called once per frame
    void Update()
    {
        tipToWorldMat = Matrix4x4.TRS(new Vector3(EE.transform.position.x, EE.transform.position.y - 0.0106f, EE.transform.position.z), EE.transform.rotation, new Vector3(1, 1, 1));
        // tipToWorldMat = Matrix4x4.TRS(new Vector3(EE.transform.position.x, EE.transform.position.y , EE.transform.position.z), EE.transform.rotation, new Vector3(1, 1, 1));
        // Debug.Log("A矩阵1"+tipToWorldMat);
        // Debug.Log(EE.transform.position);
        groundTOworld = Matrix4x4.TRS(ground.transform.position, ground.transform.rotation, new Vector3(1,1,1));
        // Debug.Log("A矩阵2"+groundTOworld);
        tipToGroundMat = groundTOworld.inverse * tipToWorldMat;
        baseMat_To_tipToWorld = RotPosBase.baseMatInv*tipToGroundMat; //RotPosTip.tipToWorldMatInv * RotPosBase.baseMat


        nC = - new Vector3 (baseMat_To_tipToWorld.m02, baseMat_To_tipToWorld.m12, baseMat_To_tipToWorld.m22).normalized;
        nEE =  new Vector3(baseMat_To_tipToWorld.m01, baseMat_To_tipToWorld.m11, baseMat_To_tipToWorld.m21).normalized;


         pEE = new Vector3(EE.transform.localPosition.x, EE.transform.localPosition.y, EE.transform.localPosition.z);
        pC = pEE - (offsetCEE * nEE);



        Base_To_C = pC - pO;
        // Debug.Log("po"+pO);
        // Debug.Log("nc"+nC);

        p = Base_To_C.normalized;

        nB = - Vector3.Cross(p, nC).normalized;
        vB =  Vector3.Cross(nC, nB).normalized;
        pO = new Vector3(RotPosBase.baseMat.m03, RotPosBase.baseMat.m13, RotPosBase.baseMat.m23);

        pB = pC + (offsetBC * vB);

        //Debug.Log("pB: " + pB.ToString("F6"));

        C_To_B = pB - pC;
        B_To_Base = pO - pB;
        Base_To_C = pC - pO;
        B_To_C = pC - pB;
        C_To_EE = pEE - pC;
        Base_To_B = pB - pO;


        joint1_yaw = Quaternion.FromToRotation(Base_To_B, nY).eulerAngles.z -180;
        if (joint1_yaw < 360 && joint1_yaw > 150)
            joint1_yaw = joint1_yaw - 360;
        joint1_yaw = -joint1_yaw;
        //Debug.Log("Value joint1_yaw: " + joint1_yaw);

        joint2_pitch = Quaternion.FromToRotation(Base_To_B, nY).eulerAngles.x;
        if (joint2_pitch < 360 && joint2_pitch > 150)
            joint2_pitch = joint2_pitch - 360;
        //Debug.Log("Value joint2_pitch: " + joint2_pitch);

        joint3_prismatic = Vector3.Distance(pB, pO) + offsetPrismatic;
        //Debug.Log("joint3_prismatic: " + (joint3_prismatic));

        // joint4_roll =  Vector3.Angle(nB, Vector3.Cross(nZ, Base_To_B)) - 180;
        // if (Vector3.Dot(B_To_Base, Vector3.Cross(Vector3.Cross(nZ, Base_To_B), nB)) <=0 ) 
        //     joint4_roll = -joint4_roll;
        // joint4_roll = Vector3.Angle(nB, Vector3.Cross(nZ, Base_To_B)) - 180;
        if (Vector3.Dot(B_To_Base, Vector3.Cross(Vector3.Cross(nZ, Base_To_B), nB)) <=0 ) 
            // joint4_roll = -joint4_roll;
        // Debug.Log("joint4_roll: " + (joint4_roll));
        // Debug.Log("nB"+nB);
        // Debug.Log("nz"+nZ);
        // Debug.Log("basetoB"+Base_To_B);

        // joint5_pitch = 180 - Vector3.Angle(B_To_C, B_To_Base);
        if (Vector3.Dot(nB , (Vector3.Cross(B_To_C, B_To_Base)))>=0)
            // joint5_pitch = -joint5_pitch;
        //Debug.Log("joint5_pitch: " + (joint5_pitch));

        // joint6_yaw = 180 -  Vector3.Angle(C_To_B, C_To_EE);
        if (Vector3.Dot(nC, (Vector3.Cross(C_To_B, C_To_EE))) <= 0 )
            // joint6_yaw = -joint6_yaw;


    // Debug.DrawRay(pB, B_To_Base.normalized, Color.green);
    // Debug.DrawRay(pC, B_To_C.normalized, Color.red);
    // Debug.DrawRay(pC,p,Color.yellow);
    // Debug.DrawRay(pC,nC, Color.cyan);
    // Debug.DrawRay(pC,nB,Color.black);
    // Debug.DrawRay(pC,vB,Color.blue);
    // // Debug.DrawRay(ptt,nZ,Color.white);
    // Debug.DrawRay(pC,C_To_EE,Color.gray);
    // Debug.DrawRay(pC,C_To_B,Color.magenta);

        independentJoints[0].SetJointValue(joint1_yaw);
        independentJoints[1].SetJointValue(joint2_pitch);
        independentJoints[2].SetJointValue(joint3_prismatic);
       joint4_roll = independentJoints[3].currentJointValue;
        joint5_pitch = independentJoints[4].currentJointValue;
        joint6_yaw = independentJoints[5].currentJointValue;
    

        /* independentJoints[0].primaryAxisRotation=-joint1_yaw;
        independentJoints[1].primaryAxisRotation=-joint2_pitch;
        prismaticJoints[0].primaryAxisRotation=joint3_prismatic;


        independentJoints[2].primaryAxisRotation=-joint4_roll;
        independentJoints[3].primaryAxisRotation=-joint5_pitch;
        
        independentJoints[4].primaryAxisRotation=-joint6_yaw;
        
        independentJoints[5].primaryAxisRotation=-joint2_pitch;
        independentJoints[6].primaryAxisRotation=joint2_pitch;
        independentJoints[7].primaryAxisRotation=-joint2_pitch;
        independentJoints[8].primaryAxisRotation=joint2_pitch;
        independentJoints[9].primaryAxisRotation=-joint2_pitch; */




         












    }
}
