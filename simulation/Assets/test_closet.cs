using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using DVRK;

public class test_clo : MonoBehaviour
{
    // public PointCloudSubscriber subscriber;
    public List<GameObject> pcd_p = new List<GameObject>();
    // List<Vector3> pcd_init = new List<Vector3>(51);
    // List<Vector3> prvposition = new List<Vector3>();
    // Vector3[] curtposition ;
    // List<Vector3> newposition = new List<Vector3>();
    // List<Vector3> prvposition = new List<Vector3>();
    // Start is called before the first frame update
    int point_count;
    private Vector3 pointF;
    public float distanceF1;
    public float distanceF2;
    public float distanceF3;
    public float distanceF4;
    public float distanceF5;

    public Vector3 closest_p1;
    public Vector3 closest_p2;
    public Vector3 closest_p3;
    public Vector3 closest_p4;
    public Vector3 closest_p5;
    public Vector3 closest_p11;
    public Vector3 closest_p22;
    public Vector3 closest_p33;
    public Vector3 closest_p44;
    public Vector3 closest_p55;

    public float distance1;
    public float distance2;
    public float distance3;
    public float distance4;
    public float distance5;
    public float distance11;
    public float distance22;
    public float distance33;
    public float distance44;
    public float distance55;

    public Transform kp1;
    public Transform kp2;
    public Transform kp3;
    public Transform kp4;
    public Transform kp5;
    public Transform ECM;
    bool init_position;
    void Start()
    {
        pointF = new Vector3(10f,10f,10f);
       
       
        // for (int i = 0; i < 61; i++)
        // { 
        //     // print("??"+i);
        //     pcd_init.Add(new Vector3(0, 0, 0));  // 使用默认值初始化，可以根据需要修改
        // }
        // for (int i = 0; i < pcd_p.Count; i++)
        // {
        //     // print("?"+i+pcd_p.Count+pcd_p[i].transform.localPosition);
        //     pcd_init[i] = pcd_p[i].transform.localPosition;
        
        // }

    }

    // Update is called once per frame
    void Update()
    { 
        distanceF1 = Vector3.Distance(kp1.transform.position,pointF);
        distanceF2 = Vector3.Distance(kp2.transform.position,pointF);
        distanceF3 = Vector3.Distance(kp3.transform.position,pointF);
        distanceF4 = Vector3.Distance(kp4.transform.position,pointF);
        distanceF5 = Vector3.Distance(kp5.transform.position,pointF);
        // if (!init_position){
            
        //     init_position=true;
        // } 
 ///////////////////////////////////////////////////////////////////////////////////////////
        // if (subscriber.GetPCL().Length <pcd_p.Count){
        //     point_count = subscriber.GetPCL().Length;
        // }
        // else{
        //     point_count = pcd_p.Count;
        // }
        // curtposition = subscriber.GetPCL();
            
        // for (int i = 0; i < point_count; i++)
        // {
        //     //  print("!");
        //     // check if the point is new
        //      bool isNewPoint = true;
        //      foreach (Vector3 prevPoint in prvposition)
        //     {
        //         // print("!!");
        //         if (Vector3.Distance(curtposition[i], prevPoint) <= 0.0001f)
        //         {
        //             // print("!!!");
        //             isNewPoint = false;
        //             break;
        //         }
        //     }
        //     if (isNewPoint)
        //     {
        //         // print("!!!!"+i+curtposition[i]/1000f+pcd_init.Count+ pcd_p.Count);
        //         newposition.Add(curtposition[i]/1000f);
        //         if(Vector3.Distance(pcd_p[i].transform.localPosition,pcd_init[i])<0.005f){
                    
        //             pcd_p[i].transform.localPosition =  curtposition[i]/1000f;
        //             // print("!!!!!"+i+pcd_p[i].transform.localPosition);
        //         }
                
        //     }
            
        // }

        // prvposition = new List<Vector3>(curtposition);
///////////////////////////////////////////////////////////////////////////////////////////
        for (int i = 0; i < pcd_p.Count; i++)
            {
                print("suoyin"+i);
                distance1 = Vector3.Distance(ECM.transform.InverseTransformPoint(pcd_p[i].transform.position), ECM.transform.InverseTransformPoint(kp1.transform.position));
                distance2 = Vector3.Distance(ECM.transform.InverseTransformPoint(pcd_p[i].transform.position), ECM.transform.InverseTransformPoint(kp2.transform.position));
                distance3 = Vector3.Distance(ECM.transform.InverseTransformPoint(pcd_p[i].transform.position), ECM.transform.InverseTransformPoint(kp3.transform.position));
                distance4 = Vector3.Distance(ECM.transform.InverseTransformPoint(pcd_p[i].transform.position), ECM.transform.InverseTransformPoint(kp4.transform.position));
                distance5 = Vector3.Distance(ECM.transform.InverseTransformPoint(pcd_p[i].transform.position), ECM.transform.InverseTransformPoint(kp5.transform.position));
                // print("bianli"+pointcloud.transform.TransformPoint(_sourceData.pointPos[1].position));
                print("juli"+distance4+distanceF4);
                
                
                // if (distance1 < distanceF1){
                //     distanceF1=distance1;
                //     // distance1=distanceF1;
                //     closest_p1 = ECM.transform.InverseTransformPoint(pcd_p[i].transform.position);
                // }
                //  if (distance2 < distanceF2){
                //     distanceF2=distance2;
                //     // distance2=distanceF2;
                //     closest_p2 = ECM.transform.InverseTransformPoint(pcd_p[i].transform.position);
                // }
                //  if (distance3 < distanceF3){
                //     distanceF3=distance3;
                //     // distance3=distanceF3;
                //     closest_p3 = ECM.transform.InverseTransformPoint(pcd_p[i].transform.position);
                // }
                 if (distance4 <= distanceF4){
                    distanceF4=distance4;
                    // distance4=distanceF4;
                    print("FFFFF"+distanceF4+distance4);
                    closest_p4 = ECM.transform.InverseTransformPoint(pcd_p[i].transform.position);
                  
                    Debug.DrawLine(kp4.transform.position, pcd_p[i].transform.position,Color.red);
                }
                // if (distance5 < distanceF5){
                //     distanceF5=distance5;
                //     distance5=distanceF5;
                //     closest_p5 = ECM.transform.InverseTransformPoint(pcd_p[i].transform.position);
                // }
                
            }

         

        distance11 = distanceF1;
        distance22 = distanceF2;
        distance33 = distanceF3;
        distance44 = distanceF4;
        distance55 = distanceF5;

        closest_p11= closest_p1;
        closest_p22= closest_p2;
        closest_p33= closest_p3;
        closest_p44= closest_p4;
        closest_p55= closest_p5;
        


            
            
        

        
    }
}
