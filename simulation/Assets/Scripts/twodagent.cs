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

public class twodagent : Agent
{
    public GameObject start;
    public GameObject goal;
    public GameObject obs1;
    public GameObject obs2;

    public GameObject ee;

    private Transform eePositionRotation;
    private Vector3 startingPos;
    private Quaternion startingRot;

    public int pos_x ;
    public int pos_z ;

    public float deltamove;
    public float distance;
    public float distanceC;
    public Vector3 eePosition;
    public Vector3 position;

    public float cost;
    // Start is called before the first frame update
    void Start()
    {
        startingPos = ee.transform.localPosition;
        // Debug.Log(startingPos);
        
        
    }

    public override void OnEpisodeBegin()
    {
        ee.transform.localPosition = startingPos;
        cost = 0f;

    }

     public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(goal.transform.localPosition);
        sensor.AddObservation(ee.transform.localPosition);
        // sensor.AddObservation(obs1.transform.localPosition);
        sensor.AddObservation(obs2.transform.localPosition);
        sensor.AddObservation(Vector3.Distance(goal.transform.position, ee.transform.position));
        // sensor.AddObservation(Vector3.Distance(obs1.transform.position, ee.transform.position));
        sensor.AddObservation(Vector3.Distance(obs2.transform.position, ee.transform.position));
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        int Pos_x = Mathf.FloorToInt(actionBuffers.DiscreteActions[0]);
        int Pos_z = Mathf.FloorToInt(actionBuffers.DiscreteActions[1]);

        if (Pos_z == 0) {  pos_z = -1; }
        if (Pos_z == 1) {  pos_z = 0; }
        if (Pos_z == 2) {  pos_z = 1; }

        if (Pos_x == 0) {  pos_x = -1; }
        if (Pos_x == 1) {  pos_x = 0; }
        if (Pos_x == 2) {  pos_x = 1; }

        position = new Vector3(pos_x*deltamove, 0f, pos_z*deltamove);

        ee.transform.localPosition += position;


    }


    // Update is called once per frame
    void Update()
    {
        distance = 1- Vector3.Distance(goal.transform.localPosition, ee.transform.localPosition)/0.33f;
        SetReward(distance);

        float obs_distance = Vector3.Distance(obs2.transform.localPosition, ee.transform.localPosition)/0.02f;
        distanceC = Vector3.Distance(obs2.transform.localPosition, ee.transform.localPosition);

        if (Vector3.Distance(obs2.transform.localPosition, ee.transform.localPosition)<0.02f && Vector3.Distance(obs2.transform.localPosition, ee.transform.localPosition)>0.015f){
            // AddReward(obs_distance-1);
            cost = 1f-obs_distance;
        }
        if (Vector3.Distance(obs2.transform.localPosition, ee.transform.localPosition)<=0.015f){
            cost = 30f;

        }
        // float obs_distance = -Vector3.Distance(obs2.transform.localPosition, ee.transform.localPosition)/0.33f;

        eePosition = ee.transform.localPosition;
        if (Vector3.Distance(goal.transform.localPosition, ee.transform.localPosition) < 0.015f) {
            
                SetReward(30f);
                
                EndEpisode();
                 }  
        
    }
}
