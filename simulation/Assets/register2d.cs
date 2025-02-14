using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.SideChannels;

public class Register2d : MonoBehaviour
{

    CostSideChannel costChannel;
    public void Awake()
    {
        // We create the Side Channel
        costChannel = new CostSideChannel();

        // When a Debug.Log message is created, we send it to the stringChannel
        // Application.logMessageReceived += costChannel.SendCostToPython;

        // The channel must be registered with the SideChannelManager class
        SideChannelManager.RegisterSideChannel(costChannel);

        // IList<float> CostList = new List<float>{GetComponent<getcost>().CostList0, GetComponent<getcost>().CostList1,GetComponent<getcost>().CostList2,GetComponent<getcost>().CostList3};
        // // IList<float> CostList = new List<float>{5,4,3,2};
        // costChannel.SendCostToPython(CostList);
        // Debug.Log("ooooooooooo哦哦哦哦哦"+GetComponent<getcost>().CostList0);
    }

    public void OnDestroy()
    {
        // De-register the Debug.Log callback
        // Application.logMessageReceived -= costChannel.SendCostToPython;
        if (Academy.IsInitialized){
            SideChannelManager.UnregisterSideChannel(costChannel);
        }
    }

    public void Update()
    {
        // Optional : If the space bar is pressed, raise an error !
        if (Input.GetKeyDown(KeyCode.Space))
        {
            Debug.LogError("This is a fake error. Space bar was pressed in Unity.");
        }

        IList<float> CostList = new List<float>{GetComponent<getcost_2d>().CostList0, 
        GetComponent<getcost>().CostList1,
        GetComponent<getcost>().CostList2,
        GetComponent<getcost>().CostList3,
        // GetComponent<getcost>().CostList4,
        // GetComponent<getcost>().CostList5,
        };
        // IList<float> CostList = new List<float>{5,4,3,2};
        costChannel.SendCostToPython(CostList);
        // Debug.Log("ooooooooooo哦哦哦哦哦"+GetComponent<getcost>().CostList0);
    }
}
