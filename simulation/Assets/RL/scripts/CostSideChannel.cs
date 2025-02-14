using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.SideChannels;
using System;

public class CostSideChannel : SideChannel
{

    public CostSideChannel()
    {
        ChannelId = new Guid("621f0a70-4f87-11ea-a6bf-784f4387d1f7");
    }

    protected override void OnMessageReceived(IncomingMessage msg)
    {
        var receivedString = msg.ReadString();
        Debug.Log("From Python : " + receivedString);
    }

     public void SendCostToPython(IList<float> cost)
    {
        // if (type == LogType.Error)
        // {
            var costToSend = cost;
            using (var msgOut = new OutgoingMessage())
            {
                msgOut.WriteFloatList(costToSend);
                QueueMessageToSend(msgOut);
            }
        // }
    }
    
}
