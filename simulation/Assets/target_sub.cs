

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class target_sub : UnitySubscriber<MessageTypes.Geometry.PoseStamped>
    {
        // public Transform PublishedTransform;
        public GameObject[] targets;

        public Vector3 position,newposition,prevposition, initpos0, initpos2, initpos3, initpos1;
        private Quaternion rotation;
        private Vector3 receivedPoint;
        public bool isMessageReceived, target_ok;

        protected override void Start()
        {
            newposition = new Vector3(-10f,-10f,-10f);
            prevposition = new Vector3(-100f,-100f,-100f);
            initpos0 = targets[0].transform.localPosition;
            initpos1 = targets[1].transform.localPosition;
            initpos2 = targets[2].transform.localPosition;
            // initpos3 = targets[3].transform.localPosition;
			base.Start();
            
		}
		
        private void Update()
        {
            if (position.x!=0f || position.y!=0f || position.z!=0f){

            
                // newposition = position;
                if (Vector3.Distance(newposition ,position)>0.001f){
                    newposition = position;
                
                //  print("test"+initpos2+targets[2].transform.localPosition);
                // print("tagte0"+newposition+position+ (Vector3.Distance(newposition,position)>0.009f));
                if (isMessageReceived && Vector3.Distance(targets[0].transform.localPosition,initpos0)<0.001f  ){
                    targets[0].transform.localPosition = position;
                    print("tagtee"+newposition+targets[0].transform.localPosition);
                }
                else if (isMessageReceived && Vector3.Distance(targets[1].transform.localPosition,initpos1)<0.001f && Vector3.Distance(targets[0].transform.localPosition,position)>0.005f)
                {
                    targets[1].transform.localPosition = position;
                    print("tagteee"+position+Vector3.Distance(targets[1].transform.localPosition,initpos2));
                }
                else if (isMessageReceived && Vector3.Distance(targets[2].transform.localPosition,initpos2)<0.001f &&
                Vector3.Distance(targets[0].transform.localPosition,position)>0.005f &&
                Vector3.Distance(targets[1].transform.localPosition,position)>0.005f )
                {
                    targets[2].transform.localPosition = position;
                     // print("tagte3"+position+Vector3.Distance(targets[2].transform.localPosition,initpos3));
                }  
                // else if (isMessageReceived && Vector3.Distance(targets[3].transform.localPosition,initpos3)<0.001f && 
                // Vector3.Distance(targets[0].transform.localPosition,position)>0.005f &&
                // Vector3.Distance(targets[1].transform.localPosition,position)>0.005f &&
                // Vector3.Distance(targets[2].transform.localPosition,position)>0.005f
                // )
                // {
                //     targets[3].transform.localPosition = position;
                    
                //     // print("tagte4"+position+Vector3.Distance(targets[3].transform.localPosition,initpos3));
                // }
                }
            
            // print("shoudaole");
             }  
             if (Vector3.Distance(targets[0].transform.localPosition,initpos0)>0.01f &&
             Vector3.Distance(targets[1].transform.localPosition,initpos1)>0.01f && 
             Vector3.Distance(targets[2].transform.localPosition,initpos2)>0.01f)// && 
            //  Vector3.Distance(targets[3].transform.localPosition,initpos3)>0.01f)
            {
                target_ok = true;
             }

        }

        protected override void ReceiveMessage(MessageTypes.Geometry.PoseStamped message)
        {
            // receivedPoint = new Vector3((float)message.x, (float)message.y, (float)message.z);
            // Debug.Log("Received point: " + receivedPoint);

            position = GetPosition(message).Ros2Unity();
            rotation = GetRotation(message).Ros2Unity();
            isMessageReceived = true;
        }

        // private void ProcessMessage()
        // {
        //     newposition = position;
        //     PublishedTransform.localPosition = newposition;
        //     PublishedTransform.localRotation = newposition;
        // }

        private Vector3 GetPosition(MessageTypes.Geometry.PoseStamped message)
        {
            return new Vector3(
                (float)message.pose.position.x,
                (float)message.pose.position.y,
                (float)message.pose.position.z);
        }

        private Quaternion GetRotation(MessageTypes.Geometry.PoseStamped message)
        {
            return new Quaternion(
                (float)message.pose.orientation.x,
                (float)message.pose.orientation.y,
                (float)message.pose.orientation.z,
                (float)message.pose.orientation.w);
        }
    }
}