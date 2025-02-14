/*
© Siemens AG, 2017-2019
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

    public class joint_state_sub_conti: UnitySubscriber<RosSharp.RosBridgeClient.MessageTypes.Sensor.JointState>
    {
        public GameObject agent;
        public GameObject JS_pub;
        public List<string> JointNames;
        public List<float> JointState= new List<float>{0f,0f,0f,0f,0f,0f};
        // public List<JointStateWriter> JointStateWriters;
        public ArticulationBody[] joints ;
        private bool isMessageReceived;
        // public List<float> JS;
        //  void Start(){
        //      agent.GetComponent<psm_visual>().enabled = true;

        // }



        protected override void ReceiveMessage(RosSharp.RosBridgeClient.MessageTypes.Sensor.JointState message)
        {
            int index;
            // if (!isMessageReceived){
                // print("JS"+isMessageReceived);
                    for (int i = 0; i < message.name.Length; i++)
                    {
                        // print("sub"+i+message.name.Length);
                        index = JointNames.IndexOf(message.name[i]);
                        // print("sub!"+index);
                        if (index != -1){
                           // print("zheli!!!!"+message.position[i]);
                            // 
                            // JointStateWriters[index].Write((float) message.position[i]);
                            JointState[index] = (float)message.position[i];
                            // print("jointstate"+((float) message.position[i]));
                           
                        }
                    }
                    // isMessageReceived = true;//
                //     print("jointstate");
                //    print("jointstate"+agent.GetComponent<psm_visual>().enabled);
                //     // psm.enabled = true;
                //     agent.GetComponent<psm_visual>().enabled = true; 
                   
                //     // print("youmeiyou?");
                //     // JS_pub.GetComponent<JointStatePublisher>().enabled = true;
                //     // JS_pub.enabled= true;
            // }
        }

       
    }


