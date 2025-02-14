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

using System.Collections.Generic;

namespace RosSharp.RosBridgeClient
{
    public class jaw_state_pub : UnityPublisher<MessageTypes.Sensor.JointState>
    {
        // public List<jointposition> JointStateReaders;
        public string FrameId = "Unity";
        public float jaw_state;

        private MessageTypes.Sensor.JointState message;    
        
        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            // int jointStateLength = JointStateReaders.Count;
            message = new MessageTypes.Sensor.JointState
            {
                header = new MessageTypes.Std.Header { frame_id = FrameId },
                name = new string[1],
                position = new double[1],
                velocity = new double[1],
                effort = new double[1]
            };
        }

        private void UpdateMessage()
        {
            message.header.Update();
            // for (int i = 0; i < JointStateReaders.Count; i++)
                UpdateJointState();

            Publish(message);
        }

        private void UpdateJointState()
        {
           
            // JointStateReaders[i].Read(
            //     out message.name[0],
            //     out float position1,
            //     out float velocity1,
            //     out float effort1);
            message.name[0] = "jaw";
            message.position[0] = jaw_state;
            // print();
            message.velocity[0] = 0;
            message.effort[0] = 0;
            
        }


    }
}
