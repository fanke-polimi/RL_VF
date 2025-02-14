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
    public class JointState_Pub : UnityPublisher<MessageTypes.Sensor.JointState>
    {
        public List<jointposition> JointStateReaders;
        public string FrameId = "Unity";

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
            int jointStateLength = JointStateReaders.Count;
            message = new MessageTypes.Sensor.JointState
            {
                header = new MessageTypes.Std.Header { frame_id = FrameId },
                name = new string[jointStateLength],
                position = new double[jointStateLength],
                velocity = new double[jointStateLength],
                effort = new double[jointStateLength]
            };
        }

        private void UpdateMessage()
        {
            message.header.Update();
            for (int i = 0; i < JointStateReaders.Count; i++)
                UpdateJointState(i);

            Publish(message);
        }

        private void UpdateJointState(int i)
        {
            if (i==0){
            JointStateReaders[i].Read(
                out message.name[i],
                out float position1,
                out float velocity1,
                out float effort1);

            message.position[i] = -position1;
            message.velocity[i] = velocity1;
            message.effort[i] = effort1;}
            if (i==1){
            JointStateReaders[i].Read(
                out message.name[i],
                out float position2,
                out float velocity2,
                out float effort2);

            message.position[i] = -position2;
            message.velocity[i] = velocity2;
            message.effort[i] = effort2;}
            if (i==2){
            JointStateReaders[i].Read(
                out message.name[i],
                out float position3,
                out float velocity3,
                out float effort3);

            message.position[i] = position3/10f;
            message.velocity[i] = velocity3;
            message.effort[i] = effort3;}
            if (i==3){
            JointStateReaders[i].Read(
                out message.name[i],
                out float position4,
                out float velocity4,
                out float effort4);

            message.position[i] = -position4;
            message.velocity[i] = velocity4;
            message.effort[i] = effort4;}
            if (i==4){
            JointStateReaders[i].Read(
                out message.name[i],
                out float position5,
                out float velocity5,
                out float effort5);

            message.position[i] = -position5;
            message.velocity[i] = velocity5;
            message.effort[i] = effort5;}
            if (i==5){
            JointStateReaders[i].Read(
                out message.name[i],
                out float position6,
                out float velocity6,
                out float effort6);

            message.position[i] = -position6;
            message.velocity[i] = velocity6;
            message.effort[i] = effort6;}
        }


    }
}
