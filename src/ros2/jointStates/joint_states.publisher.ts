// Copyright [2023] [wavem-reidlo]
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

'strict mode';

import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
import { Publisher } from '../common/common_ndoe.interface';
import { initPublish, publish } from '../common/common_node.infra';

/**
 * Class for ROS2 publish /joint_states topic to robot
 * @see rclnodejs.Publisher
 */
export default class JointStatesPublisher implements Publisher {
  
    /**
     * private boolean filed for this node is running or not
     */
    private isRunning = false;

    /**
     * private readonly field for rclnodejs.Node instance
     * @see rclnodejs.Node
     */
    private readonly node: rclnodejs.Node;

    /**
     * private readonly field for Mqtt
     * @see Mqtt
     */
    private readonly mqtt: Mqtt;

  
  /**
   * constructor for initialize field instances
   * @see node
   * @see publisher
   * @see mqtt
   * @param topic : string
   * @param mqtt : Mqtt
   */
  constructor(private readonly topic:string, mqtt:Mqtt) {
    this.node = new rclnodejs.Node('joint_states_publisher');
    this.mqtt = mqtt;
  };

  /**
   * void function for MQTT subscription & ROS2 publish by recieved MQTT message
   * @see mqtt
   * @see publihser
   * @see initPublish
   */
  start(): void {
    if (this.isRunning) return;

    this.isRunning = true;

    const publisher = initPublish(this.node, 'sensor_msgs/msg/JointState', this.topic);
    this.node.spin();
    let msg = this.genJointStatesMsg();
    // publish('wavem/1/joint_states', publisher, msg, this.mqtt);
  };

  /**
   * void function for stop ndoe spinning & MQTT unsubscribe
   */
  stop(): void {
    this.isRunning = false;
    this.mqtt.client.unsubscribe('wavem/1/joint_states');
    this.node.destroy();
  };

  /**
   * protected function for generate ROS2 publishing message object
   * @see rclnodejs.sensor_msgs.msg.JointState
   * @returns jointSateMsg : rclnodejs.sensor_msgs.msg.JointState
   */
  protected genJointStatesMsg(): rclnodejs.sensor_msgs.msg.JointState {
    let jointStateMsg = rclnodejs.createMessageObject('sensor_msgs/msg/JointState') as rclnodejs.sensor_msgs.msg.JointState;

    jointStateMsg.header.frame_id = 'joint_states';
    jointStateMsg.header.stamp = this.node.now().toMsg();

    return jointStateMsg;
  };
};