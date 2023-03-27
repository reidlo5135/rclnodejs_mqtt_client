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
import { log } from '../common/common_logger.infra';
import { Publisher } from '../common/common_ndoe.interface';
import { initPublish } from '../common/common_node.infra';

/**
 * Class for ROS2 publish /initialpose topic to robot
 * @see rclnodejs.Publisher
 */

export default class InitialPosePublisher implements Publisher {

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
     * private readonly field for Mqtt instance
     */
    private readonly mqtt: Mqtt;

    /**
     * constructor for initialize field instances
     * @see node
     * @see Mqtt
     * @param topic : string
     * @param mqtt : Mqtt
     */
    constructor(private readonly topic:string, mqtt:Mqtt) {
        this.node = new rclnodejs.Node('initialpose_publisher');
        this.mqtt = mqtt;
    };

    /**
     * void function for MQTT subscription & ROS2 publish by recieved MQTT message
     * @see Mqtt
     * @see publisher
     * @see initPublish
     */
    start(): void {
        if (this.isRunning) return;
    
        this.isRunning = true;

        const publisher = initPublish(this.node, 'geometry_msgs/msg/PoseWithCovarianceStamped', this.topic);
        this.node.spin();

        this.mqtt.subscribe('wavem/1/initialpose');
        this.mqtt.client.on("message", (topic, message) => {
            try {
                log.info(`RCL initialpose publish MQTT onMessage topic : ${topic}, message : ${message}`);
                if(topic.includes('initialpose')) {
                    let msg = this.genPWCSMsg();
                    publisher.publish(msg);
                } else return;
            } catch (error) {
                log.error(`RCL publish initialpose error : ${error}`);
            }
        });
    };

    /**
     * void function for stop node spinning & MQTT unsubscribe
     */
    stop(): void {
        this.isRunning = false;
        this.mqtt.client.unsubscribe('wavem/1/cmd_vel');
        this.node.destroy();
    };

    /**
     * protected function for generate ROS2 publishing message object
     * @see rclnodejs.geemetry_msgs.msg.PoseWithCovarianceStamped
     * @param message : any
     * @returns pwscMsg : rclnodejs.geometry_msgs.msg.PoseWithCovarianceStamped
     */
    protected genPWCSMsg(): rclnodejs.geometry_msgs.msg.PoseWithCovarianceStamped {
        let pwscMsg = rclnodejs.createMessageObject('geometry_msgs/msg/PoseWithCovarianceStamped') as rclnodejs.geometry_msgs.msg.PoseWithCovarianceStamped;

        // const pwscs = JSON.parse(message);
        pwscMsg.header.frame_id = 'initial_pose';
        pwscMsg.pose.pose.position = {x:0, y:0, z:0};
        pwscMsg.pose.pose.orientation = {x:0,y:0,z:0,w:0};
        pwscMsg.pose.covariance = [1,2,3];

        return pwscMsg;
    };
}