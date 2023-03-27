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
 * Class for ROS2 publish /transformed_global_plan topic to robot
 * @see rclnodejs.Publisher
 */
export default class TransformedGlobalPlanPublisher implements Publisher {
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
     * @see mqtt
     * @param topic : string
     * @param mqtt : Mqtt
     */
    constructor(private readonly topic:string, mqtt:Mqtt) {
        this.node = new rclnodejs.Node('local_plan_publisher');
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
        
        const publisher = initPublish(this.node, 'nav_msgs/msg/Path', this.topic);
        this.node.spin();

        this.mqtt.subscribe('wavem/1/transformed_global_plan');
        this.mqtt.client.on("message", (topic, message) => {
            try {
                log.info(`RCL transformed_global_plan publish MQTT onMessage topic : ${topic}, message : ${message}`);
                if(topic.includes('transformed_global_plan')) {
                    let msg = this.genPathMsg(message.toString());
                    publisher.publish(msg);
                } else return;
            } catch (error) {
                log.error(`RCL publish transformed_global_plan error : ${error}`);
            }
        });
    };

    /**
     * void function for stop node spinning & MQTT unsubscribe
     */
    stop(): void {
        this.isRunning = false;
        this.mqtt.client.unsubscribe('wavem/1/transformed_global_plan');
        this.node.destroy();
    };

    /**
     * protected function for generate ROS2 publishing message object
     * @see rclnodejs.nav_msgs.msg.Path
     * @param message : any
     * @returns pathMsg : rclnodejs.nav_msgs.msg.Path
     */
    protected genPathMsg(message: any): rclnodejs.nav_msgs.msg.Path {
        let pathMsg = rclnodejs.createMessageObject('nav_msgs/msg/Path') as rclnodejs.nav_msgs.msg.Path;

        const path = JSON.parse(message);

        pathMsg.header.frame_id = path.frame_id;
        pathMsg.poses = path.poses;

        return pathMsg;
    };
};