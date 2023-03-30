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
import { log } from './common_logger.infra';

/**
 * function for create ROS2 Publisher
 * @see rclnodejs
 * @see rclnodejs.Publisher<any>
 * @param node : rclnodejs.Node
 * @param type : any
 * @param topic : string
 * @returns rclnodejs.Publisher<any>
 */
export function initPublish(node:rclnodejs.Node, type:any, topic:string) : rclnodejs.Publisher<any> {
    log.info(`RCL init publish message type : ${type}, topic : ${topic}`);
    return node.createPublisher(type, topic);
};

/**
 * function for ROS2 publish after MQTT subscribe
 * @see rclnodejs.Publisher<any>
 * @see Mqtt
 * @param topic : string
 * @param publisher : rclnodejs.Publisher<any>
 * @param msg : any
 * @param mqtt : Mqtt
 */
export function publish(topic:string, publisher:rclnodejs.Publisher<any>, msg:any, mqtt:Mqtt)  {
    log.info(`RCL publish MQTT topic : ${topic}`);
    mqtt.client.subscribe(topic, function(err, granted) {
        if (err) {
            log.error(`RCL publish subscribe Error Occurred Caused By ${err}`);
            return;
        };
    });
};

/**
 * function for ROS2 subscription & MQTT publishing
 * @see rclnodejs.Node
 * @see rclnodejs.Subscription
 * @see Mqtt
 * @param node : rclnodejs.Node
 * @param type : any
 * @param topic : string
 * @param mqtt : Mqtt
 * @returns rclnodejs.Subscription
 */
export async function subscribe(node:rclnodejs.Node, type:any, topic:string, mqtt:Mqtt) : Promise<void> {
    log.info(`RCL subscription message type : ${type}, topic : ${topic}, mqtt : ${mqtt.url}`);

    node.createSubscription(type, topic, (msg) => {
        if(msg === null || msg === '') log.error(`RCL ${topic} subscription has return empty message`);
        mqtt.publish(`wavem/1${topic}`, JSON.stringify(msg));
    });
};

/**
 * function for ROS2 client
 * @see rclnodejs.Node
 * @see rclnodejs.Client<any>
 * @see rclnodejs.Service
 * @param node : rclnodejs.Node
 * @param msg_type : any
 * @param service : string
 * @returns rclnodejs.Client<any>
 */
export function client(node:rclnodejs.Node, msg_type:any, service:string) : rclnodejs.Client<any> {
    log.info(`RCL client msg_type : ${msg_type}, service : ${service}`);
    return node.createClient(msg_type, service);
};

/**
 * function for ROS2 action client
 * @see rclnodejs.Node
 * @see rclnodejs.ActionClient<any>
 * @param node : rclnodejs.Node
 * @param msg_type : any
 * @param action : string
 * @returns rclnodejs.ActionClient<any>
 */
export function actionClient(node:rclnodejs.Node, msg_type:any, action:string) : rclnodejs.ActionClient<any> {
    log.info(`RCL action client msg_type : ${msg_type}, action : ${action}`);
    return new rclnodejs.ActionClient(node, msg_type, action);
};