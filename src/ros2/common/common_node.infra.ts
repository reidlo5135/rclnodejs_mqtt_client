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
import { rclType, requestType } from './type/request.type';

const reqType: requestType = {
    pub : 'pub',
    sub : 'sub',
    action : 'action',
    service : 'service'
};

/**
 * function for create ROS2 Publisher
 * @see rclnodejs
 * @see rclnodejs.Publisher<any>
 * @param node : rclnodejs.Node
 * @param rosMessageType : any
 * @param topic : string
 * @returns rclnodejs.Publisher<any>
 */
export async function createROSPublisher(rclType: rclType, mqtt: Mqtt): Promise<void> {
    log.info(`RCL createROSPublisher nodeType : ${JSON.stringify(rclType)}`);
    const node = rclType.node;
    const topic = rclType.name;
    const rosPublisher = node.createPublisher(rclType.messageType, topic);
    
    try {
        if(rclType.rcl === reqType.pub) {
            mqtt.subscribe(topic);
            mqtt.client.on('message', (mqttTopic, mqttMessage) => {
                log.info(`RCL createROSPublisher MQTT onMessage topic : ${mqttTopic}, message : ${mqttMessage}`);
                rosPublisher.publish(mqttMessage);
            });
        } else return;
    } catch (error) {
        log.error(`RCL createROSPublisher ${error}`);
    }
};

/**
 * function for ROS2 publish after MQTT subscribe
 * @see rclnodejs.Publisher<any>
 * @see Mqtt
 * @param topic : string
 * @param rosPublisher : rclnodejs.Publisher<any>
 * @param msg : any
 * @param mqtt : Mqtt
 */
export async function publishROS(rosPublisher: rclnodejs.Publisher<any>, topic: string, mqtt: Mqtt): Promise<void> {
    log.info(`RCL publish mqttTopic : ${topic}`);
    mqtt.subscribeForROSPublisher(topic, rosPublisher);
};

/**
 * function for ROS2 subscription & MQTT publishing
 * @see rclnodejs.Node
 * @see rclnodejs.Subscription
 * @see Mqtt
 * @param node : rclnodejs.Node
 * @param rosMessageType : any
 * @param topic : string
 * @param mqtt : Mqtt
 * @returns rclnodejs.Subscription
 */
export async function createROSSubscription(rclType: rclType, mqtt: Mqtt) : Promise<void> {
    log.info(`RCL subscription nodeType : ${JSON.stringify(rclType)}`);
    const node = rclType.node;
    const topic = rclType.name;

    try {
        if(rclType.rcl === reqType.sub) {
            node.createSubscription(rclType.messageType, topic, (message) => {
                if(message === null || message === '') log.error(`RCL ${topic} subscription has return empty message`);
                mqtt.publish(`${topic}`, JSON.stringify(message));
            }); 
        } else return;
    } catch (error) {
        log.error(`RCL createROSSubscription ${error}`);
    }
};

/**
 * function for ROS2 client
 * @see rclnodejs.Node
 * @see rclnodejs.Client<any>
 * @see rclnodejs.Service
 * @param node : rclnodejs.Node
 * @param rosMessageType : any
 * @param ros_service : string
 * @returns rclnodejs.Client<any>
 */
export async function createROSClient(node: rclnodejs.Node, rosMessageType:any, ros_service:string) : Promise<rclnodejs.Client<any>> {
    log.info(`RCL client msg_type : ${rosMessageType}, service : ${ros_service}`);
    return node.createClient(rosMessageType, ros_service);
};

export async function callROSService(rosClient: rclnodejs.Client<any>, requestType: any, mqttTopic: string, mqtt: Mqtt): Promise<void> {
    const request = rclnodejs.createMessageObject(requestType);
    mqtt.subscribe(mqttTopic);
    mqtt.client.on('message', (topic, message) => {
        const service_name = rosClient.serviceName;
        log.info(`RCL callROSService MQTT onMessage topic : ${topic}, message : ${message}`);
        if(mqttTopic.includes(topic)) {
            rosClient.waitForService(1000)
                .then((result) => {
                    if(!result) {
                        log.error(`RCL ${service_name} is not available... check your ROS2 Launch Mode`);
                        return;
                    };
                    rosClient.sendRequest(request, (response) => {
                        if(response === null) log.error(`RCL call ${service_name} service call has empty response `);
                        else {
                            log.info(`RCL call ${service_name} response : ${JSON.stringify(response)}`);
                            mqtt.publish(`${service_name}`, response);
                        }
                    });
                })
                .catch((error) => {
                    log.error(`RCL ${mqttTopic} service call ${error}`);
                });
        };
    });
};

/**
 * function for ROS2 action client
 * @see rclnodejs.Node
 * @see rclnodejs.ActionClient<any>
 * @param node : rclnodejs.Node
 * @param messageType : any
 * @param action : string
 * @returns rclnodejs.ActionClient<any>
 */
export async function creatROSActionClient(node:rclnodejs.Node, messageType:any, action:string) : Promise<rclnodejs.ActionClient<any>> {
    log.info(`RCL action client msg_type : ${messageType}, action : ${action}`);
    return new rclnodejs.ActionClient(node, messageType, action);
};