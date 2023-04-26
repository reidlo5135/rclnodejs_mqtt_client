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
import { log } from '../infra/common_logger.infra';
import Mqtt from '../../../mqtt/service/mqtt.service';
import { MQTTRequest } from '../../../mqtt/type/mqtt_request.type';

/**
 * .service.ts for rclnodejs connection establishment
 * @author wavem-reidlo
 * @version 1.0.0
 * @since 2023/03/31
*/  

/**
 * type JSON for rcl request type
 * @see MQTTRequest
 */
const rclType : MQTTRequest = {
    pub : 'pub',
    sub : 'sub',
    action : 'goal',
    service : 'call'
};

/**
 * function for ROS2 subscription & MQTT publishing
 * @see rclnodejs.Node
 * @see rclnodejs.Subscription
 * @see Mqtt
 * @param node : rclnodejs.Node
 * @param messageType : any
 * @param topic : string
 * @param mqtt : Mqtt
 */
export async function createROSSubscription(node : rclnodejs.Node, messageType : any, topic : string, mqtt : Mqtt) : Promise<void> {
    log.info(`[RCL] creating {${topic}} subscription...`);
    try {
        node.createSubscription(messageType, topic, (message) => {
            const isEmpty : boolean = (message === null || message === '' || message === undefined);
            try {
                if(isEmpty) {
                    const error : string = `[RCL] {${topic}} subscription has return empty message...`;
                    log.error(error);
                    throw new Error(error);
                } else {
                    mqtt.publish(topic, JSON.stringify(message));
                };
            } catch (error : any) {
                log.error(`[RCL] creating {${topic}} subscription : ${error}`);
            };
        }); 
    } catch (error : any) {
        log.error(`[RCL] createROSSubscription ${error}`);
    };
};

/**
 * function for create ROS2 Publisher
 * @see rclnodejs
 * @see rclnodejs.Publisher<any>
 * @see Mqtt
 * @param node : rclnodejs.Node
 * @param messageType : any
 * @param topic : string
 * @param mqtt : Mqtt
 */
export async function createROSPublisher(node : rclnodejs.Node, messageType : any, topic : string, mqtt : Mqtt) : Promise<void> {
    // log.info(`[RCL] creating {${topic}} publisher...`);
    try {
        const rosPublisher = node.createPublisher(messageType, topic);
        
        mqtt.subscribe(topic);
        mqtt.client.on('message', (mqttTopic : string, mqttMessage : any) => {
            try {
                const isTopicEqual : boolean = (topic === mqttTopic);
                                                
                if(isTopicEqual) {
                    const parsedMQTT : any = parseMQTT(mqttMessage.toString());
                    const isPublisher : boolean = (parsedMQTT.mode === rclType.pub);

                    if(isPublisher) {
                        log.info(`[RCL] publihser topic ${topic}, mqttTopic : ${mqttTopic}`);
                        log.info(`[RCL] {${topic}} publishing data : ${JSON.stringify(parsedMQTT.data)}`);
                        rosPublisher.publish(parsedMQTT.data);
                    } else {
                        return;
                    };
                } else return;
            } catch (error : any) {
                log.error(`RCL creating {${topic}} publisher : ${error}`);
            };            
        });
    } catch (error : any) {
        log.error(`RCL createROSPublisher ${error}`);
    };
};

/**
 * async function for create ROS2 action client
 * @see rclnodejs.Node
 * @see rclnodejs.ActionClient<any>
 * @param node : rclnodejs.Node
 * @param messageType : any
 * @param action : string
 * @returns Promise<rclnodejs.ActionClient<any>
 */
export async function createROSActionClient(node : rclnodejs.Node, messageType : any, action : string) : Promise<rclnodejs.ActionClient<any> | undefined> {
    log.info(`[RCL] creating action client for {${action}} with type [${messageType}]...`);
    let actionClient;
    try {
        actionClient = new rclnodejs.ActionClient(node, messageType, action);
        log.info(`[RCL] action client for {${action}} created`);
    } catch (error : any) {
        log.error(`[RCL] createROSActionClient ${error}`);
    };
    return actionClient;
};

/**
 * function for ROS2 request action server with action client
 * @see rclnodejs.ActionClient<any>
 * @see Mqtt
 * @param rosActionClient : rclnodejs.ActionClient<any>
 * @param goal : any
 * @param topic : string
 * @param mqtt : Mqtt
 */
export function requestROSActionServer(rosActionClient : rclnodejs.ActionClient<any>, goal : any, topic : string, mqtt : Mqtt) : void {
    log.info(`[RCL] action client {${topic}}, goal : {${goal}}`);
    const parsedMQTTTopic : string = topic + '/request';

    mqtt.subscribe(parsedMQTTTopic);
    try {
        mqtt.client.on('message', (mqttTopic : string, mqttMessage : any) => {
            const isTopicEqual : boolean = (parsedMQTTTopic === mqttTopic);
                        
            if(isTopicEqual) {
                log.info(`[RCL] action client topic ${parsedMQTTTopic}, mqttTopic : ${mqttTopic}`);
                const parsedMQTT : any = parseMQTT(mqttMessage.toString());
                const isAction : boolean = (parsedMQTT.mode === rclType.action);

                if(isAction) {
                    rosActionClient.waitForServer(1000)
                        .then((result : boolean) => {
                            if(!result || !rosActionClient.isActionServerAvailable()) {
                                log.error(`[RCL] action {${topic}} is not available... check your ROS2 Launch Mode`);
                                return;
                            };
                            try {
                                rosActionClient.sendGoal(goal, (feedback : any) => {
                                    try {
                                        const feedbackMessage : string = JSON.stringify(feedback);
                                        // log.info(`[RCL] action server goalHandle feedback : {${feedbackMessage}}`);
                                        mqtt.publish(`${topic}/feedback`, feedbackMessage);
                                    } catch (error : any) {
                                        log.error(`[RCL] action server feedback ${error}`);
                                    };
                                }).then((response : rclnodejs.ClientGoalHandle<any>) => {
                                    try {
                                        const responseMessage : string = JSON.stringify(response.getResult());
                                        log.info(`[RCL] action server goalHandle response : {${responseMessage}}`);
                                        if(response.status === rclnodejs.GoalResponse.ACCEPT) {
                                            mqtt.publish(`${topic}/response`, responseMessage);
                                        };
                                    } catch (error : any) {
                                        log.error(`[RCL] action server response ${error}`);
                                    };
                                }).catch((error : any) => {
                                    log.error(`[RCL] action server communication error : ${error}`);
                                });
                            } catch (error : any) {
                                log.error(`[RCL] action sendGoal ${error}`);
                            };
                        })
                        .catch((error : any) => {
                            log.error(`[RCL] action server error : ${error}`);
                            throw new Error(error);
                        });
                } else {
                    log.error(`[RCL] {${mqttTopic}} is not a action type...`);
                    return;
                };               
            } else return;
        });
    } catch (error : any) {
        log.error(`[RCL] action client error : ${error}`);
    };
};

/**
 * async function for ROS2 create service client
 * @see rclnodejs.Node
 * @see rclnodejs.Client<any>
 * @see rclnodejs.Service
 * @param node : rclnodejs.Node
 * @param messageType : any
 * @param rosService : string
 * @returns Promise<rclnodejs.Client<any>
 */
export async function createROSServiceClient(node : rclnodejs.Node, messageType : any, rosService : string) : Promise<rclnodejs.Client<any> | undefined> {
    log.info(`[RCL] creating service client for {${rosService}} ...`);
    let serviceClient;
    try {
        serviceClient = node.createClient(messageType, rosService);
    } catch (error : any) {
        log.error(`[RCL] create service client : ${error}`);
    };
    log.info(`[RCL] service client for {${rosService}} created`);
    return serviceClient;
};

/**
 * function for ROS2 call service with service client
 * @see rclnodejs.Node
 * @see rclnodejs.Client<any>
 * @see rclnodejs.Service
 * @param node : rclnodejs.Node
 * @param requestType : any
 * @param topic : string
 * @param mqtt : Mqtt
 */
export function requestROSServiceServer(rosClient : rclnodejs.Client<any>, requestType : any, topic : string, mqtt : Mqtt) : void {
    log.info(`[RCL] service client {${topic}}, requestType : {${requestType}}`);
    const request : any = rclnodejs.createMessageObject(requestType);
    const parsedMQTTTopic : string = topic + '/request';

    mqtt.subscribe(parsedMQTTTopic);
    try {
        mqtt.client.on('message', (mqttTopic : string, mqttMessage : any) => {
            const isTopicEqual : boolean = (parsedMQTTTopic === mqttTopic);
            
            if(isTopicEqual) {
                log.info(`[RCL] service client topic ${parsedMQTTTopic}, mqttTopic : ${mqttTopic}, mqttMessage : ${mqttMessage.toString()}`);
                const parsedMQTT : any = parseMQTT(mqttMessage.toString());
                const isService : boolean = (parsedMQTT.mode === rclType.service);

                if(isService) {
                    const rosService : string = rosClient.serviceName;
                    rosClient.waitForService(1000)
                        .then((result : boolean) => {
                            if(!result || !rosClient.isServiceServerAvailable()) {
                                log.error(`[RCL] service {${rosService}} is not available... check your ROS2 Launch Mode`);
                                return;
                            };
                            rosClient.sendRequest(request, (response : any) => {
                                try {
                                    if(response === null) log.error(`[RCL] call {${rosService}} service call has empty response...`);
                                    else {
                                        const result : string = JSON.stringify(response);
                                        mqtt.publish(`${rosService}/response`, result);
                                    };
                                } catch (error : any) {
                                    log.error(`[RCL] call {${rosService}} : [${error}]`);
                                };
                            });
                        })
                        .catch((error : any) => {
                            log.error(`[RCL] {${topic}} service call ${error}`);
                            throw new Error(error);
                        });
                } else {
                    log.error(`[RCL] {${mqttTopic}} is not a service type...`);
                    return;
                };
            } else return;
        });
    } catch (error : any) {
        log.error(`[RCL] {${topic} service call ${error}}`);
    };
};

/**
 * function for parse MQTT message into JSON
 * @see JSON
 * @param mqttMessage : string
 * @returns parsedMQTT : any
 */
function parseMQTT(mqttMessage : string) : any {
    let parsedMQTT : any = {};
    try {
        parsedMQTT = JSON.parse(mqttMessage);
    } catch (error : any) {
        log.error(`[RCL] MQTT-JSON parser throws ${error}`);
    };
    return parsedMQTT;
};