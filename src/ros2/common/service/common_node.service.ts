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
import { mqtt_request } from '../../../mqtt/type/mqtt_request.type';

/**
 * .service.ts for rclnodejs connection establishment
 * @author wavem-reidlo
 * @version 1.0.0
 * @since 2023/03/31
*/  

/**
 * type JSON for rcl request type
 * @see mqtt_request
 */
const mqtt_rcl_request_type : mqtt_request = {
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
 * @param message_type : any
 * @param topic : string
 * @param mqtt : Mqtt
 */
export async function create_rcl_subscription(node : rclnodejs.Node, message_type : any, topic : string, mqtt : Mqtt) : Promise<void> {
    log.info(`[RCL] creating {${topic}} subscription...`);
    try {
        node.createSubscription(message_type, topic, (message) => {
            const is_rcl_message_empty : boolean = (message === null || message === '' || message === undefined);
            try {
                if(is_rcl_message_empty) {
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
        log.error(`[RCL] create_rcl_subscription ${error}`);
    };
};

/**
 * function for create ROS2 Publisher
 * @see rclnodejs
 * @see rclnodejs.Publisher<any>
 * @see Mqtt
 * @param node : rclnodejs.Node
 * @param message_type : any
 * @param topic : string
 * @param mqtt : Mqtt
 */
export async function create_rcl_publisher(node : rclnodejs.Node, message_type : any, topic : string, mqtt : Mqtt) : Promise<void> {
    log.info(`[RCL] creating {${topic}} publisher...`);
    try {
        const rcl_publisher = node.createPublisher(message_type, topic);
        mqtt.subscribe(topic);
        mqtt.client.on('message', (mqttTopic : string, mqttMessage : any) => {
            try {
                const is_topic_equals : boolean = (topic === mqttTopic);
                                                
                if(is_topic_equals) {
                    const parsed_mqtt : any = parse_mqtt_message_to_json(mqttMessage.toString());
                    const is_message_publisher_type : boolean = (parsed_mqtt.mode === mqtt_rcl_request_type.pub);

                    if(is_message_publisher_type) {
                        log.info(`[RCL] publihser topic ${topic}, mqttTopic : ${mqttTopic}`);
                        log.info(`[RCL] {${topic}} publishing data : ${JSON.stringify(parsed_mqtt.data)}`);
                        rcl_publisher.publish(parsed_mqtt.data);
                    } else {
                        return;
                    };
                } else return;
            } catch (error : any) {
                log.error(`RCL creating {${topic}} publisher : ${error}`);
            };            
        });
    } catch (error : any) {
        log.error(`[RCL] create_rcl_publisher ${error}`);
    };
};

/**
 * async function for create ROS2 action client
 * @see rclnodejs.Node
 * @see rclnodejs.ActionClient<any>
 * @param node : rclnodejs.Node
 * @param message_type : any
 * @param action : string
 * @returns Promise<rclnodejs.ActionClient<any>
 */
export async function create_rcl_action_client(node : rclnodejs.Node, message_type : any, action : string) : Promise<rclnodejs.ActionClient<any> | undefined> {
    log.info(`[RCL] creating action client for {${action}} with type [${message_type}]...`);
    let rcl_action_client;
    try {
        rcl_action_client = new rclnodejs.ActionClient(node, message_type, action);
        log.info(`[RCL] action client for {${action}} created`);
    } catch (error : any) {
        log.error(`[RCL] create_rcl_action_client ${error}`);
    };
    return rcl_action_client;
};

/**
 * function for ROS2 request action server with action client
 * @see rclnodejs.ActionClient<any>
 * @see Mqtt
 * @param rcl_action_client : rclnodejs.ActionClient<any>
 * @param goal : any
 * @param topic : string
 * @param mqtt : Mqtt
 */
export function request_to_rcl_action_server(rcl_action_client : rclnodejs.ActionClient<any>, goal : any, topic : string, mqtt : Mqtt) : void {
    log.info(`[RCL] action client {${topic}}, goal : {${goal}}`);
    try {
        const parsed_mqtt_topic : string = topic + '/request';
        mqtt.subscribe(parsed_mqtt_topic);
        mqtt.client.on('message', (mqtt_topic : string, mqtt_message : any) => {
            const is_topic_equals : boolean = (parsed_mqtt_topic === mqtt_topic);
                        
            if(is_topic_equals) {
                log.info(`[RCL] action client topic ${parsed_mqtt_topic}, mqttTopic : ${mqtt_topic}`);
                const parsed_mqtt : any = parse_mqtt_message_to_json(mqtt_message.toString());
                const is_message_action_type : boolean = (parsed_mqtt.mode === mqtt_rcl_request_type.action);

                if(is_message_action_type) {
                    rcl_action_client.waitForServer(1000)
                        .then((result : boolean) => {
                            if(!result || !rcl_action_client.isActionServerAvailable()) {
                                log.error(`[RCL] action {${topic}} is not available... check your ROS2 Launch Mode`);
                                return;
                            };
                            try {
                                rcl_action_client.sendGoal(goal, (feedback : any) => {
                                    try {
                                        const feedback_message : string = JSON.stringify(feedback);
                                        // log.info(`[RCL] action server goalHandle feedback : {${feedbackMessage}}`);
                                        mqtt.publish(`${topic}/feedback`, feedback_message);
                                    } catch (error : any) {
                                        log.error(`[RCL] action server feedback ${error}`);
                                    };
                                }).then((response : rclnodejs.ClientGoalHandle<any>) => {
                                    try {
                                        const response_message : string = JSON.stringify(response.getResult());
                                        log.info(`[RCL] action server goalHandle response : {${response_message}}`);
                                        if(response.status === rclnodejs.GoalResponse.ACCEPT) {
                                            mqtt.publish(`${topic}/response`, response_message);
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
                    log.error(`[RCL] {${mqtt_topic}} is not a action type...`);
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
 * @param message_type : any
 * @param rcl_service_name : string
 * @returns Promise<rclnodejs.Client<any>
 */
export async function create_rcl_service_client(node : rclnodejs.Node, message_type : any, rcl_service_name : string) : Promise<rclnodejs.Client<any> | undefined> {
    log.info(`[RCL] creating service client for {${rcl_service_name}} ...`);
    let rcl_service_client;
    try {
        rcl_service_client = node.createClient(message_type, rcl_service_name);
    } catch (error : any) {
        log.error(`[RCL] create service client : ${error}`);
    };
    log.info(`[RCL] service client for {${rcl_service_name}} created`);
    return rcl_service_client;
};

/**
 * function for ROS2 call service with service client
 * @see rclnodejs.Node
 * @see rclnodejs.Client<any>
 * @see rclnodejs.Service
 * @param node : rclnodejs.Node
 * @param request_type : any
 * @param topic : string
 * @param mqtt : Mqtt
 */
export function request_to_rcl_service_server(rcl_client : rclnodejs.Client<any>, request_type : any, topic : string, mqtt : Mqtt) : void {
    log.info(`[RCL] service client {${topic}}, requestType : {${request_type}}`);
    try {
        const request : any = rclnodejs.createMessageObject(request_type);
        const parsed_mqtt_topic : string = topic + '/request';
        mqtt.subscribe(parsed_mqtt_topic);
        mqtt.client.on('message', (mqtt_topic : string, mqtt_message : any) => {
            const is_topic_equals : boolean = (parsed_mqtt_topic === mqtt_topic);
            
            if(is_topic_equals) {
                log.info(`[RCL] service client topic ${parsed_mqtt_topic}, mqttTopic : ${mqtt_topic}, mqttMessage : ${mqtt_message.toString()}`);
                const parsed_mqtt : any = parse_mqtt_message_to_json(mqtt_message.toString());
                const is_message_service_type : boolean = (parsed_mqtt.mode === mqtt_rcl_request_type.service);

                if(is_message_service_type) {
                    const rcl_service_name : string = rcl_client.serviceName;
                    rcl_client.waitForService(1000)
                        .then((result : boolean) => {
                            if(!result || !rcl_client.isServiceServerAvailable()) {
                                log.error(`[RCL] service {${rcl_service_name}} is not available... check your ROS2 Launch Mode`);
                                return;
                            };
                            rcl_client.sendRequest(request, (response : any) => {
                                try {
                                    if(response === null) log.error(`[RCL] call {${rcl_service_name}} service call has empty response...`);
                                    else {
                                        if(rcl_service_name.includes("/map_server/map")) {
                                            const parsed_map_data : Array<number> = Array.from(response.map.data);
                                            const parsed_map_json : any = {
                                                map: {
                                                    header : response.map.header,
                                                    info : response.map.info,
                                                    data : parsed_map_data
                                                }
                                            };
                                            const parsed_result : string = JSON.stringify(parsed_map_json);
                                            mqtt.publish(`${rcl_service_name}/response`, parsed_result);
                                        } else {
                                            const result : string = JSON.stringify(response);
                                            mqtt.publish(`${rcl_service_name}/response`, result);
                                        }
                                    };
                                } catch (error : any) {
                                    log.error(`[RCL] call {${rcl_service_name}} : [${error}]`);
                                };
                            });
                        })
                        .catch((error : any) => {
                            log.error(`[RCL] {${topic}} service call ${error}`);
                            throw new Error(error);
                        });
                } else {
                    log.error(`[RCL] {${mqtt_topic}} is not a service type...`);
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
 * @param mqtt_message : string
 * @returns parsedMQTT : any
 */
function parse_mqtt_message_to_json(mqtt_message : string) : any {
    let parsed_mqtt_json : any = {};
    try {
        parsed_mqtt_json = JSON.parse(mqtt_message);
    } catch (error : any) {
        log.error(`[RCL] MQTT-JSON parser throws ${error}`);
    };
    return parsed_mqtt_json;
};