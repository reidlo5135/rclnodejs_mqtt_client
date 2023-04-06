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
    action : 'action',
    service : 'service'
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
export function createROSSubscription(node : rclnodejs.Node, messageType : any, topic : string, mqtt : Mqtt) : void {
    log.info(`[RCL] creating {${topic}} subscription...`);
    try {
        node.createSubscription(messageType, topic, (message) => {
            try {
                if(message === null || message === '') log.error(`RCL ${topic} subscription has return empty message`);
                mqtt.publish(topic, JSON.stringify(message));
            } catch (error) {
                log.error(`RCL {${topic}} error : ${error}`);
            };
        }); 
        log.info(`[RCL] {${topic}} subscription created`);
    } catch (error) {
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
export function createROSPublisher(node : rclnodejs.Node, messageType : any, topic : string, mqtt : Mqtt) : void {
    log.info(`[RCL] creating {${topic}} publisher...`);
    try {
        const rosPublisher = node.createPublisher(messageType, topic);
        log.info(`[RCL] {${topic}} publisher created`);

        mqtt.subscribe(topic);
        mqtt.client.on('message', (mqttTopic, mqttMessage) => {
            try {
                const parsedMQTT = JSON.parse(mqttMessage.toString());
                const isPub = (parsedMQTT.mode === rclType.pub);
                const isTopicEqual = (topic === mqttTopic);

                if(isPub && isTopicEqual) {
                    log.info(`[RCL] {${topic}} publishing data : ${JSON.stringify(parsedMQTT.data)}`);
                    rosPublisher.publish(parsedMQTT.data);
                } else return;
            } catch (error) {
                log.error(`RCL createPublisher : ${error}`);
            };            
        });
    } catch (error) {
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
 * @returns Promise<rclnodejs.ActionClient<any>>
 */
export async function createROSActionClient(node : rclnodejs.Node, messageType : any, action : string) : Promise<rclnodejs.ActionClient<any> | undefined> {
    log.info(`[RCL] creating action client for {${action}} ...`);
    let actionClient;
    try {
        actionClient = new rclnodejs.ActionClient(node, messageType, action);
    } catch (error) {
        log.error(`[RCL] create action client ${error}`);
    };
    log.info(`[RCL] action client for {${action}} created`);
    return actionClient;
};

/**
 * function for ROS2 request action server with action client
 * @see rclnodejs.ActionClient<amy>
 * @see Mqtt
 * @param rosActionClient : rclnodejs.ActionClient<amy>
 * @param goal : any
 * @param topic : string
 * @param mqtt : Mqtt
 */
export function requestROSActionServer(rosActionClient : rclnodejs.ActionClient<any>, goal : any, topic : string, mqtt : Mqtt) : void {
    log.info(`[RCL] action client mqttTopic : {${topic}}, goal : {${goal}}`);
    const parsedMQTTTopic = topic + '/reqeust';

    mqtt.subscribe(parsedMQTTTopic);
    mqtt.client.on('message', (mqttTopic, mqttMessage) => {
        const parsedMQTT = JSON.parse(mqttMessage.toString());
        const isAction = (parsedMQTT.mode === rclType.action);
        const isTopicEqual = (topic === mqttTopic);
        
        log.info(`[RCL] request action server MQTT onMessage topic : {${mqttTopic}}, parsedMQTT : {${JSON.stringify(parsedMQTT)}}`);

        if(isAction && isTopicEqual) {
            rosActionClient.waitForServer(1000)
                .then((result) => {
                    if(!result || !rosActionClient.isActionServerAvailable()) {
                        log.error(`[RCL] action server is not available...`);
                        return;
                    };
                    rosActionClient.sendGoal(goal, (feedback) => {
                        const feedbackMessage = JSON.stringify(feedback);
                        log.info(`[RCL] action client goalHandle feedback : {${feedbackMessage}}`);
                        mqtt.publish(`${mqttTopic}/feedback`, feedbackMessage);
                    }).then((response) => {
                        const responseMessage = JSON.stringify(response);
                        log.info(`[RCL] action client goalHandle response : {${responseMessage}}`);
                        if(response.status === rclnodejs.GoalResponse.ACCEPT) {
                            mqtt.publish(`${mqttTopic}/response`, responseMessage);
                        };
                    }).catch((error) => {
                        log.error(`[RCL] action client communication error : ${error}`);
                    });
                })
                .catch((error) => {
                    log.error(`[RCL] action client error : ${error}`);
                });
        };
    });
};

/**
 * async function for ROS2 create service client
 * @see rclnodejs.Node
 * @see rclnodejs.Client<any>
 * @see rclnodejs.Service
 * @param node : rclnodejs.Node
 * @param messageType : any
 * @param rosService : string
 * @returns Promise<rclnodejs.Client<any>>
 */
export async function createROSServiceClient(node : rclnodejs.Node, messageType : any, rosService : string) : Promise<rclnodejs.Client<any> | undefined> {
    log.info(`[RCL] creating service client for {${rosService}} ...`);
    let serviceClient;
    try {
        serviceClient = node.createClient(messageType, rosService);
    } catch (error) {
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
    const request = rclnodejs.createMessageObject(requestType);
    mqtt.subscribe(topic);
    mqtt.client.on('message', (mqttTopic, mqttMessage) => {
        const parsedMQTT = JSON.parse(mqttMessage.toString());
        const isService = (parsedMQTT.mode === rclType.service);
        const isTopicEqual = (topic === mqttTopic);

        log.info(`[RCL] requestROSServiceServer MQTT onMessage topic : {${mqttTopic}}, parsedMQTT : {${JSON.stringify(parsedMQTT)}}`);

        if(isService && isTopicEqual) {
            const rosService = rosClient.serviceName;
            rosClient.waitForService(1000)
                .then((result) => {
                    if(!result || !rosClient.isServiceServerAvailable()) {
                        log.error(`[RCL] {${rosService}} is not available... check your ROS2 Launch Mode`);
                        return;
                    };
                    rosClient.sendRequest(request, (response) => {
                        if(response === null) log.error(`[RCL] call {${rosService}} service call has empty response `);
                        else {
                            log.info(`[RCL] call {${rosService}} response : {${JSON.stringify(response)}}`);
                            mqtt.publish(`${rosService}`, response);
                        }
                    });
                })
                .catch((error) => {
                    log.error(`[RCL] {${topic}} service call ${error}`);
                });
        };
    });
};