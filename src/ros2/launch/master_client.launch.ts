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

import { IPublishPacket } from 'mqtt';
import * as rclnodejs from 'rclnodejs';
import Mqtt from "../../mqtt/service/mqtt.service";
import { log } from "../common/infra/common_logger.infra";
import { MQTTRequest } from '../../mqtt/type/mqtt_request.type';
import { createROSPublisher, createROSSubscription, createROSServiceClient, requestROSServiceServer, createROSActionClient, requestROSActionServer } from '../common/service/common_node.service';

/**
 * Class for MQTT publish/subscribe & generate ROS2 publisher/subscriber/action/service
 * @author wavem-reidlo
 * @version 1.0.0
 * @since 2023/03/31
*/
class MasterClientLaunch {

    /**
     * constructor for create master rcl node & initialize Mqtt class instance & invoke this runRCL
     * @see rclnodejs.Node
     * @see Mqtt
     * @see runRCL
     */
    constructor() {
        rclnodejs.init()
            .then(() => {
                const master : rclnodejs.Node = new rclnodejs.Node('master_mqtt_client_launch');
                const URL_DOODLE : string = 'tcp://10.223.188.12:1883';
                const mqtt : Mqtt = new Mqtt(URL_DOODLE);
                this.runRCL(master, mqtt);
                master.spin();
            })
            .catch((err) => {
                log.error(`[RCL] MasterClientLaunch ${err}`);
            });
    };

    /**
     * private async function for MQTT init subscribe & filter MQTT payload for run RCL
     * @see rclnodejs.Node
     * @see Mqtt
     * @see MQTTRequest
     * @see IPublishPacket
     * @param master : rclnodejs.Node
     * @param mqtt : MQTT
     */
    private async runRCL(master : rclnodejs.Node, mqtt : Mqtt) : Promise<void> {
        const defaultTopic : string = 'ros_message_init';
        mqtt.subscribe(defaultTopic);

        const reqType : MQTTRequest = {
            pub : 'pub',
            sub : 'sub',
            action : 'goal',
            service : 'call'
        };

        mqtt.client.on('message', (mqttTopic : string, mqttMessage : string, mqttPacket : IPublishPacket) => {    
            if(mqttPacket.topic === 'ros_message_init') {
                log.info(`[RCL] {ros_message_init} init : ${mqttMessage.toString()}`);
                try {
                    const json = JSON.parse(mqttMessage);
                    
                    for(let raw of json) {
                        if(raw.type === reqType.sub)  {
                            createROSSubscription(master, raw.message_type, raw.name, mqtt);
                        } else if(raw.type === reqType.pub) {
                            createROSPublisher(master, raw.message_type, raw.name, mqtt);
                        } else if(raw.type === reqType.action) {
                            createROSActionClient(master, raw.message_type, raw.name)
                                .then((client) => {
                                    requestROSActionServer(client!, raw.name, raw.name, mqtt);
                                })
                                .catch((error) => {
                                    log.error(`[RCL] request action client [${error}]`);
                                });
                        } else if(raw.type === reqType.service) {
                            createROSServiceClient(master, raw.message_type, raw.name)
                                .then((client) => {
                                    requestROSServiceServer(client!, raw.request_type, mqttTopic, mqtt);
                                })
                                .catch((error) => {
                                    log.error(`[RCL] service client [${error}]`);
                                });
                        };
                    };
                } catch (error) {
                    log.error(`[RCL] ros_message_init : [${error}]`);
                };
            } else return;
        });
    };
};

/**
 * async function for invoke MasterClientLaunch class instance
 * @see MasterClientLaunch
 */
async function run() : Promise<void> {
    new MasterClientLaunch();
};

/**
 * function for welcome logging
 */
function welcome() : void {
    console.log('  _____   ____   _____ ___    __  __  ____ _______ _______    _____ _      _____ ______ _   _ _______ ');
    console.log(' |  __ \\ / __ \\ / ____|__ \\  |  \\/  |/ __ \\__   __|__   __|  / ____| |    |_   _|  ____| \\ | |__   __|');
    console.log(' | |__) | |  | | (___    ) | | \\  / | |  | | | |     | |    | |    | |      | | | |__  |  \\| |  | |   ');
    console.log(" |  _  /| |  | |\\___ \\  / /  | |\\/| | |  | | | |     | |    | |    | |      | | |  __| | . ` |  | |   ");
    console.log(' | | \\ \\| |__| |____) |/ /_  | |  | | |__| | | |     | |    | |____| |____ _| |_| |____| |\\  |  | |   ');
    console.log(' |_|  \\_\\\\____/|_____/|____| |_|  |_|\\___\\_\\ |_|     |_|     \\_____|______|_____|______|_| \\_|  |_|   ');
    console.log('                                                                                                      ');
    log.info('[RCL-MASTER] Client is ready for RCL!!');
};

/**
 * async function for main runtime
 * @see run
 * @see welcome
 */
(async function main() : Promise<void> {
    run()
    .then(() => welcome())
    .catch((err) => log.error(`[RCL-MASTER] Client has crashed by.. ${err} `));
})().catch((e): void => {
    log.error(`[RCL-MASTER] Client has crashed by.. ${e}`);
});