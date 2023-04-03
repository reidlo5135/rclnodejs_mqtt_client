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

import * as rclnodejs from 'rclnodejs';
import Mqtt from "../../../mqtt/mqtt.infra";
import { log } from "../../common/common_logger.infra";
import { createROSPublisher, publishROS, createROSSubscription, createROSClient, callROSService } from '../../common/common_node.infra';
import { rclType, requestType } from '../../common/type/request.type';
import { IPublishPacket } from 'mqtt';
import { error } from 'console';

/**
 * Class for MQTT publish/subscribe & generate ROS2 publisher/subscriber/action/service
 * @see Mqtt
 */
class MasterClientLaunch {

    constructor() {
        rclnodejs.init()
            .then(() => {
                const master = new rclnodejs.Node('master_mqtt_client_launch');
                const mqtt:Mqtt = new Mqtt();
                this.runSubscriptions(master, mqtt);
                master.spin();
            })
            .catch((err) => {
                log.error(`RCL MasterClientLaunch ${err}`);
            });
    };

    private async runSubscriptions(master: rclnodejs.Node, mqtt: Mqtt): Promise<void> {
        mqtt.subscribe('/test');
        mqtt.subscribe('ros_message_init');

        const reqType: requestType = {
            pub : 'pub',
            sub : 'sub',
            action : 'action',
            service : 'service'
        };

        mqtt.client.on('message', (mqttTopic: string, payload: string, packet: IPublishPacket) => {
            log.info(`RCL Master runSubscription topic : ${mqttTopic}, payload : ${payload}, packet : ${packet.payload}`);
            
            if(packet.topic === 'ros_message_init') {
                try {
                    const json = JSON.parse(payload);

                    for(let raw of json) {
                        log.info(`RCL Master arr : ${JSON.stringify(raw)}`);
                        if(raw.type === reqType.sub)  {
                            createROSSubscription(this.setRclType(reqType.sub, master, raw), mqtt);
                        } else if(raw.type === reqType.pub) {
                            createROSPublisher(this.setRclType(reqType.pub, master, raw), mqtt);
                        } else if(raw.type === reqType.action) {
        
                        } else if(raw.type === reqType.service) {
                            createROSClient(master, raw.message_type, raw.name)
                                .then((client) => {
                                    callROSService(client, raw.request_type, mqttTopic, mqtt);
                                })
                                .catch((error) => {
                                    log.error(`RCL service client ${error}`);
                                });
                        };
                    };
                } catch (error) {
                    log.error(`RCL ros_message_init ${error}`);
                }
            } else return;
        });
    };

    private setRclType(reqType: string, master: rclnodejs.Node, raw: any): rclType {
        const rclType: rclType = {
            rcl: reqType,
            node: master,
            messageType: raw.message_type,
            name: raw.name
        };
        return rclType;
    };
};

async function run() {
    new MasterClientLaunch();
};

function welcome() {
    console.log('  _____   ____   _____ ___    __  __  ____ _______ _______    _____ _      _____ ______ _   _ _______ ');
    console.log(' |  __ \\ / __ \\ / ____|__ \\  |  \\/  |/ __ \\__   __|__   __|  / ____| |    |_   _|  ____| \\ | |__   __|');
    console.log(' | |__) | |  | | (___    ) | | \\  / | |  | | | |     | |    | |    | |      | | | |__  |  \\| |  | |   ');
    console.log(" |  _  /| |  | |\\___ \\  / /  | |\\/| | |  | | | |     | |    | |    | |      | | |  __| | . ` |  | |   ");
    console.log(' | | \\ \\| |__| |____) |/ /_  | |  | | |__| | | |     | |    | |____| |____ _| |_| |____| |\\  |  | |   ');
    console.log(' |_|  \\_\\\\____/|_____/|____| |_|  |_|\\___\\_\\ |_|     |_|     \\_____|______|_____|______|_| \\_|  |_|   ');
    console.log('                                                                                                      ');
    log.info('ROS2-MQTT [MASTER] Client is ready for RCL!!');
};

/**
 * async function for main runtime
 * @returns : Promise<void>
 */
(async function main(): Promise<void> {
    run()
    .then(() => welcome())
    .catch((err) => log.error(`ROS2-MQTT [MASTER] Client has crashed by.. ${err} `));
})().catch((e): void => {
    log.error('ROS2-MQTT [MASTER] error : ', e);
    process.exit(1);
});