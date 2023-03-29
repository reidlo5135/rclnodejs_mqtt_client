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
import { subscribe } from '../../common/common_node.infra';
import { requestType } from '../../common/type/request.type';

/**
 * Class for MQTT publish/subscribe & generate ROS2 publisher/subscriber/action/service
 * @see Mqtt
 */
class MasterClientLaunch {

    private readonly mqtt:Mqtt;

    constructor() {
        this.mqtt = new Mqtt();
        rclnodejs.init()
            .then(() => {
                const master = new rclnodejs.Node('master_mqtt_client_launch');
                this.runSubscriptions(master);
                master.spin();
            })
            .catch((err) => {
                log.error(`RCL MasterClientLaunch ${err}`);
            });
    };

    private runSubscriptions(master: rclnodejs.Node) {
        // this.mqtt.subscribe('JaraTwo/1/request');
        this.mqtt.subscribe('ros_message_init');
        const arr = [
            {
              "messageType":"nav_msgs/msg/OccupancyGrid",
              "name":"/map",
              "type":"sub"
            },
            {
              "messageType":"nav2_msgs/msg/NavigationToGoal",
              "name":"/navigationToGoal",
              "type":"action"
            },
            {
                "messageType":"nav_msgs/msg/Odometry",
                "name":"/odom",
                "type":"sub"
            }
        ];

        const reqType: requestType = {
            pub : 'pub',
            sub : 'sub',
            action : 'action',
            service : 'service'
        };

        this.mqtt.client.on('message', (mqttTopic:string, payload:string) => {
            log.info(`RCL Master runSubscription topic : ${mqttTopic}, payload : ${payload}`);
            const json = JSON.parse(payload);            
            for(let raw of json) {
                log.info(`RCL Master arr : ${JSON.stringify(raw)}`);
                if(raw.type === reqType.sub)  {
                    subscribe(master, raw.message_type, raw.name, this.mqtt)
                        .then(() => {
                            log.info(`RCL ${raw.type} is subscribing on ${raw.name}`);
                        })
                        .catch((error) => {
                            log.error(`RCL subscription ${error}`);
                        });
                } else if(raw.type === reqType.pub) {

                } else if(raw.type === reqType.action) {

                } else if(raw.type === reqType.service) {

                }
            };
        });
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
    log.info('ROS2-MQTT [ROS-ONE-READY] Client is ready for RCL!!');
};

/**
 * async function for main runtime
 * @returns : Promise<void>
 */
(async function main(): Promise<void> {
    run()
    .then(() => welcome())
    .catch((err) => log.error(`ROS2-MQTT [ROS-ONE-READY] Client has crashed by.. ${err} `));
})().catch((e): void => {
    log.error('overall_ros_one_ready error : ', e);
    process.exitCode = 1
});