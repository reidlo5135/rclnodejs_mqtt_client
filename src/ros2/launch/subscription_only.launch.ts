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
import Mqtt from "../../mqtt/service/mqtt.service";
import { log } from "../common/infra/common_logger.infra";
import { createROSSubscription } from '../common/service/common_node.service';

/**
 * Class for MQTT publish/subscribe & generate ROS2 publisher/subscriber/action/service
 * @author wavem-reidlo
 * @version 1.0.0
 * @since 2023/04/04
*/
class SubscriptionOnlyLaunch {

    /**
     * constructor for create master rcl node & initialize Mqtt class instance & invoke this runRCL
     * @see rclnodejs.Node
     * @see Mqtt
     * @see runRCLSubscription
     */
    constructor() {
        rclnodejs.init()
            .then(() => {
                const master : rclnodejs.Node = new rclnodejs.Node('subscription_only_launch');
                const URL_REIDLO_LINUX : string = 'tcp://192.168.0.187:1883';
                const mqtt : Mqtt = new Mqtt(URL_REIDLO_LINUX);
                this.runRCLSubscription(master, mqtt);
                master.spin();
            })
            .catch((err) => {
                log.error(`RCL MasterClientLaunch ${err}`);
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
    private async runRCLSubscription(master : rclnodejs.Node, mqtt : Mqtt) : Promise<void> {
        const target : Array<any> = [
            {
                messageType: 'nav_msgs/msg/Odometry',
                topic: "/odom"
            },
            {
                messageType: 'sensor_msgs/msg/Imu',
                topic: "/imu/data"
            },
            {
                messageType: 'geometry_msgs/msg/Pose',
                topic: "/robot_pose"
            },
            {
                messageType: 'tf2_msgs/msg/TFMessage',
                topic: '/tf'
            }
        ]
        for(let raw of target) {
            createROSSubscription(master, raw.messageType, raw.topic, mqtt);
        };
    };
};

/**
 * async function for invoke MasterClientLaunch class instance
 * @see MasterClientLaunch
 */
async function run() : Promise<void> {
    new SubscriptionOnlyLaunch();
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
    log.info('ROS2-MQTT [SubscriptionOnly] Client is ready for RCL!!');
};

/**
 * async function for main runtime
 * @see run
 * @see welcome
 */
(async function main() : Promise<void> {
    run()
    .then(() => welcome())
    .catch((err) => log.error(`ROS2-MQTT [SubscriptionOnly] Client has crashed by.. ${err} `));
})().catch((e): void => {
    log.error(`ROS2-MQTT [SubscriptionOnly] Client has crashed by.. ${e}`);
    process.exit(1);
});