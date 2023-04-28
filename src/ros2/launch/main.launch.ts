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
import { mqtt_request } from '../../mqtt/type/mqtt_request.type';
import { create_rcl_publisher, create_rcl_subscription, create_rcl_service_client, request_to_rcl_service_server, create_rcl_action_client, request_to_rcl_action_server } from '../common/service/common_node.service';

/**
 * Class for MQTT publish/subscribe & generate ROS2 publisher/subscriber/action/service
 * @author wavem-reidlo
 * @version 1.0.0
 * @since 2023/03/31
*/
class MainLaunch {

    /**
     * constructor for create master rcl node & initialize Mqtt class instance & invoke this runRCL
     * @see rclnodejs.Node
     * @see Mqtt
     * @see runRCL
     */
    constructor() {
        rclnodejs.init()
            .then(() => {
                const master : rclnodejs.Node = new rclnodejs.Node('rclnodejs_mqtt_bridge');
                if(master.spinning) {
                    master.destroy();
                };
                const MQTT_BROKER_URL : string = 'tcp://localhost:1883';
                const mqtt : Mqtt = new Mqtt(MQTT_BROKER_URL);
                this.runRCL(master, mqtt);
                master.spin();
            })
            .catch((err) => {
                log.error(`[RCL] MainLaunch ${err}`);
            });
    };

    /**
     * private async function for MQTT init subscribe & filter MQTT payload for run RCL
     * @see rclnodejs.Node
     * @see Mqtt
     * @see mqtt_request
     * @see IPublishPacket
     * @param master : rclnodejs.Node
     * @param mqtt : MQTT
     */
    private runRCL(master : rclnodejs.Node, mqtt : Mqtt) : void {
        const default_topic : string = 'ros_message_init';
        try {
            mqtt.unsubscribe_after_connection_check(default_topic);
            mqtt.subscribe(default_topic);
        } catch (error : any) {
            log.error(`[MQTT] default topic : ${error}`);
        };

        const mqtt_request_type : mqtt_request = {
            pub : 'pub',
            sub : 'sub',
            action : 'goal',
            service : 'call'
        };

        mqtt.client.on('message', (mqtt_topic : string, mqtt_message : string, mqtt_packet : IPublishPacket) : void => {
            const is_mqtt_message_init : boolean = (mqtt_packet.topic === default_topic);
            if(is_mqtt_message_init) {
                try {
                    const json = JSON.parse(mqtt_message);
                    
                    for(let raw of json) {
                        if(raw.type === mqtt_request_type.sub)  {
                            create_rcl_subscription(master, raw.message_type, raw.name, mqtt)
                                .then(() => {
                                    log.info(`[RCL] {${raw.name}} subscription created`);
                                })
                                .catch((error : Error) => {
                                    log.error(`[RCL] {${raw.name}} subscription throws : ${error}`);
                                });
                        } else if(raw.type === mqtt_request_type.pub) {
                            create_rcl_publisher(master, raw.message_type, raw.name, mqtt)
                                .then(() => {
                                    log.info(`[RCL] {${raw.name}} publisher created`);
                                })
                                .catch((error : Error) => {
                                    log.error(`[RCL] {${raw.name}} publisher throws : ${error}`);
                                });
                        } else if(raw.type === mqtt_request_type.action) {
                            create_rcl_action_client(master, raw.message_type, raw.name)
                                .then((client) => {
                                    request_to_rcl_action_server(client!, raw.request_type, raw.name, mqtt);
                                })
                                .catch((error) => {
                                    log.error(`[RCL] request action client [${error}]`);
                                });
                        } else if(raw.type === mqtt_request_type.service) {
                            create_rcl_service_client(master, raw.message_type, raw.name)
                                .then((client) => {
                                    request_to_rcl_service_server(client!, raw.request_type, raw.name, mqtt);
                                })
                                .catch((error) => {
                                    log.error(`[RCL] service client [${error}]`);
                                });
                        };
                    };
                } catch (error : any) {
                    log.error(`[RCL] ros_message_init : [${error}]`);
                    throw new Error(error);
                };
            } else return;
        });
    };
};

/**
 * async function for invoke MainLaunch class instance
 * @see MainLaunch
 */
async function run() : Promise<void> {
    new MainLaunch();
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
    log.info('[RCL] Client is ready for RCL!!');
};

/**
 * async function for main runtime
 * @see run
 * @see welcome
 */
(async function main() : Promise<void> {
    await run()
    .then(() => welcome())
    .catch((err : Error) => log.error(`[RCL] Client has crashed by.. ${err} `));
})().catch((e : Error) : void => {
    log.error(`[RCL] Client has crashed by.. ${e}`);
});