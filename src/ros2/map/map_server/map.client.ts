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
import Mqtt from '../../../mqtt/mqtt.infra';
import { log } from '../../common/common_logger.infra';
import { client } from '../../common/common_node.infra';

/**
 * Class for ROS2 client for call /map_server/map service
 * @see rclnodejs.Client
 */
export default class MapClient {

    /**
     * private boolean filed for this node is running or not
     */
    private isRunning = false;

    /**
     * private readonly field for rclnodejs.Node instance
     * @see rclnodejs.Node
     */
    private readonly node: rclnodejs.Node;

    /**
     * private readonly field for rclnodejs.Client<any> instance
     * @see rclnodejs.Client<any>
     */
    private readonly client: rclnodejs.Client<any>;

    /**
     * private readonly field for rclnodejs.Messag instance
     */
    private readonly request: rclnodejs.Message;

    /**
     * private readonly field for Mqtt
     * @see Mqtt
     */
    private readonly mqtt:Mqtt;
    
    /**
     * constructor for initialize field instance
     * @see node
     * @see client
     * @see mqtt
     * @see rclnodejs.createMessageObject
     * @param msg_type : any
     * @param req_type : any
     * @param service : string
     * @param mqtt : Mqtt
     */
    constructor(private readonly msg_type:any, private readonly req_type:any, private readonly service:string, mqtt:Mqtt) {
        this.node = new rclnodejs.Node('map_client', 'map_server');
        this.client = client(this.node, msg_type, service);
        this.request = rclnodejs.createMessageObject(req_type);
        this.mqtt = mqtt;
        this.node.spin();
    };

    /**
     * void function for MQTT subscription & create ROS2 client for call ROS2 service & convert recieved MQTT message(map data) into hex string
     * @see mqtt
     * @see client
     * @see client.waitForService
     */
    call(): void {
        this.mqtt.subscribe('wavem/1/requestMap');
        this.mqtt.client.on('message', (topic, message) => {
            log.info(`RCL call MQTT onMessage topic : ${topic}, message : ${message}`);
            if(topic.includes('requestMap')) {
                this.client.waitForService(1000)
                    .then((result) => {
                        if(!result) {
                            log.error(`RCL ${this.service} is not available... check your ROS2 Launch Mode`);
                            return;
                        };

                        log.info(`RCL call ${this.service}`);
                        this.client.sendRequest(this.request, (response) => {
                            if(response === null) log.info(`RCL call ${this.service} service call is null `);
                            
                            const buffer = Buffer.from(response.map.data);
                
                            const bmpData = {
                                width: response.map.info.width,
                                height: response.map.info.height
                            };
                            
                            const data = JSON.stringify(bmpData) + '/' + buffer.toString('hex');
                            this.mqtt.publish(`wavem/1${this.service}`, data);
                        });
                    })
                    .catch((e) => {
                        log.error(`RCL client call ${this.service} error occurred : ${JSON.stringify(e)}`);
                    });
            } else return;
        });
    };
};