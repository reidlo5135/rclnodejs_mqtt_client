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
import { Subscriber } from '../common/common_ndoe.interface';
import { subscribe } from '../common/common_node.infra';

/**
 * Class for ROS2 subscribe /tf_static topic from object
 * @see rclnodejs.Subscription
 * @see Subscriber
 */

export default class TfStaticSubscriber implements Subscriber {

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
     * private readonly field for Mqtt instance
     */
    private readonly mqtt: Mqtt;

    /**
     * constructor for initialize field instances
     * @see node
     * @see Mqtt
     * @param topic : string
     * @param type : any
     * @param mqtt : Mqtt
     */
    constructor(private readonly topic:string, private readonly type:any, mqtt:Mqtt) {
        this.node = new rclnodejs.Node('tf_static_subscriber');
        this.mqtt = mqtt;
    };

    start(): void {
        this.isRunning = true;
        subscribe(this.node, this.type, this.topic, this.mqtt);
        this.node.spin();
    };

    stop(): void {
        this.isRunning = false;
        this.mqtt.client.unsubscribe(this.topic);
        this.node.destroy();
    };
}