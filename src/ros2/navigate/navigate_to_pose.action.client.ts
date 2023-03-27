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
import { log } from '../common/common_logger.infra';
import { actionClient } from '../common/common_node.infra';

/**
 * Class for ROS2 action client /navigate_to_pose
 * @see rclnodejs.ActionClient<any>
 */
export default class NavigateToPoseActionClient {

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
     * private readonly field for Mqtt
     * @see Mqtt
     */
    private readonly mqtt: Mqtt;

    /**
     * constructor for initialize field instances
     * @see node
     * @see mqtt
     * @param topic : string
     * @param mqtt : Mqtt
     */
    constructor(private readonly topic:string, mqtt:Mqtt) {
        this.node = new rclnodejs.Node('navigate_to_pose_action_client_node');
        this.mqtt = mqtt;
    };

    sendGoal(): void {
        const client = actionClient(this.node, 'nav2_msgs/action/NavigateToPose', 'navigate_to_pose_action_client');
        client.waitForServer(1000)
            .then((result) => {
                if(!result || !client.isActionServerAvailable()) {
                    log.error(`RCL action server is not available... check your ROS2 Launch Mode`);
                    return;
                };
            })
            .catch((err) => {
                log.error(`RCL action client error occurred : ${JSON.stringify(err)}`);
            });
    }
};