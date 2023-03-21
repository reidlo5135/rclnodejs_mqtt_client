'use strict';

import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
import { subscribe } from '../common/common_node.infra';

export default class RobotPoseSubscriber {
    private isRunning = false;
    private readonly node: rclnodejs.Node;
    private subscriber: rclnodejs.Subscription;

    constructor(public readonly topic:string, public readonly type: any, mqtt:Mqtt) {
        this.node = new rclnodejs.Node('robot_pose_subscriber');
        this.isRunning = true;
        this.topic = topic;
        this.type = type;
        this.subscriber = subscribe(this.node, this.type, this.topic, mqtt);
        this.node.spin();
    };
};