'strict mode';

import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
import { subscribe } from '../common/common_node.infra';

export default class ImuDataSubscriber {
    private isRunning = false;
    private subscriber: rclnodejs.Subscription;

    constructor(private readonly node: rclnodejs.Node, public readonly topic:string, public readonly type: any, mqtt:Mqtt) {
        this.isRunning = true;
        this.subscriber = subscribe(this.node, this.type, this.topic, mqtt);
        this.node.spin();
    };
};