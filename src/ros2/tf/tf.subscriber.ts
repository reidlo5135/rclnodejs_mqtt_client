'strict mode';

import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
import { subscribe } from '../common/common_node.infra';

export default class TfSubscriber {
    private isRunning = false;
    private readonly node: rclnodejs.Node;
    private readonly subscriber: rclnodejs.Subscription;

    constructor(private readonly topic:string, private readonly type:any, mqtt:Mqtt) {
        this.node = new rclnodejs.Node('tf_subscriber');
        this.isRunning = true;
        this.subscriber = subscribe(this.node, this.type, this.topic, mqtt);
        this.node.spin();
    };
};