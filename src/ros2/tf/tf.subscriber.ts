'strict mode';

import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
import { subscribe } from '../common/common_node.infra';

/**
 * Class for ROS2 subscribe /tf topic from object
 */
export default class TfSubscriber {

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
     * private readonly field for rclnodejs.Subscription instance
     */
    private readonly subscriber: rclnodejs.Subscription;

    /**
     * constructor for initialize field instances
     * @see node
     * @see subscriber;
     * @param topic : string
     * @param type : any
     * @param mqtt : Mqtt
     */
    constructor(private readonly topic:string, private readonly type:any, mqtt:Mqtt) {
        this.node = new rclnodejs.Node('tf_subscriber');
        this.isRunning = true;
        this.subscriber = subscribe(this.node, this.type, this.topic, mqtt);
        this.node.spin();
    };
};