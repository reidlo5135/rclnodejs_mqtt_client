'strict mode';

import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
import { log } from '../common/common_logger.infra';
import { initPublish, publish } from '../common/common_node.infra';

export default class CmdVelPublisher {

    private isRunning = false;
    private readonly node: rclnodejs.Node;
    private publisher: rclnodejs.Publisher<'geometry_msgs/msg/Twist'>;
    private mqtt: Mqtt;

    constructor(public readonly topic:string, mqtt:Mqtt) {
        this.node = new rclnodejs.Node('cmd_vel_publisher');
        this.publisher = initPublish(this.node, 'geometry_msgs/msg/Twist', topic);
        this.mqtt = mqtt;
        this.node.spin();
    };

    start(): void {
        if (this.isRunning) return;
    
        this.isRunning = true;

        let msg = this.genTwistMsg();
        publish('wavem/1/cmd_vel', this.publisher, msg, this.mqtt);
    };

    stop(): void {
        this.isRunning = false;
        this.mqtt.client.unsubscribe('wavem/1/cmd_vel');
    };

    protected genTwistMsg(range = 10): rclnodejs.geometry_msgs.msg.Twist {
        let twistMsg = rclnodejs.createMessageObject('geometry_msgs/msg/Twist') as rclnodejs.geometry_msgs.msg.Twist;

        twistMsg.linear = {x: 0, y: 0, z: 0};
        twistMsg.angular = {x: 0, y: 0, z: 0};

        return twistMsg;
    };
};

