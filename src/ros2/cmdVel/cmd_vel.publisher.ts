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
        
        this.mqtt.subscribe('wavem/1/cmd_vel');
        this.mqtt.client.on("message", (topic, message) => {
            log.info(`RCL publish MQTT onMessage topic : ${topic}, message : ${message}`);
            let msg = this.genTwistMsg(message.toString());
            this.publisher.publish(msg);
        });
    };

    stop(): void {
        this.isRunning = false;
        this.mqtt.client.unsubscribe('wavem/1/cmd_vel');
    };

    protected genTwistMsg(message: any): rclnodejs.geometry_msgs.msg.Twist {
        let twistMsg = rclnodejs.createMessageObject('geometry_msgs/msg/Twist') as rclnodejs.geometry_msgs.msg.Twist;

        const twist = JSON.parse(message);

        twistMsg.linear = twist.linear;
        twistMsg.angular = twist.angular;

        return twistMsg;
    };
};

