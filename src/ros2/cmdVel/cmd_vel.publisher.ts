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

        publish('wavem/1/cmd_vel', this.publisher, '', this.mqtt);

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

        let liX = 0.0;
        let angZ = 0.0;

        const command = JSON.parse(message).command;

        if(command === 'forward') {
            liX += 0.25;
            log.info(`RCL cmd_vel move forward : liX ${liX}, angZ : ${angZ}`);
        } else if (command === 'backward') {
            liX -= 0.25;
            log.info(`RCL cmd_vel move backward : liX ${liX}, angZ : ${angZ}`);
        } else if (command === 'left') {
            angZ -= 0.35;
            log.info(`RCL cmd_vel move left : liX ${liX}, angZ : ${angZ}`);
        } else if (command === 'right') {
            angZ += 0.35;
            log.info(`RCL cmd_vel move right : liX ${liX}, angZ : ${angZ}`);
        } else if (command === 'stop') {
            liX = 0.0;
            angZ = 0.0;
            log.info(`RCL cmd_vel move stop : liX ${liX}, angZ : ${angZ}`);
        } else {
            liX = 0.0;
            angZ = 0.0;
        }

        twistMsg.linear = {x: liX, y: 0, z: 0};
        twistMsg.angular = {x: 0, y: 0, z: angZ};

        return twistMsg;
    };
};

