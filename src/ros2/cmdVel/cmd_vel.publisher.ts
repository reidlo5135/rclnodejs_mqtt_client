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
            liX += 0.125;
            angZ -= 0.35;
            log.info(`RCL cmd_vel move left : liX ${liX}, angZ : ${angZ}`);
        } else if (command === 'twForLeft') {
            liX += 0.20;
            angZ -= 0.30;
            log.info(`RCL cmd_vel move twForLeft : liX ${liX}, angZ : ${angZ}`);
        } else if (command === 'twBackLeft') {
            liX -= 0.20;
            angZ -= 0.30;
            log.info(`RCL cmd_vel move twBackLeft : liX ${liX}, angZ : ${angZ}`);
        } else if (command === 'right') {
            liX += 0.125;
            angZ += 0.35;
            log.info(`RCL cmd_vel move right : liX ${liX}, angZ : ${angZ}`);
        } else if (command === 'twForRight') {
            liX += 0.125;
            angZ += 0.30;
            log.info(`RCL cmd_vel move twForRight : liX ${liX}, angZ : ${angZ}`);
        } else if (command === 'twBackRight') {
            liX -= 0.125;
            angZ += 0.30;
            log.info(`RCL cmd_vel move twBackRight : liX ${liX}, angZ : ${angZ}`);
        } else if (command === 's') {
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

