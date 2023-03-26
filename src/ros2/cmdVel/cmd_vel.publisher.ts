'strict mode';

import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
import { log } from '../common/common_logger.infra';
import { initPublish } from '../common/common_node.infra';

/**
 * Class for ROS2 publish /cmd_vel topic to robot
 * @see rclnodejs.Publisher
 */
export default class CmdVelPublisher {

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
     * private readonly field for rclnodejs.Publisher<'geometry_msgs/msg/Twist'> instance
     * @see rclnodejs.Publisher
     * @see geometry_msgs/msgs/Twist
     */
    private readonly publisher: rclnodejs.Publisher<'geometry_msgs/msg/Twist'>;

    /**
     * private readonly field for Mqtt
     * @see Mqtt
     */
    private readonly mqtt: Mqtt;

    /**
     * constructor for initialize field instances
     * @see node
     * @see publisher
     * @see mqtt
     * @see initPublish
     * @param topic : string
     * @param mqtt : Mqtt
     */
    constructor(private readonly topic:string, mqtt:Mqtt) {
        this.node = new rclnodejs.Node('cmd_vel_publisher');
        this.publisher = initPublish(this.node, 'geometry_msgs/msg/Twist', topic);
        this.mqtt = mqtt;
        this.node.spin();
    };

    /**
     * void function for MQTT subscription & ROS2 publish by recieved MQTT message
     * @see mqtt
     * @see publisher
     */
    start(): void {
        if (this.isRunning) return;
    
        this.isRunning = true;
        
        this.mqtt.subscribe('wavem/1/cmd_vel');
        this.mqtt.client.on("message", (topic, message) => {
            log.info(`RCL cmd_vel publish MQTT onMessage topic : ${topic}, message : ${message}`);
            if(topic.includes('cmd_vel')) {
                let msg = this.genTwistMsg(message.toString());
                this.publisher.publish(msg);
            } else return;
        });
    };

    /**
     * void function for stop node spinning & MQTT unsubscribe
     */
    stop(): void {
        this.isRunning = false;
        this.mqtt.client.unsubscribe('wavem/1/cmd_vel');
    };

    /**
     * protected function for generate ROS2 publishing message object
     * @see rclnodejs.geemetry_msgs.msg.Twist
     * @param message : any
     * @returns twistMsg : rclnodejs.geemetry_msgs.msg.Twist
     */
    protected genTwistMsg(message: any): rclnodejs.geometry_msgs.msg.Twist {
        let twistMsg = rclnodejs.createMessageObject('geometry_msgs/msg/Twist') as rclnodejs.geometry_msgs.msg.Twist;

        const twist = JSON.parse(message);

        if((twist.linear === null || twist.linear === '') || (twist.angular === null || twist.angular === '')) {
            twistMsg.linear = {x:0,y:0,z:0};
            twistMsg.angular = {x:0,y:0,z:0};
            throw new Error('RCL cmdVel publish values is empty...');
        };

        twistMsg.linear = twist.linear;
        twistMsg.angular = twist.angular;

        return twistMsg;
    };
};

