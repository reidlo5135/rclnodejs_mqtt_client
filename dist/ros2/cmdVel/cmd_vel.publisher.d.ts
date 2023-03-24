import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
export default class CmdVelPublisher {
    private readonly topic;
    private isRunning;
    private readonly node;
    private readonly publisher;
    private readonly mqtt;
    constructor(topic: string, mqtt: Mqtt);
    start(): void;
    stop(): void;
    protected genTwistMsg(message: any): rclnodejs.geometry_msgs.msg.Twist;
}
//# sourceMappingURL=cmd_vel.publisher.d.ts.map