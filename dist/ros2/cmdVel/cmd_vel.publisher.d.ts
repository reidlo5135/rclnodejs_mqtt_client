import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
export default class CmdVelPublisher {
    readonly topic: string;
    private isRunning;
    private readonly node;
    private publisher;
    private mqtt;
    constructor(topic: string, mqtt: Mqtt);
    start(): void;
    stop(): void;
    protected genTwistMsg(range?: number): rclnodejs.geometry_msgs.msg.Twist;
}
//# sourceMappingURL=cmd_vel.publisher.d.ts.map