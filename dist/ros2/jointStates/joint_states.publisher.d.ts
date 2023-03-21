import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
export default class JointStatesPublisher {
    readonly topic: string;
    private isRunning;
    private readonly node;
    private publisher;
    private mqtt;
    constructor(topic: string, mqtt: Mqtt);
    start(): void;
    stop(): void;
    protected genJointStatesMsg(range?: number): rclnodejs.sensor_msgs.msg.JointState;
}
//# sourceMappingURL=joint_states.publisher.d.ts.map