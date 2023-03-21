import Mqtt from '../../mqtt/mqtt.infra';
export default class RobotPoseSubscriber {
    readonly topic: string;
    readonly type: any;
    private isRunning;
    private readonly node;
    private subscriber;
    constructor(topic: string, type: any, mqtt: Mqtt);
}
//# sourceMappingURL=robot_pose.subscriber.d.ts.map