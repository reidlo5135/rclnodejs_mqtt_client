import Mqtt from '../../mqtt/mqtt.infra';
export default class RobotPoseSubscriber {
    private readonly topic;
    private readonly type;
    private isRunning;
    private readonly node;
    private readonly subscriber;
    constructor(topic: string, type: any, mqtt: Mqtt);
}
//# sourceMappingURL=robot_pose.subscriber.d.ts.map