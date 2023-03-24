import Mqtt from '../../mqtt/mqtt.infra';
export default class OdometrySubscriber {
    private readonly topic;
    private readonly type;
    private isRunning;
    private readonly node;
    private readonly subscriber;
    constructor(topic: string, type: any, mqtt: Mqtt);
}
//# sourceMappingURL=odometry.subscriber.d.ts.map