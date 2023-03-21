import Mqtt from '../../mqtt/mqtt.infra';
export default class OdometrySubscriber {
    readonly topic: string;
    readonly type: any;
    private isRunning;
    private readonly node;
    private subscriber;
    constructor(topic: string, type: any, mqtt: Mqtt);
}
//# sourceMappingURL=odometry.subscriber.d.ts.map