import Mqtt from '../../mqtt/mqtt.infra';
export default class ImuDataSubscriber {
    readonly topic: string;
    readonly type: any;
    private isRunning;
    private readonly node;
    private subscriber;
    constructor(topic: string, type: any, mqtt: Mqtt);
}
//# sourceMappingURL=imu_data.subscriber.d.ts.map