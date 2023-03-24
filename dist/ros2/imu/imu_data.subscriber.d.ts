import Mqtt from '../../mqtt/mqtt.infra';
export default class ImuDataSubscriber {
    private readonly topic;
    private readonly type;
    private isRunning;
    private readonly node;
    private readonly subscriber;
    constructor(topic: string, type: any, mqtt: Mqtt);
}
//# sourceMappingURL=imu_data.subscriber.d.ts.map