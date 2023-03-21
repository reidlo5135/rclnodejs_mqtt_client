import Mqtt from '../../mqtt/mqtt.infra';
export default class ScanSubscriber {
    readonly topic: string;
    readonly type: any;
    private isRunning;
    private readonly node;
    private subscriber;
    constructor(topic: string, type: any, mqtt: Mqtt);
}
//# sourceMappingURL=scan.subscriber.d.ts.map