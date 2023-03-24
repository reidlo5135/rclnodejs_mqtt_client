import Mqtt from '../../mqtt/mqtt.infra';
export default class TfSubscriber {
    private readonly topic;
    private readonly type;
    private isRunning;
    private readonly node;
    private readonly subscriber;
    constructor(topic: string, type: any, mqtt: Mqtt);
}
//# sourceMappingURL=tf.subscriber.d.ts.map