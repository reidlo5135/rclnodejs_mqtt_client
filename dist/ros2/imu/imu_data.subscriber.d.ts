import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
export default class ImuDataSubscriber {
    private readonly node;
    readonly topic: string;
    readonly type: any;
    private isRunning;
    private subscriber;
    constructor(node: rclnodejs.Node, topic: string, type: any, mqtt: Mqtt);
}
//# sourceMappingURL=imu_data.subscriber.d.ts.map