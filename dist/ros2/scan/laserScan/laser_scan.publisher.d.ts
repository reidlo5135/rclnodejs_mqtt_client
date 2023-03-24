import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../../mqtt/mqtt.infra';
export default class LaserScanPublisher {
    private readonly topic;
    private isRunning;
    private readonly node;
    private readonly publisher;
    private publisherTimer;
    private readonly mqtt;
    constructor(topic: string, mqtt: Mqtt);
    start(interval?: number): void;
    stop(): void;
    protected genLaserScanMsg(range?: number): rclnodejs.sensor_msgs.msg.LaserScan;
}
//# sourceMappingURL=laser_scan.publisher.d.ts.map