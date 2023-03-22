import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../../mqtt/mqtt.infra';
export default class LaserScanPublisher {
    readonly topic: string;
    private isRunning;
    private readonly node;
    private publisher;
    private publisherTimer;
    private mqtt;
    constructor(topic: string, mqtt: Mqtt);
    start(interval?: number): void;
    stop(): void;
    protected genLaserScanMsg(range?: number): rclnodejs.sensor_msgs.msg.LaserScan;
}
//# sourceMappingURL=laser_scan.publisher.d.ts.map