import * as rclnodejs from 'rclnodejs';
export declare class LaserScanPublisher {
    readonly topic: string;
    private isRunning;
    private readonly node;
    private publisher;
    private publisherTimer;
    constructor(topic: string);
    start(interval?: number): void;
    stop(): void;
    protected genLaserScanMsg(range?: number): rclnodejs.sensor_msgs.msg.LaserScan;
}
//# sourceMappingURL=laser_scan.publisher.d.ts.map