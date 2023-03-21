import * as rclnodejs from 'rclnodejs';
/**
 * ROS2 publisher of simulated LaserScan messages.
 * A timer is used to create and publish a laserScan message once per second (1 Hz).
 * Use start() and stop() to initiate and terminate messgage publication.
 */
export declare class LaserScanPublisher {
    readonly node: rclnodejs.Node;
    readonly topic: string;
    private isRunning;
    private publisher;
    private publisherTimer;
    /**
     * Create an instance
     *
     * @param node - The node that to which this publisher belongs.
     * @param topic - The topic name to which laserScan msgs will be published.
     */
    constructor(node: rclnodejs.Node, topic?: string);
    /**
     * Start the laserScan message generation and publishing process.
     *
     * @param interval - The unit of time (milliseconds) to wait before running the next .
     */
    start(interval?: number): void;
    /**
     * Stop creating and publishing laserScan messages.
     */
    stop(): void;
    /**
     * Creates a simulated forward facing LaserScan message.
     * Scan data consists of 180 measurements on a 180 degree arc centered on the
     * x-axis extending directly forward of the virutal lidar device.
     *
     * @param range  - The distance of simulated lidar range readings
     * @returns A new LaserScan message
     */
    protected genLaserScanMsg(range?: number): rclnodejs.sensor_msgs.msg.LaserScan;
}
//# sourceMappingURL=laser_scan.publisher.test.node.d.ts.map