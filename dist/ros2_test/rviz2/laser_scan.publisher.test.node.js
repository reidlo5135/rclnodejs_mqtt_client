"use strict";
'strict mode';
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || function (mod) {
    if (mod && mod.__esModule) return mod;
    var result = {};
    if (mod != null) for (var k in mod) if (k !== "default" && Object.prototype.hasOwnProperty.call(mod, k)) __createBinding(result, mod, k);
    __setModuleDefault(result, mod);
    return result;
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.LaserScanPublisher = void 0;
const rclnodejs = __importStar(require("rclnodejs"));
/**
 * ROS2 publisher of simulated LaserScan messages.
 * A timer is used to create and publish a laserScan message once per second (1 Hz).
 * Use start() and stop() to initiate and terminate messgage publication.
 */
class LaserScanPublisher {
    /**
     * Create an instance
     *
     * @param node - The node that to which this publisher belongs.
     * @param topic - The topic name to which laserScan msgs will be published.
     */
    constructor(node, topic = 'laser_frame') {
        this.node = node;
        this.topic = topic;
        this.isRunning = false;
        this.publisher = node.createPublisher('sensor_msgs/msg/LaserScan', topic);
    }
    ;
    /**
     * Start the laserScan message generation and publishing process.
     *
     * @param interval - The unit of time (milliseconds) to wait before running the next .
     */
    start(interval = 1000) {
        if (this.isRunning)
            return;
        this.isRunning = true;
        this.publisherTimer = this.node.createTimer(interval, () => {
            let msg = this.genLaserScanMsg();
            console.log('publish msg at ', msg.header.stamp.sec);
            this.publisher.publish(msg);
        });
    }
    ;
    /**
     * Stop creating and publishing laserScan messages.
     */
    stop() {
        this.publisherTimer.cancel();
        this.publisherTimer = null;
        this.isRunning = false;
    }
    ;
    /**
     * Creates a simulated forward facing LaserScan message.
     * Scan data consists of 180 measurements on a 180 degree arc centered on the
     * x-axis extending directly forward of the virutal lidar device.
     *
     * @param range  - The distance of simulated lidar range readings
     * @returns A new LaserScan message
     */
    genLaserScanMsg(range = 10) {
        // create empty laserScan msg
        let laserScanMsg = rclnodejs.createMessageObject('sensor_msgs/msg/LaserScan');
        // configure msg header 
        laserScanMsg.header.frame_id = 'laser_frame';
        laserScanMsg.header.stamp = this.node.now().toMsg();
        // generate range and intensity data
        let sample_cnt = 180;
        let ranges = new Array(sample_cnt).fill(range);
        // configure msg data
        laserScanMsg.angle_min = 0;
        laserScanMsg.angle_max = Math.PI / 2.0;
        laserScanMsg.angle_increment = Math.PI / 180.0;
        laserScanMsg.time_increment = 1.0 / sample_cnt;
        laserScanMsg.scan_time = 1.0;
        laserScanMsg.range_min = range - 1;
        laserScanMsg.range_max = range + 1;
        laserScanMsg.ranges = ranges;
        return laserScanMsg;
    }
    ;
}
exports.LaserScanPublisher = LaserScanPublisher;
;
//# sourceMappingURL=laser_scan.publisher.test.node.js.map