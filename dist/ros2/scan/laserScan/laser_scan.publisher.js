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
const rclnodejs = __importStar(require("rclnodejs"));
const common_node_infra_1 = require("../../common/common_node.infra");
class LaserScanPublisher {
    constructor(topic, mqtt) {
        this.topic = topic;
        this.isRunning = false;
        this.node = new rclnodejs.Node('laser_scan_publisher');
        this.publisher = (0, common_node_infra_1.initPublish)(this.node, 'sensor_msgs/msg/LaserScan', topic);
        this.mqtt = mqtt;
        this.node.spin();
    }
    ;
    start(interval = 1000) {
        if (this.isRunning)
            return;
        this.isRunning = true;
        let msg = this.genLaserScanMsg();
        this.publisherTimer = this.node.createTimer(interval, () => {
            this.publisher.publish(msg);
        });
        // publish('wavem/1/laser_frame', this.publisher, msg, this.mqtt);
    }
    ;
    stop() {
        this.publisherTimer.cancel();
        this.publisherTimer = null;
        this.isRunning = false;
    }
    ;
    genLaserScanMsg(range = 10) {
        let laserScanMsg = rclnodejs.createMessageObject('sensor_msgs/msg/LaserScan');
        laserScanMsg.header.frame_id = 'laser_frame';
        laserScanMsg.header.stamp = this.node.now().toMsg();
        let sample_cnt = 180;
        let ranges = new Array(sample_cnt).fill(range);
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
exports.default = LaserScanPublisher;
;
//# sourceMappingURL=laser_scan.publisher.js.map