"use strict";
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
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
const rclnodejs = __importStar(require("rclnodejs"));
const mqtt_infra_1 = __importDefault(require("./mqtt/mqtt.infra"));
const odometry_subscriber_1 = __importDefault(require("./ros2/odom/odometry.subscriber"));
const imu_data_subscriber_1 = __importDefault(require("./ros2/imu/imu_data.subscriber"));
const robot_pose_subscriber_1 = __importDefault(require("./ros2/robotPose/robot_pose.subscriber"));
const scan_subscriber_1 = __importDefault(require("./ros2/scan/scan.subscriber"));
const laser_scan_publisher_1 = require("./ros2/scan/laserScan/laser_scan.publisher");
const joint_states_publisher_1 = __importDefault(require("./ros2/jointStates/joint_states.publisher"));
function run() {
    return __awaiter(this, void 0, void 0, function* () {
        yield rclnodejs.init();
        const mqtt = new mqtt_infra_1.default();
        const imuData = new imu_data_subscriber_1.default('/imu/data', 'sensor_msgs/msg/Imu', mqtt);
        const odom = new odometry_subscriber_1.default('/odom', 'nav_msgs/msg/Odometry', mqtt);
        const robotPose = new robot_pose_subscriber_1.default('/robot_pose', 'geometry_msgs/msg/Pose', mqtt);
        const scan = new scan_subscriber_1.default('/scan', 'sensor_msgs/msg/LaserScan', mqtt);
        const laserScanPublisher = new laser_scan_publisher_1.LaserScanPublisher('laser_frame');
        laserScanPublisher.start();
        const jointStatesPublisher = new joint_states_publisher_1.default('joint_state', mqtt);
        jointStatesPublisher.start();
    });
}
;
(function main() {
    return __awaiter(this, void 0, void 0, function* () {
        run();
    });
})().catch(() => {
    process.exitCode = 1;
});
//# sourceMappingURL=index.js.map