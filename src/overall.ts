import Mqtt from "./mqtt/mqtt.infra";
import * as rclnodejs from 'rclnodejs';
import TfSubscriber from './ros2/tf/tf.subscriber';
import { log } from './ros2/common/common_logger.infra';
import ScanSubscriber from './ros2/scan/scan.subscriber';
import MapClient from './ros2/map/map_server/map.client';
import CmdVelPublisher from './ros2/cmdVel/cmd_vel.publisher';
import ImuDataSubscriber from './ros2/imu/imu_data.subscriber';
import OdometrySubscriber from './ros2/odom/odometry.subscriber';
import RobotPoseSubscriber from './ros2/robotPose/robot_pose.subscriber';
import LaserScanPublisher from './ros2/scan/laserScan/laser_scan.publisher';
import JointStatesPublisher from './ros2/jointStates/joint_states.publisher';

async function run() {
    await rclnodejs.init();

    const mqtt:Mqtt = new Mqtt();  

    const laserScanPublisher = new LaserScanPublisher('laser_frame', mqtt);
    laserScanPublisher.start();

    const jointStatesPublisher = new JointStatesPublisher('joint_states', mqtt);
    jointStatesPublisher.start();  

    const cmdVelPublisher = new CmdVelPublisher('cmd_vel', mqtt);
    cmdVelPublisher.start();
    
    const imuData = new ImuDataSubscriber('/imu/data', 'sensor_msgs/msg/Imu', mqtt);
    const odom = new OdometrySubscriber('/odom', 'nav_msgs/msg/Odometry', mqtt);
    const robotPose = new RobotPoseSubscriber('/robot_pose', 'geometry_msgs/msg/Pose', mqtt);
    const scan = new ScanSubscriber('/scan', 'sensor_msgs/msg/LaserScan', mqtt);
    const tf = new TfSubscriber('/tf', 'tf2_msgs/msg/TFMessage', mqtt);

    const mapServerMapClient = new MapClient('nav_msgs/srv/GetMap', 'nav_msgs/srv/GetMap_Request', '/map_server/map', mqtt);
    mapServerMapClient.call();
};

function welcome() {
    console.log('  _____   ____   _____ ___    __  __  ____ _______ _______    _____ _      _____ ______ _   _ _______ ');
    console.log(' |  __ \\ / __ \\ / ____|__ \\  |  \\/  |/ __ \\__   __|__   __|  / ____| |    |_   _|  ____| \\ | |__   __|');
    console.log(' | |__) | |  | | (___    ) | | \\  / | |  | | | |     | |    | |    | |      | | | |__  |  \\| |  | |   ');
    console.log(" |  _  /| |  | |\\___ \\  / /  | |\\/| | |  | | | |     | |    | |    | |      | | |  __| | . ` |  | |   ");
    console.log(' | | \\ \\| |__| |____) |/ /_  | |  | | |__| | | |     | |    | |____| |____ _| |_| |____| |\\  |  | |   ');
    console.log(' |_|  \\_\\\\____/|_____/|____| |_|  |_|\\___\\_\\ |_|     |_|     \\_____|______|_____|______|_| \\_|  |_|   ');
    console.log('                                                                                                      ');
    log.info('ROS2-MQTT Client is ready for RCL!!');
};

(async function main(): Promise<void> {
    run()
    .then(() => welcome())
    .catch((err) => log.error(`ROS2-MQTT Client has crashed by.. ${err} `));
    
})().catch((e): void => {
    log.error('overall error : ', e);
    process.exitCode = 1
});