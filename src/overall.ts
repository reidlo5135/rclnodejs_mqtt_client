import * as rclnodejs from 'rclnodejs';
import Mqtt from "./mqtt/mqtt.infra";
import LaserScanPublisher from './ros2/scan/laserScan/laser_scan.publisher';
import JointStatesPublisher from './ros2/jointStates/joint_states.publisher';
import CmdVelPublisher from './ros2/cmdVel/cmd_vel.publisher';
import OdometrySubscriber from './ros2/odom/odometry.subscriber';
import ImuDataSubscriber from './ros2/imu/imu_data.subscriber';
import RobotPoseSubscriber from './ros2/robotPose/robot_pose.subscriber';
import ScanSubscriber from './ros2/scan/scan.subscriber';
import { log } from './ros2/common/common_logger.infra';
import MapClient from './ros2/map/map_server/map.client';
import TfSubscriber from './ros2/tf/tf.subscriber';

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

(async function main(): Promise<void> {
    run()
    .then(() => log.info('ROS2-MQTT Client is ready for RCL'))
    .catch((err) => log.error(`ROS2-MQTT Client has crashed by.. ${err} `));
})().catch((e): void => {
    log.error('overall error : ', e);
    process.exitCode = 1
});