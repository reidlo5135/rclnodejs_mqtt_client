import * as rclnodejs from 'rclnodejs';
import Mqtt from "./mqtt/mqtt.infra";
import OdometrySubscriber from './ros2/odom/odometry.subscriber';
import ImuDataSubscriber from './ros2/imu/imu_data.subscriber';
import RobotPoseSubscriber from './ros2/robotPose/robot_pose.subscriber';
import ScanSubscriber from './ros2/scan/scan.subscriber';
import { log } from './ros2/common/common_logger.infra';


export async function runSubscriptions() {
  await rclnodejs.init();

  const mqtt:Mqtt = new Mqtt();  

  const imuData = new ImuDataSubscriber('/imu/data', 'sensor_msgs/msg/Imu', mqtt);
  const odom = new OdometrySubscriber('/odom', 'nav_msgs/msg/Odometry', mqtt);
  const robotPose = new RobotPoseSubscriber('/robot_pose', 'geometry_msgs/msg/Pose', mqtt);
  const scan = new ScanSubscriber('/scan', 'sensor_msgs/msg/LaserScan', mqtt);
};

(async function main(): Promise<void> {
  runSubscriptions();
})().catch((e): void => {
    log.error('overall subscriptions error : ', e);
    process.exitCode = 1
});