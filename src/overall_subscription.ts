import Mqtt from "./mqtt/mqtt.infra";
import * as rclnodejs from 'rclnodejs';
import TfSubscriber from './ros2/tf/tf.subscriber';
import { log } from './ros2/common/common_logger.infra';
import ScanSubscriber from './ros2/scan/scan.subscriber';
import ImuDataSubscriber from './ros2/imu/imu_data.subscriber';
import OdometrySubscriber from './ros2/odom/odometry.subscriber';
import RobotPoseSubscriber from './ros2/robotPose/robot_pose.subscriber';

/**
 * async function for run ROS2-MQTT Subscriptions
 */
export async function runSubscriptions() {

  /**
   * await function for rclnodejs.init();
   */
  await rclnodejs.init();

  /**
   * const instance for mqtt class
   * @see Mqtt
   */
  const mqtt:Mqtt = new Mqtt();  

  /**
   * const instance for ImuDataSubscriber class
   * @see ImuDataSubscriber
   * @see mqtt
   */
  const imuData = new ImuDataSubscriber('/imu/data', 'sensor_msgs/msg/Imu', mqtt);

  /**
   * const instacne for OdometrySubscriber class
   * @see OdometrySubscriber
   * @see mqtt
   */
  const odom = new OdometrySubscriber('/odom', 'nav_msgs/msg/Odometry', mqtt);

  /**
   * const instance for RobotPoseSubscriber class
   * @see RobotPoseSubscriber
   * @see mqtt
   */
  const robotPose = new RobotPoseSubscriber('/robot_pose', 'geometry_msgs/msg/Pose', mqtt);

  /**
   * const instance for ScanSubscriber class
   * @see RobotPoseSubscriber
   * @see mqtt
   */
  const scan = new ScanSubscriber('/scan', 'sensor_msgs/msg/LaserScan', mqtt);

  /**
   * const instance for TfSubscriber class
   * @see TfSubscriber
   * @see mqtt
   */
  const tf = new TfSubscriber('/tf', 'tf2_msgs/msg/TFMessage', mqtt);
};

/**
 * function for logging when run() started
 */
function welcome() {
  console.log('  _____   ____   _____ ___    __  __  ____ _______ _______    _____ _      _____ ______ _   _ _______ ');
  console.log(' |  __ \\ / __ \\ / ____|__ \\  |  \\/  |/ __ \\__   __|__   __|  / ____| |    |_   _|  ____| \\ | |__   __|');
  console.log(' | |__) | |  | | (___    ) | | \\  / | |  | | | |     | |    | |    | |      | | | |__  |  \\| |  | |   ');
  console.log(" |  _  /| |  | |\\___ \\  / /  | |\\/| | |  | | | |     | |    | |    | |      | | |  __| | . ` |  | |   ");
  console.log(' | | \\ \\| |__| |____) |/ /_  | |  | | |__| | | |     | |    | |____| |____ _| |_| |____| |\\  |  | |   ');
  console.log(' |_|  \\_\\\\____/|_____/|____| |_|  |_|\\___\\_\\ |_|     |_|     \\_____|______|_____|______|_| \\_|  |_|   ');
  console.log('                                                                                                      ');
  log.info('ROS2-MQTT OnlySubscription is ready for RCL');
};

/**
 * async function for main runtime
 * @returns : Promise<void>
 */
(async function main(): Promise<void> {
  runSubscriptions()
    .then(() => welcome())
    .catch((err) => log.error(`ROS2-MQTT OnlySubscription has crashed by.. ${err} `));
})().catch((e): void => {
    log.error('overall subscriptions error : ', e);
    process.exitCode = 1
});