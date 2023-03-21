import * as rclnodejs from 'rclnodejs';
import { Mqtt } from "./mqtt/mqtt.infra";
import imu_data_subscriber_node from "./ros2/imu/imu_data.subscriber.node";
import odometry_subscriber_node from "./ros2/odom/odometry.subscriber.node";
import robot_pose_subscriber_node from "./ros2/robotPose/robot_pose.subscriber.node";
import scan_subscriber_node from "./ros2/scan/scan.subscriber.node";

async function run() {

  await rclnodejs.init();
  const mqtt:Mqtt = new Mqtt();
    
  imu_data_subscriber_node(mqtt);
  odometry_subscriber_node(mqtt);
  scan_subscriber_node(mqtt);
  robot_pose_subscriber_node(mqtt);
};

(async function main(): Promise<void> {
  run();
})().catch((): void => {
  process.exitCode = 1
});