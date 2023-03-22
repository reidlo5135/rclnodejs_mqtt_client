import * as rclnodejs from 'rclnodejs';
import Mqtt from "./mqtt/mqtt.infra";
import LaserScanPublisher from './ros2/scan/laserScan/laser_scan.publisher';
import JointStatesPublisher from './ros2/jointStates/joint_states.publisher';
import CmdVelPublisher from './ros2/cmdVel/cmd_vel.publisher';
import { log } from './ros2/common/common_logger.infra';


export async function runPublishers() {
  await rclnodejs.init();

  const mqtt:Mqtt = new Mqtt();  

  const laserScanPublisher = new LaserScanPublisher('laser_frame', mqtt);
  laserScanPublisher.start();

  const jointStatesPublisher = new JointStatesPublisher('joint_states', mqtt);
  jointStatesPublisher.start();  

  const cmdVelPublisher = new CmdVelPublisher('cmd_vel', mqtt);
  cmdVelPublisher.start();
};

(async function main(): Promise<void> {
  runPublishers();
})().catch((e): void => {
    log.error('overall publishers error : ', e);
    process.exitCode = 1
});