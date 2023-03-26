import Mqtt from "./mqtt/mqtt.infra";
import * as rclnodejs from 'rclnodejs';
import { log } from './ros2/common/common_logger.infra';
import CmdVelPublisher from './ros2/cmdVel/cmd_vel.publisher';
import LaserScanPublisher from './ros2/scan/laserScan/laser_scan.publisher';
import JointStatesPublisher from './ros2/jointStates/joint_states.publisher';

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

function welcome() {
  console.log('  _____   ____   _____ ___    __  __  ____ _______ _______    _____ _      _____ ______ _   _ _______ ');
  console.log(' |  __ \\ / __ \\ / ____|__ \\  |  \\/  |/ __ \\__   __|__   __|  / ____| |    |_   _|  ____| \\ | |__   __|');
  console.log(' | |__) | |  | | (___    ) | | \\  / | |  | | | |     | |    | |    | |      | | | |__  |  \\| |  | |   ');
  console.log(" |  _  /| |  | |\\___ \\  / /  | |\\/| | |  | | | |     | |    | |    | |      | | |  __| | . ` |  | |   ");
  console.log(' | | \\ \\| |__| |____) |/ /_  | |  | | |__| | | |     | |    | |____| |____ _| |_| |____| |\\  |  | |   ');
  console.log(' |_|  \\_\\\\____/|_____/|____| |_|  |_|\\___\\_\\ |_|     |_|     \\_____|______|_____|______|_| \\_|  |_|   ');
  console.log('                                                                                                      ');
  log.info('ROS2-MQTT OnlyPublisher is ready for RCL')
};

(async function main(): Promise<void> {
  runPublishers()
    .then(() => welcome())
    .catch((err) => log.error(`ROS2-MQTT OnlyPublisher has crashed by.. ${err} `));
})().catch((e): void => {
    log.error('overall publishers error : ', e);
    process.exitCode = 1
});