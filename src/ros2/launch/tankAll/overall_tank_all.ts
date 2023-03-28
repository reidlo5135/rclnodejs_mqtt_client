// Copyright [2023] [wavem-reidlo]
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import Mqtt from "../../../mqtt/mqtt.infra";
import * as rclnodejs from 'rclnodejs';
import TfSubscriber from "../../tf/tf.subscriber";
import { log } from "../../common/common_logger.infra";
import ScanSubscriber from "../../scan/scan.subscriber";
import CmdVelPublisher from "../../cmdVel/cmd_vel.publisher";
import ImuDataSubscriber from "../../imu/imu_data.subscriber";
import OdometrySubscriber from "../../odom/odometry.subscriber";
import LaserScanPublisher from "../../scan/laserScan/laser_frame.publisher";
import JointStatesPublisher from "../../jointStates/joint_states.publisher";

/**
 * async function for run Overall ROS2-MQTT Client
 */
async function run() {

    /**
     * await function for rclnodejs.init();
     */
    await rclnodejs.init();
    const master = new rclnodejs.Node('mqtt_tank_all');

    /**
     * const instance for mqtt class
     * @see Mqtt
     */
    const mqtt:Mqtt = new Mqtt();  

    /**
     * const instance for LaserScanPublihser class
     * @see LaserScanPublihser
     * @see LaserScanPublihser.start
     * @see mqtt
     */
    const laserScanPublisher = new LaserScanPublisher('laser_frame', mqtt);
    laserScanPublisher.start();

    /**
     * const instance for JointStatesPublisher class
     * @see JointStatesPublisher
     * @see JointStatesPublisher.start
     * @see mqtt
     */
    const jointStatesPublisher = new JointStatesPublisher('joint_states', mqtt);
    jointStatesPublisher.start();  

    /**
     * const instance for CmdVelPublisher class
     * @see CmdVelPublisher
     * @see CmdVelPublisher.start
     * @see mqtt
     */
    const cmdVelPublisher = new CmdVelPublisher('cmd_vel', mqtt);
    cmdVelPublisher.start();
    
    /**
     * const instance for ImuDataSubscriber class
     * @see ImuDataSubscriber
     * @see Mqtt
     */
    const imuData = new ImuDataSubscriber('/imu/data', 'sensor_msgs/msg/Imu', mqtt);
    imuData.start();

    /**
     * const instacne for OdometrySubscriber class
     * @see OdometrySubscriber
     * @see Mqtt
     */
    const odom = new OdometrySubscriber(master, '/odom', 'nav_msgs/msg/Odometry', mqtt);
    odom.start();

    /**
     * const instance for ScanSubscriber class
     * @see RobotPoseSubscriber
     * @see Mqtt
     */
    const scan = new ScanSubscriber('/scan', 'sensor_msgs/msg/LaserScan', mqtt);
    scan.start();

    /**
     * const instance for TfSubscriber class
     * @see TfSubscriber
     * @see Mqtt
     */
    const tf = new TfSubscriber('/tf', 'tf2_msgs/msg/TFMessage', mqtt);
    tf.start();
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
    log.info('ROS2-MQTT [TANK-ALL] Client is ready for RCL!!');
};

/**
 * async function for main runtime
 * @returns : Promise<void>
 */
(async function main(): Promise<void> {
    run()
    .then(() => welcome())
    .catch((err) => log.error(`ROS2-MQTT [TANK-ALL] Client has crashed by.. ${err} `));
})().catch((e): void => {
    log.error('overall_tankAll error : ', e);
    process.exitCode = 1
});