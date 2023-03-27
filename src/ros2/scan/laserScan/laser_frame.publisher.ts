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

'strict mode';

import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../../mqtt/mqtt.infra';
import { Publisher } from '../../common/common_ndoe.interface';
import { initPublish } from '../../common/common_node.infra';

/**
 * Class for ROS2 publish /laser_frame topic to robot
 * @see rclnodejs.Publisher
 */
export default class LaserScanPublisher implements Publisher {

  /**
   * private boolean field for this node is running or not
   */
  private isRunning = false;

  /**
   * private readonly field for rclnodejs.Node instance
   */
  private readonly node: rclnodejs.Node;


  /**
   * private field for publishTimer
   * @see rclnodejs.publisherTimer
   */
  private publisherTimer: any;

  /**
   * private readonly field for Mqtt
   * @see Mqtt
   */
  private readonly mqtt:Mqtt;

  /**
   * constructor for initialize field instances
   * @see node
   * @see publisher
   * @see mqtt
   * @param topic : string
   * @param mqtt : Mqtt
   */
  constructor(private readonly topic:string, mqtt:Mqtt) {
    this.node = new rclnodejs.Node('laser_scan_publisher');
    this.mqtt = mqtt;
  };

  /**
   * void function for ROS2 publishing loop
   * @see genLaserScanMsg
   * @see publisherTimer
   * @see createTimer
   * @see initPublish
   * @param interval : number
   */
  start(interval = 1000): void {
    if (this.isRunning) return;

    this.isRunning = true;

    const publisher = initPublish(this.node, 'sensor_msgs/msg/LaserScan', this.topic);
    this.node.spin();

    let msg = this.genLaserScanMsg();
    this.publisherTimer = this.node.createTimer(interval, () => {
      publisher.publish(msg);
    });
  };

  /**
   * void function for cancel publisher timer
   */
  stop(): void {
    this.publisherTimer.cancel();
    this.publisherTimer = null;
    this.isRunning = false;
    this.node.destroy();
  };

  /**
   * protected function for generate ROS2 publishing message object
   * @see rclnodejs.sensor_msgs.msg.LaserScan
   * @param range : number
   * @returns laserScanMsg : rclnodejs.sensor_msgs.msg.LaserScan
   */
  protected genLaserScanMsg(range = 10): rclnodejs.sensor_msgs.msg.LaserScan {
    let laserScanMsg = rclnodejs.createMessageObject('sensor_msgs/msg/LaserScan') as rclnodejs.sensor_msgs.msg.LaserScan;

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
  };
};
