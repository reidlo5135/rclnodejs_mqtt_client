'strict mode';

import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../../mqtt/mqtt.infra';
import { log } from '../../common/common_logger.infra';
import { initPublish, publish } from '../../common/common_node.infra';

export default class LaserScanPublisher {

  private isRunning = false;
  private readonly node: rclnodejs.Node;
  private publisher: rclnodejs.Publisher<'sensor_msgs/msg/LaserScan'>;
  private publisherTimer: any;
  private mqtt:Mqtt;

  constructor(public readonly topic:string, mqtt:Mqtt) {
    this.node = new rclnodejs.Node('laser_scan_publisher');
    this.publisher = initPublish(this.node, 'sensor_msgs/msg/LaserScan', topic);
    this.mqtt = mqtt;
    this.node.spin();
  };

  start(interval = 1000): void {
    if (this.isRunning) return;

    this.isRunning = true;

    this.publisherTimer = this.node.createTimer(interval, () => {
      
    });
    let msg = this.genLaserScanMsg();
    publish('wavem/1/laser_frame', this.publisher, msg, this.mqtt);
  };

  stop() {
    this.publisherTimer.cancel();
    this.publisherTimer = null;
    this.isRunning = false;
  };

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
