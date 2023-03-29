import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
import { log } from '../common/common_logger.infra';

rclnodejs.init().then(() => {
    const node = new rclnodejs.Node('odom_subscriber_test');
    const mqtt: Mqtt = new Mqtt();

    node.createSubscription('nav_msgs/msg/Odometry', '/odom', (msg) => {
        log.info(`odom subsciber test msg : ${JSON.stringify(msg)}`);
        mqtt.publish(`wavem/1/odom`, JSON.stringify(msg));
    });

    node.spin();
});