'use strict';

import { init } from "rclnodejs";
import { Mqtt } from "../../mqtt/mqtt.infra";
import { node, subscribe } from "../../ros2/common/common_node.infra";

init().then(() => {
    const mqtt:Mqtt = new Mqtt();

    subscribe(node('imu_data_subscriber_test_node'), 'sensor_msgs/msg/Imu', '/imu/data', mqtt);
});