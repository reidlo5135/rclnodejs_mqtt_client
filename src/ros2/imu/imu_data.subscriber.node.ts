'use strict';

import { Mqtt } from '../../mqtt/mqtt.infra';
import { node, subscribe } from '../common/common_node.infra';

export default function imu_data_subscriber_node(mqtt:Mqtt) {
    subscribe(node('imu_data_subscriber_node'), 'sensor_msgs/msg/Imu', '/imu/data', mqtt);
};