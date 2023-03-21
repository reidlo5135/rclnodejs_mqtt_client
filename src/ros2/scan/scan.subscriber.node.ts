'use strict';

import { Mqtt } from '../../mqtt/mqtt.infra';
import { node, subscribe } from '../common/common_node.infra';

export default function scan_subscriber_node(mqtt:Mqtt) {
    subscribe(node('scan_subscriber_node'), 'sensor_msgs/msg/LaserScan', '/scan', mqtt);
};