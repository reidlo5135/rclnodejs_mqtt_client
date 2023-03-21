'use strict';

import { Mqtt } from '../../mqtt/mqtt.infra';
import { node, subscribe } from '../common/common_node.infra';

export default function odometry_subscriber_node(mqtt:Mqtt) {
    subscribe(node('odoemtry_subscriber_node'), 'nav_msgs/msg/Odometry', '/odom', mqtt);
};