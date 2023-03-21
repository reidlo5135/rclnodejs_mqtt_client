'use strict';

import { Mqtt } from '../../mqtt/mqtt.infra';
import { node, subscribe } from '../common/common_node.infra';

export default function dummy_map_subscriber_node(mqtt:Mqtt) {
     subscribe(node('dummy_map_subscriber_node'), 'nav_msgs/msg/OccupancyGrid', '/map', mqtt);
};
