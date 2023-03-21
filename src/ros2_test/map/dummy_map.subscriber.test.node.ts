'use strict';

import { init } from "rclnodejs";
import { Mqtt } from "../../mqtt/mqtt.infra";
import { node, subscribe } from "../../ros2/common/common_node.infra";

init().then(() => {
    const mqtt:Mqtt = new Mqtt();

    subscribe(node('dummy_map_subscriber_test_node'), 'nav_msgs/msg/OccupancyGrid', '/map', mqtt);
});