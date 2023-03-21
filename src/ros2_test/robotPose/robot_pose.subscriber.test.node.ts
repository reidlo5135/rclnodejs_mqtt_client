'use strict';

import { init } from "rclnodejs";
import { Mqtt } from "../../mqtt/mqtt.infra";
import { node, subscribe } from "../../ros2/common/common_node.infra";

init().then(() => {
    const mqtt:Mqtt = new Mqtt();

    subscribe(node('robot_pose_subscriber_test_node'), 'geometry_msgs/msg/Pose', '/robot_pose', mqtt);
});