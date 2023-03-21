'use strict';

import { Mqtt } from "../../mqtt/mqtt.infra";
import { node, subscribe } from "../common/common_node.infra";

export default function robot_pose_subscriber_node(mqtt:Mqtt) {
    subscribe(node('robot_pose_subscriber_node'), 'geometry_msgs/msg/Pose', '/robot_pose', mqtt);
};