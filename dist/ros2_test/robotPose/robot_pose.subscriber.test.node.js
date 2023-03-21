'use strict';
Object.defineProperty(exports, "__esModule", { value: true });
const rclnodejs_1 = require("rclnodejs");
const mqtt_infra_1 = require("../../mqtt/mqtt.infra");
const common_node_infra_1 = require("../../ros2/common/common_node.infra");
(0, rclnodejs_1.init)().then(() => {
    const mqtt = new mqtt_infra_1.Mqtt();
    (0, common_node_infra_1.subscribe)((0, common_node_infra_1.node)('robot_pose_subscriber_test_node'), 'geometry_msgs/msg/Pose', '/robot_pose', mqtt);
});
//# sourceMappingURL=robot_pose.subscriber.test.node.js.map