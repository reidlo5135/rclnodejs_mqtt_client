'use strict';
Object.defineProperty(exports, "__esModule", { value: true });
const common_node_infra_1 = require("../common/common_node.infra");
function robot_pose_subscriber_node(mqtt) {
    (0, common_node_infra_1.subscribe)((0, common_node_infra_1.node)('robot_pose_subscriber_node'), 'geometry_msgs/msg/Pose', '/robot_pose', mqtt);
}
exports.default = robot_pose_subscriber_node;
;
//# sourceMappingURL=robot_pose.subscriber.node.js.map