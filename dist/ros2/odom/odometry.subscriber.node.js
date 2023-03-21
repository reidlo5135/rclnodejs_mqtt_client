'use strict';
Object.defineProperty(exports, "__esModule", { value: true });
const common_node_infra_1 = require("../common/common_node.infra");
function odometry_subscriber_node(mqtt) {
    (0, common_node_infra_1.subscribe)((0, common_node_infra_1.node)('odoemtry_subscriber_node'), 'nav_msgs/msg/Odometry', '/odom', mqtt);
}
exports.default = odometry_subscriber_node;
;
//# sourceMappingURL=odometry.subscriber.node.js.map