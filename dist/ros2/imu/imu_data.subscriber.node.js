'use strict';
Object.defineProperty(exports, "__esModule", { value: true });
const common_node_infra_1 = require("../common/common_node.infra");
function imu_data_subscriber_node(mqtt) {
    (0, common_node_infra_1.subscribe)((0, common_node_infra_1.node)('imu_data_subscriber_node'), 'sensor_msgs/msg/Imu', '/imu/data', mqtt);
}
exports.default = imu_data_subscriber_node;
;
//# sourceMappingURL=imu_data.subscriber.node.js.map