'use strict';
Object.defineProperty(exports, "__esModule", { value: true });
const common_node_infra_1 = require("../common/common_node.infra");
function scan_subscriber_node(mqtt) {
    (0, common_node_infra_1.subscribe)((0, common_node_infra_1.node)('scan_subscriber_node'), 'sensor_msgs/msg/LaserScan', '/scan', mqtt);
}
exports.default = scan_subscriber_node;
;
//# sourceMappingURL=scan.subscriber.node.js.map