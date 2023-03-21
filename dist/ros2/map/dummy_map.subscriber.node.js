'use strict';
Object.defineProperty(exports, "__esModule", { value: true });
const common_node_infra_1 = require("../common/common_node.infra");
function dummy_map_subscriber_node(mqtt) {
    (0, common_node_infra_1.subscribe)((0, common_node_infra_1.node)('dummy_map_subscriber_node'), 'nav_msgs/msg/OccupancyGrid', '/map', mqtt);
}
exports.default = dummy_map_subscriber_node;
;
//# sourceMappingURL=dummy_map.subscriber.node.js.map