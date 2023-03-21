"use strict";
'strict mode';
Object.defineProperty(exports, "__esModule", { value: true });
const common_node_infra_1 = require("../common/common_node.infra");
class ImuDataSubscriber {
    constructor(node, topic, type, mqtt) {
        this.node = node;
        this.topic = topic;
        this.type = type;
        this.isRunning = false;
        this.isRunning = true;
        this.subscriber = (0, common_node_infra_1.subscribe)(this.node, this.type, this.topic, mqtt);
        this.node.spin();
    }
    ;
}
exports.default = ImuDataSubscriber;
;
//# sourceMappingURL=imu_data.subscriber.js.map