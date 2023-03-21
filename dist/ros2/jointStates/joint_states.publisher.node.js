"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
function join_states_publisher_node(mqtt) {
    mqtt.subscribeForROSPublisher('wavem/1/joint_states');
}
exports.default = join_states_publisher_node;
//# sourceMappingURL=joint_states.publisher.node.js.map