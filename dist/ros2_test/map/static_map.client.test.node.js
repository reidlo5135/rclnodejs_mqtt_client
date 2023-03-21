'use strict';
Object.defineProperty(exports, "__esModule", { value: true });
const rclnodejs_1 = require("rclnodejs");
const mqtt_infra_1 = require("../../mqtt/mqtt.infra");
(0, rclnodejs_1.init)().then(() => {
    const mqtt = new mqtt_infra_1.Mqtt();
    mqtt.subscribeForServiceCall('wavem/1/requestMap');
});
//# sourceMappingURL=static_map.client.test.node.js.map