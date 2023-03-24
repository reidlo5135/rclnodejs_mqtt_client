'use strict';
Object.defineProperty(exports, "__esModule", { value: true });
exports.client = exports.subscribe = exports.publish = exports.initPublish = void 0;
const common_logger_infra_1 = require("./common_logger.infra");
function initPublish(node, type, topic) {
    common_logger_infra_1.log.info(`RCL init publish message type : ${type}, topic : ${topic}`);
    return node.createPublisher(type, topic);
}
exports.initPublish = initPublish;
;
function publish(topic, publisher, msg, mqtt) {
    common_logger_infra_1.log.info(`RCL publish MQTT topic : ${topic}`);
    mqtt.client.subscribe(topic, function (err, granted) {
        if (err) {
            common_logger_infra_1.log.error(`RCL publish subscribe Error Occurred Caused By ${err}`);
            return;
        }
        ;
    });
    // mqtt.client.on("message", (topic, message) => {
    //     log.info(`RCL publish MQTT onMessage topic : ${topic}, message : ${message}`);
    //     publisher.publish(msg);
    // });
}
exports.publish = publish;
;
function subscribe(node, type, topic, mqtt) {
    common_logger_infra_1.log.info(`RCL subscription message type : ${type}, topic : ${topic} `);
    const subscription = node.createSubscription(type, topic, (msg) => {
        if (msg === null || msg === '')
            common_logger_infra_1.log.error(`RCL ${topic} subscription has return empty message`);
        mqtt.publish(`wavem/1${topic}`, JSON.stringify(msg));
    });
    return subscription;
}
exports.subscribe = subscribe;
;
function client(node, msg_type, service) {
    common_logger_infra_1.log.info(`RCL client msg_type : ${msg_type}, service : ${service}`);
    return node.createClient(msg_type, service);
}
exports.client = client;
;
//# sourceMappingURL=common_node.infra.js.map