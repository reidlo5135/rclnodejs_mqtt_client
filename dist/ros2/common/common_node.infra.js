'use strict';
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || function (mod) {
    if (mod && mod.__esModule) return mod;
    var result = {};
    if (mod != null) for (var k in mod) if (k !== "default" && Object.prototype.hasOwnProperty.call(mod, k)) __createBinding(result, mod, k);
    __setModuleDefault(result, mod);
    return result;
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.clientForMap = exports.subscribe = exports.publish = exports.initPublish = void 0;
const rclnodejs = __importStar(require("rclnodejs"));
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
        mqtt.publish(`wavem/1${topic}`, JSON.stringify(msg));
    });
    return subscription;
}
exports.subscribe = subscribe;
;
function clientForMap(msg_type, req_type, service, mqtt) {
    common_logger_infra_1.log.info(`RCL clientForMap msg_type : ${msg_type}, service : ${service}`);
    const node = new rclnodejs.Node('map_server_map_client_test');
    const client = node.createClient(msg_type, service);
    const request = rclnodejs.createMessageObject(req_type);
    client.waitForService(1000).then((result) => {
        if (!result) {
            common_logger_infra_1.log.error(`RCL ${service} is not available...`);
            rclnodejs.shutdown();
            return;
        }
        ;
        common_logger_infra_1.log.info(`RCL sending to ${service} with ${typeof request}`, request);
        client.sendRequest(request, (response) => {
            common_logger_infra_1.log.info(`${service} service call is null? `, response === null);
            const buffer = Buffer.from(response.map.data);
            const bmpData = {
                width: response.map.info.width,
                height: response.map.info.height
            };
            const data = JSON.stringify(bmpData) + '/' + buffer.toString('hex');
            mqtt.publish(`wavem/1${service}`, data);
        });
        node.spin();
        if (node.getNodeNames().includes(node.name())) {
            node.destroy();
        }
        ;
    }).catch((e) => {
        common_logger_infra_1.log.error(`${node.name} error occurred : ${e}`);
    });
}
exports.clientForMap = clientForMap;
;
//# sourceMappingURL=common_node.infra.js.map