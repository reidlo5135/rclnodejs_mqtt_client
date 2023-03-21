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
exports.clientForMap = exports.subscribe = exports.publish = exports.node = void 0;
const rclnodejs = __importStar(require("rclnodejs"));
const common_logger_infra_1 = require("./common_logger.infra");
function node(name) {
    const node = new rclnodejs.Node(name);
    common_logger_infra_1.log.info(`${name} running...`);
    return node;
}
exports.node = node;
;
function publish(topic, type) {
    const topArr = topic.split('/');
    common_logger_infra_1.log.info(`RCL publish message type : ${type}, topic : ${topArr[2]} `);
    const node = new rclnodejs.Node(topArr[2] + '_publisher_node');
    const publisher = node.createPublisher(type, topArr[2]);
    console.log(`Publishing message: Hello ROS`);
    publisher.publish({
        header: {
            stamp: {
                sec: 123456,
                nanosec: 789,
            },
            frame_id: 'main frame',
        },
        name: ['Tom', 'Jerry'],
        position: [1, 2],
        velocity: [2, 3],
        effort: [4, 5, 6],
    });
    node.spin();
    destroyDuplicateNode(node);
}
exports.publish = publish;
;
function subscribe(node, type, topic, mqtt) {
    common_logger_infra_1.log.info(`RCL subscription message type : ${type}, topic : ${topic} `);
    node.createSubscription(type, topic, (msg) => {
        // log.info(`topic ${topic} result msg : ${JSON.stringify(msg)}`);
        mqtt.publish(`wavem/1${topic}`, JSON.stringify(msg));
    });
    node.spin();
}
exports.subscribe = subscribe;
;
function clientForMap(node, msg_type, req_type, service, mqtt) {
    common_logger_infra_1.log.info(`RCL clientForMap msg_type : ${msg_type}, service : ${service}`);
    const client = node.createClient(msg_type, service);
    const request = rclnodejs.createMessageObject(req_type);
    client.waitForService(1000).then((result) => {
        if (!result) {
            common_logger_infra_1.log.error(`${service} is not available...`);
            rclnodejs.shutdown();
            return;
        }
        ;
        common_logger_infra_1.log.info(`[INFO] sending to ${service} with ${typeof request}`, request);
        client.sendRequest(request, (response) => {
            // log.info(`[INFO] ${service} service call response result : ${typeof response}`, response);
            common_logger_infra_1.log.info(`${service} service call is null? `, response === null);
            const buffer = Buffer.from(response.map.data);
            // const buffer = response.map.data;
            // log.info(`[INFO] ${service} buffer : ` , buffer);
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
function destroyDuplicateNode(node) {
    if (node.getNodeNames().includes(node.name())) {
        node.destroy();
    }
    ;
}
//# sourceMappingURL=common_node.infra.js.map