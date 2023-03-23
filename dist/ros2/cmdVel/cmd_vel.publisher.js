"use strict";
'strict mode';
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
const rclnodejs = __importStar(require("rclnodejs"));
const common_logger_infra_1 = require("../common/common_logger.infra");
const common_node_infra_1 = require("../common/common_node.infra");
class CmdVelPublisher {
    constructor(topic, mqtt) {
        this.topic = topic;
        this.isRunning = false;
        this.node = new rclnodejs.Node('cmd_vel_publisher');
        this.publisher = (0, common_node_infra_1.initPublish)(this.node, 'geometry_msgs/msg/Twist', topic);
        this.mqtt = mqtt;
        this.node.spin();
    }
    ;
    start() {
        if (this.isRunning)
            return;
        this.isRunning = true;
        this.mqtt.subscribe('wavem/1/cmd_vel');
        this.mqtt.client.on("message", (topic, message) => {
            common_logger_infra_1.log.info(`RCL publish MQTT onMessage topic : ${topic}, message : ${message}`);
            let msg = this.genTwistMsg(message.toString());
            this.publisher.publish(msg);
        });
    }
    ;
    stop() {
        this.isRunning = false;
        this.mqtt.client.unsubscribe('wavem/1/cmd_vel');
    }
    ;
    genTwistMsg(message) {
        let twistMsg = rclnodejs.createMessageObject('geometry_msgs/msg/Twist');
        const twist = JSON.parse(message);
        twistMsg.linear = twist.linear;
        twistMsg.angular = twist.angular;
        return twistMsg;
    }
    ;
}
exports.default = CmdVelPublisher;
;
//# sourceMappingURL=cmd_vel.publisher.js.map