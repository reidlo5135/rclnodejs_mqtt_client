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
const common_logger_infra_1 = require("../../common/common_logger.infra");
const common_node_infra_1 = require("../../common/common_node.infra");
class MapClient {
    constructor(msg_type, req_type, service, mqtt) {
        this.msg_type = msg_type;
        this.req_type = req_type;
        this.service = service;
        this.isRunning = false;
        this.node = new rclnodejs.Node('map_client', 'map_server');
        this.client = (0, common_node_infra_1.client)(this.node, msg_type, service);
        this.request = rclnodejs.createMessageObject(req_type);
        this.mqtt = mqtt;
        this.node.spin();
    }
    ;
    call() {
        this.mqtt.subscribe('wavem/1/requestMap');
        this.mqtt.client.on('message', (topic, message) => {
            common_logger_infra_1.log.info(`RCL call MQTT onMessage topic : ${topic}, message : ${message}`);
            if (topic.includes('requestMap')) {
                this.client.waitForService(1000)
                    .then((result) => {
                    if (!result) {
                        common_logger_infra_1.log.error(`RCL ${this.service} is not available...`);
                        rclnodejs.shutdown();
                        return;
                    }
                    ;
                    common_logger_infra_1.log.info(`RCL call ${this.service}`);
                    this.client.sendRequest(this.request, (response) => {
                        if (response === null)
                            common_logger_infra_1.log.info(`RCL call ${this.service} service call is null `);
                        const buffer = Buffer.from(response.map.data);
                        const bmpData = {
                            width: response.map.info.width,
                            height: response.map.info.height
                        };
                        const data = JSON.stringify(bmpData) + '/' + buffer.toString('hex');
                        this.mqtt.publish(`wavem/1${this.service}`, data);
                    });
                })
                    .catch((e) => {
                    common_logger_infra_1.log.error(`RCL client call ${this.service} error occurred : ${JSON.stringify(e)}`);
                });
            }
            else
                return;
        });
    }
    ;
}
exports.default = MapClient;
;
//# sourceMappingURL=map.client.js.map