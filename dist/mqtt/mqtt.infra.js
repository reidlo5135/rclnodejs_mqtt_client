"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
const mqtt_1 = __importDefault(require("mqtt"));
const common_logger_infra_1 = require("../ros2/common/common_logger.infra");
/**
 * Class for MQTT Connections
 * @see Mqtt
 */
class Mqtt {
    constructor() {
        this.url = 'tcp://192.168.0.187:1883';
        this.client = mqtt_1.default.connect(this.url);
        this.onConnect();
    }
    ;
    onConnect() {
        this.client.on("connect", () => {
            if (!this.client.connected) {
                common_logger_infra_1.log.error('MQTT disconnected');
            }
        });
        this.client.on("error", (err) => {
            common_logger_infra_1.log.error(`MQTT Error Occurred Caused By ${err}`);
            process.exit(1);
        });
    }
    ;
    publish(topic, message) {
        // log.info(`MQTT publish topic : ${topic}`);
        this.client.publish(topic, message);
    }
    ;
    subscribe(topic) {
        this.client.subscribe(topic, function (err, granted) {
            if (err) {
                common_logger_infra_1.log.error(`RCL ${topic} subscribe Error Occurred Caused By ${err}`);
                return;
            }
            ;
        });
    }
    ;
    subscribeForROSPublisher(topic, publisher, msg) {
        common_logger_infra_1.log.info(`MQTT subscribeForROSPublisher topic : ${topic}`);
        this.client.subscribe(topic, function (err, granted) {
            if (err) {
                common_logger_infra_1.log.error(`MQTT subscribeForROSPublisher Error Occurred Caused By ${err}`);
                return;
            }
            ;
            common_logger_infra_1.log.info(`MQTT subscribeForROSPublisher ${typeof granted}`, granted);
        });
        this.client.on("message", (topic, message) => {
            common_logger_infra_1.log.info(`MQTT onMessage topic : ${topic}, message : ${message}`);
            publisher.publish(msg);
        });
    }
    ;
}
exports.default = Mqtt;
;
//# sourceMappingURL=mqtt.infra.js.map