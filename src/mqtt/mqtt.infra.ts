import mqtt from 'mqtt';
import * as rclnodejs from 'rclnodejs';
import { log } from '../ros2/common/common_logger.infra';

/**
 * Class for MQTT Connections
 * @see Mqtt
 */
export default class Mqtt {
    url;
    client;

    constructor() {
        this.url = 'tcp://192.168.0.187:1883';
        this.client = mqtt.connect(this.url);
        this.onConnect();
    };

    private onConnect() {
        this.client.on("connect", () => {
            if(!this.client.connected) {
                log.error('MQTT disconnected');
            }
        });
        this.client.on("error", (err) => {
            log.error(`MQTT Error Occurred Caused By ${err}`);
            process.exit(1);
        });
    };

    public publish(topic:string, message:string) {
        // log.info(`MQTT publish topic : ${topic}`);
        this.client.publish(topic, message);
    };

    public subscribe(topic:string) {
        this.client.subscribe(topic, function(err, granted) {
            if (err) {
                log.error(`RCL ${topic} subscribe Error Occurred Caused By ${err}`);
                return;
            };
        });
    };

    public subscribeForROSPublisher(topic: string, publisher: rclnodejs.Publisher<any>, msg:any) {
        log.info(`MQTT subscribeForROSPublisher topic : ${topic}`);
        this.client.subscribe(topic, function(err, granted) {
            if (err) {
                log.error(`MQTT subscribeForROSPublisher Error Occurred Caused By ${err}`);
                return;
            };

            log.info(`MQTT subscribeForROSPublisher ${typeof granted}`, granted);
            
        });

        this.client.on("message", (topic, message) => {
            log.info(`MQTT onMessage topic : ${topic}, message : ${message}`);
            publisher.publish(msg);
        });
    };
};