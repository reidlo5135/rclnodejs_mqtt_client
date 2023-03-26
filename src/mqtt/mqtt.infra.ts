import mqtt from 'mqtt';
import * as rclnodejs from 'rclnodejs';
import { log } from '../ros2/common/common_logger.infra';

/**
 * Class for MQTT Connections
 * @see Mqtt
 */
export default class Mqtt {

    /**
     * field for MQTT url
     */
    url;

    /**
     * fiedl for this class instance
     */
    client;

    /**
     * constructor for initialize url, client & invoke this#onConnect()
     * @see onConnect
     */
    constructor() {
        this.url = 'tcp://192.168.0.187:1883';
        this.client = mqtt.connect(this.url);
        this.onConnect();
    };

    /**
     * private void function for connect MQTT & handle MQTT Connections' Error
     */
    private onConnect() {
        this.client.on("connect", () => {
            if(!this.client.connected) {
                log.error('MQTT disconnected');
            };
        });
        this.client.on("error", (err) => {
            log.error(`MQTT Error Occurred Caused By ${err}`);
            process.exit(1);
        });
    };

    /**
     * public void function for MQTT publishing
     * @param topic 
     * @param message
     * @see Mqtt
     * @see client 
     */
    public publish(topic:string, message:string) {
        this.client.publish(topic, message);
    };

    /**
     * public void function for MQTT subscription
     * @param topic 
     * @see Mqtt
     * @see client
     */
    public subscribe(topic:string) {
        this.client.subscribe(topic, function(err, granted) {
            if (err) {
                log.error(`RCL ${topic} subscribe Error Occurred Caused By ${err}`);
                return;
            };
        });
    };

    /**
     * public void function for MQTT subscription & ROS Publishing
     * @param topic 
     * @see Mqtt
     * @see client
     * @see rclnodejs.Publisher
     */
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