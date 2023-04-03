// Copyright [2023] [wavem-reidlo]
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
    public subscribeForROSPublisher(topic: string, ros_publisher: rclnodejs.Publisher<any>) {
        log.info(`MQTT subscribeForROSPublisher topic : ${topic}`);
        this.client.subscribe(topic, function(err, granted) {
            if (err) {
                log.error(`MQTT subscribeForROSPublisher Error Occurred Caused By ${err}`);
                return;
            };

            log.info(`MQTT subscribeForROSPublisher ${typeof granted}`, granted);
            
        });

        this.client.on("message", (mqttTopic, mqtt_message) => {
            log.info(`MQTT onMessage topic : ${mqttTopic}, message : ${mqtt_message}`);
            if(topic.includes(mqttTopic)) {
                ros_publisher.publish(mqtt_message);
            } else return;
        });
    };
};