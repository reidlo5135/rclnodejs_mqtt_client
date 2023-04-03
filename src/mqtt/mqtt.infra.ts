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
        const url_jaraTwo = 'tcp://192.168.0.119:1883';
        const url_wavem = 'tcp://192.168.0.187:1883';
        const url_reidlo = 'tcp://192.168.0.132:1883';
        this.url = url_reidlo;
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
        try {
            this.client.publish(topic, message);
        } catch (error) {
            log.error(`MQTT publisher errror : ${error}`);
        }
    };

    /**
     * public void function for MQTT subscription
     * @param topic 
     * @see Mqtt
     * @see client
     */
    public async subscribe(topic:string): Promise<void> {
        this.client.subscribe(topic, function(err, granted) {
            if (err) {
                log.error(`MQTT ${topic} subscribe Error Occurred Caused By ${err}`);
                return;
            };
            log.info(`MQTT subscribe granted : ${granted[0].topic}`);
        });
    };

    /**
     * public void function for MQTT subscription & ROS Publishing
     * @param topic 
     * @see Mqtt
     * @see client
     * @see rclnodejs.Publisher<any>
     */
    public subscribeForROSPublisher(topic: string, rosPublisher: rclnodejs.Publisher<any>) {
        log.info(`MQTT subscribeForROSPublisher topic : ${topic}`);
        if(topic.includes('pub')) {
            const parsedTopic = topic.split('/')[1];
            this.subscribe(parsedTopic);
            this.client.on("message", (mqttTopic, mqttMessage, packet) => {
                log.info(`MQTT onMessage topic : ${mqttTopic}, message : ${mqttMessage}`);
                // log.info(`MQTT publish is Equal ${topic.includes(mqttTopic)}`);
                if(topic.includes(mqttTopic)) {
                    rosPublisher.publish(mqttMessage);
                } else return;
            });
        } else return;
    };
};