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
import { log } from '../../ros2/common/infra/common_logger.infra';

/**
 * Class for MQTT Connections
 * @author wavem-reidlo
 * @version 1.0.0
 * @since 2023.03.27
 * @see Mqtt
 */
export default class Mqtt {

    /**
     * field for MQTT url
     */
    url : string;

    /**
     * fiedl for this class instance
     */
    client : mqtt.MqttClient;

    /**
     * constructor for initialize url, client & invoke this#onConnect()
     * @see onConnect
     */
    constructor(url_target : string) {
        this.url = url_target;
        this.client = mqtt.connect(this.url);
        this.onConnect();
    };

    /**
     * private void function for connect MQTT & handle MQTT Connections' Error
     */
    private onConnect() : void {
        log.info(`[MQTT] connected with {${this.url}}`);
        this.client.on("connect", () => {
            if(!this.client.connected) {
                log.error('[MQTT] connection discarded');
            };
        });
        this.client.on("error", (err) => {
            log.error(`[MQTT] connection on ${err}`);
        });
    };

    /**
     * public void function for MQTT publishing
     * @param topic 
     * @param message
     * @see Mqtt
     * @see client 
     */
    public publish(topic : string, message : string) : void {
        try {
            this.client.publish(topic, message);
        } catch (error : any) {
            log.error(`[MQTT] publishing errror : ${error}`);
            throw new Error(error);
        };
    };

    /**
     * public void function for MQTT subscriptions
     * @param topic 
     * @see Mqtt
     * @see client
     */
    public subscribe(topic : string) : void {
        try {
            this.client.subscribe(topic, function(err, granted) {
                if (err) {
                    log.error(`[MQTT] ${topic} subscription on ${err}`);
                    return;
                };
                log.info(`[MQTT] subscription has granted by topic {${granted[0].topic}}`);
            });
        } catch (error : any) {
            log.error(`[MQTT] {${topic}} subscription : ${error}`);
            throw new Error(error);
        };
    };

    public unsubscribe(topic : string) : void {
        try {
            if(this.client.connected) {
                this.client.unsubscribe(topic);
                this.client.reconnect();
            };
        } catch (error : any) {
            log.error(`[MQTT] {${topic}} subscription : ${error}`);
            throw new Error(error);
        }
    };
};