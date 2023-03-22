'use strict';

import * as rclnodejs from 'rclnodejs';
import { log } from './common_logger.infra';
import Mqtt from '../../mqtt/mqtt.infra';

export function initPublish(node:rclnodejs.Node, type:any, topic:string) : rclnodejs.Publisher<any> {
    log.info(`RCL init publish message type : ${type}, topic : ${topic}`);
    return node.createPublisher(type, topic);
};

export function publish(topic:string, publisher:rclnodejs.Publisher<any>, msg:any, mqtt:Mqtt)  {
    log.info(`RCL publish MQTT topic : ${topic}`);
    mqtt.client.subscribe(topic, function(err, granted) {
        if (err) {
            log.error(`RCL publish subscribe Error Occurred Caused By ${err}`);
            return;
        };
        log.info(`RCL publish MQTT subscribe ${typeof granted}`, granted);
    });

    mqtt.client.on("message", (topic, message) => {
        log.info(`RCL publish MQTT onMessage topic : ${topic}, message : ${message}`);
        publisher.publish(msg);
    });
};

export function subscribe(node:rclnodejs.Node, type:any, topic:string, mqtt:Mqtt) : rclnodejs.Subscription {
    log.info(`RCL subscription message type : ${type}, topic : ${topic} `);
    
    const subscription = node.createSubscription(type, topic, (msg) => {
        mqtt.publish(`wavem/1${topic}`, JSON.stringify(msg));
    });

    return subscription;
};

export function clientForMap(msg_type:any, req_type:any, service:string, mqtt:Mqtt) {
    log.info(`RCL clientForMap msg_type : ${msg_type}, service : ${service}`);
    
    const node = new rclnodejs.Node('map_server_map_client_test');
    const client = node.createClient(msg_type, service);
    const request = rclnodejs.createMessageObject(req_type);

    client.waitForService(1000).then((result) => {
        if(!result) {
            log.error(`${service} is not available...`);
            rclnodejs.shutdown();
            return;
        };

        log.info(`[INFO] sending to ${service} with ${typeof request}`, request);
        client.sendRequest(request, (response) => {
            log.info(`${service} service call is null? `, response === null);
            
            const buffer = Buffer.from(response.map.data);

            const bmpData = {
                width: response.map.info.width,
                height: response.map.info.height
            };
            
            const data = JSON.stringify(bmpData) + '/' + buffer.toString('hex');
            mqtt.publish(`wavem/1${service}`, data);
        });
            
        node.spin();

        if(node.getNodeNames().includes(node.name())) {
            node.destroy();
        };
    }).catch((e) => {
        log.error(`${node.name} error occurred : ${e}`);
    });
};