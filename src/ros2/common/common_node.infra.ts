'use strict';

import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
import { log } from './common_logger.infra';

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
    });
};

export function subscribe(node:rclnodejs.Node, type:any, topic:string, mqtt:Mqtt) : rclnodejs.Subscription {
    log.info(`RCL subscription message type : ${type}, topic : ${topic} `);
    
    const subscription = node.createSubscription(type, topic, (msg) => {
        if(msg === null || msg === '') log.error(`RCL ${topic} subscription has return empty message`);
        mqtt.publish(`wavem/1${topic}`, JSON.stringify(msg));
    });

    return subscription;
};

export function client(node:rclnodejs.Node, msg_type:any, service:string) : rclnodejs.Client<any> {
    log.info(`RCL client msg_type : ${msg_type}, service : ${service}`);

    return node.createClient(msg_type, service);
};