'use strict';

import * as rclnodejs from 'rclnodejs';
import {log} from './common_logger.infra';
import { Mqtt } from '../../mqtt/mqtt.infra';

export function node(name:string) {
    const node = new rclnodejs.Node(name);
    log.info(`${name} running...`);
    
    return node;
};

export function publish(topic:string, type:any) {
    const topArr = topic.split('/');
    log.info(`RCL publish message type : ${type}, topic : ${topArr[2]} `);

    const node = new rclnodejs.Node(topArr[2] + '_publisher_node');
    const publisher = node.createPublisher(type, topArr[2]);
    console.log(`Publishing message: Hello ROS`);

    publisher.publish({
        header: {
        stamp: {
            sec: 123456,
            nanosec: 789,
        },
        frame_id: 'main frame',
        },
        name: ['Tom', 'Jerry'],
        position: [1, 2],
        velocity: [2, 3],
        effort: [4, 5, 6],
    }); 

    node.spin();
    destroyDuplicateNode(node);    
};

export function subscribe(node:rclnodejs.Node, type:any, topic:string, mqtt:Mqtt) {
    log.info(`RCL subscription message type : ${type}, topic : ${topic} `);
    
    node.createSubscription(type, topic, (msg) => {
        // log.info(`topic ${topic} result msg : ${JSON.stringify(msg)}`);
        mqtt.publish(`wavem/1${topic}`, JSON.stringify(msg));
    });

    node.spin();
};

export function clientForMap(node:rclnodejs.Node, msg_type:any, req_type:any, service:string, mqtt:Mqtt) {
    log.info(`RCL clientForMap msg_type : ${msg_type}, service : ${service}`);
    
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
            // log.info(`[INFO] ${service} service call response result : ${typeof response}`, response);
            log.info(`${service} service call is null? `, response === null);
            
            const buffer = Buffer.from(response.map.data);
            // const buffer = response.map.data;
            // log.info(`[INFO] ${service} buffer : ` , buffer);

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

function destroyDuplicateNode(node:rclnodejs.Node) {
    if(node.getNodeNames().includes(node.name())) {
        node.destroy();
    };
}