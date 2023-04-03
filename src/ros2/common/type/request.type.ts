import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../../mqtt/mqtt.infra';

export type requestType = {
    pub: string
    sub: string
    action: string
    service: string
};

export type rclType = {
    rcl: string
    node: rclnodejs.Node
    messageType: any
    name: string
};