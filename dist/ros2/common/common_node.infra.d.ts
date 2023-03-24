import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
export declare function initPublish(node: rclnodejs.Node, type: any, topic: string): rclnodejs.Publisher<any>;
export declare function publish(topic: string, publisher: rclnodejs.Publisher<any>, msg: any, mqtt: Mqtt): void;
export declare function subscribe(node: rclnodejs.Node, type: any, topic: string, mqtt: Mqtt): rclnodejs.Subscription;
export declare function client(node: rclnodejs.Node, msg_type: any, service: string): rclnodejs.Client<any>;
//# sourceMappingURL=common_node.infra.d.ts.map