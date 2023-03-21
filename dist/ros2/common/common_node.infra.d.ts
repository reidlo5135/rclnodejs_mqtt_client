import * as rclnodejs from 'rclnodejs';
import { Mqtt } from '../../mqtt/mqtt.infra';
export declare function node(name: string): rclnodejs.Node;
export declare function publish(topic: string, type: any): void;
export declare function subscribe(node: rclnodejs.Node, type: any, topic: string, mqtt: Mqtt): void;
export declare function clientForMap(node: rclnodejs.Node, msg_type: any, req_type: any, service: string, mqtt: Mqtt): void;
//# sourceMappingURL=common_node.infra.d.ts.map