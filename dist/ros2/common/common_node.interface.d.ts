import * as rclnodejs from 'rclnodejs';
export interface Subscriber {
    isRunning: boolean;
    subscriber: rclnodejs.Subscription;
}
export interface Publisher {
    isRunning: boolean;
    publisher: rclnodejs.Publisher<any>;
    timer: rclnodejs.Timer;
    start(): void;
    stop(): void;
}
//# sourceMappingURL=common_node.interface.d.ts.map