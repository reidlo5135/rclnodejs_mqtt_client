import mqtt from 'mqtt';
import * as rclnodejs from 'rclnodejs';
/**
 * Class for MQTT Connections
 * @see Mqtt
 */
export default class Mqtt {
    url: string;
    client: mqtt.MqttClient;
    constructor();
    private onConnect;
    publish(topic: string, message: string): void;
    subscribe(topic: string): void;
    subscribeForROSPublisher(topic: string, publisher: rclnodejs.Publisher<any>, msg: any): void;
    subscribeForServiceCall(topic: string): void;
}
//# sourceMappingURL=mqtt.infra.d.ts.map