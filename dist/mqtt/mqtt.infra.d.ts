import mqtt from 'mqtt';
/**
 * Class for MQTT Connections
 * @see Mqtt
 */
export declare class Mqtt {
    url: string;
    client: mqtt.MqttClient;
    constructor();
    private onConnect;
    publish(topic: string, message: string): void;
    subscribeForROSPublisher(topic: string): void;
    subscribeForServiceCall(topic: string): void;
}
//# sourceMappingURL=mqtt.infra.d.ts.map