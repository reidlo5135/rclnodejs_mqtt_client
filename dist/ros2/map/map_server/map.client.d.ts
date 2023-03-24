import Mqtt from '../../../mqtt/mqtt.infra';
export default class MapClient {
    private readonly msg_type;
    private readonly req_type;
    private readonly service;
    private isRunning;
    private readonly node;
    private readonly client;
    private readonly request;
    private readonly mqtt;
    constructor(msg_type: any, req_type: any, service: string, mqtt: Mqtt);
    call(): void;
}
//# sourceMappingURL=map.client.d.ts.map