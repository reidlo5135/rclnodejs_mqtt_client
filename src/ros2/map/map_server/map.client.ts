'strict mode';

import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../../mqtt/mqtt.infra';
import { log } from '../../common/common_logger.infra';
import { client } from '../../common/common_node.infra';

export default class MapClient {
    private isRunning = false;
    private readonly node: rclnodejs.Node;
    private readonly client: rclnodejs.Client<any>;
    private readonly request: rclnodejs.Message;
    private readonly mqtt:Mqtt;
    
    constructor(private readonly msg_type:any, private readonly req_type:any, private readonly service:string, mqtt:Mqtt) {
        this.node = new rclnodejs.Node('map_client', 'map_server');
        this.client = client(this.node, msg_type, service);
        this.request = rclnodejs.createMessageObject(req_type);
        this.mqtt = mqtt;
        this.node.spin();
    };

    call(): void {
        this.mqtt.subscribe('wavem/1/requestMap');
        this.mqtt.client.on('message', (topic, message) => {
            log.info(`RCL call MQTT onMessage topic : ${topic}, message : ${message}`);
            if(topic.includes('requestMap')) {
                this.client.waitForService(1000)
                    .then((result) => {
                        if(!result) {
                            log.error(`RCL ${this.service} is not available...`);
                            rclnodejs.shutdown();
                            return;
                        };

                        log.info(`RCL call ${this.service}`);
                        this.client.sendRequest(this.request, (response) => {
                            if(response === null) log.info(`RCL call ${this.service} service call is null `);
                            
                            const buffer = Buffer.from(response.map.data);
                
                            const bmpData = {
                                width: response.map.info.width,
                                height: response.map.info.height
                            };
                            
                            const data = JSON.stringify(bmpData) + '/' + buffer.toString('hex');
                            this.mqtt.publish(`wavem/1${this.service}`, data);
                        });
                    })
                    .catch((e) => {
                        log.error(`RCL client call ${this.service} error occurred : ${JSON.stringify(e)}`);
                    });
            } else return;
        });
    };
};