'strict mode';

import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
import { initPublish, publish } from '../common/common_node.infra';

export default class JointStatesPublisher {
  
    private isRunning = false;
    private readonly node: rclnodejs.Node;
    private readonly publisher: rclnodejs.Publisher<'sensor_msgs/msg/JointState'>;
    private mqtt: Mqtt;

  constructor(private readonly topic:string, mqtt:Mqtt) {
    this.node = new rclnodejs.Node('joint_states_publisher');
    this.publisher = initPublish(this.node, 'sensor_msgs/msg/JointState', topic);
    this.mqtt = mqtt;
    this.node.spin();
  };

  start(): void {
    if (this.isRunning) return;

    this.isRunning = true;

    let msg = this.genJointStatesMsg();
    publish('wavem/1/joint_states', this.publisher, msg, this.mqtt);
  };

  stop(): void {
    this.isRunning = false;
    this.mqtt.client.unsubscribe('wavem/1/joint_states');
  };

  protected genJointStatesMsg(range = 10): rclnodejs.sensor_msgs.msg.JointState {
    let jointStateMsg = rclnodejs.createMessageObject('sensor_msgs/msg/JointState') as rclnodejs.sensor_msgs.msg.JointState;

    jointStateMsg.header.frame_id = 'joint_states';
    jointStateMsg.header.stamp = this.node.now().toMsg();

    return jointStateMsg;
  };
};