'strict mode';

import * as rclnodejs from 'rclnodejs';
import Mqtt from '../../mqtt/mqtt.infra';
import { initPublish, publish } from '../common/common_node.infra';

/**
 * Class for ROS2 publish /joint_states topic to robot
 * @see rclnodejs.Publisher
 */
export default class JointStatesPublisher {
  
    /**
     * private boolean filed for this node is running or not
     */
    private isRunning = false;

    /**
     * private readonly field for rclnodejs.Node instance
     * @see rclnodejs.Node
     */
    private readonly node: rclnodejs.Node;

    /**
     * private readonly field for rclnodejs.Publisher<'sensor_msgs/msg/JointState'> instance
     * @see rclnodejs.Publisher
     * @see sensor_msgs/msg/JointState
     */
    private readonly publisher: rclnodejs.Publisher<'sensor_msgs/msg/JointState'>;

    /**
     * private readonly field for Mqtt
     * @see Mqtt
     */
    private mqtt: Mqtt;

  
  /**
   * constructor for initialize field instances
   * @see node
   * @see publisher
   * @see mqtt
   * @see initPublish
   * @param topic : string
   * @param mqtt : Mqtt
   */
  constructor(private readonly topic:string, mqtt:Mqtt) {
    this.node = new rclnodejs.Node('joint_states_publisher');
    this.publisher = initPublish(this.node, 'sensor_msgs/msg/JointState', topic);
    this.mqtt = mqtt;
    this.node.spin();
  };

  /**
   * void function for MQTT subscription & ROS2 publish by recieved MQTT message
   * @see mqtt
   * @see publihser
   */
  start(): void {
    if (this.isRunning) return;

    this.isRunning = true;

    let msg = this.genJointStatesMsg();
    publish('wavem/1/joint_states', this.publisher, msg, this.mqtt);
  };

  /**
   * void function for stop ndoe spinning & MQTT unsubscribe
   */
  stop(): void {
    this.isRunning = false;
    this.mqtt.client.unsubscribe('wavem/1/joint_states');
  };

  /**
   * protected function for generate ROS2 publishing message object
   * @see rclnodejs.sensor_msgs.msg.JointState
   * @returns jointSateMsg : rclnodejs.sensor_msgs.msg.JointState
   */
  protected genJointStatesMsg(): rclnodejs.sensor_msgs.msg.JointState {
    let jointStateMsg = rclnodejs.createMessageObject('sensor_msgs/msg/JointState') as rclnodejs.sensor_msgs.msg.JointState;

    jointStateMsg.header.frame_id = 'joint_states';
    jointStateMsg.header.stamp = this.node.now().toMsg();

    return jointStateMsg;
  };
};