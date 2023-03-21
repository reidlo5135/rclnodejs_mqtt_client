'use strict';

import { init } from "rclnodejs";
import { Mqtt } from "../../mqtt/mqtt.infra";

init().then(() => {
    const mqtt:Mqtt = new Mqtt();
    mqtt.subscribeForROSPublisher('wavem/1/joint_states');
});