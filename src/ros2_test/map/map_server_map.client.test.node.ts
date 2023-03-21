'use strict';

import { init } from "rclnodejs";
import { Mqtt } from "../../mqtt/mqtt.infra";

init().then(() => {
    const mqtt:Mqtt = new Mqtt();
    mqtt.subscribeForServiceCall('wavem/1/requestMap');
});