# ROS2-MQTT Client - For ROS2-MQTT connection

[![npm-rclnodejs](https://img.shields.io/npm/v/rclnodejs.svg)](https://www.npmjs.com/package/rclnodejs)
[![node](https://img.shields.io/node/v/rclnodejs.svg)](https://nodejs.org/en/download/releases/)
[![npm type definitions](https://img.shields.io/npm/types/rclnodejs)](https://www.npmjs.com/package/rclnodejs)

## Document
- [ROS2-MQTT Client - For ROS2-MQTT connection](#ros2-mqtt-client---for-ros2-mqtt-connection)
  - [Document](#document)
  - [Installation](#installation)
    - [Prerequisites](#prerequisites)
    - [Installing node(nodejs) \& npm](#installing-nodenodejs--npm)
  - [Clone Project](#clone-project)
  - [Build Project](#build-project)
  - [Generate ROS2 Javascript Message](#generate-ros2-javascript-message)
  - [Launch Client](#launch-client)

## Installation

### Prerequisites

Before installing, please ensure the following softare is installed and configured on your system:

- [node](https://nodejs.org/en/) version required 18.15.0 (npm 9.5.0)
  
- [nodejs](https://nodejs.org/en/) version required between 10.23.1 - 12.x.

- [ROS2 setup](https://index.ros.org/doc/ros2/Installation/) for install rclnodejs by npm -
  **INSTALL [ROS2 Foxy-Fitzroy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)**

### Installing node(nodejs) & npm

Install the node(nodejs) & npm into your Linux(Ubuntu 20.0.4LTS)

For the most current version of nodejs :

```bash
sudo install nodejs
```

For the most current version of npm(node package manager) :

```bash
sudo install npm -g
```

Install node(nodejs) & npm latest version by n :

```bash
sudo npm i n
```

```bash
sudo n 18.15.0
```

Check Your node(nodejs) & npm version

```bash
node -v
```
```bash
nodejs -v
```

```bash
npm -v
```

## Clone Project

Clone proejct into your own directories

```bash
mkdir ${your directory} && cd ${your directory}

git clone -b JaraTwo_Release_v1.0.0 https://github.com/WaveM-Robot/MqttClient.git
```

## Build Project

Before usage install node modules & build project

For the install required node modules

```bash
cd ${your directory}/ros2_mqtt_client

npm i
```

If error occurs during installing npm modules apply your ROS2 setup.bash

```bash
source /opt/ros/foxy/setup.bash
```

For the npm build project

```bash
npm run build
```

For the colcon build to apply into colcon workspace

```bash
colcon build --symlink-install
```

For the source installed setup.bash

```bash
source install/setup.bash
```

## Generate ROS2 Javascript Message

For generate & apply new message type after install new message type that you want to use

```bash
npx generate-ros-message
npm i
```

Check it installed or not

```typescript
import * as rclnodejs from 'rclnodejs';

const message = rclnodejs.createMessageObject('${installed message type}');
```

## Launch Client

Launch ROS2-MQTT Client

```bash
ros2 launch ros2_mqtt_client overall.launch.py
```
Launch Result Console

```

  _____   ____   _____ ___    __  __  ____ _______ _______    _____ _      _____ ______ _   _ _______ 
 |  __ \ / __ \ / ____|__ \  |  \/  |/ __ \__   __|__   __|  / ____| |    |_   _|  ____| \ | |__   __|
 | |__) | |  | | (___    ) | | \  / | |  | | | |     | |    | |    | |      | | | |__  |  \| |  | |   
 |  _  /| |  | |\___ \  / /  | |\/| | |  | | | |     | |    | |    | |      | | |  __| | . ` |  | |   
 | | \ \| |__| |____) |/ /_  | |  | | |__| | | |     | |    | |____| |____ _| |_| |____| |\  |  | |   
 |_|  \_\\____/|_____/|____| |_|  |_|\___\_\ |_|     |_|     \_____|______|_____|______|_| \_|  |_|   
                                                                                                      

info: ROS2-MQTT Client is ready for RCL!!                                                                                                                            
```