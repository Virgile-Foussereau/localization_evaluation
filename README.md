# localization_evaluation
Evaluation node of dead-reckoning methods for AI Racing Tech in the Indy Autonomous Challenge.

## Overview

![Overview](https://i.ibb.co/WPWn6cz/Pasted-Graphic-1.png)

This package is used alongside a rosbag (or a source of sensors messages) and a sensor-fusion module. It will redirect the sensors messages to the sensor-fusion module, with drop out on GPS, and evaluate its performance.

This node subscribes to:
- `/gps_bot/pose`
- `/gps_bot/orientation`
- `/gps_bot/imu`
- `/filtered_publisher/pose`

It publishes to:
- `/intermediate/gps_pose`
- `/intermediate/gps_orientation`

It republises `/gps_bot/pose` and `/gps_bot/orientation` to `/intermediate/gps_pose` and `/intermediate/gps_orientation` outside the GPS denied zone. In the GPS denied zone, only IMU data is not blocke. The GPS zone is defined by the user using the command line arguments when running this package.


