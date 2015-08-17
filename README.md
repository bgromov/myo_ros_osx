# myo_ros_osx

## Overview

An OS X fork of [myo_ros_windows](https://github.com/clearpathrobotics/myo_ros_windows) repository. The ROS message interface is slightly modified version of original [myo_ros](https://github.com/clearpathrobotics/myo_ros) package. So, make sure to download the version from [https://github.com/bgromov/myo_ros.git](https://github.com/bgromov/myo_ros.git). The main difference with the original package is that stamped versions of messages are being used. A timestamp is calculated with respect to ROS time.

This version is based on a regular ROS distribution and does not use rosserial as the original package does.

The package is shipped with Myo SDK v0.9.0 (see lib/myo.framework folder).

## ROS API
### Subscribed Topics
- vibration (myo_ros/Vibration.msg): A vibration pattern for the Myo
- unlock_override (std_msgs/Bool.msg): Set to true to disable the Myo locking after it's been locked for the first time.

### Published Topics
- rotation (geometry_msgs/QuaternionStamped.msg): The rotation of the Myo. Myo coordinate frames still TBA by Thalmic.
- gesture (myo_ros/GestureStamped.msg): Gesture as detected by the Myo
- gyro (geometry_msgs/Vector3Stamped): Gyroscope data in deg/s
- accel (geometry_msgs/Vector3Stamped): Accelerometer data in g
- status (myo_ros/StatusStamped.msg): Status of the Myo
