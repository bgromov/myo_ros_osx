# myo_ros_osx

## Overview

An OS X fork of [my_ros_windows](https://github.com/clearpathrobotics/myo_ros_windows) repository. The ROS message interface is kept same as defined by [myo_ros](https://github.com/clearpathrobotics/myo_ros) package, however this version is based on a regular ROS distribution and does not use rosserial as the original package does.

The package is shipped with Myo SDK v0.9.0 (see lib/myo.framework folder).

## ROS API
### Subscribed Topics
- vibration (myo_ros/Vibration.msg): A vibration pattern for the Myo
- unlock_override (std_msgs/Bool.msg): Set to true to disable the Myo locking after it's been locked for the first time.

### Published Topics
- rotation (geometry_msgs/Quaternion.msg): The rotation of the Myo. Myo coordinate frames still TBA by Thalmic.
- gesture (myo_ros/Gesture.msg): Gesture as detected by the Myo
- gyro (geometry_msgs/Vector3): Gyroscope data in deg/s
- accel (geometry_msgs/Vector3): Accelerometer data in g
- status (myo_ros/Status.msg): Status of the Myo
