// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Confidential and not for redistribution. See LICENSE.txt.
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <stdint.h>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <myo_ros/GestureStamped.h>
#include <myo_ros/Vibration.h>
#include <myo_ros/StatusStamped.h>
#include <std_msgs/Bool.h>

myo::Myo* my_myo = NULL;

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener
{
  std::string fixed_frame_id_;
  std::string myo_frame_id_;
  int64_t time_offset_us_;

public:
  // Information that will be going out
  geometry_msgs::QuaternionStamped msg_rotation_;
  tf::TransformBroadcaster br_;
  myo_ros::GestureStamped msg_gesture_;
  myo_ros::StatusStamped msg_status_;
  geometry_msgs::Vector3Stamped msg_gyro_;
  geometry_msgs::Vector3Stamped msg_accel_;

  ros::Publisher pub_rotation_;
  ros::Publisher pub_gesture_;
  ros::Publisher pub_gyro_;
  ros::Publisher pub_accel_;
  ros::Publisher pub_status_;
  ros::Subscriber sub_vibration_;
  ros::Subscriber sub_unlock_override_;

  DataCollector(ros::NodeHandlePtr &nh): time_offset_us_(0), myo_frame_id_("myo_frame"), fixed_frame_id_("/world")
  {
    // Subscribe & publish setup
    pub_rotation_ = nh->advertise<geometry_msgs::QuaternionStamped>("rotation", 100);
    pub_gesture_ = nh->advertise<myo_ros::GestureStamped>("gesture", 100);
    pub_gyro_ = nh->advertise<geometry_msgs::Vector3Stamped>("gyro", 100);
    pub_accel_ = nh->advertise<geometry_msgs::Vector3Stamped>("accel", 100);
    pub_status_ = nh->advertise<myo_ros::StatusStamped>("status", 100);

    sub_vibration_ = nh->subscribe<myo_ros::Vibration>("vibration", 100, boost::bind(&DataCollector::vibrationCallback, this, _1));
    sub_unlock_override_ = nh->subscribe<std_msgs::Bool>("unlock_override", 100, boost::bind(&DataCollector::unlockCallback, this, _1));
  }

  inline ros::Time myoToRosTime(uint64_t timestamp) {
    // Myo timestamp is in microseconds
    uint64_t new_ts = timestamp + time_offset_us_;
    return ros::Time(new_ts / 1000000, (new_ts % 1000000) * 1000);
  }

  // Creates vibrations
  void vibrationCallback(const myo_ros::VibrationConstPtr& vibration)
  {
    if (my_myo)
    {
      my_myo->vibrate(static_cast<myo::Myo::VibrationType>(vibration->vibration));
    }
  }

  // Sets the unlock timing
  void unlockCallback(const std_msgs::BoolConstPtr& unlock)
  {
    if (my_myo)
    {
      if (unlock->data == true)
      {
        my_myo->unlock(myo::Myo::unlockHold);
      }
      else
      {
        my_myo->unlock(myo::Myo::unlockTimed);
      }
    }
  }

  // onConnect() is called when a paired Myo is connected
  void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
  {
    if (time_offset_us_ == 0)
    {
      time_offset_us_ = ros::Time::now().toNSec() / 1000 - timestamp; // time difference between ROS and Myo in microseconds
    }

    my_myo = myo;
    ROS_INFO("Connected");
  }

  // onConnect() is called when a paired Myo is disconnected
  void onDisconnect(myo::Myo* myo, uint64_t timestamp)
  {
    my_myo = NULL;
    time_offset_us_ = 0;
    ROS_INFO("Disconnected");
  }

  // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
  // as a unit quaternion.
  void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
  {
    tf::Transform myo_tf;
    myo_tf.setOrigin(tf::Vector3(0.0, 0.0, 1.5));
    myo_tf.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));

    msg_rotation_.header.frame_id = myo_frame_id_;
    msg_rotation_.header.stamp = myoToRosTime(timestamp);
    msg_rotation_.quaternion.w = quat.w();
    msg_rotation_.quaternion.x = quat.x();
    msg_rotation_.quaternion.y = quat.y();
    msg_rotation_.quaternion.z = quat.z();

    pub_rotation_.publish(msg_rotation_);
    br_.sendTransform(tf::StampedTransform(myo_tf, myoToRosTime(timestamp), fixed_frame_id_, myo_frame_id_));
  }

  // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
  // making a fist, or not making a fist anymore.
  void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
  {
    msg_gesture_.header.frame_id = myo_frame_id_;
    msg_gesture_.header.stamp = myoToRosTime(timestamp);
    msg_gesture_.gesture.gesture = pose.type();
    pub_gesture_.publish(msg_gesture_);
  }

  // onGyroscopeData() called when a paired Myo has provided new gyroscope data in units of deg/s
  void onGyroscopeData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3<float> &gyro)
  {
    msg_gyro_.header.frame_id = myo_frame_id_;
    msg_gyro_.header.stamp = myoToRosTime(timestamp);
    msg_gyro_.vector.x = gyro.x();
    msg_gyro_.vector.y = gyro.y();
    msg_gyro_.vector.z = gyro.z();
    pub_gyro_.publish(msg_gyro_);
  }

  // onAccelerometerData() called when a paired Myo has provided new accelerometer data in units of g.
  void onAccelerometerData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3<float> &accel)
  {
    msg_accel_.header.frame_id = myo_frame_id_;
    msg_accel_.header.stamp = myoToRosTime(timestamp);
    msg_accel_.vector.x = accel.x();
    msg_accel_.vector.y = accel.y();
    msg_accel_.vector.z = accel.z();
    pub_accel_.publish(msg_accel_);
  }

  // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
  // arm. This lets Myo know which arm it's on and which way it's facing.
  void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
  {
    msg_status_.header.frame_id = myo_frame_id_;
    msg_status_.header.stamp = myoToRosTime(timestamp);
    msg_status_.status.sync = true;
    msg_status_.status.unlock = false;
    msg_status_.status.arm = static_cast<int8_t>(arm);
    msg_status_.status.direction = static_cast<int8_t>(xDirection);
    pub_status_.publish(msg_status_);
  }

  // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
  // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
  // when Myo is moved around on the arm.
  void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
  {
    msg_status_.header.frame_id = myo_frame_id_;
    msg_status_.header.stamp = myoToRosTime(timestamp);
    msg_status_.status.sync = false;
    msg_status_.status.unlock = false;
    msg_status_.status.arm = static_cast<int8_t>(myo::armUnknown);
    msg_status_.status.direction = static_cast<int8_t>(myo::xDirectionUnknown);
    pub_status_.publish(msg_status_);
  }

  // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
  void onUnlock(myo::Myo* myo, uint64_t timestamp)
  {
    msg_status_.header.frame_id = myo_frame_id_;
    msg_status_.header.stamp = myoToRosTime(timestamp);
    msg_status_.status.unlock = true;
    pub_status_.publish(msg_status_);
  }

  // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
  void onLock(myo::Myo* myo, uint64_t timestamp)
  {
    msg_status_.header.frame_id = myo_frame_id_;
    msg_status_.header.stamp = myoToRosTime(timestamp);
    msg_status_.status.unlock = false;
    pub_status_.publish(msg_status_);
  }
};

int main(int argc, char** argv)
{
  // We catch any exceptions that might occur below -- see the catch statement for more details.
  try
  {
    // Initialize ROS
    ros::init(argc, argv, "myo_ros_osx");

    // First, we create a Hub. The Hub provides access to one or more Myos.
    myo::Hub hub("org.ros.myo_ros_windows");

    ROS_INFO("Attempting to find Myo...");

    // Next, we try to find a Myo (any Myo) that's nearby and connect to it. waitForAnyMyo() takes a timeout
    // value in milliseconds. In this case we will try to find a Myo for 10 seconds, and if that fails, the function
    // will return a null pointer.
    my_myo = hub.waitForMyo(10000);

    // If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
    if (!my_myo)
    {
      ROS_FATAL("Unable to find Myo!");
      return -1;
    }

    // We've found a Myo, let's output its MAC address.
    ROS_INFO("Found Myo");

    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle());

    // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
    DataCollector collector(nh);

    // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
    // Hub::run() to send events to all registered device listeners.
    hub.addListener(&collector);

    // Finally we enter our main loop.
    while (ros::ok())
    {
      // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
      // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
      hub.run(1000 / 20);

      // Spin ROS
      ros::spinOnce();
    }

    // If a standard exception occurred, we print out its message and exit.
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
    std::cerr << "Press enter to continue.";
    std::cin.ignore();
    return 1;
  }
}
