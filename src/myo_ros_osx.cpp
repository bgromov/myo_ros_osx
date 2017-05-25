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

// This class serves as a ROS namespace wrapper and allows
// to publish individual Myo's data in a separate namespace.
class MyoRos {
  ros::NodeHandlePtr pnode_;
  myo::Myo* myo_;
  size_t id_;
  std::string frame_id_;
  std::string fixed_frame_id_;
  std::string ns_;

  uint64_t time_offset_us_;

  bool use_ros_timestamps_;

  // Information that will be going out
  geometry_msgs::QuaternionStamped msg_rotation_;
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

  tf::TransformBroadcaster br_;
public:

  MyoRos(ros::NodeHandlePtr& parent, myo::Myo* myo, size_t id, uint64_t time_offset_us)
    : myo_(myo), id_(id), fixed_frame_id_("/world"), time_offset_us_(time_offset_us), use_ros_timestamps_(false)
  {
    ros::NodeHandlePtr pnode_root = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    use_ros_timestamps_ = pnode_root->param("use_ros_timestamps", false);

    if (pnode_root->hasParam("static_myo_ids"))
    {
      XmlRpc::XmlRpcValue v;
      pnode_root->param("static_myo_ids", v, v);
      for (int i = 0; i < v.size(); i++)
      {
        if (std::string(v[i]) == myo->getMacAddress())
        {
          id_ = id = i + 1;
          break;
        }
      }
    }

    ns_ = std::string("myo") + std::to_string(id_);
    frame_id_ = std::string("myo") + std::to_string(id_) + std::string("_frame");
    pnode_ = ros::NodeHandlePtr(new ros::NodeHandle(*parent, ns_));

    // Subscribe & publish setup
    pub_rotation_ = pnode_->advertise<geometry_msgs::QuaternionStamped>("rotation", 100);
    pub_gesture_ = pnode_->advertise<myo_ros::GestureStamped>("gesture", 100);
    pub_gyro_ = pnode_->advertise<geometry_msgs::Vector3Stamped>("gyro", 100);
    pub_accel_ = pnode_->advertise<geometry_msgs::Vector3Stamped>("accel", 100);
    pub_status_ = pnode_->advertise<myo_ros::StatusStamped>("status", 100);

    sub_vibration_ = pnode_->subscribe<myo_ros::Vibration>("vibration", 100, boost::bind(&MyoRos::vibrationCallback, this, _1));
    sub_unlock_override_ = pnode_->subscribe<std_msgs::Bool>("unlock_override", 100, boost::bind(&MyoRos::unlockCallback, this, _1));

    if (use_ros_timestamps_)
    {
      ROS_INFO("Paired Myo (%s) with ID: %s. Using ROS time stamps", myo->getName().c_str(), ns_.c_str());
    }
    else
    {
      ROS_INFO("Paired Myo (%s) with ID: %s. The time offset with ROS is: %15.3f", myo->getName().c_str(), ns_.c_str(), myoToRosTime(0).toSec());
    }
  }

  ~MyoRos()
  {
    myo_->lock();
  }

  inline ros::Time myoToRosTime(uint64_t timestamp) {
    // Myo timestamp is in microseconds
    uint64_t new_ts = timestamp + time_offset_us_;
    if (use_ros_timestamps_)
    {
      return ros::Time::now();
    }
    else
    {
      return ros::Time(new_ts / 1000000, (new_ts % 1000000) * 1000);
    }
  }

  std::string getNs() const
  {
    return ns_;
  }

  // Creates vibrations
  void vibrationCallback(const myo_ros::VibrationConstPtr& vibration)
  {
    if (myo_)
    {
      myo_->vibrate(static_cast<myo::Myo::VibrationType>(vibration->vibration));
    }
  }

  // Sets the unlock timing
  void unlockCallback(const std_msgs::BoolConstPtr& unlock)
  {
    if (myo_)
    {
      if (unlock->data == true)
      {
        myo_->unlock(myo::Myo::unlockHold);
      }
      else
      {
        myo_->unlock(myo::Myo::unlockTimed);
      }
    }
  }

  void publishRotation(geometry_msgs::QuaternionStamped& msg)
  {
    tf::Transform myo_tf;
    tf::Quaternion tf_quat;

//    myo_tf.setOrigin(tf::Vector3(0.0, 0.0, 1.5));
    tf::quaternionMsgToTF(msg.quaternion, tf_quat);
    myo_tf.setRotation(tf_quat);

    // Myo returns data in global inertial frame, e.g. /world
    msg.header.frame_id = fixed_frame_id_;

    pub_rotation_.publish(msg);
    br_.sendTransform(tf::StampedTransform(myo_tf, msg.header.stamp, fixed_frame_id_, frame_id_));

  }

  void publishGesture(myo_ros::GestureStamped& msg)
  {
    msg.header.frame_id = fixed_frame_id_;

    pub_gesture_.publish(msg);
  }

  void publishGyro(geometry_msgs::Vector3Stamped& msg)
  {
    msg.header.frame_id = fixed_frame_id_;

    pub_gyro_.publish(msg);
  }

  void publishAccel(geometry_msgs::Vector3Stamped& msg)
  {
    msg.header.frame_id = fixed_frame_id_;

    pub_accel_.publish(msg);
  }

  void publishStatus(myo_ros::StatusStamped& msg)
  {
    msg.header.frame_id = fixed_frame_id_;

    pub_status_.publish(msg);
  }
};

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener
{
  typedef boost::shared_ptr<MyoRos> MyoRosPtr;

  size_t myo_counter_;
  std::map<myo::Myo*, MyoRosPtr> myo_map_;

  ros::NodeHandlePtr node_;

public:
  DataCollector(ros::NodeHandlePtr& nh): myo_counter_(0)
  {
    node_ = nh;
  }

  void addMyo(myo::Myo* myo, int64_t time_offset_us)
  {
    MyoRosPtr mt;

    // If new Myo found assign new ID
    if (myo_map_.find(myo) == myo_map_.end())
    {
      myo_map_[myo] = MyoRosPtr(new MyoRos(node_, myo, ++myo_counter_, time_offset_us));
    }
  }

  void removeMyo(myo::Myo* myo)
  {
    myo_map_.erase(myo);
  }

  void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
  {
    // time difference between ROS and Myo in microseconds
    int64_t time_offset_us = ros::Time::now().toNSec() / 1000 - timestamp;

    this->addMyo(myo, time_offset_us);
  }

  void onUnpair(myo::Myo* myo, uint64_t timestamp)
  {
    this->removeMyo(myo);
  }

  // onConnect() is called when a paired Myo is connected
  void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
  {
    ROS_INFO("Connected to %s", myo_map_[myo]->getNs().c_str());
  }

  // onDisconnect() is called when a paired Myo is disconnected
  void onDisconnect(myo::Myo* myo, uint64_t timestamp)
  {
    ROS_INFO("Disconnected from %s", myo_map_[myo]->getNs().c_str());
  }

  // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
  // as a unit quaternion.
  void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
  {
    geometry_msgs::QuaternionStamped msg;

    // We have to fill in all data here, except for frame id
    msg.header.stamp = myo_map_[myo]->myoToRosTime(timestamp);
    msg.quaternion.w = quat.w();
    msg.quaternion.x = quat.x();
    msg.quaternion.y = quat.y();
    msg.quaternion.z = quat.z();

    // Publish rotation and tf frame
    myo_map_[myo]->publishRotation(msg);
  }

  // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
  // making a fist, or not making a fist anymore.
  void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
  {
    myo_ros::GestureStamped msg;

    msg.header.stamp = myo_map_[myo]->myoToRosTime(timestamp);
    msg.gesture.gesture = pose.type();

    myo_map_[myo]->publishGesture(msg);
  }

  // onGyroscopeData() called when a paired Myo has provided new gyroscope data in units of deg/s
  void onGyroscopeData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3<float> &gyro)
  {
    geometry_msgs::Vector3Stamped msg;

    msg.header.stamp = myo_map_[myo]->myoToRosTime(timestamp);
    msg.vector.x = gyro.x();
    msg.vector.y = gyro.y();
    msg.vector.z = gyro.z();

    myo_map_[myo]->publishGyro(msg);
  }

  // onAccelerometerData() called when a paired Myo has provided new accelerometer data in units of g.
  void onAccelerometerData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3<float> &accel)
  {
    geometry_msgs::Vector3Stamped msg;

    msg.header.stamp = myo_map_[myo]->myoToRosTime(timestamp);
    msg.vector.x = accel.x();
    msg.vector.y = accel.y();
    msg.vector.z = accel.z();

    myo_map_[myo]->publishAccel(msg);
}

  // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
  // arm. This lets Myo know which arm it's on and which way it's facing.
  void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
  {
    myo_ros::StatusStamped msg;

    msg.header.stamp = myo_map_[myo]->myoToRosTime(timestamp);
    msg.status.sync = true;
    msg.status.unlock = false;
    msg.status.arm = static_cast<int8_t>(arm);
    msg.status.direction = static_cast<int8_t>(xDirection);

    myo_map_[myo]->publishStatus(msg);
  }

  // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
  // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
  // when Myo is moved around on the arm.
  void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
  {
    myo_ros::StatusStamped msg;

    msg.header.stamp = myo_map_[myo]->myoToRosTime(timestamp);
    msg.status.sync = false;
    msg.status.unlock = false;
    msg.status.arm = static_cast<int8_t>(myo::armUnknown);
    msg.status.direction = static_cast<int8_t>(myo::xDirectionUnknown);

    myo_map_[myo]->publishStatus(msg);
  }

  // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
  void onUnlock(myo::Myo* myo, uint64_t timestamp)
  {
    myo_ros::StatusStamped msg;

    msg.header.stamp = myo_map_[myo]->myoToRosTime(timestamp);
    msg.status.unlock = true;

    myo_map_[myo]->publishStatus(msg);
  }

  // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
  void onLock(myo::Myo* myo, uint64_t timestamp)
  {
    myo_ros::StatusStamped msg;

    msg.header.stamp = myo_map_[myo]->myoToRosTime(timestamp);
    msg.status.unlock = false;

    myo_map_[myo]->publishStatus(msg);
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
    myo::Hub hub("org.ros.myo_ros_osx");

//    ROS_INFO("Attempting to find Myo...");
//
//    // Next, we try to find a Myo (any Myo) that's nearby and connect to it. waitForAnyMyo() takes a timeout
//    // value in milliseconds. In this case we will try to find a Myo for 10 seconds, and if that fails, the function
//    // will return a null pointer.
//    my_myo = hub.waitForMyo(10000);
//
//    // If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
//    if (!my_myo)
//    {
//      ROS_FATAL("Unable to find Myo!");
//      return -1;
//    }
//
//    // We've found a Myo, let's output its MAC address.
//    ROS_INFO("Found Myo");

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
