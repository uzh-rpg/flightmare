#pragma once
#include <ros/ros.h>
#include <flightlib/common/types.hpp>
#include <sensor_msgs/Image.h>
#include <dvs_msgs/EventArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>

using namespace flightlib;

namespace flightros {
inline std::string getTopicName(int i, const std::string& suffix)
{
  std::stringstream ss;
  ss << "cam" << i << "/" << suffix;
  return ss.str();
}

inline std::string getTopicName(const std::string& prefix, int i, const std::string& suffix)
{
  std::stringstream ss;
  ss << prefix << "/" << getTopicName(i, suffix);
  return ss.str();
}

inline ros::Time toRosTime(int64_t t)
{
  ros::Time ros_time;
  ros_time.fromNSec(t);
  return ros_time;
}

void imageToMsg(const cv::Mat_<ImageFloatType>& image, int64_t t, sensor_msgs::ImagePtr& msg);
void imageToMsg(const cv::Mat& image, int64_t t, sensor_msgs::ImagePtr& msg);
void imageFloatToMsg(const cv::Mat& image, int64_t t, sensor_msgs::ImagePtr& msg);
void eventsToMsg(const EventsVector& events, int width, int height, dvs_msgs::EventArrayPtr& msg, int64_t starting_time);

} // namespace flightros
