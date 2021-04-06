#pragma once

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <rosbag/bag.h>
#include <tf/tfMessage.h>

#include <chrono>
#include <flightlib/common/types.hpp>
#include <flightros/ros_utils.hpp>
#include <ze/common/time_conversions.hpp>
#include <ze/common/transformation.hpp>

namespace flightros {

class RosbagWriter {
 public:
  RosbagWriter(const std::string& path_to_output_bag);
  RosbagWriter(const std::string& path_to_output_bag, int64_t stime);

  ~RosbagWriter();

  void imageCallback(const ImagePtr& images, int64_t t);
  void imageRGBCallback(const RGBImagePtr& images, int64_t t);
  void imageOFCallback(const RGBImagePtr& image, int64_t t);
  void imageDepthCallback(const RGBImagePtr& image, int64_t t);
  void eventsCallback(const EventsVector& events, int64_t t);
  void poseCallback(const ze::Transformation& T_W_C, int64_t t);
  void imageEventCallback(const RGBImagePtr& images, int64_t t);

 private:
  size_t num_cameras_;
  cv::Size sensor_size_;
  rosbag::Bag bag_;
  int64_t starting_time;
  const std::string topic_name_prefix_ = "";

  int64_t last_published_camera_info_time_;
  int64_t last_published_image_time_;
};

}  // namespace flightros
