#pragma once

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "flightlib/common/logger.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/sensors/sensor_base.hpp"

namespace flightlib {

struct Event {
  int coord_x;
  int coord_y;
  int polarity;
  float time;
};

class EventCamera : SensorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EventCamera();
  ~EventCamera();

  // public set functions
  bool setRelPose(const Ref<Vector<3>> B_r_BC, const Ref<Matrix<3, 3>> R_BC);
  bool setWidth(const int width);
  bool setHeight(const int height);
  bool setFOV(const Scalar fov);
  bool setCp(const float cp);
  bool setCm(const float cm);
  bool setsigmaCp(const float sigma_cp);
  bool setsigmaCm(const float sigma_cm);
  bool setRefractory(const uint64_t refractory_period);
  bool setLogEps(const float log_eps);
  bool setImgStore(const bool img_store);

  bool changeTime(TimeMessage_t time_msg);
  double getSecSimTime();
  int64_t getMicroSimTime();
  int64_t getMicroTime();
  int64_t getNanoSimTime();

  bool feedImageQueue(const cv::Mat& image_mat);
  bool feedEventImageQueue(const cv::Mat& image_mat);
  bool feedEventQueue(std::vector<Event_t>& events);
  bool deleteEventQueue();

  // public get functions
  Matrix<4, 4> getRelPose(void) const;
  int getChannels(void) const;
  int getWidth(void) const;
  int getHeight(void) const;
  Scalar getFOV(void) const;
  float getCm(void) const;
  float getCp(void) const;
  float getsigmaCm(void) const;
  float getsigmaCp(void) const;
  uint64_t getRefractory(void) const;
  float getLogEps(void) const;
  bool getImgStore(void);


  // Scalar getDepthScale(void) const;
  bool getRGBImage(cv::Mat& rgb_img);
  bool getEventImages(cv::Mat& image_mat);
  std::vector<Event_t> getEvents();
  cv::Mat createEventimages();

 private:
  Logger logger_{"RBGCamera"};

  // camera parameters
  int channels_;
  int width_;
  int height_;
  Scalar fov_;
  float cm_;
  float cp_;
  float sigma_cp_;
  float sigma_cm_;
  int64_t refractory_period_ns_;
  float log_eps_;

  int64_t sim_time = 0;
  int64_t delta_time = 0;
  int64_t real_time;


  // Camera relative
  Vector<3> B_r_BC_;
  Matrix<4, 4> T_BC_;

  // image data buffer
  std::mutex queue_mutex_;
  const int queue_size_ = 1;

  std::deque<cv::Mat> rgb_queue_;
  std::deque<cv::Mat> event_image_queue_;
  std::deque<std::vector<Event_t>> event_queue_;
  std::vector<Event_t> event_queue_for_img;
  std::vector<Event_t> event_queue_for_test;
  std::vector<Event_t> event_queue_sum;
  bool store_image_ = false;
};
}  // namespace flightlib
