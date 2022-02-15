#pragma once

#include <yaml-cpp/yaml.h>

#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "flightlib/common/logger.hpp"
#include "flightlib/common/math.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/sensors/sensor_base.hpp"

namespace flightlib {

enum CameraLayer { DepthMap = 1, Segmentation = 2, OpticalFlow = 3 };

class RGBCamera : SensorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RGBCamera();
  ~RGBCamera();

  // public set functions
  bool setRelPose(const Ref<Vector<3>> B_r_BC, const Ref<Matrix<3, 3>> R_BC);
  bool setWidth(const int width);
  bool setHeight(const int height);
  bool setChannels(const int height);
  bool setFOV(const Scalar fov);
  bool setDepthScale(const Scalar depth_scale);
  bool setPostProcessing(const std::vector<bool>& enabled_layers);
  bool feedImageQueue(const int image_layer, const cv::Mat& image_mat);

  //
  Vector<2> projectPointToImage(const Ref<Matrix<4, 4>> T_WB,
                                const Ref<Vector<3>> point_w,
                                const bool normalized = false);

  // public get functions
  std::vector<bool> getEnabledLayers(void) const;
  Matrix<4, 4> getRelPose(void) const;
  int getChannels(void) const;
  int getWidth(void) const;
  int getHeight(void) const;
  Scalar getFOV(void) const;
  Scalar getDepthScale(void) const;
  Matrix<3, 3> getIntrinsic(void) const;
  bool getRGBImage(cv::Mat& rgb_img);
  bool getDepthMap(cv::Mat& depth_map);
  bool getSegmentation(cv::Mat& segmentation);
  bool getOpticalFlow(cv::Mat& opticalflow);

  // auxiliary functions
  void enableDepth(const bool on);
  void enableOpticalFlow(const bool on);
  void enableSegmentation(const bool on);

 private:
  bool updateCameraIntrinsics(void);
  Logger logger_{"RBGCamera"};

  // camera parameters
  int channels_;
  int width_;
  int height_;
  Scalar fov_;
  Scalar depth_scale_;

  // Camera relative
  Vector<3> B_r_BC_;
  // camera
  Matrix<4, 4> T_BC_;
  // camera intrinsic
  Matrix<3, 3> K_;

  // image data buffer
  std::mutex queue_mutex_;
  const long unsigned int queue_size_{1};

  std::deque<cv::Mat> rgb_queue_;
  std::deque<cv::Mat> depth_queue_;
  std::deque<cv::Mat> opticalflow_queue_;
  std::deque<cv::Mat> segmentation_queue_;

  // [depth, segmentation, optical flow]
  std::vector<bool> enabled_layers_;
};

}  // namespace flightlib
