#include "flightlib/sensors/rgb_camera.hpp"

namespace flightlib {

RGBCamera::RGBCamera()
  : channels_(3),
    width_(640),
    height_(480),
    fov_{90.0},
    depth_scale_{0.2},
    T_BC_{(Matrix<4, 4>() << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)
            .finished()},
    K_{(Matrix<3, 3>() << 174.246, 0.0, 320.0, 0.0, 130.684, 240.0, 0.0, 0.0,
        1.0)
         .finished()},
    enabled_layers_({false, false, false}) {}

RGBCamera::~RGBCamera() {}

bool RGBCamera::feedImageQueue(const int image_layer,
                               const cv::Mat& image_mat) {
  queue_mutex_.lock();
  switch (image_layer) {
    case 0:  // rgb image
      if (rgb_queue_.size() > queue_size_) rgb_queue_.resize(queue_size_);
      rgb_queue_.push_back(image_mat);
      break;
    case CameraLayer::DepthMap:
      if (depth_queue_.size() > queue_size_) depth_queue_.resize(queue_size_);
      depth_queue_.push_back(image_mat);
      break;
    case CameraLayer::Segmentation:
      if (segmentation_queue_.size() > queue_size_)
        segmentation_queue_.resize(queue_size_);
      segmentation_queue_.push_back(image_mat);
      break;
    case CameraLayer::OpticalFlow:
      if (opticalflow_queue_.size() > queue_size_)
        opticalflow_queue_.resize(queue_size_);
      opticalflow_queue_.push_back(image_mat);
      break;
  }
  queue_mutex_.unlock();
  return true;
}

bool RGBCamera::setRelPose(const Ref<Vector<3>> B_r_BC,
                           const Ref<Matrix<3, 3>> R_BC) {
  if (!B_r_BC.allFinite() || !R_BC.allFinite()) {
    logger_.error(
      "The setting value for Camera Relative Pose Matrix is not valid, discard "
      "the setting.");
    return false;
  }
  B_r_BC_ = B_r_BC;
  T_BC_.block<3, 3>(0, 0) = R_BC;
  T_BC_.block<3, 1>(0, 3) = B_r_BC;
  T_BC_.row(3) << 0.0, 0.0, 0.0, 1.0;
  return true;
}

bool RGBCamera::setWidth(const int width) {
  if (width <= 0) {
    logger_.warn(
      "The setting value for Image Width is not valid, discard the setting.");
    return false;
  }
  width_ = width;
  updateCameraIntrinsics();
  return true;
}

bool RGBCamera::setHeight(const int height) {
  if (height <= 0) {
    logger_.warn(
      "The setting value for Image Height is not valid, discard the "
      "setting.");
    return false;
  }
  height_ = height;
  updateCameraIntrinsics();
  return true;
}

bool RGBCamera::updateCameraIntrinsics(void) {
  K_(0, 0) = (height_ / 2) / (std::tanh(M_PI * fov_ / 180));
  K_(1, 1) = (height_ / 2) / (std::tanh(M_PI * fov_ / 180));
  K_(0, 2) = (int)width_ / 2;
  K_(1, 2) = (int)height_ / 2;
  K_(2, 2) = 1;
  return true;
}

bool RGBCamera::setChannels(const int channels) {
  if (!(channels == 1 || channels == 2 || channels == 3)) {
    logger_.warn(
      "The setting value for Image channel is not valid, discard the setting. "
      "%d",
      channels);
    return false;
  }
  channels_ = channels;
  return true;
}

bool RGBCamera::setFOV(const Scalar fov) {
  if (fov <= 0.0) {
    logger_.warn(
      "The setting value for Camera Field-of-View is not valid, discard the "
      "setting.");
    return false;
  }
  fov_ = fov;
  updateCameraIntrinsics();
  return true;
}

bool RGBCamera::setDepthScale(const Scalar depth_scale) {
  if (depth_scale_ < 0.0 || depth_scale_ > 1.0) {
    logger_.warn(
      "The setting value for Camera Depth Scale is not valid, discard the "
      "setting.");
    return false;
  }
  depth_scale_ = depth_scale;
  return true;
}

bool RGBCamera::setPostProcessing(const std::vector<bool>& enabled_layers) {
  if (enabled_layers_.size() != enabled_layers.size()) {
    logger_.warn(
      "Vector size does not match. The vector size should be equal to %d.",
      enabled_layers_.size());
    return false;
  }
  enabled_layers_ = enabled_layers;
  return true;
}

Vector<2> RGBCamera::projectPointToImage(const Ref<Matrix<4, 4>> T_BW,
                                         const Ref<Vector<3>> point_W,
                                         const bool normalized) {
  if (!T_BW.allFinite()) {
    logger_.error(
      "Transformation matrix is not valid. Cannot project point to the image "
      "plane");
  }
  // transformation matrix from body frame to the camera frame
  Matrix<4, 4> T_BC = T_BC_;
  T_BC.block<3, 3>(0, 0) =
    (AngleAxis(-90.0 * M_PI / 180.0, Vector<3>::UnitZ()) *
     AngleAxis(-90 * M_PI / 180.0, Vector<3>::UnitX()))
      .toRotationMatrix();
  Matrix<4, 4> T_CB = inversePoseMatrix(T_BC);

  // transfer point in the world frame to the camera frame
  const Vector<4> point_C =
    T_CB * T_BW * (Vector<4>() << point_W, 1.0).finished();

  //
  Vector<2> point_uv;
  point_uv[0] = (K_(0, 0) * point_C[0]) / point_C[2] + K_(0, 2);
  point_uv[1] = (K_(1, 1) * point_C[1]) / point_C[2] + K_(1, 2);

  if (normalized) {
    point_uv[0] = 2.0 * (point_uv[0] - (Scalar)width_ / 2.0) / (Scalar)width_;
    point_uv[1] = 2.0 * (point_uv[1] - (Scalar)height_ / 2.0) / (Scalar)height_;
    point_uv = point_uv.cwiseMax(-1.0).cwiseMin(1.0);
  }
  return point_uv;
}

std::vector<bool> RGBCamera::getEnabledLayers(void) const {
  return enabled_layers_;
}

Matrix<4, 4> RGBCamera::getRelPose(void) const { return T_BC_; }

int RGBCamera::getChannels(void) const { return channels_; }

int RGBCamera::getWidth(void) const { return width_; }

int RGBCamera::getHeight(void) const { return height_; }

Scalar RGBCamera::getFOV(void) const { return fov_; }

Scalar RGBCamera::getDepthScale(void) const { return depth_scale_; }

Matrix<3, 3> RGBCamera::getIntrinsic(void) const { return K_; }

void RGBCamera::enableDepth(const bool on) {
  if (enabled_layers_[CameraLayer::DepthMap] == on) {
    logger_.warn("Depth layer was already %s.", on ? "on" : "off");
  }
  enabled_layers_[CameraLayer::DepthMap] = on;
}

void RGBCamera::enableSegmentation(const bool on) {
  if (enabled_layers_[CameraLayer::Segmentation] == on) {
    logger_.warn("Segmentation layer was already %s.", on ? "on" : "off");
  }
  enabled_layers_[CameraLayer::Segmentation] = on;
}

void RGBCamera::enableOpticalFlow(const bool on) {
  if (enabled_layers_[CameraLayer::OpticalFlow] == on) {
    logger_.warn("Optical Flow layer was already %s.", on ? "on" : "off");
  }
  enabled_layers_[CameraLayer::OpticalFlow] = on;
}

bool RGBCamera::getRGBImage(cv::Mat& rgb_img) {
  if (!rgb_queue_.empty()) {
    rgb_img = rgb_queue_.front();
    rgb_queue_.pop_front();
    return true;
  }
  return false;
}

bool RGBCamera::getDepthMap(cv::Mat& depth_map) {
  if (!depth_queue_.empty()) {
    depth_map = depth_queue_.front();
    depth_queue_.pop_front();
    return true;
  }
  return false;
}

bool RGBCamera::getSegmentation(cv::Mat& segmentation) {
  if (!segmentation_queue_.empty()) {
    segmentation = segmentation_queue_.front();
    segmentation_queue_.pop_front();
    return true;
  }
  return false;
}

bool RGBCamera::getOpticalFlow(cv::Mat& opticalflow) {
  if (!opticalflow_queue_.empty()) {
    opticalflow = opticalflow_queue_.front();
    opticalflow_queue_.pop_front();
    return true;
  }
  return false;
}

}  // namespace flightlib