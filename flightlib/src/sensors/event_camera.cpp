#include "flightlib/sensors/event_camera.hpp"

namespace flightlib {

EventCamera::EventCamera()
  : channels_(3), width_(720), height_(480), fov_{70.0} {
  auto time = std::chrono::high_resolution_clock::now();
}

EventCamera::~EventCamera() {}

bool EventCamera::feedEventImageQueue(const cv::Mat& event) {
  queue_mutex_.lock();
  event_image_queue_.push_back(event);
  queue_mutex_.unlock();
  return true;
}

bool EventCamera::feedEventQueue(std::vector<Event_t>& events) {
  queue_mutex_.lock();
  event_queue_.push_back(events);
  event_queue_for_img.resize(events.size());
  event_queue_for_img = events;
  event_queue_for_test = events;
  event_queue_sum.insert(std::end(event_queue_sum), std::begin(events),
                         std::end(events));
  queue_mutex_.unlock();
  std::string amount = std::to_string(event_queue_for_img.size());
  logger_.warn(amount);
  return true;
}

bool EventCamera::setRelPose(const Ref<Vector<3>> B_r_BC,
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

bool EventCamera::setWidth(const int width) {
  if (width <= 0.0) {
    logger_.warn(
      "The setting value for Image Width is not valid, discard the setting.");
    return false;
  }
  width_ = width;
  return true;
}

bool EventCamera::setHeight(const int height) {
  if (height <= 0.0) {
    logger_.warn(
      "The setting value for Image Height is not valid, discard the "
      "setting.");
    return false;
  }
  height_ = height;
  return true;
}

bool EventCamera::setFOV(const Scalar fov) {
  if (fov <= 0.0) {
    logger_.warn(
      "The setting value for Camera Field-of-View is not valid, discard the "
      "setting.");
    return false;
  }
  fov_ = fov;
  return true;
}
bool EventCamera::setCp(const float cp) {
  if (cp <= 0.0) {
    logger_.warn(
      "The setting value for Eventcamera Cp is not valid, discard the "
      "setting.");
    return false;
  }
  cp_ = cp;
  return true;
}
bool EventCamera::setCm(const float cm) {
  if (cm <= 0.0) {
    logger_.warn(
      "The setting value for Eventcamera Cm is not valid, discard the "
      "setting.");
    return false;
  }
  cm_ = cm;
  return true;
}
bool EventCamera::setsigmaCp(const float sigma_cp) {
  if (sigma_cp <= 0.0) {
    logger_.warn(
      "The setting value for Eventcamera sigmaCm is not valid, discard the "
      "setting.");
    return false;
  }
  sigma_cp_ = sigma_cp;
  return true;
}
bool EventCamera::setsigmaCm(const float sigma_cm) {
  if (sigma_cm <= 0.0) {
    logger_.warn(
      "The setting value for Eventcamera sigmaCm is not valid, discard the "
      "setting.");
    return false;
  }
  sigma_cm_ = sigma_cm;
  return true;
}
bool EventCamera::setRefractory(const uint64_t refractory_period) {
  if (refractory_period <= 0) {
    logger_.warn(
      "The setting value for Eventcamera refractory period is not valid, "
      "discard the "
      "setting.");
    return false;
  }
  refractory_period_ns_ = refractory_period;
  return true;
}
bool EventCamera::setLogEps(const float log_eps) {
  if (log_eps <= 0.0) {
    logger_.warn(
      "The setting value for Eventcamera log_eps is not valid, discard the "
      "setting.");
    return false;
  }
  log_eps_ = log_eps;
  return true;
}
bool EventCamera::setImgStore(const bool img_store) {
  store_image_ = img_store;
  return true;
}

bool EventCamera::changeTime(TimeMessage_t time_msg) {
  if (time_msg.next_timestep <= 0) {
    logger_.warn("timestep is zero or invalid");
    return false;
  }
  sim_time = time_msg.current_time;
  delta_time = time_msg.next_timestep;
  return true;
}
double EventCamera::getSecSimTime() { return sim_time / 1000000.0; }
int64_t EventCamera::getMicroSimTime() { return sim_time; }
int64_t EventCamera::getNanoSimTime() { return sim_time * 1000; }
int64_t EventCamera::getMicroTime() { return real_time; }

Matrix<4, 4> EventCamera::getRelPose(void) const { return T_BC_; }

int EventCamera::getWidth(void) const { return width_; }

int EventCamera::getHeight(void) const { return height_; }

Scalar EventCamera::getFOV(void) const { return fov_; }
float EventCamera::getCm(void) const { return cm_; }
float EventCamera::getCp(void) const { return cp_; }

float EventCamera::getsigmaCm(void) const { return sigma_cm_; }
float EventCamera::getsigmaCp(void) const { return sigma_cp_; }
uint64_t EventCamera::getRefractory(void) const {
  return refractory_period_ns_;
}
float EventCamera::getLogEps(void) const { return log_eps_; }
bool EventCamera::getImgStore(void) { return store_image_; }

bool EventCamera::getEventImages(cv::Mat& event) {
  if (!event_image_queue_.empty()) {
    // seems wrong here
    event = event_image_queue_.front();
    event_image_queue_.pop_front();
    return true;
  }
  return false;
}

std::vector<Event_t> EventCamera::getEvents() {
  std::vector<Event_t> events;
  if (!event_queue_for_test.empty()) {
    // seems wrong here
    events = event_queue_for_test;
    // event_queue_for_test.clear();
    return events;
  }
  logger_.error("empty events buffer");
  return events;
}

bool EventCamera::deleteEventQueue() {
  if (!event_queue_for_test.empty()) {
    event_queue_for_test.clear();
    return true;
  }
  logger_.error("already empty event buffer");
  return false;
}

cv::Mat EventCamera::createEventimages() {
  int wid = getWidth();
  int hei = getHeight();
  cv::Mat image = cv::Mat::zeros(cv::Size(wid, hei), CV_64FC1);
  std::vector<Event_t> events;
  int count = 0;

  for (auto event : event_queue_sum) {
    if (event.coord_x > wid || event.coord_y > hei) {
      logger_.error("coord out of the image");
    }
    if (event.polarity == 1) {
      image.at<cv::Vec3b>(event.coord_y, event.coord_x)[0] = 0;
      image.at<cv::Vec3b>(event.coord_y, event.coord_x)[1] = 0;
      image.at<cv::Vec3b>(event.coord_y, event.coord_x)[2] = 255;
    } else if (event.polarity == -1) {
      count++;
      image.at<cv::Vec3b>(event.coord_y, event.coord_x)[0] = 255;
      image.at<cv::Vec3b>(event.coord_y, event.coord_x)[1] = 0;
      image.at<cv::Vec3b>(event.coord_y, event.coord_x)[2] = 0;
    }
  }
  event_queue_sum.clear();
  return image;
}

bool EventCamera::feedImageQueue(const cv::Mat& image_mat) {
  queue_mutex_.lock();

  if (rgb_queue_.size() > queue_size_) rgb_queue_.resize(queue_size_);
  rgb_queue_.push_back(image_mat);

  queue_mutex_.unlock();
  return true;
}

bool EventCamera::getRGBImage(cv::Mat& rgb_img) {
  if (!rgb_queue_.empty()) {
    rgb_img = rgb_queue_.front();
    rgb_queue_.pop_front();
    return true;
  }
  return false;
}

}  // namespace flightlib