#include <flightros/ros_utils.hpp>
#include <cv_bridge/cv_bridge.h>

namespace flightros {

void imageToMsg(const cv::Mat_<ImageFloatType>& image, int64_t t,
                sensor_msgs::ImagePtr& msg) {
  cv_bridge::CvImage cv_image;
  cv_image.image = image;
  cv_image.encoding = "bgr8";
  cv_image.header.stamp = toRosTime(t);
  msg = cv_image.toImageMsg();
}
void imageToMsg(const cv::Mat& image, int64_t t, sensor_msgs::ImagePtr& msg) {
  cv_bridge::CvImage cv_image;
  cv_image.image = image;
  cv_image.encoding = "bgr8";
  cv_image.header.stamp = toRosTime(t);
  msg = cv_image.toImageMsg();
}
void imageFloatToMsg(const cv::Mat& image, int64_t t, sensor_msgs::ImagePtr& msg) {
  cv_bridge::CvImage cv_image;
  cv_image.image = image;
  cv_image.encoding = "32FC1";
  cv_image.header.stamp = toRosTime(t);
  msg = cv_image.toImageMsg();
}

void eventsToMsg(const EventsVector& events, int width, int height,
                 dvs_msgs::EventArrayPtr& msg, int64_t starting_time) {
  std::vector<dvs_msgs::Event> events_list;
  for (const Event_t& e : events) {
    dvs_msgs::Event ev;
    ev.x = e.coord_x;
    ev.y = e.coord_y;
    int64_t event_time=e.time;
    ev.ts = toRosTime((event_time*1000 + starting_time) );
    ev.polarity = e.polarity;
    if(e.time>0){
    events_list.push_back(ev);
    }
  }

  msg->events = events_list;
  msg->height = height;
  msg->width = width;
  msg->header.stamp = events_list.back().ts;
}

}  // namespace flightros
