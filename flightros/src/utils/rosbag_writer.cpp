#include <flightros/rosbag_writer.hpp>

DECLARE_double(ros_publisher_camera_info_rate);
DECLARE_double(ros_publisher_frame_rate);
DECLARE_double(ros_publisher_depth_rate);
DECLARE_double(ros_publisher_pointcloud_rate);
DECLARE_double(ros_publisher_optic_flow_rate);
DEFINE_string(path_to_output_bag, "",
              "Path to which save the output bag file.");

namespace flightros {
RosbagWriter::RosbagWriter(const std::string& path_to_output_bag) {
  num_cameras_ = 1;
  sensor_size_ = cv::Size(100, 100);
  starting_time = 1;
  try {
    bag_.open(path_to_output_bag, rosbag::bagmode::Write);
  } catch (rosbag::BagIOException e) {
    LOG(FATAL) << "Error: could not open rosbag: " << FLAGS_path_to_output_bag
               << std::endl;
    return;
  }

  last_published_camera_info_time_ = 0;
  last_published_image_time_ = 0;
}
RosbagWriter::RosbagWriter(const std::string& path_to_output_bag,
                           int64_t stime) {
  num_cameras_ = 1;
  sensor_size_ = cv::Size(100, 100);

  starting_time = 1;  // stime
  try {
    bag_.open(path_to_output_bag, rosbag::bagmode::Write);
  } catch (rosbag::BagIOException e) {
    LOG(FATAL) << "Error: could not open rosbag: " << FLAGS_path_to_output_bag
               << std::endl;
    return;
  }
  last_published_camera_info_time_ = 0;
  last_published_image_time_ = 0;
}

RosbagWriter::~RosbagWriter() {
  LOG(INFO) << "Finalizing the bag...";
  bag_.close();
  LOG(INFO) << "Finished writing to bag: " << FLAGS_path_to_output_bag;
}

void RosbagWriter::imageCallback(const ImagePtr& image, int64_t t) {
  sensor_size_ = image->size();
  if (image) {
    sensor_msgs::ImagePtr msg;
    imageToMsg(*image, t + starting_time, msg);
    bag_.write(getTopicName(topic_name_prefix_, 0, "image_raw"),
               msg->header.stamp, msg);
  }
  last_published_image_time_ = t + starting_time;
}
void RosbagWriter::imageRGBCallback(const RGBImagePtr& image, int64_t t) {
  sensor_size_ = image->size();
  ROS_INFO_STREAM("writing" << image);
  if (image) {
    sensor_msgs::ImagePtr msg;
    imageToMsg(*image, t + starting_time, msg);
    bag_.write(getTopicName(topic_name_prefix_, 0, "image_rgb_raw"),
               msg->header.stamp, msg);
  }
  last_published_image_time_ = t + starting_time;
}
void RosbagWriter::imageEventCallback(const RGBImagePtr& image, int64_t t) {
  sensor_size_ = image->size();
  if (image) {
    sensor_msgs::ImagePtr msg;
    imageToMsg(*image, t + starting_time, msg);
    bag_.write(getTopicName(topic_name_prefix_, 0, "image_event"),
               msg->header.stamp, msg);
  }
  last_published_image_time_ = t + starting_time;
}

void RosbagWriter::imageOFCallback(const RGBImagePtr& image, int64_t t) {
  sensor_size_ = image->size();
  if (image) {
    sensor_msgs::ImagePtr msg;
    imageToMsg(*image, t + starting_time, msg);
    bag_.write(getTopicName(topic_name_prefix_, 0, "image_of"),
               msg->header.stamp, msg);
  }
  last_published_image_time_ = t + starting_time;
}
void RosbagWriter::imageDepthCallback(const RGBImagePtr& image, int64_t t) {
  sensor_size_ = image->size();
  if (image) {
    sensor_msgs::ImagePtr msg;
    imageFloatToMsg(*image, t + starting_time, msg);
    bag_.write(getTopicName(topic_name_prefix_, 0, "image_depth"),
               msg->header.stamp, msg);
  }

  last_published_image_time_ = t + starting_time;
}


void RosbagWriter::eventsCallback(const EventsVector& events, int64_t t) {
  if (sensor_size_.width == 0 || sensor_size_.height == 0) {
    ROS_WARN_STREAM("width to small");
    return;
  }

  if (events.empty()) {
    ROS_WARN_STREAM("empty ");
    return;
  }

  dvs_msgs::EventArrayPtr msg;
  msg.reset(new dvs_msgs::EventArray);
  eventsToMsg(events, sensor_size_.width, sensor_size_.height, msg,
              starting_time);

  bag_.write(getTopicName(topic_name_prefix_, 0, "events"), msg->header.stamp,
             msg);
}

void RosbagWriter::poseCallback(const ze::Transformation& T_W_C, int64_t t) {
  geometry_msgs::PoseStamped pose_stamped_msg;
  geometry_msgs::TransformStamped transform_stamped_msg;
  transform_stamped_msg.header.frame_id = "map";
  transform_stamped_msg.header.stamp = toRosTime(t + starting_time);
  tf::tfMessage tf_msg;

  tf::poseStampedKindrToMsg(T_W_C, toRosTime(t + starting_time), "map",
                            &pose_stamped_msg);
  bag_.write(getTopicName(topic_name_prefix_, 0, "pose"),
             toRosTime(t + starting_time), pose_stamped_msg);

  // Write tf transform to bag
  std::stringstream ss;
  ss << "cam";
  transform_stamped_msg.child_frame_id = ss.str();
  tf::transformKindrToMsg(T_W_C, &transform_stamped_msg.transform);
  tf_msg.transforms.push_back(transform_stamped_msg);
  bag_.write("/tf", toRosTime(t + starting_time), tf_msg);
}
}  // namespace flightros