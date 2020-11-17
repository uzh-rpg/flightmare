#include "flightros/camera/camera.hpp"

bool camera::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    std::cout << "Unity Bridge is created." << std::endl;
  }
  return true;
}

bool camera::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}


int main(int argc, char *argv[]) {
  // initialize ROS
  ros::init(argc, argv, "flightmare_rviz");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");
  ros::Rate(50.0);

  // initialize publishers
  image_transport::ImageTransport it(pnh);
  camera::rgb_pub_ = it.advertise("/rgb", 1);
  camera::depth_pub_ = it.advertise("/depth", 1);
  camera::segmentation_pub_ = it.advertise("/segmentation", 1);
  camera::opticalflow_pub_ = it.advertise("/opticalflow", 1);

  // quad initialization
  camera::quad_ptr_ = std::make_unique<Quadrotor>();
  // add mono camera
  camera::rgb_camera_ = std::make_unique<RGBCamera>();

  // Flightmare
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  camera::rgb_camera_->setFOV(90);
  camera::rgb_camera_->setWidth(720);
  camera::rgb_camera_->setHeight(480);
  camera::rgb_camera_->setRelPose(B_r_BC, R_BC);
  camera::rgb_camera_->setPostProcesscing(
    std::vector<bool>{true, true, true});  // depth, segmentation, optical flow
  camera::quad_ptr_->addRGBCamera(camera::rgb_camera_);


  // // initialization
  camera::quad_state_.setZero();

  camera::quad_ptr_->reset(camera::quad_state_);

  // connect unity
  camera::setUnity(camera::unity_render_);
  camera::connectUnity();

  while (ros::ok() && camera::unity_render_ && camera::unity_ready_) {
    camera::quad_state_.x[QS::POSX] = (Scalar)0;
    camera::quad_state_.x[QS::POSY] = (Scalar)0;
    camera::quad_state_.x[QS::POSZ] = (Scalar)0;
    camera::quad_state_.x[QS::ATTW] = (Scalar)0;
    camera::quad_state_.x[QS::ATTX] = (Scalar)0;
    camera::quad_state_.x[QS::ATTY] = (Scalar)0;
    camera::quad_state_.x[QS::ATTZ] = (Scalar)0;

    camera::quad_ptr_->setState(camera::quad_state_);

    camera::unity_bridge_ptr_->getRender(0);
    camera::unity_bridge_ptr_->handleOutput();

    cv::Mat img;

    camera::rgb_camera_->getRGBImage(img);
    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    rgb_msg->header.stamp.fromNSec(camera::counter);
    camera::rgb_pub_.publish(rgb_msg);

    camera::rgb_camera_->getDepthMap(img);
    sensor_msgs::ImagePtr depth_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    depth_msg->header.stamp.fromNSec(camera::counter);
    camera::depth_pub_.publish(depth_msg);

    camera::rgb_camera_->getSegmentation(img);
    sensor_msgs::ImagePtr segmentation_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    segmentation_msg->header.stamp.fromNSec(camera::counter);
    camera::segmentation_pub_.publish(segmentation_msg);

    camera::rgb_camera_->getOpticalFlow(img);
    sensor_msgs::ImagePtr opticflow_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    opticflow_msg->header.stamp.fromNSec(camera::counter);
    camera::opticalflow_pub_.publish(opticflow_msg);

    camera::counter++;
  }

  return 0;
}
