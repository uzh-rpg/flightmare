#include "flightros/testing.hpp"

bool testing::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    std::cout << "Unity Bridge is created." << std::endl;
  }
  return true;
}

bool testing::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

// return image type
std::string testing::type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
    case CV_8U:
      r = "8U";
      break;
    case CV_8S:
      r = "8S";
      break;
    case CV_16U:
      r = "16U";
      break;
    case CV_16S:
      r = "16S";
      break;
    case CV_32S:
      r = "32S";
      break;
    case CV_32F:
      r = "32F";
      break;
    case CV_64F:
      r = "64F";
      break;
    default:
      r = "User";
      break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}
// saves the events into a .txt file
void testing::saveToFile(std::vector<Event_t> events) {
  for (const Event_t& e : events) {
    if (e.polarity != 0) {
      testing::events_text_file_ << e.time << " " << e.coord_x << " "
                                 << e.coord_y << " " << e.polarity << std::endl;
    }
  }
}

int main(int argc, char* argv[]) {
  // initialize ROS
  ros::init(argc, argv, "flightmare_gates");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");
  ros::Rate(50.0);
  image_transport::ImageTransport my_image_transport(nh);
  std::vector<ze::real_t> times, mean_errors, stddev_errors;

  // quad initialization
  testing::quad_ptr_ = std::make_unique<Quadrotor>();
  // add camera
  testing::rgb_camera_ = std::make_unique<RGBCamera>();
  testing::event_camera_ = std::make_unique<EventCamera>();

  testing::scene_id_ = 4;
  // Flightmare
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  // initialize rgb camera
  testing::rgb_camera_->setFOV(83);
  testing::rgb_camera_->setWidth(512);
  testing::rgb_camera_->setHeight(352);
  testing::rgb_camera_->setRelPose(B_r_BC, R_BC);
  testing::rgb_camera_->setPostProcesscing(
    std::vector<bool>{true, false, true});  // depth, segmentation, optical flow
  testing::quad_ptr_->addRGBCamera(testing::rgb_camera_);
  
  // initialize event camera, careful with the image size.
  // h,w should be a mutliple of 8 otherwise the thread groups in the compute 
  // shader need to be adapted
  testing::event_camera_->setFOV(83);
  testing::event_camera_->setWidth(512);
  testing::event_camera_->setHeight(352);
  testing::event_camera_->setRelPose(B_r_BC, R_BC);
  testing::event_camera_->setCp(0.2);
  testing::event_camera_->setCm(0.2);
  testing::event_camera_->setsigmaCm(0.0);
  testing::event_camera_->setsigmaCp(0.0);
  testing::event_camera_->setRefractory(1);
  testing::event_camera_->setLogEps(0.0001);
  testing::quad_ptr_->addEventCamera(testing::event_camera_);

  // scene can be changed 
  // testing::scene_id_ = 1;
  double cp = testing::event_camera_->getCp();
  double cm = testing::event_camera_->getCm();

  // // initialization
  testing::quad_state_.setZero();
  testing::quad_ptr_->reset(testing::quad_state_);

  // ROS publishers for control
  testing::rgb_pub_ =
    my_image_transport.advertise("camera/antialiasing_rgb", 1);
  testing::rgb_rgb_pub_ = my_image_transport.advertise("camera/rgb", 1);
  testing::diff_pub_ = my_image_transport.advertise("camera/diff", 1);
  testing::event_pub_ = my_image_transport.advertise("camera/event", 1);
  testing::of_pub_ = my_image_transport.advertise("camera/of", 1);
  testing::depth_pub_ = my_image_transport.advertise("camera/depth", 1);

  testing::writer_ = std::make_shared<RosbagWriter>(
    testing::path_to_output_bag, testing::event_camera_->getMicroSimTime());

  // Set unity bridge
  testing::setUnity(testing::unity_render_);

  // connect unity
  testing::connectUnity();

  // Define path 
  std::vector<Eigen::Vector3d> way_points;
  way_points.push_back(Eigen::Vector3d(0, 10, 2.5));
  way_points.push_back(Eigen::Vector3d(5, 0, 2.5));
  way_points.push_back(Eigen::Vector3d(0, -10, 2.5));
  way_points.push_back(Eigen::Vector3d(-5, 0, 2.5));

  std::size_t num_waypoints = way_points.size();
  Eigen::VectorXd segment_times(num_waypoints);
  segment_times << 10.0, 10.0, 10.0, 10.0;
  Eigen::VectorXd minimization_weights(num_waypoints);
  minimization_weights << 1.0, 1.0, 1.0, 1.0;

  polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings =
    polynomial_trajectories::PolynomialTrajectorySettings(
      way_points, minimization_weights, 7, 4);

  polynomial_trajectories::PolynomialTrajectory trajectory =
    polynomial_trajectories::minimum_snap_trajectories::
      generateMinimumSnapRingTrajectory(segment_times, trajectory_settings,
                                        20.0, 20.0, 6.0);

  testing::events_text_file_.open("/home/gian/Desktop/events");

  // Start testing
  bool is_first_image = true;
  Image I, L, L_reconstructed, L_last;

  int frame = 0;
  float time_ = 0;
  while (ros::ok() && testing::unity_render_ && testing::unity_ready_ &&
         frame < 200) {

    quadrotor_common::TrajectoryPoint desired_pose =
      polynomial_trajectories::getPointFromTrajectory(trajectory,
                                                      ros::Duration(time_));
    ROS_INFO_STREAM("pose time " << time_);
    testing::quad_state_.x[QS::POSX] = (Scalar)desired_pose.position.x();
    testing::quad_state_.x[QS::POSY] = (Scalar)desired_pose.position.y();
    testing::quad_state_.x[QS::POSZ] = (Scalar)desired_pose.position.z();
    testing::quad_state_.x[QS::ATTW] = (Scalar)desired_pose.orientation.w();
    testing::quad_state_.x[QS::ATTX] = (Scalar)desired_pose.orientation.x();
    testing::quad_state_.x[QS::ATTY] = (Scalar)desired_pose.orientation.y();
    testing::quad_state_.x[QS::ATTZ] = (Scalar)desired_pose.orientation.z();

    ze::Transformation tcv;
    testing::quad_state_.qx.normalize();
    tcv.getRotation() = ze::Quaternion(
      testing::quad_state_.x[QS::ATTW], testing::quad_state_.x[QS::ATTX],
      testing::quad_state_.x[QS::ATTY], testing::quad_state_.x[QS::ATTZ]);
    tcv.getPosition() = ze::Position((Scalar)desired_pose.position.x(),
                                     (Scalar)desired_pose.position.y(),
                                     (Scalar)desired_pose.position.z());
    testing::quad_ptr_->setState(testing::quad_state_);
    ROS_INFO_STREAM("time " << testing::event_camera_->getSecSimTime());

    testing::unity_bridge_ptr_->getRender(0);
    testing::unity_bridge_ptr_->handleOutput(true);

    //make some checks
    cv::Mat new_image, rgb_img, of_img, depth_img;
    testing::event_camera_->getRGBImage(new_image);

    // publish rgb image
    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", new_image).toImageMsg();
    testing::rgb_pub_.publish(rgb_msg);


    // publish rgb image without antialiasing
    testing::rgb_camera_->getRGBImage(rgb_img);
    sensor_msgs::ImagePtr rrgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_img).toImageMsg();
    testing::rgb_rgb_pub_.publish(rrgb_msg);

    // publish event image
    cv::Mat ev_img = testing::event_camera_->createEventimages();
    sensor_msgs::ImagePtr ev_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", ev_img).toImageMsg();
    testing::event_pub_.publish(ev_msg);

    // publish OF image
    bool was_of_empty = testing::rgb_camera_->getOpticalFlow(of_img);
    sensor_msgs::ImagePtr of_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", of_img).toImageMsg();
    testing::of_pub_.publish(of_msg);

    // publish depth image
    bool was_depth_empty = testing::rgb_camera_->getDepthMap(depth_img);
    sensor_msgs::ImagePtr depth_msg =
      cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_img).toImageMsg();
    testing::depth_pub_.publish(depth_msg);

    ROS_INFO_STREAM("recieved OF and depth " << was_of_empty << " / "<< was_depth_empty);
    ROS_INFO_STREAM("new image type " << testing::type2str(of_img.type()));

    // coonvert rgb image to gray
    cv::cvtColor(new_image, new_image, CV_BGR2GRAY);
    new_image.convertTo(I, cv::DataType<ImageFloatType>::type, 1. / 255.);

    // calculate the events to test
    Image dummie =
      cv::Mat::zeros(I.rows, I.cols, cv::DataType<ImageFloatType>::type);

    // convert image to log image
    Image log_img;
    cv::log(I + 0.0001, log_img);

    cv::log(dummie + 0.0001, dummie);
    L = log_img - dummie;


    if (is_first_image) {
      // Initialize reconstructed image from the ground truth image
      L_reconstructed = L.clone();
      L_last = L.clone();
      is_first_image = false;
    }

    int counter = 0;
    int counter_ = 0;
    int count = 0;
    bool first_check = true;

    // create illuminance difference image from events
    Image event_image =
      cv::Mat::zeros(I.rows, I.cols, cv::DataType<ImageFloatType>::type);

    for (const Event_t& e : testing::event_camera_->getEvents()) {
      counter_++;
      if (e.polarity != 0) {
        int pol;
        if (e.polarity == 1) {
          pol = 1;
          count++;
        } else if (e.polarity == -1) {
          pol = -1;
          counter++;
        } else
          pol = 0;

        L_reconstructed(e.coord_y, e.coord_x) += pol * cp;
        event_image(e.coord_y, e.coord_x) += pol * cp;

        if (first_check) {
          ROS_INFO_STREAM("Time of first event: " << e.time);
          first_check = false;
        }
      }
    }

    ROS_INFO_STREAM("Amount of pos events  " << count << " of neg " << counter
                                             << " of " << counter_);

    const EventsVector& events = testing::event_camera_->getEvents();
    testing::saveToFile(testing::event_camera_->getEvents());
    // clear the buffer
    testing::event_camera_->deleteEventQueue();

    //  starting evaluations
    // total error
    ImageFloatType total_error = 0;
    for (int y = 0; y < I.rows; ++y) {
      for (int x = 0; x < I.cols; ++x) {
        const ImageFloatType reconstruction_error =
          std::fabs(L_reconstructed(y, x) - L(y, x));
        total_error += reconstruction_error;
      }
    }
    ROS_INFO_STREAM("Total error " << total_error);

    // calculate total difference of two consecutive images
    total_error = 0;
    float true_mean = 0;
    float true_error_mean = 0;
    int count_for_mean = 0;
    for (int y = 0; y < I.rows; ++y) {
      for (int x = 0; x < I.cols; ++x) {
        total_error += std::fabs(L_last(y, x) - L(y, x));
        if (std::fabs(L_last(y, x) - L(y, x)) != 0) {
          true_mean += std::fabs(L_last(y, x) - L(y, x));
          true_error_mean += std::fabs(L_reconstructed(y, x) - L(y, x));
          count_for_mean++;
        }
      }
    }
    float number = static_cast<float>(count_for_mean);
    true_mean = true_mean / (number);
    true_error_mean = true_error_mean / (number);
    ROS_INFO_STREAM("Total diference between two images " << total_error);

    ROS_INFO_STREAM("True means " << true_mean
                                  << ", error mean : " << true_error_mean);

    // compute std deviation and mean of the error
    Image error, real_diff;
    cv::absdiff(L_reconstructed, L, error);
    cv::absdiff(L, L_last, real_diff);


    double minVal, maxVal, minVal_, maxVal_;
    cv::Point max, min, max_, min_;
    cv::minMaxLoc(error, &minVal, &maxVal, &min, &max);
    cv::minMaxLoc(real_diff, &minVal_, &maxVal_, &min_, &max_);

    // publish the two images
    cv::Mat draw;
    error.convertTo(draw, CV_8U, 255.0 / (maxVal - minVal), -minVal);
    sensor_msgs::ImagePtr diff_msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", draw).toImageMsg();
    testing::diff_pub_.publish(diff_msg);


    cv::Scalar mean_error, stddev_error, mean, stddevv;
    cv::meanStdDev(real_diff, mean, stddevv);
    cv::meanStdDev(error, mean_error, stddev_error);
    ROS_INFO_STREAM("Mean of real diff: " << mean << ", Stddev: " << stddevv);
    ROS_INFO_STREAM("Mean error: " << mean_error
                                   << ", Stddev: " << stddev_error);
    cv::Mat draw_;
    real_diff.convertTo(draw_, CV_8U, 255.0 / (maxVal_ - minVal_), -minVal_);
    ROS_INFO_STREAM("The biggest val of last " << maxVal_ << " and " << minVal_
                                               << " at " << max_.y << "/"
                                               << max_.x);
    ROS_INFO_STREAM("The biggest val of reconstructed "
                    << maxVal << " and " << minVal << " at " << max.y << "/"
                    << max.x);

    // setup variables for next rendering
    if (frame < 10 || frame % 1 == 0) L_reconstructed = L.clone();
    else if (frame > 10) {
      times.push_back(frame);
      mean_errors.push_back(mean_error[0]);
      stddev_errors.push_back(stddev_error[0]);
    }
    L_last = L.clone();

    time_ += 0.001;
    frame++;
  }
  // Plot the mean and stddev reconstruction error over time
  ze::plt::plot(times, mean_errors, "r");
  ze::plt::plot(times, stddev_errors, "b--");
  std::stringstream title;
  title << "C = " << cp << ", sigma = " << testing::event_camera_->getsigmaCm()
        << ", bias = " << 0;
  ze::plt::title(title.str());
  ze::plt::xlabel("Time (s)");
  ze::plt::ylabel("Mean / Stddev reconstruction error");
  ze::plt::grid(true);
  ze::plt::show();
  testing::events_text_file_.close();
  return 0;
}
