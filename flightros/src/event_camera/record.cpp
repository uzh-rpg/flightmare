#include "flightros/record.hpp"

bool record::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    std::cout << "Unity Bridge is created." << std::endl;
  }
  return true;
}

bool record::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

void record::samplePolynomial(
  quadrotor_common::Trajectory& trajectory,
  const polynomial_trajectories::PolynomialTrajectory& polynomial,
  const double sampling_frequency) {
  if (polynomial.trajectory_type ==
      polynomial_trajectories::TrajectoryType::UNDEFINED) {
    ROS_WARN_STREAM("Trajectory is undefined.");
  }

  trajectory.points.push_back(polynomial.start_state);

  const ros::Duration dt(1.0 / sampling_frequency);
  ros::Duration time_from_start = polynomial.start_state.time_from_start + dt;
  ROS_INFO_STREAM("Sampling." << polynomial.T << "/" << dt);
  ROS_INFO_STREAM("From start." << time_from_start);

  while (time_from_start < polynomial.T) {
    trajectory.points.push_back(polynomial_trajectories::getPointFromTrajectory(
      polynomial, time_from_start));
    time_from_start += dt;
  }
  ROS_INFO_STREAM("From start to end." << time_from_start);

  trajectory.points.push_back(polynomial.end_state);

  trajectory.trajectory_type =
    quadrotor_common::Trajectory::TrajectoryType::GENERAL;
}

std::string record::type2str(int type) {
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

void record::createMinSnap(const std::vector<Eigen::Vector3d> waypoints,
                           quadrotor_common::Trajectory* trajectory) {
  // identify the segment times
  Eigen::VectorXd segment_times = Eigen::VectorXd::Ones(waypoints.size() - 1);
  double desired_speed = 1.0;
  for (int i = 0; i < waypoints.size() - 1; i++) {
    segment_times[i] =
      (waypoints.at(i + 1) - waypoints.at(i)).norm() / desired_speed;
    ROS_INFO_STREAM("seg time" << segment_times[i]);
  }
  Eigen::VectorXd minimization_weights(4);
  minimization_weights << 0.0, 0.0, 0.0, 100.0;
  double sampling_freq = 50.0;
  quadrotor_common::TrajectoryPoint start_state;
  start_state.position = waypoints.front();
  start_state.velocity =
    Eigen::Vector3d::UnitX() *
    0.05;  // hack to get a smooth heading trajectory at the beginning
  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = waypoints.back();
  std::vector<Eigen::Vector3d> waypoints_no_start_end = waypoints;
  waypoints_no_start_end.erase(waypoints_no_start_end.begin());
  waypoints_no_start_end.pop_back();
  std::cout << "generating trajectory through " << waypoints_no_start_end.size()
            << " waypoints with " << segment_times.size() << " segment times"
            << std::endl;
  polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings;
  trajectory_settings.way_points = waypoints_no_start_end;
  trajectory_settings.minimization_weights = minimization_weights;
  trajectory_settings.polynomial_order = 11;
  trajectory_settings.continuity_order = 4;
  double max_velocity = 5.0;
  double max_collective_thrust = 20.0;
  double max_roll_pitch_rate = 3.0;


  *trajectory =
    trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
      start_state, end_state, 3, max_velocity, max_collective_thrust,
      max_roll_pitch_rate, sampling_freq);
}

polynomial_trajectories::PolynomialTrajectory record::createOwnSnap(
  const std::vector<Eigen::Vector3d> waypoints_in,
  Eigen::VectorXd segment_times_in) {
  double desired_speed_in = 1.0;
  for (int i = 0; i < waypoints_in.size() - 1; i++) {
    segment_times_in[i] =
      (waypoints_in.at(i + 1) - waypoints_in.at(i)).norm() / desired_speed_in;
    ROS_INFO_STREAM("seg time: " << segment_times_in[i]);
  }

  Eigen::VectorXd minimization_weights_in(4);
  minimization_weights_in << 0.0, 0.0, 0.0, 100.0;
  double sampling_freq_in = 50.0;
  quadrotor_common::TrajectoryPoint start_state_in;
  start_state_in.position = waypoints_in.front();
  start_state_in.velocity =
    Eigen::Vector3d::UnitX() *
    0.05;  // hack to get a smooth heading trajectory at the beginning
  quadrotor_common::TrajectoryPoint end_state_in;
  end_state_in.position = waypoints_in.back();
  // maybe the problem lies in the time from start
  std::vector<Eigen::Vector3d> waypoints_no_start_end_in = waypoints_in;
  waypoints_no_start_end_in.erase(waypoints_no_start_end_in.begin());
  waypoints_no_start_end_in.pop_back();
  std::cout << "generating trajectory through "
            << waypoints_no_start_end_in.size() << " waypoints with "
            << segment_times_in.size() << " segment times" << std::endl;
  polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings_in;
  trajectory_settings_in.way_points = waypoints_no_start_end_in;
  trajectory_settings_in.minimization_weights = minimization_weights_in;
  trajectory_settings_in.polynomial_order = 18;
  trajectory_settings_in.continuity_order = 4;
  double max_velocity_in = 5.0;
  double max_collective_thrust_in = 20.0;
  double max_roll_pitch_rate_in = 3.0;

  polynomial_trajectories::PolynomialTrajectory trajectory_in =
    polynomial_trajectories::minimum_snap_trajectories::
      generateMinimumSnapTrajectory(
        segment_times_in, start_state_in, end_state_in, trajectory_settings_in,
        max_velocity_in, max_collective_thrust_in, max_roll_pitch_rate_in);

  return trajectory_in;
}

void record::saveToFile(std::vector<Event_t> events) {
  for (const Event_t& e : events) {
    if (e.polarity != 0) {
      record::events_text_file_ << e.time << " " << e.coord_x << " "
                                << e.coord_y << " " << e.polarity << std::endl;
    }
  }
}

int main(int argc, char* argv[]) {
  // initialize ROS
  ros::init(argc, argv, "flightmare_gates");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");
  image_transport::ImageTransport my_image_transport(nh);

  ros::Rate(50.0);

  // Settings
  bool rotate = false;
  bool trajectory_rotation = false;
  bool position_trajectory;
  bool ring_traj = false;
  int scene;
  double cp;
  double cm;
  float time_for_rotation;
  std::string which_trajectory;
  // pull nitialization param from param server
  pnh.getParam("/rosbag/rotate", rotate);
  pnh.getParam("/rosbag/trajectory_rotation", trajectory_rotation);
  pnh.getParam("/rosbag/position_trajectory", position_trajectory);
  pnh.getParam("/rosbag/path", record::path_to_output_bag);
  pnh.getParam("/rosbag/ring_traj", ring_traj);
  pnh.getParam("/rosbag/scene_id", scene);
  pnh.getParam("/rosbag/trajectory", which_trajectory);
  pnh.getParam("/rosbag/cp", cp);
  pnh.getParam("/rosbag/cm", cm);
  pnh.getParam("/rosbag/time_for_rotation", time_for_rotation);


  record::scene_id_ = scene;

  // quad initialization
  record::quad_ptr_ = std::make_unique<Quadrotor>();

  // add camera
  record::rgb_camera_ = std::make_unique<RGBCamera>();
  record::event_camera_ = std::make_unique<EventCamera>();

  record::rgb_pub_ = my_image_transport.advertise("camera/rgb", 1);
  ros::Publisher trajectory_pub_ =
    nh.advertise<quadrotor_msgs::Trajectory>("autopilot/trajectory", 1);
  ros::Publisher traj_pub_ = nh.advertise<nav_msgs::Path>("path/trajectory", 1);
  ros::Publisher line_pub_ = nh.advertise<nav_msgs::Path>("path/straight", 1);


  std::vector<float> position_vec;
  std::vector<float> orientation_vec;
  std::vector<float> waypts_x;
  std::vector<float> waypts_y;
  std::vector<float> waypts_z;

  pnh.getParam("/record/single_position", position_vec);
  pnh.getParam("/record/single_orientation", orientation_vec);
  pnh.getParam(which_trajectory + "/x", waypts_x);
  pnh.getParam(which_trajectory + "/y", waypts_y);
  pnh.getParam(which_trajectory + "/z", waypts_z);

  ROS_INFO_STREAM("writing to rosbag: " << record::path_to_output_bag);


  int frame = 0;
  // Flightmare
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  record::rgb_camera_->setFOV(83);
  record::rgb_camera_->setWidth(512);
  record::rgb_camera_->setHeight(352);
  record::rgb_camera_->setRelPose(B_r_BC, R_BC);
  record::rgb_camera_->setPostProcesscing(
    std::vector<bool>{true, false, true});  // depth, segmentation, optical flow
  record::quad_ptr_->addRGBCamera(record::rgb_camera_);
  record::event_camera_->setFOV(83);
  record::event_camera_->setWidth(512);
  record::event_camera_->setHeight(352);
  record::event_camera_->setRelPose(B_r_BC, R_BC);
  record::event_camera_->setCp(0.1);
  record::event_camera_->setCm(0.1);
  record::event_camera_->setsigmaCm(cp);
  record::event_camera_->setsigmaCp(cm);
  record::event_camera_->setRefractory(1);
  record::event_camera_->setLogEps(0.0001);

  record::quad_ptr_->addEventCamera(record::event_camera_);


  // double cp = record::event_camera_->getCp();
  // double cm = record::event_camera_->getCm();
  bool record = false;
  // // initialization
  record::quad_state_.setZero();

  record::quad_ptr_->reset(record::quad_state_);
  // record::quad_ptr_->setState()

  record::writer_ = std::make_shared<RosbagWriter>(record::path_to_output_bag);

  // Set unity bridge
  record::setUnity(record::unity_render_);

  // connect unity
  record::connectUnity();
  std::vector<Eigen::Vector3d> way_points;

  Eigen::VectorXd seg_times(waypts_x.size() - 1);

  quadrotor_common::Trajectory sampled_trajectory_;

  // create trajectory ring or not
  if (ring_traj) {
    ROS_WARN_STREAM("Creating RING trajectory");
    way_points.push_back(Eigen::Vector3d(0, 10, 2.5));
    way_points.push_back(Eigen::Vector3d(5, 0, 2.5));
    way_points.push_back(Eigen::Vector3d(0, -10, 2.5));
    way_points.push_back(Eigen::Vector3d(-5, 0, 2.5));
    way_points.push_back(Eigen::Vector3d(-7, 10, 2.5));
    way_points.push_back(Eigen::Vector3d(-5, 20, 2.5));

    std::size_t num_waypoints = way_points.size();
    Eigen::VectorXd segment_times(num_waypoints);
    segment_times << 50.0, 50.0, 50.0, 50.0, 50.0, 50.0;
    Eigen::VectorXd minimization_weights(num_waypoints);
    minimization_weights << 0.0, 0.0, 0.0, 1.0;

    polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings =
      polynomial_trajectories::PolynomialTrajectorySettings(
        way_points, minimization_weights, 7, 4);

    polynomial_trajectories::PolynomialTrajectory trajectory =
      polynomial_trajectories::minimum_snap_trajectories::
        generateMinimumSnapRingTrajectory(segment_times, trajectory_settings,
                                          20.0, 20.0, 6.0);
  } else {
    for (int it = 0; it < waypts_x.size(); it++) {
      way_points.push_back(
        Eigen::Vector3d(waypts_x.at(it), waypts_y.at(it), waypts_z.at(it)));
    }
    for (int it = 0; it < waypts_x.size() - 1; it++) {
      seg_times[it] = 10.0;
    }
  }
  polynomial_trajectories::PolynomialTrajectory trajectory_ =
    record::createOwnSnap(way_points, seg_times);

  record::samplePolynomial(sampled_trajectory_, trajectory_, 10.0);

  nav_msgs::Path path;
  path.header.frame_id = "/map";
  for (auto pt : sampled_trajectory_.points) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/map";
    pose.pose.position.x = pt.position[0];
    pose.pose.position.y = pt.position[1];
    pose.pose.position.z = pt.position[2];
    path.poses.push_back(pose);
    ROS_INFO_STREAM("pt   " << pt.position[0] << "/" << pt.position[1] << "/"
                            << pt.position[2]);
  }

  // Desired path in lines
  nav_msgs::Path desired_path;
  desired_path.header.frame_id = "/map";

  for (auto pt : way_points) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/map";
    pose.pose.position.x = pt[0];
    pose.pose.position.y = pt[1];
    pose.pose.position.z = pt[2];
    desired_path.poses.push_back(pose);
  }

  cv::Mat new_image, of_image, depth_image, ev_image;
  Image I;

  ROS_INFO_STREAM(
    "========================================================================"
    "=");

  while (ros::ok() && record::unity_render_ && record::unity_ready_) {
    quadrotor_common::TrajectoryPoint desired_pose =
      polynomial_trajectories::getPointFromTrajectory(
        trajectory_, ros::Duration(record::event_camera_->getSecSimTime()));

    // different options are considered:
    // pure rotation, pure trnaslation or both
    if (position_trajectory) {
      record::quad_state_.x[QS::POSX] = (Scalar)desired_pose.position.x();
      record::quad_state_.x[QS::POSY] = (Scalar)desired_pose.position.y();
      record::quad_state_.x[QS::POSZ] = (Scalar)desired_pose.position.z();

    } else {
      record::quad_state_.x[QS::POSX] = position_vec.at(0);
      record::quad_state_.x[QS::POSY] = position_vec.at(1);
      record::quad_state_.x[QS::POSZ] = position_vec.at(2);
    }
    if (trajectory_rotation) {
      record::quad_state_.x[QS::ATTW] = (Scalar)desired_pose.orientation.w();
      record::quad_state_.x[QS::ATTX] = (Scalar)desired_pose.orientation.x();
      record::quad_state_.x[QS::ATTY] = (Scalar)desired_pose.orientation.y();
      record::quad_state_.x[QS::ATTZ] = (Scalar)desired_pose.orientation.z();
    } else if (rotate) {
      record::quad_state_.x[QS::ATTW] =
        2 * std::cos(record::event_camera_->getSecSimTime() * 0.1) *
        std::cos(record::event_camera_->getSecSimTime() * 3.1415 /
                 time_for_rotation);
      // should be changing velocity and directions
      record::quad_state_.x[QS::ATTX] = 0.0;
      record::quad_state_.x[QS::ATTY] = 0.0;
      record::quad_state_.x[QS::ATTZ] =
        2 * std::cos(record::event_camera_->getSecSimTime() * 0.1) *
        std::sin(record::event_camera_->getSecSimTime() * 3.1425 /
                 time_for_rotation);
    } else {
      record::quad_state_.x[QS::ATTW] = orientation_vec.at(3);
      record::quad_state_.x[QS::ATTX] = orientation_vec.at(0);
      record::quad_state_.x[QS::ATTY] = orientation_vec.at(1);
      record::quad_state_.x[QS::ATTZ] = orientation_vec.at(2);
    }

    // publish trajectory for visualization
    traj_pub_.publish(path);
    line_pub_.publish(desired_path);

    // define new position and orientation
    ze::Transformation twc;
    record::quad_state_.qx.normalize();
    twc.getRotation() = ze::Quaternion(
      record::quad_state_.x[QS::ATTW], record::quad_state_.x[QS::ATTX],
      record::quad_state_.x[QS::ATTY], record::quad_state_.x[QS::ATTZ]);
    twc.getPosition() = ze::Position(record::quad_state_.x[QS::POSX],
                                     record::quad_state_.x[QS::POSY],
                                     record::quad_state_.x[QS::POSZ]);
    // writ position to rosbag
    record::writer_->poseCallback(twc, record::event_camera_->getNanoSimTime());

    ROS_INFO_STREAM("pose " << record::quad_state_.x[QS::POSX] << "/"
                            << record::quad_state_.x[QS::POSY] << "/"
                            << record::quad_state_.x[QS::POSZ] << "/"
                            << record::event_camera_->getSecSimTime());

    // send state to server
    record::quad_ptr_->setState(record::quad_state_);
    record::unity_bridge_ptr_->getRender(0);
    record::unity_bridge_ptr_->handleOutput();

    // get images
    record::event_camera_->getRGBImage(ev_image);
    record::rgb_camera_->getRGBImage(new_image);
    record::rgb_camera_->getOpticalFlow(of_image);
    record::rgb_camera_->getDepthMap(depth_image);

    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_image).toImageMsg();
    record::rgb_pub_.publish(rgb_msg);

    RGBImagePtr rgb_img_ptr = std::make_shared<RGBImage>(ev_image);
    RGBImagePtr of_img_ptr = std::make_shared<RGBImage>(of_image);
    RGBImagePtr depth_img_ptr = std::make_shared<RGBImage>(depth_image);

    // triggers for starting recording
    if (frame > 10 && record::event_camera_->getImgStore()) {
      ROS_INFO_STREAM("starting to record");
      record = true;
    }

    // write to rosbag
    if (record) {
      if (record::event_camera_->getImgStore()) {
        cv::Mat event_img = record::event_camera_->createEventimages();
        RGBImagePtr event_img_ptr = std::make_shared<RGBImage>(event_img);

        record::writer_->imageRGBCallback(
          rgb_img_ptr, record::event_camera_->getNanoSimTime());
        record::writer_->imageOFCallback(
          of_img_ptr, record::event_camera_->getNanoSimTime());
        record::writer_->imageDepthCallback(
          depth_img_ptr, record::event_camera_->getNanoSimTime());
        record::writer_->imageEventCallback(
          event_img_ptr, record::event_camera_->getNanoSimTime());
      }

      const EventsVector& events = record::event_camera_->getEvents();
      record::writer_->eventsCallback(events,
                                      record::event_camera_->getNanoSimTime());
    }
    // clear the buffer
    record::event_camera_->deleteEventQueue();
    frame++;
  }
  record::events_text_file_.close();
  return 0;
}
