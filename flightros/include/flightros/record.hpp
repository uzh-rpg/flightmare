#pragma once

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "flightros/ros_utils.hpp"
#include "flightros/rosbag_writer.hpp"
#include "opencv2/imgcodecs.hpp"


// standard libraries
#include <assert.h>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <thread>
#include <vector>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/objects/static_gate.hpp"
#include "flightlib/sensors/event_camera.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

// trajectory
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <polynomial_trajectories/polynomial_trajectories_common.h>
#include <polynomial_trajectories/polynomial_trajectory.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/trajectory_point.h>
#include <quadrotor_msgs/Trajectory.h>
#include <fstream>
#include "trajectory_generation_helper/polynomial_trajectory_helper.h"

using namespace flightlib;
using namespace flightros;

namespace record {

bool setUnity(const bool render);
bool connectUnity(void);
std::string type2str(int type);
void saveToFile(std::vector<Event_t>);
void createMinSnap(const std::vector<Eigen::Vector3d> waypoints,
                   quadrotor_common::Trajectory* trajectory);
polynomial_trajectories::PolynomialTrajectory createOwnSnap(
  const std::vector<Eigen::Vector3d> waypoints_in,
  Eigen::VectorXd segment_times_in);
void samplePolynomial(
  quadrotor_common::Trajectory& trajectory,
  const polynomial_trajectories::PolynomialTrajectory& polynomial,
  const double sampling_frequency);
image_transport::Publisher rgb_pub_;

// unity quadrotor
std::shared_ptr<Quadrotor> quad_ptr_;
std::shared_ptr<RGBCamera> rgb_camera_;
std::shared_ptr<EventCamera> event_camera_;
QuadState quad_state_;

// Flightmare(Unity3D)
std::shared_ptr<UnityBridge> unity_bridge_ptr_;
SceneID scene_id_{UnityScene::WAREHOUSE};
bool unity_ready_{false};
bool unity_render_{true};
RenderMessage_t unity_output_;
uint16_t receive_id_{0};
std::ofstream events_text_file_;
int count_;
std::vector<float> errors;
int num_cam = 1;
std::string path_to_output_bag = "/home/gian/bags_flightmare/record.bag";

std::shared_ptr<RosbagWriter> writer_;
}  // namespace record