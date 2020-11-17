#pragma once

// standard libraries
#include <assert.h>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstring>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <thread>
#include <vector>

// Open3D
#include <Open3D/Geometry/KDTreeFlann.h>
#include <Open3D/Geometry/PointCloud.h>
#include <Open3D/IO/ClassIO/PointCloudIO.h>

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

// TinyPly
#define TINYPLY_IMPLEMENTATION
#include "tinyply.h"

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

// ros
#include <ros/ros.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace flightlib;

namespace motion_planning {

class manual_timer {
  std::chrono::high_resolution_clock::time_point t0;
  double timestamp{0.0};

 public:
  void start() { t0 = std::chrono::high_resolution_clock::now(); }
  void stop() {
    timestamp = std::chrono::duration<double>(
                  std::chrono::high_resolution_clock::now() - t0)
                  .count() *
                1000.0;
  }
  const double &get() { return timestamp; }
};

struct float2 {
  float x, y;
};
struct float3 {
  float x, y, z;
};
struct double3 {
  double x, y, z;
};
struct uint3 {
  uint32_t x, y, z;
};
struct uint4 {
  uint32_t x, y, z, w;
};
std::vector<float3> verts;
float range = 1;
bool solution_found = false;
bool trajectory_found = false;

std::vector<float3> readPointCloud();
float3 min_bounds;
float3 max_bounds;

Eigen::Vector3d stateToEigen(const ompl::base::State *state);

std::vector<ompl::base::State *> path_;
std::vector<Eigen::Vector3d> vecs_;

void getBounds();

void plan();

bool isStateValid(const ob::State *state);

bool isInRange(float x, float y, float z);

open3d::geometry::KDTreeFlann kd_tree_;
Eigen::MatrixXd points_;
bool searchRadius(const Eigen::Vector3d &query_point, const double radius);

void executePath();

// void setupQuad();
bool setUnity(const bool render);
bool connectUnity(void);

// unity quadrotor
std::shared_ptr<Quadrotor> quad_ptr_;
std::shared_ptr<RGBCamera> rgb_camera_;
QuadState quad_state_;

// Flightmare(Unity3D)
std::shared_ptr<UnityBridge> unity_bridge_ptr_;
SceneID scene_id_{UnityScene::NATUREFOREST};
bool unity_ready_{false};
bool unity_render_{true};
RenderMessage_t unity_output_;
uint16_t receive_id_{0};


}  // namespace motion_planning
