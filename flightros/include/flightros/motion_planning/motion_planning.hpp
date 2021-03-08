// standard libraries
#include <assert.h>
#include <Eigen/Dense>
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

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace flightlib;

namespace motion_planning {

struct float3 {
  float x, y, z;
};

struct Bounds {
  float3 min;
  float3 max;
};

class MotionPlanner {
 public:
  MotionPlanner();
  ~MotionPlanner();
  void run();
  void readPointCloud();
  void getBounds();
  bool plan();
  void executePath();

 private:
  std::vector<ompl::base::State *> path_;
  std::vector<Eigen::Vector3d> vecs_;
  std::vector<float3> verts_;
  open3d::geometry::KDTreeFlann kd_tree_;
  Eigen::MatrixXd points_;
  Bounds bounds_;

  // unity
  SceneID scene_id_{UnityScene::NATUREFOREST};

  // ompl methods
  bool searchRadius(const Eigen::Vector3d &query_point, const double radius);
  Eigen::Vector3d stateToEigen(const ompl::base::State *state);
  bool isStateValid(const ob::State *state);
};

}  // namespace motion_planning
