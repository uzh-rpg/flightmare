#include "flightros/motion_planning/motion_planning.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace flightlib;
using namespace tinyply;

namespace motion_planning {

MotionPlanner::MotionPlanner() {}

MotionPlanner::~MotionPlanner() {}

void MotionPlanner::run() {
  std::cout << "Read PointCloud" << std::endl;
  readPointCloud();

  std::cout << "Get Bounds" << std::endl;
  getBounds();

  bool solution_found = false;
  std::cout << "Plan" << std::endl;
  while (!solution_found) {
    solution_found = plan();
  }

  std::cout << "Execute" << std::endl;
  executePath();
}

void MotionPlanner::readPointCloud() {
  std::shared_ptr<std::istream> file_stream;
  std::vector<uint8_t> byte_buffer;
  std::string filepath =
    std::experimental::filesystem::path(__FILE__).parent_path().string() +
    "/data/point_cloud.ply";
  try {
    file_stream.reset(new std::ifstream(filepath, std::ios::binary));

    if (!file_stream || file_stream->fail())
      throw std::runtime_error("file_stream failed to open " + filepath);

    PlyFile file;
    file.parse_header(*file_stream);

    std::cout << "\t[ply_header] Type: "
              << (file.is_binary_file() ? "binary" : "ascii") << std::endl;
    for (const auto &c : file.get_comments())
      std::cout << "\t[ply_header] Comment: " << c << std::endl;
    for (const auto &c : file.get_info())
      std::cout << "\t[ply_header] Info: " << c << std::endl;

    for (const auto &e : file.get_elements()) {
      std::cout << "\t[ply_header] element: " << e.name << " (" << e.size << ")"
                << std::endl;
      for (const auto &p : e.properties) {
        std::cout << "\t[ply_header] \tproperty: " << p.name
                  << " (type=" << tinyply::PropertyTable[p.propertyType].str
                  << ")";
        if (p.isList)
          std::cout << " (list_type=" << tinyply::PropertyTable[p.listType].str
                    << ")";
        std::cout << std::endl;
      }
    }

    // Because most people have their own mesh types, tinyply treats parsed data
    // as structured/typed byte buffers. See examples below on how to marry your
    // own application-specific data structures with this one.
    std::shared_ptr<PlyData> vertices, normals, colors, texcoords, faces,
      tripstrip;

    // The header information can be used to programmatically extract properties
    // on elements known to exist in the header prior to reading the data. For
    // brevity of this sample, properties like vertex position are hard-coded:
    try {
      vertices =
        file.request_properties_from_element("vertex", {"x", "y", "z"});
    } catch (const std::exception &e) {
      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    file.read(*file_stream);

    if (vertices)
      std::cout << "\tRead " << vertices->count << " total vertices "
                << std::endl;


    const size_t numVerticesBytes = vertices->buffer.size_bytes();


    std::vector<float3> verts(vertices->count);
    std::memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);
    verts_ = verts;

    int idx = 0;
    for (const auto &point_tinyply : verts) {
      if (idx == 0) {
        points_ = Eigen::Vector3d(static_cast<double>(point_tinyply.x),
                                  static_cast<double>(point_tinyply.y),
                                  static_cast<double>(point_tinyply.z));
      } else {
        points_.conservativeResize(points_.rows(), points_.cols() + 1);
        points_.col(points_.cols() - 1) =
          Eigen::Vector3d(static_cast<double>(point_tinyply.x),
                          static_cast<double>(point_tinyply.y),
                          static_cast<double>(point_tinyply.z));
      }
      idx += 1;
    }

    kd_tree_.SetMatrixData(points_);

  } catch (const std::exception &e) {
    std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
  }
}

void MotionPlanner::getBounds() {
  bounds_.min = verts_[0];
  bounds_.max = verts_[0];


  for (const auto &ver : verts_) {
    if (ver.x < bounds_.min.x) {
      bounds_.min.x = ver.x;
    }

    if (ver.x > bounds_.max.x) {
      bounds_.max.x = ver.x;
    }

    if (ver.y < bounds_.min.y) {
      bounds_.min.y = ver.y;
    }

    if (ver.y > bounds_.max.y) {
      bounds_.max.y = ver.y;
    }

    if (ver.z > bounds_.max.z) {
      bounds_.max.z = ver.z;
    }
  }
}

bool MotionPlanner::plan() {
  bool solution_found = false;

  // construct the state space we are planning in
  auto space(std::make_shared<ob::SE3StateSpace>());

  // set the bounds for the R^3 part of SE(3)
  ob::RealVectorBounds bounds(3);

  bounds.setLow(0, bounds_.min.x);
  bounds.setLow(1, bounds_.min.y);
  bounds.setLow(2, bounds_.min.z);
  bounds.setHigh(0, bounds_.max.x);
  bounds.setHigh(1, bounds_.max.y);
  bounds.setHigh(2, bounds_.max.z);

  space->setBounds(bounds);

  // define a simple setup class
  og::SimpleSetup ss(space);

  // set state validity checking for this space
  ss.setStateValidityChecker(
    [&](const ob::State *state) { return isStateValid(state); });

  // create a random start state
  ob::ScopedState<> start(space);
  start.random();

  // create a random goal state
  ob::ScopedState<> goal(space);
  goal.random();

  // set the start and goal states
  ss.setStartAndGoalStates(start, goal);

  // this call is optional, but we put it in to get more output information
  ss.setup();
  ss.print();

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = ss.solve(1.0);

  if (solved) {
    std::cout << "Found solution:" << std::endl;

    ss.simplifySolution();
    ss.getSolutionPath().print(std::cout);

    path_.clear();
    path_ = ss.getSolutionPath().getStates();

    vecs_.clear();
    for (auto const &pos : path_) {
      vecs_.push_back(stateToEigen(pos));
    }

    if (path_.size() > 1) {
      solution_found = true;
    }
  } else {
    std::cout << "No solution found" << std::endl;
  }

  return solution_found;
}

Eigen::Vector3d MotionPlanner::stateToEigen(const ompl::base::State *state) {
  // cast the abstract state type to the type we expect
  const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

  // extract the first component of the state and cast it to what we expect
  const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

  // extract the second component of the state and cast it to what we expect
  //  const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

  // check validity of state defined by pos & rot
  float x = pos->values[0];
  float y = pos->values[1];
  float z = pos->values[2];

  Eigen::Vector3d query_pos{x, y, z};
  return query_pos;
}

bool MotionPlanner::isStateValid(const ob::State *state) {
  // cast the abstract state type to the type we expect
  const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

  // extract the first component of the state and cast it to what we expect
  const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

  // extract the second component of the state and cast it to what we expect
  //  const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

  // check validity of state defined by pos & rot
  float x = pos->values[0];
  float y = pos->values[1];
  float z = pos->values[2];

  Eigen::Vector3d query_pos{x, y, z};
  float range = 1;

  return searchRadius(query_pos, range);
}

bool MotionPlanner::searchRadius(const Eigen::Vector3d &query_point,
                                 const double radius) {
  std::vector<int> indices;
  std::vector<double> distances_squared;
  kd_tree_.SearchRadius(query_point, radius, indices, distances_squared);

  if (indices.size() == 0) {
    return true;
  }

  for (const auto &close_point_idx : indices) {
    // get point, check if within drone body
    Eigen::Vector3d close_point = points_.col(close_point_idx);
    // project point on each body axis and check distance
    Eigen::Vector3d close_point_body = (close_point - query_point);
    if (std::abs(close_point_body.x()) <= radius &&
        std::abs(close_point_body.y()) <= radius &&
        std::abs(close_point_body.z()) <= radius) {
      // point is in collision
      return false;
    }
  }

  return true;
}

void MotionPlanner::executePath() {
  // initialization
  assert(!vecs_.empty());
  assert(vecs_.at(0).x() != NULL);

  // compute trajectory
  std::size_t num_waypoints = vecs_.size();
  int scaling = 10;

  std::vector<Eigen::Vector3d> way_points;

  for (int i = 0; i < int(num_waypoints - 1); i++) {
    Eigen::Vector3d diff_vec = vecs_.at(i + 1) - vecs_.at(i);
    double norm = diff_vec.norm();
    for (int j = 0; j < int(norm * scaling); j++) {
      way_points.push_back(vecs_.at(i) + j * (diff_vec / (norm * scaling)));
    }
  }

  // Flightmare
  std::shared_ptr<Quadrotor> quad_ptr_ = std::make_shared<Quadrotor>();
  std::shared_ptr<RGBCamera> rgb_camera_ = std::make_shared<RGBCamera>();

  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(720);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  quad_ptr_->addRGBCamera(rgb_camera_);


  QuadState quad_state_;
  quad_state_.setZero();
  quad_state_.x[QS::POSX] = (Scalar)vecs_.at(0).x();
  quad_state_.x[QS::POSY] = (Scalar)vecs_.at(0).y();
  quad_state_.x[QS::POSZ] = (Scalar)vecs_.at(0).z();

  quad_ptr_->reset(quad_state_);

  // connect unity
  std::shared_ptr<UnityBridge> unity_bridge_ptr_ = UnityBridge::getInstance();
  unity_bridge_ptr_->addQuadrotor(quad_ptr_);
  bool unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  assert(unity_ready_);

  int counter = 0;

  while (unity_ready_) {
    quad_state_.x[QS::POSX] = (Scalar)way_points.at(counter).x();
    quad_state_.x[QS::POSY] = (Scalar)way_points.at(counter).y();
    quad_state_.x[QS::POSZ] = (Scalar)way_points.at(counter).z();
    quad_state_.x[QS::ATTW] = (Scalar)0;
    quad_state_.x[QS::ATTX] = (Scalar)0;
    quad_state_.x[QS::ATTY] = (Scalar)0;
    quad_state_.x[QS::ATTZ] = (Scalar)0;

    quad_ptr_->setState(quad_state_);

    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();

    counter += 1;
    counter = counter % way_points.size();
  }
}

}  // namespace motion_planning

int main(int argc, char *argv[]) {
  motion_planning::MotionPlanner motion_planner;

  motion_planner.run();

  return 0;
}