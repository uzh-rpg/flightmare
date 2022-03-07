
#include "flightlib/envs/vision_env/vision_env.hpp"

namespace flightlib {

VisionEnv::VisionEnv()
  : VisionEnv(getenv("FLIGHTMARE_PATH") +
                std::string("/flightpy/configs/vision/config.yaml"),
              0) {
  std::cout << "Oh shit" << std::endl;
}

VisionEnv::VisionEnv(const std::string &cfg_path, const int env_id)
  : EnvBase() {
  std::cout << "hello" << std::endl;
  // check if configuration file exist
  if (!(file_exists(cfg_path))) {
    logger_.error("Configuration file %s does not exists.", cfg_path);
  }
  // load configuration file
  cfg_ = YAML::LoadFile(cfg_path);
  //
  init();
  env_id_ = env_id;
}

VisionEnv::VisionEnv(const YAML::Node &cfg_node, const int env_id) : EnvBase() {
  std::cout << "hello" << std::endl;
  std::cout << cfg_node << std::endl;
  cfg_ = cfg_node;
  std::cout << cfg_ << std::endl;
  sleep(5);

  //
  std::cout << "init1" << std::endl;
  init();
  std::cout << "init2" << std::endl;
  env_id_ = env_id;
}

void VisionEnv::init() {
  std::cout << "init" << std::endl;
  //
  unity_render_offset_ << 0.0, 0.0, 0.0;
  goal_pos_ << 80.0, 0.0, 1.0;
  start_pos_ << 0.0, 0.0, 1.0;

  cmd_.setZeros();

  // create quadrotors
  quad_ptr_ = std::make_shared<Quadrotor>();
  // update dynamics
  QuadrotorDynamics dynamics;
  dynamics.updateParams(cfg_);
  quad_ptr_->updateDynamics(dynamics);


  // define a bounding box {xmin, xmax, ymin, ymax, zmin, zmax}
  world_box_ << -100, 100, -100, 100, -0.0, 100;
  if (!quad_ptr_->setWorldBox(world_box_)) {
    logger_.error("cannot set wolrd box");
  };

  // define input and output dimension for the environment
  obs_dim_ = visionenv::kNObs;
  act_dim_ = visionenv::kNAct;
  rew_dim_ = 0;
  num_detected_obstacles_ = visionenv::kNObstacles;

  std::cout << "init" << std::endl;
  // load parameters
  loadParam(cfg_);
  std::cout << "load params" << std::endl;

  // add camera
  if (!configCamera(cfg_)) {
    logger_.error(
      "Cannot config RGB Camera. Something wrong with the config file");
  }

  // add dynamic objects
  std::string dynamic_object_yaml =
    getenv("FLIGHTMARE_PATH") +
    std::string("/flightpy/configs/vision/dynamic_obstacles.yaml");
  if (!configDynamicObjects(dynamic_object_yaml)) {
    logger_.error(
      "Cannot config Dynamic Object Yaml. Something wrong with the config "
      "file");
  }

  // add static objects
  static_object_csv_ =
    getenv("FLIGHTMARE_PATH") +
    std::string("/flightpy/configs/vision/static_obstacles.csv");
  if (!configStaticObjects(static_object_csv_)) {
    logger_.error(
      "Cannot config Static Object. Something wrong with the config file");
  }

  // use single rotor control or bodyrate control
  if (rotor_ctrl_ == 0) {
    act_mean_ = Vector<visionenv::kNAct>::Ones() *
                quad_ptr_->getDynamics().getSingleThrustMax() / 2;
    act_std_ = Vector<visionenv::kNAct>::Ones() *
               quad_ptr_->getDynamics().getSingleThrustMax() / 2;
  } else if (rotor_ctrl_ == 1) {
    Scalar max_force = quad_ptr_->getDynamics().getForceMax();
    Vector<3> max_omega = quad_ptr_->getDynamics().getOmegaMax();
    //
    act_mean_ << (max_force / quad_ptr_->getMass()) / 2, 0.0, 0.0, 0.0;
    act_std_ << (max_force / quad_ptr_->getMass()) / 2, max_omega.x(),
      max_omega.y(), max_omega.z();
  }
}

VisionEnv::~VisionEnv() {}

bool VisionEnv::reset(Ref<Vector<>> obs) {
  quad_state_.setZero();
  pi_act_.setZero();
  obstacle_collision_ = false;

  // randomly reset the quadrotor state
  // reset position
  quad_state_.x(QS::POSX) = uniform_dist_(random_gen_);
  quad_state_.x(QS::POSY) = uniform_dist_(random_gen_);
  quad_state_.x(QS::POSZ) = uniform_dist_(random_gen_) + 5;
  if (quad_state_.x(QS::POSZ) < -0.0)
    quad_state_.x(QS::POSZ) = -quad_state_.x(QS::POSZ);
  // reset linear velocity
  quad_state_.x(QS::VELX) = uniform_dist_(random_gen_);
  quad_state_.x(QS::VELY) = uniform_dist_(random_gen_);
  quad_state_.x(QS::VELZ) = uniform_dist_(random_gen_);
  // reset orientation
  // quad_state_.x(QS::ATTW) = uniform_dist_(random_gen_);
  // quad_state_.x(QS::ATTX) = uniform_dist_(random_gen_);
  // quad_state_.x(QS::ATTY) = uniform_dist_(random_gen_);
  // quad_state_.x(QS::ATTZ) = uniform_dist_(random_gen_);
  // quad_state_.qx /= quad_state_.qx.norm();

  // reset quadrotor with random states
  quad_ptr_->reset(quad_state_);

  // reset control command
  cmd_.t = 0.0;
  cmd_.setCmdMode(rotor_ctrl_);
  if (rotor_ctrl_ == 0) {
    cmd_.thrusts.setZero();
  } else if (rotor_ctrl_ == 1) {
    cmd_.collective_thrust = 0;
    cmd_.omega.setZero();
  }

  start_pos_ = quad_state_.p;
  // obtain observations
  getObs(obs);
  return true;
}

bool VisionEnv::reset(Ref<Vector<>> obs, bool random) { return reset(obs); }

bool VisionEnv::getObs(Ref<Vector<>> obs) {
  if (obs.size() != obs_dim_) {
    logger_.error("Observation dimension mismatch. %d != %d", obs.size(),
                  obs_dim_);
    return false;
  }

  Vector<9> ori = Map<Vector<>>(quad_state_.R().data(), quad_state_.R().size());

  Vector<visionenv::kNObstacles * visionenv::kNObstaclesState> obstacle_obs;

  getObstacleState(obstacle_obs);

  // 3 + 9 + 3 + 3 + 3*N
  obs << quad_state_.p - goal_pos_, ori, quad_state_.v, quad_state_.w,
    obstacle_obs;
  return true;
}

bool VisionEnv::getObstacleState(Ref<Vector<>> obs_state) {
  quad_ptr_->getState(&quad_state_);
  // compute relative distance to dynamic obstacles
  std::vector<Scalar> relative_pos_norm;
  std::vector<Vector<3>> relative_pos;
  for (int i = 0; i < (int)dynamic_objects_.size(); i++) {
    Vector<3> delta_pos = quad_state_.p - dynamic_objects_[i]->getPos();
    relative_pos.push_back(delta_pos);
    relative_pos_norm.push_back(delta_pos.norm());
  }

  // compute relatiev distance to static obstacles
  for (int i = 0; i < (int)static_objects_.size(); i++) {
    Vector<3> delta_pos = quad_state_.p - static_objects_[i]->getPos();
    relative_pos.push_back(delta_pos);
    relative_pos_norm.push_back(delta_pos.norm());
  }

  size_t idx = 0;
  for (size_t sort_idx : sort_indexes(relative_pos_norm)) {
    if (idx >= visionenv::kNObstacles) break;

    if (idx < relative_pos.size()) {
      // if enough obstacles in the environment
      if (relative_pos_norm[sort_idx] <= max_detection_range_) {
        // if obstacles are within detection range
        obs_state.segment<3>(idx * 3) << relative_pos[sort_idx];
      } else {
        // if obstacles are beyong detection range
        obs_state.segment<3>(idx * 3) = Vector<3>(
          max_detection_range_, max_detection_range_, max_detection_range_);
      }
    } else {
      // if not enough obstacles in the environment
      obs_state.segment<3>(idx * 3) = Vector<3>(
        max_detection_range_, max_detection_range_, max_detection_range_);
    }
    idx += 1;
  }
  return true;
}

bool VisionEnv::step(const Ref<Vector<>> act, Ref<Vector<>> obs,
                     Ref<Vector<>> reward) {
  if (!act.allFinite() || act.rows() != act_dim_ || rew_dim_ != reward.rows()) {
    return false;
    logger_.error(
      "Cannot run environment simulation. dimension mismatch or invalid "
      "actions.");
  }
  //
  pi_act_ = act.cwiseProduct(act_std_) + act_mean_;

  cmd_.t += sim_dt_;
  quad_state_.t += sim_dt_;

  if (rotor_ctrl_ == 0) {
    cmd_.thrusts = pi_act_;
  } else if (rotor_ctrl_ == 1) {
    cmd_.collective_thrust = pi_act_(0);
    cmd_.omega = pi_act_.segment<3>(1);
  }

  // simulate quadrotor
  quad_ptr_->run(cmd_, sim_dt_);
  // update quadrotor state and old quad_state
  quad_old_state_ = quad_state_;
  quad_ptr_->getState(&quad_state_);

  // simulate dynamic obstacles
  simDynamicObstacles(sim_dt_);

  // update observations
  getObs(obs);

  return computeReward(reward);
}

bool VisionEnv::simDynamicObstacles(const Scalar dt) {
  if (dynamic_objects_.size() <= 0) {
    logger_.warn(
      "No Dynamic Obstacles defined. Skipping dynamic obstacles simulation.");
    return false;
  }
  for (int i = 0; i < int(dynamic_objects_.size()); i++) {
    dynamic_objects_[i]->run(sim_dt_);
  }
  return true;
}

bool VisionEnv::computeReward(Ref<Vector<>> reward) {
  // Vector<3> goal_lin_vel = Vector<3>(5, 0.0, 0.0);
  // Scalar lin_vel_reward =
  //   -0.03 * std::pow((quad_state_.v - goal_lin_vel).norm() / 5.0, 2);
  Scalar prog_rew = 0.0;
  const Scalar gate_dist = (goal_pos_ - quad_state_.p).norm();
  const Scalar gate_dist_prev = (goal_pos_ - quad_old_state_.p).norm();
  // std::cout << gate_dist << "  " << gate_dist_prev << std::endl;
  prog_rew = (gate_dist_prev - gate_dist);


  // get obstacle observations
  Vector<visionenv::kNObstaclesState * visionenv::kNObstacles> obstacle_obs;
  getObstacleState(obstacle_obs);

  Scalar distance_reward = 0.0;
  for (int idx = 0; idx < visionenv::kNObstacles; idx++) {
    Vector<3> delta_pos = obstacle_obs.segment<3>(idx * 3);
    // Scalar dist = std::max(current->getDistance(quad_state_.p),
    // (Scalar)0.001);
    const Scalar dist = delta_pos.norm();
    distance_reward += -0.03 * ((1 / dist) - (1 / max_detection_range_));

    if (dist <= 0.5) {
      obstacle_collision_ = true;
    }
  }
  // ---------------------- reward function design
  // - position tracking
  // const Scalar pos_reward = -0.002 * (quad_state_.p - goal_pos_).norm();

  // - orientation tracking
  const Scalar ori_reward =
    -0.002 * (quad_state_.q().toRotationMatrix().eulerAngles(2, 1, 0)).norm();

  // - linear velocity tracking
  // const Scalar lin_vel_reward = -0.0001 * quad_state_.v.norm();

  // - angular velocity tracking
  const Scalar ang_vel_reward = -0.0001 * quad_state_.w.norm();

  const Scalar total_reward = prog_rew + distance_reward + ang_vel_reward;

  reward << prog_rew, distance_reward, ori_reward, ang_vel_reward, total_reward;
  return true;
}

bool VisionEnv::isTerminalState(Scalar &reward) {
  if (obstacle_collision_) {
    reward = -1.0;
    return true;
  }
  if (quad_state_.x(QS::POSZ) <= world_box_(QS::POSZ, 0) + 0.01) {
    reward = -1.0;
    return true;
  }

  if (quad_state_.x(QS::POSZ) >= world_box_(QS::POSZ, 1) - 0.01) {
    reward = -1.0;
    return true;
  }

  if (cmd_.t >= max_t_ - sim_dt_) {
    reward = 0.0;
    return true;
  }

  // project current position onto current path segment
  Vector<3> V = quad_state_.p - start_pos_;
  Vector<3> P12 = goal_pos_ - start_pos_;
  Scalar proj = V.cross(P12).norm() / P12.norm();
  if (proj >= 3) {
    reward = -1.0;
    return true;
  }

  bool x_valid = quad_state_.p(QS::POSX) > world_box_(QS::POSX, 0) &&
                 quad_state_.p(QS::POSX) < world_box_(QS::POSX, 1);
  bool y_valid = quad_state_.p(QS::POSY) > world_box_(QS::POSY, 0) &&
                 quad_state_.p(QS::POSY) < world_box_(QS::POSY, 1);

  if (!x_valid || !y_valid) {
    reward = -1.0;
    return true;
  }

  if (quad_state_.p(QS::POSX) > goal_pos_.x()) {
    reward = 10.0;
    return true;
  }

  return false;
}


bool VisionEnv::getQuadAct(Ref<Vector<>> act) const {
  if (cmd_.t >= 0.0 && pi_act_.allFinite() && (act.size() == pi_act_.size())) {
    act = pi_act_;
    return true;
  }
  return false;
}

bool VisionEnv::getQuadState(Ref<Vector<>> obs) const {
  if (quad_state_.t >= 0.0 && (obs.rows() == visionenv::kNQuadState)) {
    obs << quad_state_.t, quad_state_.p, quad_state_.qx, quad_state_.v,
      quad_state_.w, quad_state_.a, quad_ptr_->getMotorOmega(),
      quad_ptr_->getMotorThrusts();
    return true;
  }
  logger_.error("Get Quadrotor state failed.");
  return false;
}

bool VisionEnv::getDepthImage(Ref<DepthImgVector<>> depth_img) {
  if (!rgb_camera_ || !rgb_camera_->getEnabledLayers()[0]) {
    logger_.error(
      "No RGB Camera or depth map is not enabled. Cannot retrieve depth "
      "images.");
    return false;
  }
  rgb_camera_->getDepthMap(depth_img_);

  depth_img = Map<DepthImgVector<>>((float_t *)depth_img_.data,
                                    depth_img_.rows * depth_img_.cols);
  return true;
}


bool VisionEnv::getImage(Ref<ImgVector<>> img, const bool rgb) {
  if (!rgb_camera_) {
    logger_.error("No Camera! Cannot retrieve Images.");
    return false;
  }

  rgb_camera_->getRGBImage(rgb_img_);

  if (rgb_img_.rows != img_height_ || rgb_img_.cols != img_width_) {
    logger_.error(
      "Image resolution mismatch. Aborting.. Image rows %d != %d, Image cols "
      "%d != %d",
      rgb_img_.rows, img_height_, rgb_img_.cols, img_width_);
    return false;
  }

  if (!rgb) {
    // converting rgb image to gray image
    cvtColor(rgb_img_, gray_img_, CV_RGB2GRAY);
    // map cv::Mat data to Eiegn::Vector
    img = Map<ImgVector<>>(gray_img_.data, gray_img_.rows * gray_img_.cols);
  } else {
    img = Map<ImgVector<>>(rgb_img_.data, rgb_img_.rows * rgb_img_.cols *
                                            rgb_camera_->getChannels());
  }
  return true;
}


bool VisionEnv::loadParam(const YAML::Node &cfg) {
  std::cout << "init 1" << std::endl;
  if (cfg["simulation"]) {
    sim_dt_ = cfg["simulation"]["sim_dt"].as<Scalar>();
    max_t_ = cfg["simulation"]["max_t"].as<Scalar>();
    rotor_ctrl_ = cfg["simulation"]["rotor_ctrl"].as<int>();
  } else {
    logger_.error("Cannot load [quadrotor_env] parameters");
    return false;
  }

  std::cout << "init 2" << std::endl;
  if (cfg["rewards"]) {
    // load reinforcement learning related parameters
    dummy_coeff_ = cfg["rewards"]["dummy_coeff"].as<Scalar>();
    // load reward settings
    reward_names_ = cfg["rewards"]["names"].as<std::vector<std::string>>();

    rew_dim_ = cfg["rewards"]["names"].as<std::vector<std::string>>().size();
  } else {
    logger_.error("Cannot load [rewards] parameters");
    return false;
  }

  std::cout << "init 3" << std::endl;
  // environment
  if (cfg["unity"]) {
    unity_render_ = cfg["unity"]["render"].as<bool>();
    scene_id_ = cfg["unity"]["scene_id"].as<SceneID>();
  }
  // std::string scene_file =
  //   getenv("FLIGHTMARE_PATH") + std::string("/flightpy/configs/scene.yaml");
  // // check if configuration file exist
  // if (!(file_exists(scene_file))) {
  //   logger_.error("Unity scene configuration file %s does not exists.",
  //                 scene_file);
  // }
  // // load configuration file
  // YAML::Node scene_cfg_node = YAML::LoadFile(scene_file);
  // std::string scene_idx = "scene_" + std::to_string(scene_id_);

  // std::vector<Scalar> render_offset =
  //   scene_cfg_node[scene_idx]["render_offset"].as<std::vector<Scalar>>();
  // unity_render_offset_ = Vector<3>(render_offset.data());
  std::cout << "init 4" << std::endl;
  return true;
}

bool VisionEnv::configDynamicObjects(const std::string &yaml_file) {
  //
  if (!(file_exists(yaml_file))) {
    logger_.error("Configuration file %s does not exists.", yaml_file);
    return false;
  }
  YAML::Node cfg_node = YAML::LoadFile(yaml_file);

  // logger_.info("Configuring dynamic objects");

  int num_objects = cfg_node["N"].as<int>();
  // create static objects
  for (int i = 0; i < num_objects; i++) {
    std::string object_id = "Object" + std::to_string(i + 1);
    std::string prefab_id = cfg_node[object_id]["prefab"].as<std::string>();
    std::shared_ptr<UnityObject> obj =
      std::make_shared<UnityObject>(object_id, prefab_id);

    // load location, rotation and size
    std::vector<Scalar> posvec =
      (cfg_node[object_id]["position"]).as<std::vector<Scalar>>();
    std::vector<Scalar> rotvec =
      (cfg_node[object_id]["rotation"]).as<std::vector<Scalar>>();
    std::vector<Scalar> scalevec =
      (cfg_node[object_id]["scale"]).as<std::vector<Scalar>>();

    obj->setPosition(Vector<3>(posvec.data()));
    obj->setRotation(Quaternion(rotvec.data()));
    obj->setSize(Vector<3>(1.0, 1.0, 1.0));
    obj->setScale(Vector<3>(scalevec.data()));

    std::string csv_name = cfg_node[object_id]["csvtraj"].as<std::string>();
    std::string csv_file = getenv("FLIGHTMARE_PATH") +
                           std::string("/flightpy/configs/vision/csvtrajs/") +
                           csv_name + std::string(".csv");
    if (!(file_exists(csv_file))) {
      logger_.error("Configuration file %s does not exists.", csv_file);
      return false;
    }
    obj->loadTrajectory(csv_file);

    dynamic_objects_.push_back(obj);
  }
  return true;
}

bool VisionEnv::configStaticObjects(const std::string &csv_file) {
  //
  if (!(file_exists(csv_file))) {
    logger_.error("Configuration file %s does not exists.", csv_file);
    return false;
  }
  std::ifstream infile(csv_file);
  int i = 0;
  for (auto &row : CSVRange(infile)) {
    // Read column 0 for time
    std::string object_id = "StaticObject" + std::to_string(i + 1);
    std::string prefab_id = (std::string)row[0];

    //
    std::shared_ptr<UnityObject> obj =
      std::make_shared<UnityObject>(object_id, prefab_id);

    //
    Vector<3> pos;
    pos << std::stod((std::string)row[1]), std::stod((std::string)row[2]),
      std::stod((std::string)row[3]);

    Quaternion quat;
    quat.w() = std::stod((std::string)row[3]);
    quat.x() = std::stod((std::string)row[4]);
    quat.y() = std::stod((std::string)row[5]);
    quat.z() = std::stod((std::string)row[6]);

    //
    obj->setPosition(pos);
    obj->setRotation(quat);
    static_objects_.push_back(obj);
  }

  return true;
}

bool VisionEnv::configCamera(const YAML::Node &cfg) {
  if (!cfg["rgb_camera"]) {
    logger_.error("Cannot config RGB Camera");
    return false;
  }

  if (!cfg["rgb_camera"]["on"].as<bool>()) {
    logger_.warn("Camera is off. Please turn it on.");
    return false;
  }

  if (quad_ptr_->getNumCamera() >= 1) {
    logger_.warn("Camera has been added. Skipping the camera configuration.");
    return false;
  }

  // create camera
  rgb_camera_ = std::make_shared<RGBCamera>();

  // load camera settings
  std::vector<Scalar> t_BC_vec =
    cfg["rgb_camera"]["t_BC"].as<std::vector<Scalar>>();
  std::vector<Scalar> r_BC_vec =
    cfg["rgb_camera"]["r_BC"].as<std::vector<Scalar>>();

  //
  Vector<3> t_BC(t_BC_vec.data());
  Matrix<3, 3> r_BC =
    (AngleAxis(r_BC_vec[2] * M_PI / 180.0, Vector<3>::UnitZ()) *
     AngleAxis(r_BC_vec[1] * M_PI / 180.0, Vector<3>::UnitY()) *
     AngleAxis(r_BC_vec[0] * M_PI / 180.0, Vector<3>::UnitX()))
      .toRotationMatrix();
  std::vector<bool> post_processing = {false, false, false};
  post_processing[0] = cfg["rgb_camera"]["enable_depth"].as<bool>();
  post_processing[1] = cfg["rgb_camera"]["enable_segmentation"].as<bool>();
  post_processing[2] = cfg["rgb_camera"]["enable_opticalflow"].as<bool>();

  //
  rgb_camera_->setFOV(cfg["rgb_camera"]["fov"].as<Scalar>());
  rgb_camera_->setWidth(cfg["rgb_camera"]["width"].as<int>());
  rgb_camera_->setChannels(cfg["rgb_camera"]["channels"].as<int>());
  rgb_camera_->setHeight(cfg["rgb_camera"]["height"].as<int>());
  rgb_camera_->setRelPose(t_BC, r_BC);
  rgb_camera_->setPostProcessing(post_processing);


  // add camera to the quadrotor
  quad_ptr_->addRGBCamera(rgb_camera_);

  // adapt parameters
  img_width_ = rgb_camera_->getWidth();
  img_height_ = rgb_camera_->getHeight();
  rgb_img_ = cv::Mat::zeros(img_height_, img_width_,
                            CV_MAKETYPE(CV_8U, rgb_camera_->getChannels()));
  depth_img_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
  return true;
}

bool VisionEnv::addQuadrotorToUnity(const std::shared_ptr<UnityBridge> bridge) {
  if (!quad_ptr_) return false;
  bridge->addQuadrotor(quad_ptr_);

  for (int i = 0; i < (int)dynamic_objects_.size(); i++) {
    bridge->addDynamicObject(dynamic_objects_[i]);
  }

  //
  bridge->setPositionOffset(unity_render_offset_);
  bridge->setObjectCSV(static_object_csv_);
  return true;
}

bool VisionEnv::setUnity(bool render) {
  unity_render_ = render;
  if (!unity_render_ || unity_bridge_ptr_ != nullptr) {
    logger_.warn(
      "Unity render is False or Flightmare Bridge has been already created. "
      "Cannot set Unity.");
    return false;
  }
  // create unity bridge
  unity_bridge_ptr_ = UnityBridge::getInstance();
  // add objects to Unity

  addQuadrotorToUnity(unity_bridge_ptr_);

  logger_.info("Flightmare Bridge created.");
  return true;
}


bool VisionEnv::connectUnity(void) {
  if (unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}


FrameID VisionEnv::updateUnity(const FrameID frame_id) {
  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(frame_id);
    return unity_bridge_ptr_->handleOutput(frame_id);
  } else {
    return 0;
  }
}


void VisionEnv::disconnectUnity(void) {
  if (unity_bridge_ptr_ != nullptr) {
    unity_bridge_ptr_->disconnectUnity();
    unity_ready_ = false;
  } else {
    logger_.warn("Flightmare Unity Bridge is not initialized.");
  }
}

std::ostream &operator<<(std::ostream &os, const VisionEnv &vision_env) {
  os.precision(3);
  os << "Vision Environment:\n"
     << "obs dim =            [" << vision_env.obs_dim_ << "]\n"
     << "act dim =            [" << vision_env.act_dim_ << "]\n"
     << "sim dt =             [" << vision_env.sim_dt_ << "]\n"
     << "max_t =              [" << vision_env.max_t_ << "]\n"
     << "act_mean =           [" << vision_env.act_mean_.transpose() << "]\n"
     << "act_std =            [" << vision_env.act_std_.transpose() << "]\n"
     << "obs_mean =           [" << vision_env.obs_mean_.transpose() << "]\n"
     << "obs_std =            [" << vision_env.obs_std_.transpose() << "]"
     << std::endl;
  os.precision();
  return os;
}

}  // namespace flightlib