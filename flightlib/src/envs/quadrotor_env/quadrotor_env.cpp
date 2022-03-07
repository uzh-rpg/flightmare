#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"

namespace flightlib {

QuadrotorEnv::QuadrotorEnv()
  : QuadrotorEnv(getenv("FLIGHTMARE_PATH") +
                   std::string("/flightpy/configs/control/config.yaml"),
                 0) {}

QuadrotorEnv::QuadrotorEnv(const std::string &cfg_path, const int env_id)
  : EnvBase() {
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

QuadrotorEnv::QuadrotorEnv(const YAML::Node &cfg_node, const int env_id)
  : EnvBase(), cfg_(cfg_node) {
  //
  init();
  env_id_ = env_id;
}

void QuadrotorEnv::init() {
  //
  goal_pos_ << 0.0, 0.0, 5.0;
  //
  quad_ptr_ = std::make_shared<Quadrotor>();
  // update dynamics
  QuadrotorDynamics dynamics;
  dynamics.updateParams(cfg_);
  quad_ptr_->updateDynamics(dynamics);

  // define a bounding box {xmin, xmax, ymin, ymax, zmin, zmax}
  world_box_ << -20, 20, -20, 20, -0.0, 20;
  if (!quad_ptr_->setWorldBox(world_box_)) {
    logger_.error("cannot set wolrd box");
  };

  // define input and output dimension for the environment
  obs_dim_ = quadenv::kNObs;
  act_dim_ = quadenv::kNAct;
  rew_dim_ = 0;

  // load parameters
  loadParam(cfg_);

  // add camera
  if (use_camera_) {
    rgb_camera_ = std::make_shared<RGBCamera>();
    if (!configCamera(cfg_, rgb_camera_)) {
      logger_.error(
        "Cannot config RGB Camera. Something wrong with the config file");
    };

    quad_ptr_->addRGBCamera(rgb_camera_);
    //
    img_width_ = rgb_camera_->getWidth();
    img_height_ = rgb_camera_->getHeight();
    rgb_img_ = cv::Mat::zeros(img_height_, img_width_,
                              CV_MAKETYPE(CV_8U, rgb_camera_->getChannels()));
    depth_img_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
  }

  // use single rotor control or bodyrate control
  if (rotor_ctrl_ == 0) {
    act_mean_ = Vector<quadenv::kNAct>::Ones() *
                quad_ptr_->getDynamics().getSingleThrustMax() / 2;
    act_std_ = Vector<quadenv::kNAct>::Ones() *
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

QuadrotorEnv::~QuadrotorEnv() {}

bool QuadrotorEnv::reset(Ref<Vector<>> obs) {
  quad_state_.setZero();
  pi_act_.setZero();

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
  quad_state_.x(QS::ATTW) = uniform_dist_(random_gen_);
  quad_state_.x(QS::ATTX) = uniform_dist_(random_gen_);
  quad_state_.x(QS::ATTY) = uniform_dist_(random_gen_);
  quad_state_.x(QS::ATTZ) = uniform_dist_(random_gen_);
  quad_state_.qx /= quad_state_.qx.norm();

  // reset quadrotor with random states
  quad_ptr_->reset(quad_state_);

  // reset control command
  cmd_.t = 0.0;
  if (rotor_ctrl_ == 0) {
    cmd_.setCmdMode(0);
    cmd_.thrusts.setZero();
  } else if (rotor_ctrl_ == 1) {
    cmd_.setCmdMode(1);
    cmd_.collective_thrust = 0;
    cmd_.omega.setZero();
  }

  // obtain observations
  getObs(obs);
  return true;
}

bool QuadrotorEnv::reset(Ref<Vector<>> obs, bool random) { return reset(obs); }

bool QuadrotorEnv::getObs(Ref<Vector<>> obs) {
  if (obs.size() != obs_dim_) {
    logger_.error("Observation dimension mismatch. %d != %d", obs.size(),
                  obs_dim_);
    return false;
  }

  quad_ptr_->getState(&quad_state_);

  // convert quaternion to euler angle
  Vector<9> ori = Map<Vector<>>(quad_state_.R().data(), quad_state_.R().size());

  // observation dim : 3 + 9 + 3 = 15
  obs.segment<quadenv::kNObs>(quadenv::kObs) << quad_state_.p, ori,
    quad_state_.v;

  // use the following observations if use single rotor thrusts as input
  // observation dim : 3 + 9 + 3 + 3= 18
  // obs.segment<quadenv::kNObs>(quadenv::kObs) << quad_state_.p, ori,
  //   quad_state_.v, quad_state_.w;
  return true;
}

bool QuadrotorEnv::step(const Ref<Vector<>> act, Ref<Vector<>> obs,
                        Ref<Vector<>> reward) {
  if (!act.allFinite() || act.rows() != act_dim_ || rew_dim_ != reward.rows())
    return false;
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

  // update observations
  getObs(obs);

  // ---------------------- reward function design
  // - position tracking
  const Scalar pos_reward = pos_coeff_ * (quad_state_.p - goal_pos_).norm();

  // - orientation tracking
  const Scalar ori_reward =
    ori_coeff_ *
    (quad_state_.q().toRotationMatrix().eulerAngles(2, 1, 0)).norm();

  // - linear velocity tracking
  const Scalar lin_vel_reward = lin_vel_coeff_ * quad_state_.v.norm();

  // - angular velocity tracking
  const Scalar ang_vel_reward = ang_vel_coeff_ * quad_state_.w.norm();

  const Scalar total_reward =
    pos_reward + ori_reward + lin_vel_reward + ang_vel_reward;

  reward << pos_reward, ori_reward, lin_vel_reward, ang_vel_reward,
    total_reward;
  return true;
}

bool QuadrotorEnv::isTerminalState(Scalar &reward) {
  if (quad_state_.x(QS::POSZ) <= 0.02) {
    reward = -1.0;
    return true;
  }

  if (cmd_.t >= max_t_ - sim_dt_) {
    reward = 0.0;
    return true;
  }
  return false;
}


bool QuadrotorEnv::getQuadAct(Ref<Vector<>> act) const {
  if (cmd_.t >= 0.0 && pi_act_.allFinite() && (act.size() == pi_act_.size())) {
    act = pi_act_;
    return true;
  }
  return false;
}

bool QuadrotorEnv::getQuadState(Ref<Vector<>> obs) const {
  if (quad_state_.t >= 0.0 && (obs.rows() == quadenv::kNQuadState)) {
    obs << quad_state_.t, quad_state_.p, quad_state_.qx, quad_state_.v,
      quad_state_.w, quad_state_.a, quad_ptr_->getMotorOmega(),
      quad_ptr_->getMotorThrusts();
    return true;
  }
  logger_.error("Get Quadrotor state failed.");
  return false;
}

bool QuadrotorEnv::getDepthImage(Ref<DepthImgVector<>> depth_img) {
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


bool QuadrotorEnv::getImage(Ref<ImgVector<>> img, const bool rgb) {
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


bool QuadrotorEnv::loadParam(const YAML::Node &cfg) {
  if (cfg["simulation"]) {
    sim_dt_ = cfg["simulation"]["sim_dt"].as<Scalar>();
    max_t_ = cfg["simulation"]["max_t"].as<Scalar>();
    rotor_ctrl_ = cfg["simulation"]["rotor_ctrl"].as<int>();
  } else {
    logger_.error("Cannot load [quadrotor_env] parameters");
    return false;
  }

  if (cfg["rewards"]) {
    // load reinforcement learning related parameters
    pos_coeff_ = cfg["rewards"]["pos_coeff"].as<Scalar>();
    ori_coeff_ = cfg["rewards"]["ori_coeff"].as<Scalar>();
    lin_vel_coeff_ = cfg["rewards"]["lin_vel_coeff"].as<Scalar>();
    ang_vel_coeff_ = cfg["rewards"]["ang_vel_coeff"].as<Scalar>();
    // load reward settings
    reward_names_ = cfg["rewards"]["names"].as<std::vector<std::string>>();

    rew_dim_ = cfg["rewards"]["names"].as<std::vector<std::string>>().size();
  } else {
    logger_.error("Cannot load [rewards] parameters");
    return false;
  }
  return true;
}

bool QuadrotorEnv::configCamera(const YAML::Node &cfg,
                                const std::shared_ptr<RGBCamera> camera) {
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

bool QuadrotorEnv::addQuadrotorToUnity(
  const std::shared_ptr<UnityBridge> bridge) {
  if (!quad_ptr_) return false;
  bridge->addQuadrotor(quad_ptr_);
  return true;
}

std::ostream &operator<<(std::ostream &os, const QuadrotorEnv &quad_env) {
  os.precision(3);
  os << "Quadrotor Environment:\n"
     << "obs dim =            [" << quad_env.obs_dim_ << "]\n"
     << "act dim =            [" << quad_env.act_dim_ << "]\n"
     << "sim dt =             [" << quad_env.sim_dt_ << "]\n"
     << "max_t =              [" << quad_env.max_t_ << "]\n"
     << "act_mean =           [" << quad_env.act_mean_.transpose() << "]\n"
     << "act_std =            [" << quad_env.act_std_.transpose() << "]\n"
     << "obs_mean =           [" << quad_env.obs_mean_.transpose() << "]\n"
     << "obs_std =            [" << quad_env.obs_std_.transpose() << "]"
     << std::endl;
  os.precision();
  return os;
}

}  // namespace flightlib