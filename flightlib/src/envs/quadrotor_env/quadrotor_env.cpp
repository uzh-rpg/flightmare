#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"

namespace flightlib {

QuadrotorEnv::QuadrotorEnv()
  : QuadrotorEnv(getenv("FLIGHTMARE_PATH") +
                 std::string("/flightlib/configs/quadrotor_env.yaml")) {}

QuadrotorEnv::QuadrotorEnv(const std::string &cfg_path)
  : EnvBase(),
    pos_coeff_(0.0),
    ori_coeff_(0.0),
    lin_vel_coeff_(0.0),
    ang_vel_coeff_(0.0),
    act_coeff_(0.0),
    goal_state_((Vector<quadenv::kNObs>() << 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0)
                  .finished()) {
  // load configuration file
  YAML::Node cfg_ = YAML::LoadFile(cfg_path);

  quadrotor_ptr_ = std::make_shared<Quadrotor>();
  // update dynamics
  QuadrotorDynamics dynamics;
  dynamics.updateParams(cfg_);
  quadrotor_ptr_->updateDynamics(dynamics);

  std::vector<float> camera_ori = (cfg_["quadrotor_env"]["camera_rot"]).as<std::vector<float>>();
  std::vector<float> camera_pos = (cfg_["quadrotor_env"]["camera_pos"]).as<std::vector<float>>();
  frame_height_ = quadenv::frame_height;
  frame_width_ = quadenv::frame_width;
  // define a bounding box
  std::vector<float> world_box = (cfg_["quadrotor_env"]["world_box"]).as<std::vector<float>>();
  world_box_ = Matrix<3, 2>(world_box.data());
  quadrotor_ptr_->setWorldBox(world_box_);

  // define input and output dimension for the environment
  obs_dim_ = quadenv::kNObs;
  act_dim_ = quadenv::kNAct;

  Scalar mass = quadrotor_ptr_->getMass();
  act_mean_ = Vector<quadenv::kNAct>::Ones() * (-mass * Gz) / 4;
  act_std_ = Vector<quadenv::kNAct>::Ones() * (-mass * 2 * Gz) / 4;

  // load parameters
  loadParam(cfg_);
  Vector<3> B_r_BC(camera_pos.data());
  // std::cout <<  "camera pos set" << std::endl;
  Matrix<3, 3> R_BC = Quaternion(euler2Quaternion(Vector<3>(camera_ori.data()))).toRotationMatrix();
  Quaternion quat(R_BC);
  // std::cout << "ROTATION MATRIX IS " << std::endl << R_BC << std::endl;
  // logger_.warn("camera pose is: x " + std::to_string(quat. x()) + " y "+ std::to_string(quat.y()) + " z " + std::to_string(quat.z()) + " w " + std::to_string(quat.w()));
  // std::cout <<  "camera ori set" << std::endl;
  rgb_camera = std::make_shared<RGBCamera>();
  rgb_camera->setFOV(quadenv::camera_FOV);
  rgb_camera->setHeight(frame_height_);
  rgb_camera->setWidth(frame_width_);
  rgb_camera->setRelPose(B_r_BC, R_BC);
  rgb_camera->setPostProcesscing(
                    std::vector<bool>{true, false, false});  // depth, segmentation, optical flow
  quadrotor_ptr_->addRGBCamera(rgb_camera);
  img_depth_ = cv::Mat::zeros(frame_width_, frame_height_, CV_32FC1);
}

QuadrotorEnv::~QuadrotorEnv() {}


//--------------------------------//
//----      Drone reset       ----//
//--------------------------------//


void QuadrotorEnv::setResetPose(Vector<3> &resetPosition, Vector<3> &resetRotation) {
  resetPosition_ = resetPosition;
  resetRotation_ = resetRotation;
  quadrotor_ptr_->box_center_ = resetPosition_;

  int goalDistance = 35;

  // Adjust goal position accordingly.
  goal_state_(0) = resetPosition(0);
  goal_state_(1) = resetPosition(1); 
  goal_state_(2) = resetPosition(2);

  // Euler angles
  goal_state_(3) = 0;
  goal_state_(4) = 0;
  goal_state_(5) = 0;


  // World box relative to drone reset position.
  world_box_ << world_box_(0)+resetPosition_(0), world_box_(3)+resetPosition_(0), world_box_(1)+resetPosition_(1),
                world_box_(4)+resetPosition_(1), world_box_(2), world_box_(5);

  quadrotor_ptr_->setWorldBox(world_box_);

  // Set quadrotor state.
  setQuadstateToResetState();
}

void QuadrotorEnv::setQuadstateToResetState(){
  quad_state_.setZero();
  quad_act_.setZero();

  quad_state_.x(QS::POSX) = resetPosition_(0);
  quad_state_.x(QS::POSY) = resetPosition_(1);
  quad_state_.x(QS::POSZ) = resetPosition_(2);

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
}

bool QuadrotorEnv::reset(Ref<Vector<>> obs, Ref<DepthImage<>> img, const bool random){
  resetObs(obs);

  resetImages(img);

  // resetWorldBox();
  return true;
}

bool QuadrotorEnv::resetObs(Ref<Vector<>> obs, const bool random) {
  quad_state_.setZero();
  quad_act_.setZero();

  setQuadstateToResetState();
  
  // reset quadrotor with random states
  quadrotor_ptr_->reset(quad_state_);

  // reset control command
  cmd_.t = 0.0;
  cmd_.thrusts.setZero();

  // obtain observations

  getObs(obs);

  return true;
}

bool QuadrotorEnv::resetImages(Ref<DepthImage<>> img) {  
  img_depth_ = cv::Mat::zeros(frame_width_, frame_height_, CV_32FC1);

  getImages(img);

  return true;
}
//--------------------------------//
//----      Observations      ----//
//--------------------------------//

bool QuadrotorEnv::getImages(Ref<DepthImage<>> img) {
  
//Update image observation
  rgb_camera->getDepthMap(img_depth_);

  cv::cv2eigen(img_depth_, depth_img_mat_);
  Map<DepthImage<>> img_vec_(depth_img_mat_.data(), depth_img_mat_.size());
  img.block<quadenv::frame_height*quadenv::frame_width,1>(0,0) = img_vec_;

  return true;
}

bool QuadrotorEnv::getObs(Ref<Vector<>> obs) {
  quadrotor_ptr_->getState(&quad_state_);

  // convert quaternion to euler angle
  Vector<3> euler_zyx = quad_state_.q().toRotationMatrix().eulerAngles(2, 1, 0);
  // quaternionToEuler(quad_state_.q(), euler);
  quad_obs_ << quad_state_.p, euler_zyx, quad_state_.v, quad_state_.w, goal_state_.segment<quadenv::kNGoal>(quadenv::kGoal);

  obs.segment<quadenv::kNObs>(quadenv::kObs) = quad_obs_;
  return true;
}


//--------------------------------//
//----          Step          ----//
//--------------------------------//


Scalar QuadrotorEnv::step(const Ref<Vector<>> act, Ref<Vector<>> obs) {
  quad_act_ = act.cwiseProduct(act_std_) + act_mean_;
  cmd_.t += sim_dt_;
  cmd_.thrusts = quad_act_;

  // simulate quadrotor
  quadrotor_ptr_->run(cmd_, sim_dt_);

  // update observations
  getObs(obs);

  Matrix<3, 3> rot = quad_state_.q().toRotationMatrix();

  //--- REWARD DESIGN IS DONE HERE --//
  // this reward will not work for the task, it's just to show a use case 
  Scalar pos_reward =
    pos_coeff_ * (quad_obs_.segment<quadenv::kNPos>(quadenv::kPos) -
                  goal_state_.segment<quadenv::kNPos>(quadenv::kPos))
                   .squaredNorm();
  // - orientation tracking
  Scalar ori_reward =
    ori_coeff_ * (quad_obs_.segment<quadenv::kNOri>(quadenv::kOri) -
                  goal_state_.segment<quadenv::kNOri>(quadenv::kOri))
                   .squaredNorm();
  // - linear velocity tracking
  Scalar lin_vel_reward =
    lin_vel_coeff_ * (quad_obs_.segment<quadenv::kNLinVel>(quadenv::kLinVel) -
                      goal_state_.segment<quadenv::kNLinVel>(quadenv::kLinVel))
                       .squaredNorm();
  // - angular velocity tracking
  Scalar ang_vel_reward =
    ang_vel_coeff_ * (quad_obs_.segment<quadenv::kNAngVel>(quadenv::kAngVel) -
                      goal_state_.segment<quadenv::kNAngVel>(quadenv::kAngVel))
                       .squaredNorm();

  // - control action penalty
  Scalar act_reward = act_coeff_ * act.cast<Scalar>().norm();

  Scalar total_reward =
    pos_reward + ori_reward + lin_vel_reward + ang_vel_reward + act_reward;

  // survival reward
  total_reward += 0.1;

  return total_reward;
}

//--------------------------------//
//----    Terminal analysis   ----//
//--------------------------------//



bool QuadrotorEnv::isTerminalState(Scalar &reward) {
  // Check out of bounds x. 
  if (quad_state_.x(QS::POSX) <= world_box_(0) + 0.5 || quad_state_.x(QS::POSX) >= world_box_(3) - 0.5 ) {
    reward = 0;
    // logger_.warn("out of bound x " + std::to_string(quad_state_.x(QS::POSX)) + " outside " + "[" + std::to_string(world_box_(0) + 0.5) + ", " + std::to_string(world_box_(3) - 0.5) + "]");
  }
  // Check out of bounds y.
  if (quad_state_.x(QS::POSY) <= world_box_(1) + 0.5 || quad_state_.x(QS::POSY) >= world_box_(4) - 0.5 ) {
    reward = 0;
    // logger_.warn("out of bound y " + std::to_string(quad_state_.x(QS::POSY)) + " outside " + "[" + std::to_string(world_box_(1) + 0.5) + ", " + std::to_string(world_box_(4) - 0.5) + "]");
  }
  // Check out of bounds z.
  if (quad_state_.x(QS::POSZ) <= world_box_(2) + 0.5  || quad_state_.x(QS::POSZ) >= world_box_(5) - 0.5 ) {
    reward = 0;
    // logger_.warn("out of bound z " + std::to_string(quad_state_.x(QS::POSZ)) + " outside " + "[" + std::to_string(world_box_(2) + 0.5) + ", " + std::to_string(world_box_(5) - 0.5) + "]");
  }

  if (reward==0.0) {
    return false;
  }
  return true;
}

bool QuadrotorEnv::isTerminalStateUnity(Scalar &reward) {
  // Check collision. 
  // Need justHadCollision flag because otherwise collision with a single object is thrown for two consecutive timesteps.
  if (quadrotor_ptr_->getCollision()) {
    // Collision is actually happening. 
    if (!justHadCollision)
    {
      // logger_.warn("quadrotor_ptr_->getCollision : " + std::to_string(quadrotor_ptr_->getCollision()));
      justHadCollision = true;
      return true;
    } else { // Collision is due to previous timestep.
      collision_step_count++;
      if(collision_step_count == 10){
      justHadCollision = false;
      collision_step_count = 0;
      }
    }
  }
}

bool QuadrotorEnv::loadParam(const YAML::Node &cfg) {
  if (cfg["quadrotor_env"]) {
    sim_dt_ = cfg["quadrotor_env"]["sim_dt"].as<Scalar>();
    max_t_ = cfg["quadrotor_env"]["max_t"].as<Scalar>();
  } else {
    return false;
  }

  if (cfg["rl"]) {
    // load reinforcement learning related parameters
    pos_coeff_ = cfg["rl"]["pos_coeff"].as<Scalar>();
    ori_coeff_ = cfg["rl"]["ori_coeff"].as<Scalar>();
    lin_vel_coeff_ = cfg["rl"]["lin_vel_coeff"].as<Scalar>();
    ang_vel_coeff_ = cfg["rl"]["ang_vel_coeff"].as<Scalar>();
    act_coeff_ = cfg["rl"]["act_coeff"].as<Scalar>();
  } else {
    return false;
  }
  return true;
}

bool QuadrotorEnv::getAct(Ref<Vector<>> act) const {
  if (cmd_.t >= 0.0 && quad_act_.allFinite()) {
    act = quad_act_;
    return true;
  }
  return false;
}

bool QuadrotorEnv::getAct(Command *const cmd) const {
  if (!cmd_.valid()) return false;
  *cmd = cmd_;
  return true;
}

void QuadrotorEnv::addObjectsToUnity(std::shared_ptr<UnityBridge> bridge) {
  bridge->addQuadrotor(quadrotor_ptr_);
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
     << "obs_std =            [" << quad_env.obs_std_.transpose() << std::endl;
  os.precision();
  return os;
}

}  // namespace flightlib