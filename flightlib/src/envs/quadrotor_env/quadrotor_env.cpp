#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"

namespace flightlib {

QuadrotorEnv::QuadrotorEnv()
  : QuadrotorEnv(getenv("FLIGHTMARE_PATH") +
                 std::string("/flightlib/configs/quadrotor_env.yaml")) {}

QuadrotorEnv::QuadrotorEnv(const std::string &cfg_path)
  : EnvBase(),
    Q_((Vector<CtlObsAct::kObsSize>() << -1e-2, -1e-2, -1e-2, -1e-2, -1e-2,
        -1e-2, -1e-3, -1e-3, -1e-3)
         .finished()
         .asDiagonal()),
    Q_act_(
      Vector<CtlObsAct::kActSize>(-1e-4, -1e-4, -1e-4, -1e-4).asDiagonal()),
    goal_state_((Vector<CtlObsAct::kObsSize>() << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0)
                  .finished()) {
  // load configuration file
  YAML::Node cfg_ = YAML::LoadFile(cfg_path);

  // update dynamics
  QuadrotorDynamics dynamics;
  dynamics.updateParams(cfg_);
  quadrotor_.updateDynamics(dynamics);
  world_box_ << -10, 10, -10, 10, 0, 10;
  quadrotor_.setWorldBox(world_box_);

  // define input and output dimension for the environment
  obs_dim_ = CtlObsAct::kObsSize;
  act_dim_ = CtlObsAct::kActSize;

  // load parameters
  loadParam(cfg_);
}

QuadrotorEnv::~QuadrotorEnv() {}

bool QuadrotorEnv::reset(Ref<Vector<>> obs, const bool random) {
  quad_state_.setZero();

  if (random) {
    // randomly reset the quadrotor state
    // reset position
    quad_state_.x(QS::POSX) = uniform_dist_(random_gen_);
    quad_state_.x(QS::POSY) = uniform_dist_(random_gen_);
    quad_state_.x(QS::POSZ) = uniform_dist_(random_gen_);
    if (quad_state_.x(QS::POSZ) < -1.0)
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
  }
  // reset quadrotor with random states
  quadrotor_.reset(quad_state_);

  // reset control command
  cmd_.t = 0.0;
  cmd_.collective_thrust = 0.0;
  cmd_.omega = Vector<3>::Zero();

  // obtain observations
  getObs(obs);
  return true;
}

bool QuadrotorEnv::getObs(Ref<Vector<>> obs) {
  quadrotor_.getState(&quad_state_);

  //
  Vector<3> euler;
  quaternionToEuler(quad_state_.q(), euler);
  quad_obs_ << quad_state_.p, euler, quad_state_.v;

  obs.segment<CtlObsAct::kObsSize>(kObs) = quad_obs_;
  return true;
}

Scalar QuadrotorEnv::step(const Ref<Vector<>> act, Ref<Vector<>> obs) {
  quad_act_ = act.cwiseProduct(act_std_) + act_mean_;
  cmd_.t += sim_dt_;
  cmd_.collective_thrust = quad_act_(0);
  cmd_.omega = quad_act_.segment<3>(CtlObsAct::kOmegaX);

  // simulate quadrotor
  quadrotor_.run(cmd_, sim_dt_);

  // update observations
  getObs(obs);

  // reward function design
  Scalar stage_reward =
    (quad_obs_ - goal_state_).transpose() * Q_ * (quad_obs_ - goal_state_);
  Scalar act_reward =
    (quad_act_ - act_mean_).transpose() * Q_act_ * (quad_act_ - act_mean_);

  return stage_reward + act_reward;
}

bool QuadrotorEnv::isTerminalState(Scalar &reward) {
  reward = 0.0;
  return false;
}

bool QuadrotorEnv::loadParam(const YAML::Node &cfg) {
  if (cfg["quadrotor_env"]) {
    sim_dt_ = cfg["quadrotor_env"]["sim_dt"].as<Scalar>();
  } else {
    return false;
  }

  if (cfg["rl"]) {
    // load reinforcement learning related parameters
    std::vector<Scalar> Q_pos(3), Q_ori(3), Q_lin_vel(3), Q_cmd(4);
    Q_pos = cfg["rl"]["Q_pos"].as<std::vector<Scalar>>();
    Q_ori = cfg["rl"]["Q_ori"].as<std::vector<Scalar>>();
    Q_lin_vel = cfg["rl"]["Q_lin_vel"].as<std::vector<Scalar>>();
    Q_cmd = cfg["rl"]["Q_cmd"].as<std::vector<Scalar>>();

    Q_ =
      (Vector<CtlObsAct::kObsSize>() << Q_pos[0], Q_pos[1], Q_pos[2], Q_ori[0],
       Q_ori[1], Q_ori[2], Q_lin_vel[0], Q_lin_vel[1], Q_lin_vel[2])
        .finished()
        .asDiagonal();
    Q_act_ =
      (Vector<CtlObsAct::kActSize>() << Q_cmd[0], Q_cmd[1], Q_cmd[2], Q_cmd[3])
        .finished()
        .asDiagonal();
  } else {
    return false;
  }
  return true;
}

void QuadrotorEnv::getQ(
  Ref<Matrix<CtlObsAct::kObsSize, CtlObsAct::kObsSize>> Q) const {
  Q = Q_;
}

void QuadrotorEnv::getQAct(
  Ref<Matrix<CtlObsAct::kActSize, CtlObsAct::kActSize>> Q_act) const {
  Q_act = Q_act_;
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
  bridge->addQuadrotor(&quadrotor_);
}

}  // namespace flightlib