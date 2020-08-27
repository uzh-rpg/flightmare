#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"

namespace flightlib {

QuadrotorEnv::QuadrotorEnv()
  : QuadrotorEnv(getenv("FLIGHTMARE_PATH") +
                 std::string("/flightlib/configs/quadrotor_env.yaml")) {}

QuadrotorEnv::QuadrotorEnv(const std::string &cfg_path)
  : EnvBase(),
    Q_((Vector<CtlObsAct::kObsSize>() << -1e-2, -1e-2, -1e-2, -1e-2, -1e-2,
        -1e-2, -1e-3, -1e-3, -1e-3, -1e-3, -1e-3, -1e-3)
         .finished()
         .asDiagonal()),
    Q_act_(
      Vector<CtlObsAct::kActSize>(-1e-4, -1e-4, -1e-4, -1e-4).asDiagonal()),
    goal_state_((Vector<CtlObsAct::kObsSize>() << 0.0, 0.0, 5.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                  .finished()) {
  // load configuration file
  YAML::Node cfg_ = YAML::LoadFile(cfg_path);

  // update dynamics
  QuadrotorDynamics dynamics;
  dynamics.updateParams(cfg_);
  quadrotor_.updateDynamics(dynamics);

  // define a bounding box
  world_box_ << -20, 20, -20, 20, 0, 20;
  quadrotor_.setWorldBox(world_box_);

  // define input and output dimension for the environment
  obs_dim_ = CtlObsAct::kObsSize;
  act_dim_ = CtlObsAct::kActSize;


  Scalar mass = quadrotor_.getMass();
  act_mean_ = Vector<4>::Ones() * (-mass * Gz) / 4;
  act_std_ = Vector<4>::Ones() * (-mass * 2 * Gz) / 4;

  // load parameters
  loadParam(cfg_);
}

QuadrotorEnv::~QuadrotorEnv() {}

bool QuadrotorEnv::reset(Ref<Vector<>> obs, const bool random) {
  quad_state_.setZero();
  quad_act_.setZero();

  if (random) {
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
  }
  // reset quadrotor with random states
  quadrotor_.reset(quad_state_);

  // reset control command
  cmd_.t = 0.0;
  cmd_.thrusts.setZero();

  // obtain observations
  getObs(obs);
  return true;
}

bool QuadrotorEnv::getObs(Ref<Vector<>> obs) {
  quadrotor_.getState(&quad_state_);

  // convert quaternion to euler angle
  Vector<3> euler = quad_state_.q().toRotationMatrix().eulerAngles(0, 1, 2);
  // quaternionToEuler(quad_state_.q(), euler);
  quad_obs_ << quad_state_.p, euler, quad_state_.v, quad_state_.w;

  obs.segment<CtlObsAct::kObsSize>(kObs) = quad_obs_;
  return true;
}

Scalar QuadrotorEnv::step(const Ref<Vector<>> act, Ref<Vector<>> obs) {
  quad_act_ = act.cwiseProduct(act_std_) + act_mean_;
  cmd_.t += sim_dt_;
  cmd_.thrusts = quad_act_;

  // simulate quadrotor
  quadrotor_.run(cmd_, sim_dt_);

  // update observations
  getObs(obs);

  // reward function design
  Scalar stage_reward =
    (quad_obs_ - goal_state_).transpose() * Q_ * (quad_obs_ - goal_state_);
  Scalar act_reward =
    (quad_act_ - act_mean_).transpose() * Q_act_ * (quad_act_ - act_mean_);

  Scalar total_reward = stage_reward + act_reward + 1;

  return total_reward;
}

bool QuadrotorEnv::isTerminalState(Scalar &reward) {
  if (quad_state_.x(QS::POSZ) <= 0.02) {
    reward = -0.02;
    return true;
  }
  reward = 0.0;
  return false;
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
     << "obs_std =            [" << quad_env.obs_std_.transpose() << "]\n"
     << "Q =                  [" << quad_env.Q_.diagonal().transpose() << "]\n"
     << "Q_act =              [" << quad_env.Q_act_.diagonal().transpose()
     << "]" << std::endl;
  os.precision();
  return os;
}

}  // namespace flightlib