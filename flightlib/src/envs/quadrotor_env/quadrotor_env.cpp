#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"

namespace flightlib {

QuadrotorEnv::QuadrotorEnv() : EnvBase() {
  // define input and output dimension for the environment
  obs_dim_ = CtlObsAct::kObsSize;
  act_dim_ = CtlObsAct::kActSize;
}

QuadrotorEnv::~QuadrotorEnv() {}

void QuadrotorEnv::reset(const Ref<Vector<>> obs) {
  quad_state_.setZero();

  // randomly reset the quadrotor state
  quad_state_.p = Vector<3>::Ones() * uniform_dist_(random_gen_);
  if (quad_state_.p(2) < -1.0) quad_state_.p(2) = -quad_state_.p(2);
  quad_state_.v = Vector<3>::Ones() * uniform_dist_(random_gen_);
  quad_state_.qx = Vector<4>::Ones() * uniform_dist_(random_gen_);
  quad_state_.qx /= quad_state_.qx.norm();

  // reset quadrotor with random states
  quadrotor_.reset(quad_state_);

  // reset control command
  cmd_.t = 0.0;
  cmd_.collective_thrust = 0.0;
  cmd_.omega = Vector<3>::Zero();

  // obtain observations
  getObs(obs);
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

Scalar QuadrotorEnv::step(Ref<Vector<>> act, Ref<Vector<>> obs) {
  quad_act_ = act.cwiseProduct(param_.act_std);
  quad_act_ += param_.act_mean;
  cmd_.t += param_.sim_dt;
  cmd_.collective_thrust = quad_act_(0);
  cmd_.omega = quad_act_.segment<3>(CtlObsAct::kOmegaX);

  // simulate quadrotor
  quadrotor_.run(cmd_, param_.sim_dt);

  // update observations
  getObs(obs);

  // reward function design
  Scalar stage_reward = (quad_obs_ - param_.goal_state).transpose() * param_.Q *
                        (quad_obs_ - param_.goal_state);
  Scalar act_reward = (quad_act_ - param_.act_mean).transpose() * param_.Q_act *
                      (quad_act_ - param_.act_mean);

  return stage_reward + act_reward;
}

bool QuadrotorEnv::setFlightmare(bool render) {
  param_.render = render;
  if (param_.render && !unity_bridge_created_) {
    logger_.info("Unity Rendering (Flightmare) is ON!\n");
    // create unity bridge and initialize connection
    unity_bridge_ = UnityBridge::getInstance();
    unity_bridge_->initializeConnections();
    connectFlightmare();

    //
    unity_bridge_created_ = true;
  }
  return true;
}

bool QuadrotorEnv::isTerminalState(Scalar &reward) {
  reward = 0.0;
  return false;
}

bool QuadrotorEnv::updateParam(QuadrotorEnvParam &param) {
  if (!param.valid()) {
    return false;
  }
  param_ = param;
  return true;
}

}  // namespace flightlib