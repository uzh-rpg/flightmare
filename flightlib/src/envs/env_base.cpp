#include "flightlib/envs/env_base.hpp"

namespace flightlib {

EnvBase::EnvBase()
  : obs_dim_(0),
    act_dim_(0),
    sim_dt_(0.0),
    unity_connection_time_out_(10.0),
    scene_id_(UnityScene::WAREHOUSE),
    render_(false),
    unity_ready_(false),
    unity_bridge_created_(false) {}

EnvBase::~EnvBase() {}

bool EnvBase::connectFlightmare() {
  Scalar time_out_count = 0;
  Scalar sleep_useconds = 0.2 * 1e5;
  while (!unity_ready_) {
    if (unity_bridge_ != nullptr) {
      // connect unity
      unity_bridge_->setScene(scene_id_);
      unity_ready_ = unity_bridge_->connectUnity();
    }
    if (unity_ready_ || time_out_count / 1e6 > unity_connection_time_out_) {
      break;
    }
    // sleep
    usleep(sleep_useconds);
    // incread time out counter
    time_out_count += sleep_useconds;
  }
  return true ? unity_ready_ : false;
}

void EnvBase::disconnectFlightmare() {
  if (unity_bridge_ != nullptr) {
    unity_bridge_->disconnectUnity();
    unity_ready_ = false;
  }
}

void EnvBase::curriculumUpdate() {}

void EnvBase::close() { disconnectFlightmare(); }

void EnvBase::render() {}

void EnvBase::updateExtraInfo() {}

bool EnvBase::isTerminalState(Scalar &reward) {
  reward = 0.f;
  return false;
}

}  // namespace flightlib
