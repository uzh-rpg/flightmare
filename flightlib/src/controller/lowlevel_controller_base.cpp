#include "flightlib/controller/lowlevel_controller_base.hpp"

namespace flightlib {

LowLevelControllerBase::LowLevelControllerBase(QuadrotorDynamics& quad_dynamics)
  : quad_dynamics_(quad_dynamics) {
  state_.setZero();
  motor_omega_des_.setZero();
}


bool LowLevelControllerBase::setState(const QuadState& state) {
  if (!state.valid()) return false;
  state_ = state;
  return true;
}


bool LowLevelControllerBase::setCommand(const Command& cmd) {
  if (!cmd.valid()) return false;
  cmd_ = cmd;
  return true;
}


bool LowLevelControllerBase::getMotorCommand(Ref<Vector<4>> motor_omega) {
  run();
  if (!motor_omega_des_.allFinite()) return false;
  motor_omega = motor_omega_des_;
  return true;
}


bool LowLevelControllerBase::updateQuadDynamics(
  const QuadrotorDynamics& quad_dynamics) {
  if (!quad_dynamics.valid()) return false;
  quad_dynamics_ = quad_dynamics;
  return true;
}

}  // namespace flightlib
