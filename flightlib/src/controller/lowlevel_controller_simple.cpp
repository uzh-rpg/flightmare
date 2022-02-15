#include "flightlib/controller/lowlevel_controller_simple.hpp"

namespace flightlib {

LowLevelControllerSimple::LowLevelControllerSimple(QuadrotorDynamics quad) {
  updateQuadDynamics(quad);
}

bool LowLevelControllerSimple::updateQuadDynamics(
  const QuadrotorDynamics& quad) {
  quad_dynamics_ = quad;
  B_allocation_ = quad.getAllocationMatrix();
  B_allocation_inv_ = B_allocation_.inverse();
  return true;
}

bool LowLevelControllerSimple::setCommand(const Command& cmd) {
  if (!cmd.valid()) return false;
  cmd_ = cmd;
  if (cmd_.isThrustRates()) {
    cmd_.collective_thrust =
      quad_dynamics_.clampCollectiveThrust(cmd_.collective_thrust);
    cmd_.omega = quad_dynamics_.clampBodyrates(cmd_.omega);
  }

  if (cmd_.isSingleRotorThrusts())
    cmd_.thrusts = quad_dynamics_.clampThrust(cmd_.thrusts);

  return true;
}


Vector<4> LowLevelControllerSimple::run(const Ref<Vector<3>> omega_des) {
  Vector<4> motor_thrusts;
  if (!cmd_.isSingleRotorThrusts()) {
    const Scalar force = quad_dynamics_.getMass() * cmd_.collective_thrust;
    const Vector<3> omega_err = cmd_.omega - omega_des;
    const Vector<3> body_torque_des =
      quad_dynamics_.getJ() * Kinv_ang_vel_tau_ * omega_err +
      omega_des.cross(quad_dynamics_.getJ() * omega_des);
    const Vector<4> thrust_torque(force, body_torque_des.x(),
                                  body_torque_des.y(), body_torque_des.z());

    motor_thrusts = B_allocation_inv_ * thrust_torque;
  } else {
    motor_thrusts = cmd_.thrusts;
  }

  motor_thrusts = quad_dynamics_.clampThrust(motor_thrusts);
  return motor_thrusts;
}


}  // namespace flightlib
