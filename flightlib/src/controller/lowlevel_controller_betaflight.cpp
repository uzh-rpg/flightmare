#include "flightlib/controller/lowlevel_controller_betaflight.hpp"

namespace flightlib {

LowLevelControllerBetaflight::LowLevelControllerBetaflight(
  QuadrotorDynamics& quad_dynamics)
  : fs(1000), I(fs), D(fs) {
  updateQuadDynamics(quad_dynamics);
}

LowLevelControllerBetaflight::LowLevelControllerBetaflight(
  QuadrotorDynamics& quad_dynamics, const float fs)
  : fs(fs), I(fs), D(fs) {
  updateQuadDynamics(quad_dynamics);
}

bool LowLevelControllerBetaflight::updateQuadDynamics(
  const QuadrotorDynamics& quad_dynamics) {
  cum_ = 0.0;
  quad_dynamics_ = quad_dynamics;
  B_allocation_ = quad_dynamics.getAllocationMatrix();
  B_allocation_inv_ = B_allocation_.inverse();

  return true;
}

bool LowLevelControllerBetaflight::setCommand(const Command& cmd) {
  if (!cmd.valid()) {
    std::cout << "invalide command " << std::endl;
    return false;
  }

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


Vector<4> LowLevelControllerBetaflight::run(const Ref<Vector<3>> omega) {
  Vector<4> motor_thrusts;
  if (!cmd_.isSingleRotorThrusts()) {
    const Scalar force = quad_dynamics_.getMass() * cmd_.collective_thrust;

    const Vector<3> omega_des = cmd_.omega;
    const Vector<3> p = P.update(omega_des, omega);
    // const Vector<3> i = I.update(omega_des, omega);
    const Vector<3> d = D.update(omega);
    const Vector<3> body_torque_des = pid_scale * (p + d);


    const Vector<4> tlmn(force, body_torque_des.x(), body_torque_des.y(),
                         body_torque_des.z());
    motor_thrusts = B_allocation_inv_ * tlmn;
  } else {
    motor_thrusts = cmd_.thrusts;
  }

  motor_thrusts = quad_dynamics_.clampThrust(motor_thrusts);
  return motor_thrusts;
}

}  // namespace flightlib
