#pragma once

#include "flightlib/common/command.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/dynamics/quadrotor_dynamics.hpp"

namespace flightlib {

class LowLevelControllerSimple {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LowLevelControllerSimple(QuadrotorDynamics quad_dynamics);
  bool setCommand(const Command& cmd);
  Vector<4> run(const Ref<Vector<3>> omega);
  bool updateQuadDynamics(const QuadrotorDynamics& quad_dynamics);

 private:
  // Quadrotor properties
  Matrix<4, 4> B_allocation_;
  Matrix<4, 4> B_allocation_inv_;

  // P gain for body rate control
  const Matrix<3, 3> Kinv_ang_vel_tau_ =
    Vector<3>(16.6, 16.6, 5.0).asDiagonal();

  // Quadcopter to which the controller is applied
  QuadrotorDynamics quad_dynamics_;

  // Motor speeds calculated by the controller
  Vector<4> motor_omega_des_;

  // Command
  Command cmd_;
};

}  // namespace flightlib
