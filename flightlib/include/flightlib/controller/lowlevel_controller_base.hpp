// from agilicious
#pragma once


#include "flightlib/common/command.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/dynamics/quadrotor_dynamics.hpp"

namespace flightlib {

class LowLevelControllerBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  LowLevelControllerBase(QuadrotorDynamics& quad_dynamics);
  virtual ~LowLevelControllerBase() = default;
  virtual bool updateQuadDynamics(const QuadrotorDynamics& quad_dynamics);
  virtual bool setState(const QuadState& state);
  virtual bool setCommand(const Command& cmd);
  virtual bool getMotorCommand(Ref<Vector<4>> motors);


 protected:
  // Command
  Command cmd_;

  // State of Quadrotor
  QuadState state_;

  // Motor speeds calculated by the controller
  Vector<4> motor_omega_des_;

  // Quadcopter to which the controller is applied
  QuadrotorDynamics quad_dynamics_;

  // Method that runs controller
  virtual void run() = 0;
};


}  // namespace flightlib
