#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>

#include "flightlib/common/command.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/controller/filter.hpp"
#include "flightlib/controller/lowlevel_controller_base.hpp"
#include "flightlib/controller/pid_parts.hpp"

namespace flightlib {

class LowLevelControllerBetaflight {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LowLevelControllerBetaflight(QuadrotorDynamics& quad);
  LowLevelControllerBetaflight(QuadrotorDynamics& quad, const float fs);
  bool setCommand(const Command& cmd);
  Vector<4> run(const Ref<Vector<3>> omega);
  bool updateQuadDynamics(const QuadrotorDynamics& quad);
  float getBattery() { return voltage_; };
  void setParamDir(const std::string& param_dir);

 private:
  // Command
  Command cmd_;

  // Motor speeds calculated by the controller
  Vector<4> motor_omega_des_;

  // Quadcopter to which the controller is applied
  QuadrotorDynamics quad_dynamics_;

  // Quadrotor properties
  // const Matrix<4, 4> B_allocation_ =
  //   (Matrix<4, 4>() << 1, -1, -1, -1, 1, 1, 1, -1, 1, -1, 1, 1, 1, 1, -1, 1)
  //     .finished();
  Matrix<4, 4> B_allocation_;
  Matrix<4, 4> B_allocation_inv_;


  // 1, -1,  1,  1,
  // 1, -1, -1, -1,
  // 1,  1,  1, -1,
  // 1,  1, -1,  1
  // Delay of one command
  Command last_cmd_;

  // Thrust Map MPC to Betaflight
  // ThrustMap tmap_;

  // Battery Voltage Simulation
  static constexpr float voltage_0_ = 16.35;
  // static constexpr float voltage_0_ = 15.7;
  static constexpr float voltage_offset_ = 0.30808;
  static constexpr float voltage_rpm_ = 0.14963;
  static constexpr float voltage_cum_ = -0.47817;
  static constexpr float voltage_rpm2_ = -0.47429;
  mutable double cum_ = 0;
  mutable float voltage_ = 0;

  // Motor normalized thrust to dshot command
  static constexpr float slope = 1888.945;
  static constexpr float offset = 157.480;

  // sbus to normalized motor command
  static constexpr float sbus_offset = -129.3667;
  static constexpr float subs_slope = 0.5993;

  static constexpr float omega_cmd_sqrt = 2.5233;
  static constexpr float omega_cmd_lin = 0.04071;
  static constexpr float omega_volt = 0.06131;
  static constexpr float omega_offset = -1.8532;

  std::string param_dir_;

  // Parameter Estimate from MATLAB, see "BetaflightID.m"
  const float fs;
  const float pid_scale = 1e-3;  // betaflight scales everything this way
  pidP P;
  pidI I;
  pidD D;
};


}  // namespace flightlib