#pragma once

#include <memory>

#include "flightlib/common/logger.hpp"
#include "flightlib/common/math.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/dynamics/dynamics_base.hpp"

namespace flightlib {

class QuadrotorDynamics : DynamicsBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  QuadrotorDynamics(const Scalar mass, const Scalar arm_l);
  ~QuadrotorDynamics();

  // dynamics function
  bool dState(const QuadState& state, QuadState* derivative) const;
  bool dState(const Ref<const Vector<QuadState::SIZE>> state,
              Ref<Vector<QuadState::SIZE>> derivative) const;

  // public get function
  DynamicsFunction getDynamicsFunction() const;

  // help functions
  bool valid() const;
  // Helpers to apply limits.
  Vector<4> clampThrust(const Vector<4> thrusts) const;
  Scalar clampThrust(const Scalar thrust) const;

  Vector<4> clampMotorOmega(const Vector<4>& omega) const;
  Vector<3> clampBodyrates(const Vector<3>& omega) const;

  inline Scalar collective_thrust_min() const { return 4.0 * thrust_min_; }
  inline Scalar collective_thrust_max() const { return 4.0 * thrust_max_; }

  // Helpers for conversion
  Vector<4> motorOmegaToThrust(const Vector<4>& omega) const;
  Vector<4> motorThrustToOmega(const Vector<4>& thrusts) const;
  Matrix<4, 4> getAllocationMatrix() const;

  //
  inline Scalar mass(void) { return mass_; }
  inline Scalar motor_tau_inv() { return motor_tau_inv_; }
  inline Matrix<3, 3> J(void) { return J_; }
  inline Matrix<3, 3> J_inv(void) { return J_inv_; }

  bool setMass(const Scalar mass);
  bool setMotortauInv(const Scalar tau_inv);
  bool setJ(const Ref<Matrix<3, 3>> J);

 private:
  Scalar mass_;
  Matrix<3, 4> t_BM_;
  Matrix<3, 3> J_;
  Matrix<3, 3> J_inv_;

  // motors
  Scalar motor_omega_min_;
  Scalar motor_omega_max_;
  Scalar motor_tau_inv_;

  // Propellers
  Vector<3> thrust_map_;
  Scalar kappa_;
  Scalar thrust_min_;
  Scalar thrust_max_;

  // Quadrotor limits
  Vector<3> omega_max_;
};

}  // namespace flightlib
