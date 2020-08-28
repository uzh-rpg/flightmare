#pragma once

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <memory>

#include "flightlib/common/logger.hpp"
#include "flightlib/common/math.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/dynamics/dynamics_base.hpp"

namespace flightlib {

class QuadrotorDynamics : DynamicsBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  QuadrotorDynamics(const Scalar mass = 1.0, const Scalar arm_l = 0.2);
  ~QuadrotorDynamics();

  // dynamics function
  bool dState(const QuadState& state, QuadState* derivative) const;
  bool dState(const Ref<const Vector<QuadState::SIZE>> state,
              Ref<Vector<QuadState::SIZE>> derivative) const;

  // public get function
  DynamicsFunction getDynamicsFunction() const;

  // help functions
  bool valid() const;
  bool updateParams(const YAML::Node& params);

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
  inline Scalar getMass(void) const { return mass_; };
  inline Scalar getArmLength(void) const { return arm_l_; };
  inline Scalar getMotorTauInv() const { return motor_tau_inv_; };
  inline Matrix<3, 3> getJ(void) const { return J_; };
  inline Matrix<3, 3> getJInv(void) const { return J_inv_; };

  bool setMass(const Scalar mass);
  bool setArmLength(const Scalar arm_length);
  bool setMotortauInv(const Scalar tau_inv);

  friend std::ostream& operator<<(std::ostream& os,
                                  const QuadrotorDynamics& quad_dymaics);

 private:
  bool updateInertiaMarix();
  Scalar mass_;
  Scalar arm_l_;
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
