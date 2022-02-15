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
  QuadrotorDynamics(const Scalar mass = 1.0);
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
  Scalar clampCollectiveThrust(const Scalar thrust) const;
  Scalar clampTotalForce(const Scalar force) const;

  Vector<4> clampMotorOmega(const Vector<4>& omega) const;
  Vector<3> clampTorque(const Vector<3>& torque) const;
  Vector<3> clampBodyrates(const Vector<3>& omega) const;
  Vector<3> clampVelocity(const Vector<3>& velocity) const;

  // Helpers for conversion
  Vector<4> motorOmegaToThrust(const Vector<4>& omega) const;
  Vector<4> motorThrustToOmega(const Vector<4>& thrusts) const;
  Matrix<4, 4> getAllocationMatrix(void) const;

  Vector<3> getBodyDrag(const Vector<3>& body_vel);
  bool updateBodyDragCoeff1(const Vector<3>& cd1);
  bool updateBodyDragCoeff3(const Vector<3>& cd3);
  bool updateBodyDragCoeffZH(const Scalar cdz_h);

  //
  inline Scalar getMass(void) const { return mass_; };
  inline Scalar getMotorTauInv() const { return motor_tau_inv_; };
  inline Matrix<3, 3> getJ(void) const { return J_; };
  inline Matrix<3, 3> getJInv(void) const { return J_inv_; };
  inline Vector<3> getOmegaMax(void) const { return omega_max_; };
  inline Vector<3> getTorqueMax(void) const {
    return force_torque_max_.segment<3>(1);
  };
  inline Scalar getSingleThrustMax(void) const { return thrust_max_; };
  inline Scalar getForceMax(void) const { return force_torque_max_(0); };

  bool setMass(const Scalar mass);
  bool setMotortauInv(const Scalar tau_inv);

  friend std::ostream& operator<<(std::ostream& os,
                                  const QuadrotorDynamics& quad_dymaics);

 private:
  bool updateInertiaMarix();
  Scalar mass_;
  Matrix<3, 4> t_BM_;
  Matrix<4, 4> B_allocation_;
  Matrix<3, 3> J_;
  Matrix<3, 3> J_inv_;

  // motors
  Scalar motor_omega_min_;
  Scalar motor_omega_max_;
  Scalar motor_tau_inv_;

  //
  Vector<4> force_torque_min_;
  Vector<4> force_torque_max_;

  // Propellers
  Vector<3> thrust_map_;
  Scalar kappa_;
  Scalar thrust_min_;
  Scalar thrust_max_;
  Scalar collective_thrust_min_;
  Scalar collective_thrust_max_;

  // body drag coefficients
  Vector<3> cd1_, cd3_;
  Scalar cdz_h_;

  // Quadrotor limits
  Vector<3> omega_max_;
};

}  // namespace flightlib
