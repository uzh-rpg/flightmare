#include "flightlib/dynamics/quadrotor_dynamics.hpp"

namespace flightlib {

QuadrotorDynamics::QuadrotorDynamics(const Scalar mass, const Scalar arm_l)
  : mass_(mass),
    t_BM_(
      arm_l * sqrt(0.5) *
      (Matrix<3, 4>() << 1, -1, -1, 1, -1, 1, -1, 1, 0, 0, 0, 0).finished()),
    J_(Matrix<3, 3>::Zero()),
    J_inv_(Matrix<3, 3>::Identity()),
    motor_omega_min_(150.0),
    motor_omega_max_(2000.0),
    motor_tau_inv_(1.0 / 0.05),
    thrust_map_(1.3298253500372892e-06, 0.0038360810526746033,
                -1.7689986848125325),
    kappa_(0.016),
    thrust_min_(0.0),
    thrust_max_(motor_omega_max_ * motor_omega_max_ * thrust_map_(0) +
                motor_omega_max_ * thrust_map_(1) + thrust_map_(2)),
    omega_max_(Vector<3>::Constant(6.0)) {}

QuadrotorDynamics::~QuadrotorDynamics() {}

bool QuadrotorDynamics::dState(const QuadState& state,
                               QuadState* dstate) const {
  return dState(state.x, dstate->x);
}

bool QuadrotorDynamics::dState(const Ref<const Vector<QuadState::SIZE>> state,
                               Ref<Vector<QuadState::SIZE>> dstate) const {
  if (!state.segment<QS::DYN>(0).allFinite()) return false;

  dstate.setZero();
  const Vector<3> omega(state(QS::OMEX), state(QS::OMEY), state(QS::OMEZ));
  const Quaternion q_omega(0, omega.x(), omega.y(), omega.z());

  dstate.segment<QS::NPOS>(QS::POS) = state.segment<QS::NVEL>(QS::VEL);
  dstate.segment<QS::NATT>(QS::ATT) =
    0.5 * Q_right(q_omega) * state.segment<QS::NATT>(QS::ATT);
  dstate.segment<QS::NVEL>(QS::VEL) = state.segment<QS::NACC>(QS::ACC);
  dstate.segment<QS::NOME>(QS::OME) =
    J_inv_ * (state.segment<QS::NTAU>(QS::TAU) - omega.cross(J_ * omega));

  return true;
}

QuadrotorDynamics::DynamicsFunction QuadrotorDynamics::getDynamicsFunction()
  const {
  return std::bind(
    static_cast<bool (QuadrotorDynamics::*)(const Ref<const Vector<QS::SIZE>>,
                                            Ref<Vector<QS::SIZE>>) const>(
      &QuadrotorDynamics::dState),
    this, std::placeholders::_1, std::placeholders::_2);
}

bool QuadrotorDynamics::valid() const {
  bool check = true;

  check &= mass_ > 0.0;
  check &= t_BM_.allFinite();
  check &= J_.allFinite();
  check &= J_inv_.allFinite();

  check &= motor_omega_min_ >= 0.0;
  check &= (motor_omega_max_ > motor_omega_min_);
  check &= motor_tau_inv_ > 0.0;

  check &= thrust_map_.allFinite();
  check &= kappa_ > 0.0;
  check &= thrust_min_ >= 0.0;
  check &= (thrust_max_ > thrust_min_);

  check &= (omega_max_.array() > 0).all();

  return check;
}

Vector<4> QuadrotorDynamics::clampThrust(const Vector<4> thrusts) const {
  return thrusts.cwiseMax(thrust_min_).cwiseMin(thrust_max_);
}

Scalar QuadrotorDynamics::clampThrust(const Scalar thrust) const {
  return std::clamp(thrust, thrust_min_, thrust_max_);
}

Vector<4> QuadrotorDynamics::clampMotorOmega(const Vector<4>& omega) const {
  return omega.cwiseMax(motor_omega_min_).cwiseMin(motor_omega_max_);
}

Vector<3> QuadrotorDynamics::clampBodyrates(const Vector<3>& omega) const {
  return omega.cwiseMax(-omega_max_).cwiseMin(omega_max_);
}

Vector<4> QuadrotorDynamics::motorOmegaToThrust(const Vector<4>& omega) const {
  const Matrix<4, 3> omega_poly =
    (Matrix<4, 3>() << omega.cwiseProduct(omega), omega, Vector<4>::Ones())
      .finished();
  return omega_poly * thrust_map_;
}

Vector<4> QuadrotorDynamics::motorThrustToOmega(
  const Vector<4>& thrusts) const {
  const Scalar scale = 1.0 / (2.0 * thrust_map_[0]);
  const Scalar offset = -thrust_map_[1] * scale;

  const Array<4, 1> root =
    (std::pow(thrust_map_[1], 2) -
     4.0 * thrust_map_[0] * (thrust_map_[2] - thrusts.array()))
      .sqrt();

  return offset + scale * root;
}

Matrix<4, 4> QuadrotorDynamics::getAllocationMatrix() const {
  return (Matrix<4, 4>() << Vector<4>::Ones().transpose(), t_BM_.topRows<2>(),
          kappa_ * Vector<4>(-1, -1, 1, 1).transpose())
    .finished();
}

}  // namespace flightlib
