#include "flightlib/dynamics/quadrotor_dynamics.hpp"

namespace flightlib {

QuadrotorDynamics::QuadrotorDynamics(const Scalar mass) : mass_(mass) {
  // all the parameters are hard coded according to the agilicious drone
  // motor drag coefficient
  kappa_ = 0.016;


  // inertia matrix
  J_ = Matrix<3, 3>(Vector<3>(0.0025, 0.0021, 0.0043).asDiagonal());
  J_inv_ = J_.inverse();

  // translational matrix. from motor to body
  t_BM_ << 0.075, -0.075, -0.075, 0.075, -0.10, 0.10, -0.10, 0.10, 0.0, 0.0,
    0.0, 0.0;

  // motor spped limits
  motor_omega_min_ = 0.0;
  motor_omega_max_ = 2000.0;

  // motor dynamics
  motor_tau_inv_ = 1.0 / 0.033;

  // thrust mapping coefficients;
  // thrust = t1 * motor_omega * motor_omega + t2 * motor_omega + t3
  thrust_map_ << 1.562522e-6, 0.0, 0.0;

  // thrust limit
  thrust_min_ = 0.0;
  thrust_max_ = motor_omega_max_ * motor_omega_max_ * thrust_map_(0) +
                motor_omega_max_ * thrust_map_(1) + thrust_map_(2);

  //
  collective_thrust_min_ = 4.0 * thrust_min_ / mass_;
  collective_thrust_max_ = 4.0 * thrust_max_ / mass_;

  //
  omega_max_ << 6.0, 6.0, 2.0;

  // allocation matrix
  B_allocation_ =
    (Matrix<4, 4>() << Vector<4>::Ones().transpose(), t_BM_.row(1),
     -t_BM_.row(0), kappa_ * Vector<4>(-1.0, -1.0, 1.0, 1.0).transpose())
      .finished();

  // body drag coefficients
  cd1_ << 0.0, 0.0, 0.0;
  cd3_ << 0.0, 0.0, 0.0;
  cdz_h_ = 0.0;
}

QuadrotorDynamics::~QuadrotorDynamics() {}

bool QuadrotorDynamics::dState(const QuadState& state,
                               QuadState* dstate) const {
  return dState(state.x, dstate->x);
}

bool QuadrotorDynamics::dState(const Ref<const Vector<QuadState::SIZE>> state,
                               Ref<Vector<QuadState::SIZE>> dstate) const {
  if (!state.segment<QS::NDYM>(0).allFinite()) return false;

  dstate.setZero();
  //
  const Vector<3> omega(state(QS::OMEX), state(QS::OMEY), state(QS::OMEZ));
  const Quaternion q_omega(0, omega.x(), omega.y(), omega.z());
  const Vector<3> body_torque = state.segment<QS::NTAU>(QS::TAU);

  // linear velocity = dx / dt
  dstate.segment<QS::NPOS>(QS::POS) = state.segment<QS::NVEL>(QS::VEL);

  // differentiate quaternion = dq / dt
  dstate.segment<QS::NATT>(QS::ATT) =
    0.5 * Q_right(q_omega) * state.segment<QS::NATT>(QS::ATT);

  // linear acceleration = dv / dt
  dstate.segment<QS::NVEL>(QS::VEL) = state.segment<QS::NACC>(QS::ACC);

  // angular accleration = domega / dt
  dstate.segment<QS::NOME>(QS::OME) =
    J_inv_ * (body_torque - omega.cross(J_ * omega));
  //
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
  check &= mass_ < 100.0;  // limit maximum mass
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

Vector<3> QuadrotorDynamics::clampTorque(const Vector<3>& torque) const {
  return torque.cwiseMax(force_torque_min_.segment<3>(1))
    .cwiseMin(force_torque_max_.segment<3>(1));
}

Scalar QuadrotorDynamics::clampTotalForce(const Scalar force) const {
  return std::clamp(force, force_torque_min_(0), force_torque_max_(0));
}

Scalar QuadrotorDynamics::clampCollectiveThrust(const Scalar thrust) const {
  return std::clamp(thrust, collective_thrust_min_, collective_thrust_max_);
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

Matrix<4, 4> QuadrotorDynamics::getAllocationMatrix(void) const {
  // compute column-wise cross product
  // tau_i = t_BM_i x F_i
  return B_allocation_;
}

Vector<3> QuadrotorDynamics::getBodyDrag(const Vector<3>& body_vel) {
  return cd1_.cwiseProduct(body_vel) +
         cd3_.cwiseProduct(body_vel.array().pow(3.0).matrix()) -
         (Vector<3>() << 0.0, 0.0,
          cdz_h_ * (body_vel.x() * body_vel.x() + body_vel.y() * body_vel.y()))
           .finished();
}

bool QuadrotorDynamics::updateBodyDragCoeff1(const Vector<3>& cd1) {
  if (!cd1.allFinite()) {
    return false;
  }
  cd1_ = cd1;
  return true;
}

bool QuadrotorDynamics::updateBodyDragCoeff3(const Vector<3>& cd3) {
  if (!cd3.allFinite()) {
    return false;
  }
  cd3_ = cd3;
  return true;
}

bool QuadrotorDynamics::updateBodyDragCoeffZH(const Scalar cdz_h) {
  cdz_h_ = cdz_h;
  return true;
}

bool QuadrotorDynamics::setMass(const Scalar mass) {
  if (mass < 0.0 || mass >= 100.) {
    return false;
  }
  mass_ = mass;
  return true;
}


bool QuadrotorDynamics::setMotortauInv(const Scalar tau_inv) {
  if (tau_inv < 1.0) {
    return false;
  }
  motor_tau_inv_ = tau_inv;
  return true;
}


bool QuadrotorDynamics::updateParams(const YAML::Node& params) {
  if (params["quadrotor_dynamics"]) {
    // load parameters from a yaml configuration file
    mass_ = params["quadrotor_dynamics"]["mass"].as<Scalar>();
    kappa_ = params["quadrotor_dynamics"]["kappa"].as<Scalar>();

    motor_omega_min_ =
      params["quadrotor_dynamics"]["motor_omega_min"].as<Scalar>();
    motor_omega_max_ =
      params["quadrotor_dynamics"]["motor_omega_max"].as<Scalar>();
    motor_tau_inv_ =
      (1.0 / params["quadrotor_dynamics"]["motor_tau"].as<Scalar>());

    //
    std::vector<Scalar> thrust_map;
    thrust_map =
      params["quadrotor_dynamics"]["thrust_map"].as<std::vector<Scalar>>();
    thrust_map_ = Map<Vector<3>>(thrust_map.data());

    // compute minmum thrust and maximum thrust
    thrust_min_ = 0.0;
    thrust_max_ = motor_omega_max_ * motor_omega_max_ * thrust_map_(0) +
                  motor_omega_max_ * thrust_map_(1) + thrust_map_(2);

    //
    collective_thrust_min_ = 4.0 * thrust_min_ / mass_;
    collective_thrust_max_ = 4.0 * thrust_max_ / mass_;

    // bodyrates constraints
    std::vector<Scalar> omega_max;
    omega_max =
      params["quadrotor_dynamics"]["omega_max"].as<std::vector<Scalar>>();
    omega_max_ = Map<Vector<3>>(omega_max.data());


    // inertia matrix
    std::vector<Scalar> inertia_vec;
    inertia_vec =
      params["quadrotor_dynamics"]["inertia"].as<std::vector<Scalar>>();
    J_ = Map<Vector<3>>(inertia_vec.data()).asDiagonal();
    J_inv_ = J_.inverse();


    std::vector<Scalar> tbm_fr;
    tbm_fr = params["quadrotor_dynamics"]["tbm_fr"].as<std::vector<Scalar>>();
    std::vector<Scalar> tbm_bl;
    tbm_bl = params["quadrotor_dynamics"]["tbm_bl"].as<std::vector<Scalar>>();
    std::vector<Scalar> tbm_br;
    tbm_br = params["quadrotor_dynamics"]["tbm_br"].as<std::vector<Scalar>>();
    std::vector<Scalar> tbm_fl;
    tbm_fl = params["quadrotor_dynamics"]["tbm_fl"].as<std::vector<Scalar>>();

    t_BM_.row(0) << tbm_fr[0], tbm_bl[0], tbm_br[0], tbm_fl[0];
    t_BM_.row(1) << tbm_fr[1], tbm_bl[1], tbm_br[1], tbm_fl[1];
    t_BM_.row(2) << tbm_fr[2], tbm_bl[2], tbm_br[2], tbm_fl[2];

    // body drag coefficients
    std::vector<Scalar> body_drag_1;
    body_drag_1 =
      params["quadrotor_dynamics"]["body_drag_1"].as<std::vector<Scalar>>();
    cd1_ = Map<Vector<3>>(body_drag_1.data());

    std::vector<Scalar> body_drag_3;
    body_drag_3 =
      params["quadrotor_dynamics"]["body_drag_3"].as<std::vector<Scalar>>();
    cd3_ = Map<Vector<3>>(body_drag_3.data());

    cdz_h_ = params["quadrotor_dynamics"]["body_drag_h"].as<Scalar>();

    // allocation matrix
    // compute column-wise cross product
    // tau_i = t_BM_i * F_i
    B_allocation_ =
      (Matrix<4, 4>() << Vector<4>::Ones().transpose(), t_BM_.row(1),
       -t_BM_.row(0), kappa_ * Vector<4>(-1.0, -1.0, 1.0, 1.0).transpose())
        .finished();

    force_torque_min_(0) = 0.0;
    force_torque_max_(0) = thrust_max_ * 4;
    // torque x
    force_torque_min_(1) =
      B_allocation_.row(1) *
      (Vector<4>() << thrust_max_, 0.0, thrust_max_, 0.0).finished();
    force_torque_max_(1) =
      B_allocation_.row(1) *
      (Vector<4>() << 0.0, thrust_max_, 0.0, thrust_max_).finished();
    // torque y
    force_torque_min_(2) =
      B_allocation_.row(2) *
      (Vector<4>() << thrust_max_, 0.0, 0.0, thrust_max_).finished();
    force_torque_max_(2) =
      B_allocation_.row(2) *
      (Vector<4>() << 0.0, thrust_max_, thrust_max_, 0.0).finished();
    // torque z
    force_torque_min_(3) =
      B_allocation_.row(3) *
      (Vector<4>() << thrust_max_, thrust_max_, 0.0, 0.0).finished();
    force_torque_max_(3) =
      B_allocation_.row(3) *
      (Vector<4>() << 0.0, 0.0, thrust_max_, thrust_max_).finished();
    return valid();
  } else {
    return false;
  }
}

bool QuadrotorDynamics::updateInertiaMarix() { return true; }

std::ostream& operator<<(std::ostream& os, const QuadrotorDynamics& quad) {
  os.precision(3);
  os << "Quadrotor Dynamics:\n"
     << "mass =             [" << quad.mass_ << "]\n"
     << "t_BM =             [" << quad.t_BM_.row(0) << "]\n"
     << "                   [" << quad.t_BM_.row(1) << "]\n"
     << "                   [" << quad.t_BM_.row(2) << "]\n"
     << "inertia =          [" << quad.J_.row(0) << "]\n"
     << "                   [" << quad.J_.row(1) << "]\n"
     << "                   [" << quad.J_.row(2) << "]\n"
     << "motor_omega_min =  [" << quad.motor_omega_min_ << "]\n"
     << "motor_omega_max =  [" << quad.motor_omega_max_ << "]\n"
     << "motor_tau_inv =    [" << quad.motor_tau_inv_ << "]\n"
     << "thrust_map =       [" << quad.thrust_map_.transpose() << "]\n"
     << "kappa =            [" << quad.kappa_ << "]\n"
     << "thrust_min =       [" << quad.thrust_min_ << "]\n"
     << "thrust_max =       [" << quad.thrust_max_ << "]\n"
     << "cthrust_min =       [" << quad.collective_thrust_min_ << "]\n"
     << "cthrust_max =       [" << quad.collective_thrust_max_ << "]\n"
     << "omega_max =        [" << quad.omega_max_.transpose() << "]"
     << std::endl;
  os.precision();
  return os;
}

}  // namespace flightlib