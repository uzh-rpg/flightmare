#include "flightlib/objects/quadrotor.hpp"

namespace flightlib {

Quadrotor::Quadrotor(const QuadrotorDynamics &dynamics)
  : dynamics_(dynamics), size_(1.0, 1.0, 1.0), collision_(false) {
  updateDynamics(dynamics);
  reset();
}

Quadrotor::~Quadrotor() {}

bool Quadrotor::run(const Command &cmd, const Scalar ctl_dt) {
  if (!setCommand(cmd)) return false;
  return run(ctl_dt);
}

bool Quadrotor::run(const Scalar ctl_dt) {
  if (!state_.valid()) return false;
  if (!cmd_.valid()) return false;

  QuadState next_state = state_;
  const Scalar max_dt = integrator_ptr_->dtMax();
  Scalar remain_ctl_dt = ctl_dt;

  // simulation loop
  while (remain_ctl_dt > 0.0) {
    const Scalar sim_dt = std::min(remain_ctl_dt, max_dt);

    const Vector<4> motor_thrusts_des =
      cmd_.isSingleRotorThrusts() ? cmd_.thrusts
                                  : runFlightCtl(sim_dt, state_.w, cmd_);

    runMotors(sim_dt, motor_thrusts_des);

    const Vector<4> force_torques = B_allocation_ * motor_thrusts_;

    // Compute linear acceleration and body torque
    const Vector<3> force(0.0, 0.0, force_torques[0]);
    state_.a = state_.R() * force / dynamics_.mass() + gz_;
    state_.tau << force_torques.segment<3>(1);

    integrator_ptr_->step(state_.x, sim_dt, next_state.x);
    state_.x = next_state.x;

    remain_ctl_dt -= sim_dt;
  }

  state_.t += ctl_dt;
  return true;
}

bool Quadrotor::reset(void) {
  state_.setZero();
  motor_omega_.setZero();
  motor_thrusts_.setZero();
  return true;
}

bool Quadrotor::reset(const QuadState &state) {
  if (!state.valid()) return false;
  state_ = state;
  motor_omega_.setZero();
  motor_thrusts_.setZero();
  return true;
}

Vector<4> Quadrotor::runFlightCtl(const Scalar sim_dt, const Vector<3> &omega,
                                  const Command &command) {
  const Scalar force = dynamics_.mass() * command.collective_thrust;

  const Vector<3> omega_err = command.omega - omega;

  const Vector<3> body_torque_des =
    dynamics_.J() * Kinv_ang_vel_tau_ * omega_err +
    state_.w.cross(dynamics_.J() * state_.w);

  const Vector<4> thrust_and_torque(force, body_torque_des.x(),
                                    body_torque_des.y(), body_torque_des.z());

  const Vector<4> motor_thrusts_des = B_allocation_inv_ * thrust_and_torque;

  return dynamics_.clampThrust(motor_thrusts_des);
}

void Quadrotor::runMotors(const Scalar sim_dt,
                          const Vector<4> &motor_thruts_des) {
  const Vector<4> motor_omega_des =
    dynamics_.motorThrustToOmega(motor_thruts_des);
  const Vector<4> motor_omega_clamped =
    dynamics_.clampMotorOmega(motor_omega_des);

  // simulate motors as a first-order system
  const Scalar c = std::exp(-sim_dt * dynamics_.motor_tau_inv());
  motor_omega_ = c * motor_omega_ + (1.0 - c) * motor_omega_clamped;

  motor_thrusts_ = dynamics_.motorOmegaToThrust(motor_omega_);
  motor_thrusts_ = dynamics_.clampThrust(motor_thrusts_);
}

bool Quadrotor::setCommand(const Command &cmd) {
  if (!cmd.valid()) return false;
  cmd_ = cmd;

  if (std::isfinite(cmd_.collective_thrust))
    cmd_.collective_thrust = dynamics_.clampThrust(cmd_.collective_thrust);

  if (cmd_.omega.allFinite()) cmd_.omega = dynamics_.clampBodyrates(cmd_.omega);

  if (cmd_.thrusts.allFinite())
    cmd_.thrusts = dynamics_.clampThrust(cmd_.thrusts);

  return true;
}

bool Quadrotor::setState(const QuadState &state) {
  if (!state.valid()) return false;

  state_ = state;
  return true;
}

bool Quadrotor::getState(QuadState *const state) const {
  if (!state_.valid()) return false;

  *state = state_;
  return true;
}

bool Quadrotor::getMotorThrusts(Ref<Vector<4>> motor_thrusts) const {
  motor_thrusts = motor_thrusts_;
  return true;
}

bool Quadrotor::getMotorOmega(Ref<Vector<4>> motor_omega) const {
  motor_omega = motor_omega_;
  return true;
}

bool Quadrotor::getDynamics(QuadrotorDynamics *const dynamics) const {
  if (!dynamics_.valid()) return false;
  *dynamics = dynamics_;
  return true;
}

const QuadrotorDynamics &Quadrotor::getDynamics() const { return dynamics_; }

bool Quadrotor::updateDynamics(const QuadrotorDynamics &dynamics) {
  if (!dynamics.valid()) {
    return false;
  }
  dynamics_ = dynamics;
  integrator_ptr_ =
    std::make_unique<IntegratorRK4>(dynamics_.getDynamicsFunction());

  B_allocation_ = dynamics_.getAllocationMatrix();
  B_allocation_inv_ = B_allocation_.inverse();
  return true;
}

Vector<3> Quadrotor::getSize(void) { return size_; }

Vector<3> Quadrotor::getPosition(void) { return size_; }

}  // namespace flightlib
