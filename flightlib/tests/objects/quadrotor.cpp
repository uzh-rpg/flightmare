#include "flightlib/objects/quadrotor.hpp"

#include <gtest/gtest.h>

#include "flightlib/common/quad_state.hpp"
#include "flightlib/dynamics/quadrotor_dynamics.hpp"

using namespace flightlib;

static constexpr Scalar CTRL_UPDATE_FREQUENCY = 50.0;
static constexpr int SIM_STEPS_N = 20;

TEST(Quadrotor, Constructor) {
  QuadrotorDynamics dynamics(1.0);
  Quadrotor quad0(dynamics);

  QuadState quad_state;
  quad0.getState(&quad_state);

  QuadState expected_state;
  expected_state.setZero();

  EXPECT_EQ(expected_state.x(QS::ATTW), quad_state.x(QS::ATTW));
  EXPECT_TRUE(quad_state.x.isApprox(expected_state.x));

  Quadrotor quad1;
  quad1.getState(&quad_state);

  EXPECT_EQ(expected_state.x(QS::ATTW), quad_state.x(QS::ATTW));
  EXPECT_TRUE(quad_state.x.isApprox(expected_state.x));

  //
  std::string cfg_path = getenv("FLIGHTMARE_PATH") +
                         std::string("/flightpy/configs/control/config.yaml");

  Quadrotor quad2(cfg_path);
  quad2.getState(&quad_state);

  EXPECT_EQ(expected_state.x(QS::ATTW), quad_state.x(QS::ATTW));
  EXPECT_TRUE(quad_state.x.isApprox(expected_state.x));
}

TEST(Quadrotor, ResetSimulator) {
  Quadrotor quad;
  QuadState initial_state;
  QuadState quad_state;
  Vector<4> motor_omega;
  Vector<4> motor_thrusts;

  // default reset
  initial_state.setZero();

  EXPECT_TRUE(quad.reset());
  EXPECT_TRUE(quad.getState(&quad_state));

  EXPECT_TRUE(quad_state.x.isApprox(initial_state.x));
  EXPECT_EQ(quad_state.t, 0.0);

  // randomly reset the quadrotor state
  initial_state.setZero();
  initial_state.x = Vector<QuadState::SIZE>::Random();

  EXPECT_TRUE(quad.reset(initial_state));
  EXPECT_TRUE(quad.getState(&quad_state));

  EXPECT_TRUE(quad_state.x.isApprox(initial_state.x));
  EXPECT_EQ(quad_state.t, 0.0);

  // check failure case
  QuadState initial_state_nan;
  EXPECT_FALSE(quad.reset(initial_state_nan));
  EXPECT_FALSE(quad.setState(initial_state_nan));
}

TEST(Quadrotor, RunQuadCmdFeedThrough) {
  Quadrotor quad;
  QuadrotorDynamics dynamics = quad.getDynamics();
  Scalar inv_tau = 1.0 / 1e-6;
  dynamics.setMotortauInv(inv_tau);
  EXPECT_TRUE(dynamics.updateBodyDragCoeff1(Vector<3>(0.0, 0.0, 0.0)));
  EXPECT_TRUE(dynamics.updateBodyDragCoeff3(Vector<3>(0.0, 0.0, 0.0)));
  EXPECT_TRUE(dynamics.updateBodyDragCoeffZH(0.0));
  EXPECT_TRUE(quad.updateDynamics(dynamics));
  const Scalar ctl_dt = (1.0 / CTRL_UPDATE_FREQUENCY);

  QuadState quad_state;
  QuadState final_state;

  // hovering test
  quad_state.setZero();
  quad_state.p << 0.0, 0.0, 1.0;
  quad.reset(quad_state);

  const Scalar mass = dynamics.getMass();

  Command cmd;
  cmd.setCmdMode(Command::SINGLEROTOR);
  cmd.t = 0.0;
  cmd.thrusts = Vector<4>::Constant(-Gz * mass) / 4.0;
  for (int i = 0; i < SIM_STEPS_N; i++) {
    // run quadrotor simulator
    EXPECT_TRUE(quad.setCommand(cmd));
    EXPECT_TRUE(quad.run(ctl_dt));
    //
    quad_state.t += ctl_dt;
  }
  quad.getState(&final_state);

  EXPECT_NEAR(final_state.t, quad_state.t, 1e-9);
  EXPECT_TRUE(quad_state.x.isApprox(final_state.x, 1e-3));

  // free fall
  quad_state.setZero();
  quad_state.p << 0.0, 0.0, 1.0;
  quad.reset(quad_state);

  cmd.t = 0.0;
  cmd.thrusts << 0.0, 0.0, 0.0, 0.0;

  for (int i = 0; i < SIM_STEPS_N; i++) {
    // run quadrotor simulator
    quad.run(cmd, ctl_dt);

    // manually update the state
    quad_state.p += quad_state.v * ctl_dt + ctl_dt * ctl_dt / 2.0 * GVEC;
    quad_state.v += ctl_dt * GVEC;
    quad_state.a = GVEC;
    quad_state.t += ctl_dt;
  }

  final_state.setZero();
  quad.getState(&final_state);

  EXPECT_NEAR(final_state.t, quad_state.t, 1e-9);
  EXPECT_TRUE(quad_state.x.isApprox(final_state.x, 1e-3));

  // taking off
  quad_state.setZero();
  quad_state.p << 0.0, 0.0, 1.0;
  quad.reset(quad_state);

  //
  cmd.t = 0.0;
  cmd.thrusts = Vector<4>::Constant(-Gz * 2 * mass) / 4.0;

  // compute acceleration
  Vector<3> acc{0.0, 0.0, (cmd.thrusts.sum() + mass * Gz) / mass};

  for (int i = 0; i < SIM_STEPS_N; i++) {
    // run quadrotor simulator
    quad.run(cmd, ctl_dt);

    // manually update the state
    // assume the orientation zero in all axes
    quad_state.p += quad_state.v * ctl_dt + ctl_dt * ctl_dt / 2.0 * acc;
    quad_state.v += ctl_dt * acc;
    quad_state.a = acc;
    quad_state.t += ctl_dt;
  }

  final_state.setZero();
  quad.getState(&final_state);
  EXPECT_NEAR(final_state.t, quad_state.t, 1e-9);
  EXPECT_TRUE(quad_state.x.isApprox(final_state.x, 1e-3));
  EXPECT_GT(quad_state.x(QS::POSZ), 1.0);
}

TEST(Quadrotor, RunSimulatorBodyRate) {
  Quadrotor quad;
  QuadrotorDynamics dynamics = quad.getDynamics();
  Scalar inv_tau = (1.0 / 1e-6);
  dynamics.setMotortauInv(inv_tau);

  EXPECT_TRUE(dynamics.updateBodyDragCoeff1(Vector<3>(0.0, 0.0, 0.0)));
  EXPECT_TRUE(dynamics.updateBodyDragCoeff3(Vector<3>(0.0, 0.0, 0.0)));
  EXPECT_TRUE(dynamics.updateBodyDragCoeffZH(0.0));
  EXPECT_TRUE(quad.updateDynamics(dynamics));

  const Scalar ctl_dt = (1.0 / CTRL_UPDATE_FREQUENCY);

  QuadState quad_state;
  QuadState final_state;

  // hovering test
  quad_state.setZero();
  quad_state.p << 0.0, 0.0, 1.0;
  quad.reset(quad_state);

  Command cmd;
  cmd.setCmdMode(Command::THRUSTRATE);
  cmd.t = 0.0;
  cmd.collective_thrust = -Gz;
  cmd.omega << 0.0, 0.0, 0.0;

  for (int i = 0; i < SIM_STEPS_N; i++) {
    // run quadrotor simulator
    EXPECT_TRUE(quad.run(cmd, ctl_dt));

    quad_state.t += ctl_dt;
  }
  quad.getState(&final_state);

  EXPECT_NEAR(final_state.t, quad_state.t, 1e-9);
  EXPECT_TRUE(quad_state.x.isApprox(final_state.x));

  // free fall
  quad_state.setZero();
  quad_state.p << 0.0, 0.0, 1.0;
  quad.reset(quad_state);

  cmd.t = 0.0;
  cmd.collective_thrust = 0.0;
  cmd.omega << 0.0, 0.0, 0.0;

  for (int i = 0; i < SIM_STEPS_N; i++) {
    // run quadrotor simulator
    quad.run(cmd, ctl_dt);

    // manually update the state
    quad_state.p += quad_state.v * ctl_dt + ctl_dt * ctl_dt / 2.0 * GVEC;
    quad_state.v += ctl_dt * GVEC;
    quad_state.a = GVEC;
    quad_state.t += ctl_dt;
  }

  final_state.setZero();
  quad.getState(&final_state);

  EXPECT_NEAR(final_state.t, quad_state.t, 1e-9);
  EXPECT_TRUE(quad_state.x.isApprox(final_state.x));
}
