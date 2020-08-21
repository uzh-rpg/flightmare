#include <gtest/gtest.h>

#include "flightlib/common/integrator_base.hpp"
#include "flightlib/common/integrator_euler.hpp"
#include "flightlib/common/integrator_rk4.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/dynamics/quadrotor_dynamics.hpp"

using namespace flightlib;

static constexpr Scalar MASS = 1.0;
static constexpr Scalar ARM_LENGTH = 0.25;

TEST(Integrators, ManualEulerAccelerationCheck) {
  static constexpr Scalar dt = 1.0;
  // Using lower tolerance for check because of accuracy of forward Euler.
  static constexpr Scalar tol = 1e-3;

  QuadState initial;
  initial.setZero();

  const QuadrotorDynamics quad(MASS, ARM_LENGTH);

  IntegratorEuler euler(quad.getDynamicsFunction());

  initial.a = Vector<3>::Random();

  QuadState expected(initial);
  expected.p = initial.p + dt * dt / 2.0 * initial.a;
  expected.v = initial.v + dt * initial.a;

  QuadState final;

  EXPECT_TRUE(euler.integrate(initial.x, dt, final.x));
  EXPECT_TRUE(final.x.isApprox(expected.x, tol))
    << "expected state:   " << expected.x.transpose() << "\n"
    << "integrated state: " << final.x.transpose() << "\n";
}

TEST(Integrators, ManualRungeKuttaAccelerationCheck) {
  static constexpr Scalar dt = 1.0;

  QuadState initial;
  initial.setZero();

  const QuadrotorDynamics quad(MASS, ARM_LENGTH);

  IntegratorRK4 rungekutta(quad.getDynamicsFunction());

  initial.a = Vector<3>::Random();

  QuadState expected(initial);
  expected.p = initial.p + dt * dt / 2.0 * initial.a;
  expected.v = initial.v + dt * initial.a;

  QuadState final;

  EXPECT_TRUE(rungekutta.integrate(initial.x, dt, final.x));
  EXPECT_TRUE(final.x.isApprox(expected.x, 1e-3))
    << "expected state:   " << expected.x.transpose() << "\n"
    << "integrated state: " << final.x.transpose() << "\n";
}

TEST(Integrators, QuadStateInterface) {
  static constexpr Scalar dt = 1.0;

  QuadState initial;
  initial.setZero();

  const QuadrotorDynamics quad(MASS, ARM_LENGTH);

  QuadState int_euler;
  QuadState int_rungekutta;
  int_euler.t = dt;
  int_rungekutta.t = dt;

  IntegratorEuler euler(quad.getDynamicsFunction());
  IntegratorRK4 rungekutta(quad.getDynamicsFunction());

  EXPECT_TRUE(euler.integrate(initial, &int_euler));
  EXPECT_TRUE(rungekutta.integrate(initial, &int_rungekutta));

  EXPECT_TRUE(int_euler.x.isApprox(initial.x));
  EXPECT_TRUE(int_rungekutta.x.isApprox(initial.x));

  int_euler.t = -0.1;
  int_rungekutta.t = -0.1;

  EXPECT_FALSE(euler.integrate(initial, &int_euler));
  EXPECT_FALSE(rungekutta.integrate(initial, &int_rungekutta));
}

TEST(Integrators, CheckEulerAgainstRungeKutta) {
  static constexpr int N = 16;  // Test not too often for speed in debug mode.
  static constexpr Scalar dt = 0.5;
  // Using lower tolerance for check because of accuracy of forward Euler.
  static constexpr Scalar tol = 1;

  const QuadrotorDynamics quad(MASS, ARM_LENGTH);
  const IntegratorEuler euler(quad.getDynamicsFunction());
  const IntegratorRK4 rungekutta(quad.getDynamicsFunction());

  for (int trials = 0; trials < N; ++trials) {
    QuadState initial(Vector<QuadState::SIZE>::Random());
    initial.qx.normalize();

    QuadState int_euler;
    QuadState int_rungekutta;

    EXPECT_TRUE(euler.integrate(initial.x, dt, int_euler.x));
    EXPECT_TRUE(rungekutta.integrate(initial.x, dt, int_rungekutta.x));
    EXPECT_TRUE(int_euler.x.isApprox(int_rungekutta.x, tol))
      << "Euler intergrated:\n"
      << int_euler.x.transpose() << std::endl
      << "RungeKutta intergrated:\n"
      << int_rungekutta.x.transpose() << std::endl;
  }
}
