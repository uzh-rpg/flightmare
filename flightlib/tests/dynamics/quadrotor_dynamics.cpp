#include "flightlib/dynamics/quadrotor_dynamics.hpp"

#include <gtest/gtest.h>

#include "flightlib/common/math.hpp"
#include "flightlib/common/quad_state.hpp"

using namespace flightlib;

static constexpr Scalar m = 1.0;
static constexpr Scalar l = 0.25;

TEST(QuadrotorDynamics, Constructor) {
  QuadrotorDynamics quad(m, l);

  QuadrotorDynamics quad_copy(quad);

  Scalar mass = quad_copy.mass();
  Matrix<3, 3> J = quad_copy.J();
  Matrix<3, 3> J_inv = quad_copy.J_inv();

  EXPECT_EQ(mass, m);
  EXPECT_TRUE(J.isApprox(Matrix<3, 3>::Zero()));
  EXPECT_TRUE(J_inv.isApprox(Matrix<3, 3>::Identity()));
}

TEST(QuadrotorDynamics, Dynamics) {
  QuadrotorDynamics quad(m, l);

  QuadState hover;
  hover.setZero();

  Vector<> derivative(hover.x);
  QuadState derivative_state;
  derivative_state.setZero();

  EXPECT_TRUE(quad.dState(hover.x, derivative));
  EXPECT_TRUE(quad.dState(hover, &derivative_state));

  EXPECT_TRUE(derivative.isZero());
  EXPECT_TRUE(derivative_state.x.isZero());

  static constexpr int N = 128;

  for (int trail = 0; trail < N; ++trail) {
    // Generate a random state.
    QuadState random_state;
    random_state.x = Vector<QuadState::SIZE>::Random();
    random_state.qx.normalize();

    // Get the differential.
    EXPECT_TRUE(quad.dState(random_state, &derivative_state));

    // Compute it manually.
    QuadState derivative_manual;
    derivative_manual.setZero();

    const Quaternion q_omega(0, random_state.w.x(), random_state.w.y(),
                             random_state.w.z());
    derivative_manual.p = random_state.v;
    derivative_manual.qx = 0.5 * Q_right(q_omega) * random_state.qx;
    derivative_manual.v = random_state.a;
    derivative_manual.w =
      quad.J_inv() *
      (random_state.tau - random_state.w.cross(quad.J() * random_state.w));

    // Compare the derivatives.
    EXPECT_TRUE(derivative_state.x.isApprox(derivative_manual.x));
  }
}

TEST(QuadrotorDynamics, VectorReference) {
  const QuadrotorDynamics quad(m, l);

  static constexpr int N = 128;
  Matrix<QuadState::SIZE, N> states = Matrix<QuadState::SIZE, N>::Random();
  Matrix<QuadState::SIZE, N> states_const =
    Matrix<QuadState::SIZE, N>::Random();
  Matrix<QuadState::SIZE, N> derivates;

  for (int i = 0; i < N; ++i) {
    QuadState derivate;
    EXPECT_TRUE(quad.dState(states.col(i), derivates.col(i)));
    EXPECT_TRUE(quad.dState(QuadState(states.col(i)), &derivate));
    EXPECT_TRUE(quad.dState(states_const.col(i), derivates.col(i)));
  }
}