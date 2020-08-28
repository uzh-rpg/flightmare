#include "flightlib/common/quad_state.hpp"

#include <gtest/gtest.h>

using namespace flightlib;

TEST(QuadState, Constructor) {
  QuadState state;
  EXPECT_TRUE(state.x.hasNaN());

  QuadState zero_static_state(Vector<QuadState::SIZE>::Zero());
  EXPECT_TRUE(zero_static_state.x.allFinite());

  QuadState zero_dynamic_state(Vector<>::Zero(QuadState::SIZE));
  EXPECT_TRUE(zero_dynamic_state.x.allFinite());
}

TEST(QuadState, Accessors) {
  const int N = QuadState::SIZE;
  Vector<> x(N);
  for (int i = 0; i < QuadState::SIZE; ++i) x(i) = i;

  QuadState state(x);

  EXPECT_EQ(state.p(0), x(0));
  EXPECT_EQ(state.p(1), x(1));
  EXPECT_EQ(state.p(2), x(2));
  EXPECT_EQ(state.qx(0), x(3));
  EXPECT_EQ(state.qx(1), x(4));
  EXPECT_EQ(state.qx(2), x(5));
  EXPECT_EQ(state.qx(3), x(6));
  EXPECT_EQ(state.v(0), x(7));
  EXPECT_EQ(state.v(1), x(8));
  EXPECT_EQ(state.v(2), x(9));
  EXPECT_EQ(state.w(0), x(10));
  EXPECT_EQ(state.w(1), x(11));
  EXPECT_EQ(state.w(2), x(12));
  EXPECT_EQ(state.a(0), x(13));
  EXPECT_EQ(state.a(1), x(14));
  EXPECT_EQ(state.a(2), x(15));
  EXPECT_EQ(state.tau(0), x(16));
  EXPECT_EQ(state.tau(1), x(17));
  EXPECT_EQ(state.tau(2), x(18));
  EXPECT_EQ(state.bw(0), x(19));
  EXPECT_EQ(state.bw(1), x(20));
  EXPECT_EQ(state.bw(2), x(21));
  EXPECT_EQ(state.ba(0), x(22));
  EXPECT_EQ(state.ba(1), x(23));
  EXPECT_EQ(state.ba(2), x(24));

  x += Vector<>::Ones(QuadState::SIZE);
  state.p += Vector<3>::Ones();
  state.qx += Vector<4>::Ones();
  state.v += Vector<3>::Ones();
  state.w += Vector<3>::Ones();
  state.a += Vector<3>::Ones();
  state.tau += Vector<3>::Ones();
  state.bw += Vector<3>::Ones();
  state.ba += Vector<3>::Ones();

  EXPECT_EQ(state.p(0), x(0));
  EXPECT_EQ(state.p(1), x(1));
  EXPECT_EQ(state.p(2), x(2));
  EXPECT_EQ(state.qx(0), x(3));
  EXPECT_EQ(state.qx(1), x(4));
  EXPECT_EQ(state.qx(2), x(5));
  EXPECT_EQ(state.qx(3), x(6));
  EXPECT_EQ(state.v(0), x(7));
  EXPECT_EQ(state.v(1), x(8));
  EXPECT_EQ(state.v(2), x(9));
  EXPECT_EQ(state.w(0), x(10));
  EXPECT_EQ(state.w(1), x(11));
  EXPECT_EQ(state.w(2), x(12));
  EXPECT_EQ(state.a(0), x(13));
  EXPECT_EQ(state.a(1), x(14));
  EXPECT_EQ(state.a(2), x(15));
  EXPECT_EQ(state.tau(0), x(16));
  EXPECT_EQ(state.tau(1), x(17));
  EXPECT_EQ(state.tau(2), x(18));
  EXPECT_EQ(state.bw(0), x(19));
  EXPECT_EQ(state.bw(1), x(20));
  EXPECT_EQ(state.bw(2), x(21));
  EXPECT_EQ(state.ba(0), x(22));
  EXPECT_EQ(state.ba(1), x(23));
  EXPECT_EQ(state.ba(2), x(24));

  EXPECT_TRUE(state.x.isApprox(x));
}

TEST(QuadState, Compare) {
  const int N = QuadState::SIZE;
  Vector<> x(N);
  for (int i = 0; i < QuadState::SIZE; ++i) x(i) = i;

  QuadState state(x);
  state.t = 0.0;
  QuadState other_state(x);
  other_state.t = 0.0;

  EXPECT_TRUE(state == other_state);
  state.p += Vector<3>::Ones();
  EXPECT_FALSE(state == other_state);
  other_state.p += Vector<3>::Ones();
  EXPECT_TRUE(state == other_state);
  state.t += 1.0;
  EXPECT_FALSE(state == other_state);
}