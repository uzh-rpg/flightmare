#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/dynamics/quadrotor_dynamics.hpp"

#include <gtest/gtest.h>

using namespace flightlib;

TEST(Quadrotor, Constructor) {
  QuadrotorDynamics dynamics(1.0, 0.2);
  Quadrotor quadrotor(dynamics);

  //
}