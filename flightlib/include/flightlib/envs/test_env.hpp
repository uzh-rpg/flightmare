#pragma once

#include "flightlib/envs/env_base.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"

namespace flightlib {

template<typename EnvBase>
class TestEnv {
 public:
  // constructor & deconstructor
  explicit TestEnv();
  ~TestEnv();
};

}  // namespace flightlib
