#include "flightlib/envs/test_env.hpp"
#include <iostream>

namespace flightlib {

template<typename EnvBase>
TestEnv<EnvBase>::TestEnv() {
  std::cout << "Hellow Test Env!" << std::endl;
}

template<typename EnvBase>
TestEnv<EnvBase>::~TestEnv() {}

template class TestEnv<QuadrotorEnv>;

}  // namespace flightlib
