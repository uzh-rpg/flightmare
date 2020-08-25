#include "flightlib/envs/test_env.hpp"
#include <iostream>

namespace flightlib {

template<typename EnvBase>
TestEnv<EnvBase>::TestEnv() {
  std::cout << "Hellow Test Env!" << std::endl;
}

template<typename EnvBase>
TestEnv<EnvBase>::~TestEnv() {}

template<typename EnvBase>
void TestEnv<EnvBase>::reset(
  Eigen::Ref<
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
    obs) {
  obs << 1, 2, 3, 4, 5, 6, 7, 8, 9;
}

template class TestEnv<QuadrotorEnv>;

}  // namespace flightlib
