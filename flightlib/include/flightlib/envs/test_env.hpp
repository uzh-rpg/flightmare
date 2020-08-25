#pragma once

// pybind11
// #include <pybind11/eigen.h>
// #include <pybind11/pybind11.h>
// #include <pybind11/stl.h>
#include <eigen3/Eigen/Eigen>

#include "flightlib/envs/env_base.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"

namespace flightlib {

template<typename EnvBase>
class TestEnv {
 public:
  // constructor & deconstructor
  TestEnv();
  ~TestEnv();

  void reset(
    Eigen::Ref<
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      obs);
};

}  // namespace flightlib
