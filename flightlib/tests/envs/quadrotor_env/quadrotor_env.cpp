#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"
#include "flightlib/common/logger.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

using namespace flightlib;

TEST(QuadrotorEnv, Constructor) {
  std::string config_path =
    getenv("FLIGHTMARE_PATH") +
    std::string("/flightlib/configs/quadrotor_env.yaml");
  Logger logger("QuadrotorEnv");
  logger.info("Environment configuration path \"%s\".", config_path.c_str());

  //
  YAML::Node cfg = YAML::LoadFile(config_path);
  QuadrotorEnv env;

  //
  QuadState quad_state;
}
