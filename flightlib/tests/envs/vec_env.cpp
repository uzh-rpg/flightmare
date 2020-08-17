#include "flightlib/envs/vec_env.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

using namespace flightlib;

TEST(VecEnv, Constructor) {
  std::string config_path =
    getenv("FLIGHTMARE_PATH") + std::string("/flightlib/configs/vec_env.yaml");


  Logger logger("VecEnv TEST");
  logger.info("Environment configuration path \"%s\".", config_path.c_str());

  VecEnv<QuadrotorEnv> env(config_path);
}


// TEST(VecEnv, ResetEnv) {}