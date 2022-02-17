#include "flightlib/envs/vision_env/vision_env.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

#include "flightlib/common/logger.hpp"

using namespace flightlib;

TEST(VisionEnv, Constructor) {
  Logger logger("VisionEnv");

  // create env and load configuration from a yaml file.
  std::string config_path = getenv("FLIGHTMARE_PATH") +
                            std::string("/flightpy/configs/vision/config.yaml");
  logger.info("Environment configuration path \"%s\".", config_path.c_str());
  const int env_id = 0;
  VisionEnv env0(config_path, env_id);

  // check observation and action dimensions
  int obs_dim = env0.getObsDim();
  int act_dim = env0.getActDim();
  std::cout << "Observation Dimension: " << obs_dim
            << ", Action Dimension: " << act_dim << std::endl;
}

TEST(VisionEnv, ResetEnv) {}

TEST(VisionEnv, StepEnv) {}
