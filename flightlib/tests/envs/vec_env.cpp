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

  // create quadrotor environment and a vector of it
  VecEnv<QuadrotorEnv> vec_env(config_path);
  QuadrotorEnv env(config_path);

  const int vec_obs_dim = vec_env.getObsDim();
  const int vec_act_dim = vec_env.getActDim();
  const int obs_dim = env.getObsDim();
  const int act_dim = env.getActDim();

  EXPECT_EQ(vec_obs_dim, obs_dim);
  EXPECT_EQ(vec_act_dim, act_dim);

  // check parameters
  YAML::Node cfg = YAML::LoadFile(config_path);

  const int seed = cfg["env"]["seed"].as<int>();
  const size_t scene_id = cfg["env"]["scene_id"].as<size_t>();
  const int num_envs = cfg["env"]["num_envs"].as<int>();
  const bool render = cfg["env"]["render"].as<bool>();

  const int vec_seed = vec_env.getSeed();
  const size_t vec_scene_id = vec_env.getSceneID();
  const int vec_num_envs = vec_env.getNumOfEnvs();
  const bool vec_render = vec_env.getUnityRender();

  //
  EXPECT_EQ(seed, vec_seed);
  EXPECT_EQ(scene_id, vec_scene_id);
  EXPECT_EQ(num_envs, vec_num_envs);
  EXPECT_EQ(render, vec_render);

  // for testing
  std::cout << vec_seed << " " << vec_scene_id << " " << vec_num_envs << " "
            << vec_render << std::endl;
}


// TEST(VecEnv, ResetEnv) {}