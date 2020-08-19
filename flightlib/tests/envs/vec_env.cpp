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
  VecEnv<QuadrotorEnv> vec_env_v0(config_path);
  QuadrotorEnv env(config_path);

  const int vec_obs_dim = vec_env_v0.getObsDim();
  const int vec_act_dim = vec_env_v0.getActDim();
  const int obs_dim = env.getObsDim();
  const int act_dim = env.getActDim();

  EXPECT_EQ(vec_obs_dim, obs_dim);
  EXPECT_EQ(vec_act_dim, act_dim);

  // check parameters
  YAML::Node cfg = YAML::LoadFile(config_path);
  VecEnv<QuadrotorEnv> vec_env_v1(cfg);

  const int seed = cfg["env"]["seed"].as<int>();
  const size_t scene_id = cfg["env"]["scene_id"].as<size_t>();
  const int num_envs = cfg["env"]["num_envs"].as<int>();
  const bool render = cfg["env"]["render"].as<bool>();

  const int vec_seed_v0 = vec_env_v0.getSeed();
  const size_t vec_scene_id_v0 = vec_env_v0.getSceneID();
  const int vec_num_envs_v0 = vec_env_v0.getNumOfEnvs();
  const bool vec_render_v0 = vec_env_v0.getUnityRender();

  const int vec_seed_v1 = vec_env_v1.getSeed();
  const size_t vec_scene_id_v1 = vec_env_v1.getSceneID();
  const int vec_num_envs_v1 = vec_env_v1.getNumOfEnvs();
  const bool vec_render_v1 = vec_env_v1.getUnityRender();

  //
  EXPECT_EQ(seed, vec_seed_v0);
  EXPECT_EQ(scene_id, vec_scene_id_v0);
  EXPECT_EQ(num_envs, vec_num_envs_v0);
  EXPECT_EQ(render, vec_render_v0);

  EXPECT_EQ(seed, vec_seed_v1);
  EXPECT_EQ(scene_id, vec_scene_id_v1);
  EXPECT_EQ(num_envs, vec_num_envs_v1);
  EXPECT_EQ(render, vec_render_v1);
}

// TEST(VecEnv, ResetEnv) {}