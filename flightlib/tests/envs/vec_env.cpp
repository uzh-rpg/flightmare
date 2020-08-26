#include "flightlib/envs/vec_env.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <vector>

using namespace flightlib;

static constexpr int SIM_STEPS_N = 20;

TEST(VecEnv, Constructor) {
  std::string config_path =
    getenv("FLIGHTMARE_PATH") + std::string("/flightlib/configs/vec_env.yaml");

  QuadrotorEnv env(config_path);
  // load configurations
  YAML::Node cfg = YAML::LoadFile(config_path);

  // constructor 0
  VecEnv<QuadrotorEnv> vec_env_v0(
    config_path);  // construct object from yaml file
  // constructor 1
  VecEnv<QuadrotorEnv> vec_env_v1(cfg);  // construct object from yaml::node
  // // constructor 2
  // VecEnv<QuadrotorEnv> vec_env_v2(
  //   "{env: { seed : 1, scene_id: 0, num_envs: 10, num_threads: 10, render: "
  //   "no}}",
  //   false);

  const int vec_obs_dim = vec_env_v0.getObsDim();
  const int vec_act_dim = vec_env_v0.getActDim();
  const int obs_dim = env.getObsDim();
  const int act_dim = env.getActDim();

  EXPECT_EQ(vec_obs_dim, obs_dim);
  EXPECT_EQ(vec_act_dim, act_dim);

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

TEST(VecEnv, ResetEnv) {
  QuadrotorEnv env;
  VecEnv<QuadrotorEnv> vec_env;
  //
  const int vec_obs_dim = vec_env.getObsDim();
  const int vec_act_dim = vec_env.getActDim();
  const int vec_num_env = vec_env.getNumOfEnvs();
  const int obs_dim = env.getObsDim();
  const int act_dim = env.getActDim();  // EXPECT_EQ(vec_obs_dim, obs_dim);

  EXPECT_EQ(vec_act_dim, act_dim);
  EXPECT_EQ(vec_obs_dim, obs_dim);

  // reset the environment
  MatrixRowMajor<> obs;
  obs.resize(vec_num_env, vec_obs_dim);
  EXPECT_TRUE(vec_env.reset(obs));
  EXPECT_TRUE(obs.allFinite());

  obs.resize(vec_num_env, vec_obs_dim + 1);
  EXPECT_FALSE(vec_env.reset(obs));
}


TEST(VecEnv, StepEnv) {
  VecEnv<QuadrotorEnv> vec_env;
  const int obs_dim = vec_env.getObsDim();
  const int act_dim = vec_env.getActDim();
  const int num_envs = vec_env.getNumOfEnvs();
  const std::vector<std::string> extra_info_names = vec_env.getExtraInfoNames();

  // vec_env.setUnity(true);

  // reset the environment
  MatrixRowMajor<> obs, act, extra_info;
  Vector<> reward;
  BoolVector<> done;

  act.resize(num_envs, act_dim);
  obs.resize(num_envs, obs_dim);
  extra_info.resize(num_envs, extra_info_names.size());
  reward.resize(num_envs);
  done.resize(num_envs);

  EXPECT_TRUE(vec_env.reset(obs));
  EXPECT_TRUE(obs.allFinite());

  // test step function
  for (int i = 0; i < SIM_STEPS_N; i++) {
    act.setRandom();
    act = act.cwiseMax(-1).cwiseMin(1);
    vec_env.step(act, obs, reward, done, extra_info);
  }
  EXPECT_TRUE(act.allFinite());
  EXPECT_TRUE(obs.allFinite());
  EXPECT_TRUE(reward.allFinite());
  EXPECT_TRUE(done.allFinite());

  // test action dimension failure case
  act.resize(num_envs, act_dim - 1);
  act.setRandom();
  act = act.cwiseMax(-1).cwiseMin(1);
  EXPECT_FALSE(vec_env.step(act, obs, reward, done, extra_info));

  // test observation dimension failure case
  act.resize(num_envs, act_dim);
  act.setRandom();
  act = act.cwiseMax(-1).cwiseMin(1);
  obs.resize(num_envs, obs_dim + 1);
  EXPECT_FALSE(vec_env.step(act, obs, reward, done, extra_info));

  // test reward dimension failure case
  obs.resize(num_envs, obs_dim);
  reward.resize(num_envs + 1);
  EXPECT_FALSE(vec_env.step(act, obs, reward, done, extra_info));

  // test done dimension failure case
  reward.resize(num_envs);
  done.resize(num_envs + 1);
  EXPECT_FALSE(vec_env.step(act, obs, reward, done, extra_info));

  // test extra info failure case
  done.resize(num_envs);
  extra_info.resize(num_envs, extra_info_names.size() + 1);
  EXPECT_FALSE(vec_env.step(act, obs, reward, done, extra_info));
}