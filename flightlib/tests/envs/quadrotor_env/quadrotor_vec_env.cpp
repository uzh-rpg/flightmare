#include "flightlib/envs/quadrotor_env/quadrotor_vec_env.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <vector>

#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"

using namespace flightlib;

static constexpr int SIM_STEPS_N = 20;
static constexpr int REW_DIM = 5;

TEST(QuadrotorVecEnv, Constructor) {
  std::string config_path =
    getenv("FLIGHTMARE_PATH") +
    std::string("/flightpy/configs/control/config.yaml");

  const int env_id = 0;
  QuadrotorEnv env(config_path, env_id);
  YAML::Node cfg = YAML::LoadFile(config_path);

  // constructor
  QuadrotorVecEnv<QuadrotorEnv> vec_env_v0;

  const int vec_obs_dim = vec_env_v0.getObsDim();
  const int vec_act_dim = vec_env_v0.getActDim();
  const int obs_dim = env.getObsDim();
  const int act_dim = env.getActDim();

  EXPECT_EQ(vec_obs_dim, obs_dim);
  EXPECT_EQ(vec_act_dim, act_dim);

  const int num_envs = cfg["simulation"]["num_envs"].as<int>();
  const int seed = cfg["simulation"]["seed"].as<int>();
  const size_t scene_id = cfg["unity"]["scene_id"].as<size_t>();
  const bool render = cfg["unity"]["render"].as<bool>();

  const int vec_seed_v0 = vec_env_v0.getSeed();
  const size_t vec_scene_id_v0 = vec_env_v0.getSceneID();
  const int vec_num_envs_v0 = vec_env_v0.getNumOfEnvs();
  const bool vec_render_v0 = vec_env_v0.getUnityRender();

  //
  EXPECT_EQ(seed, vec_seed_v0);
  EXPECT_EQ(scene_id, vec_scene_id_v0);
  EXPECT_EQ(num_envs, vec_num_envs_v0);
  EXPECT_EQ(render, vec_render_v0);
}

TEST(QuadrotorVecEnv, ResetEnv) {
  QuadrotorEnv env;
  QuadrotorVecEnv<QuadrotorEnv> vec_env;
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


TEST(QuadrotorVecEnv, StepEnv) {
  QuadrotorVecEnv<QuadrotorEnv> vec_env;
  const int obs_dim = vec_env.getObsDim();
  const int act_dim = vec_env.getActDim();
  const int rew_dim = vec_env.getRewDim();
  const int num_envs = vec_env.getNumOfEnvs();
  const std::vector<std::string> extra_info_names = vec_env.getExtraInfoNames();

  std::cout << "Obs dim: " << obs_dim << ", Act dim: " << act_dim
            << ", Rew dim: " << rew_dim << ", Env dim: " << num_envs
            << std::endl;


  // vec_env.setUnity(true);

  // reset the environment
  MatrixRowMajor<> obs, act, extra_info, reward;
  BoolVector<> done;

  act.resize(num_envs, act_dim);
  obs.resize(num_envs, obs_dim);
  extra_info.resize(num_envs, extra_info_names.size());
  reward.resize(num_envs, rew_dim);
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
  std::cout << "The RED Error messages below here is expected." << std::endl;
  EXPECT_FALSE(vec_env.step(act, obs, reward, done, extra_info));

  // test observation dimension failure case
  act.resize(num_envs, act_dim);
  act.setRandom();
  act = act.cwiseMax(-1).cwiseMin(1);
  obs.resize(num_envs, obs_dim + 1);
  std::cout << "The RED Error messages below here is expected." << std::endl;
  EXPECT_FALSE(vec_env.step(act, obs, reward, done, extra_info));

  // test reward dimension failure case
  obs.resize(num_envs, obs_dim);
  reward.resize(num_envs, rew_dim + 1);
  std::cout << "The RED  Error messages below here is expected." << std::endl;
  EXPECT_FALSE(vec_env.step(act, obs, reward, done, extra_info));

  // test done dimension failure case
  reward.resize(num_envs, rew_dim);
  done.resize(num_envs + 1);
  std::cout << "The RED  Error messages below here is expected." << std::endl;
  EXPECT_FALSE(vec_env.step(act, obs, reward, done, extra_info));

  // test extra info failure case
  done.resize(num_envs);
  extra_info.resize(num_envs, extra_info_names.size() + 1);
  std::cout << "The RED  Error messages below here is expected." << std::endl;
  EXPECT_FALSE(vec_env.step(act, obs, reward, done, extra_info));
}