#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

#include "flightlib/common/logger.hpp"

using namespace flightlib;

static constexpr int OBS_DIM = 15;
static constexpr int ACT_DIM = 4;
static constexpr int REW_DIM = 5;
static constexpr int SIM_STEPS_N = 20;
static constexpr Scalar TOL = 1e-3;

TEST(QuadrotorEnv, Constructor) {
  Logger logger("QuadrotorEnv");

  // create env and load configuration from a yaml file.
  std::string config_path =
    getenv("FLIGHTMARE_PATH") +
    std::string("/flightpy/configs/control/config.yaml");
  logger.info("Environment configuration path \"%s\".", config_path.c_str());
  const int env_id = 0;
  QuadrotorEnv env0(config_path, env_id);

  Vector<3> debug{3.0, 3.0, 3.0};

  // check observation and action dimensions
  int obs_dim = env0.getObsDim();
  int act_dim = env0.getActDim();
  EXPECT_EQ(obs_dim, OBS_DIM);
  EXPECT_EQ(act_dim, ACT_DIM);

  // load parameters via the yaml file
  YAML::Node cfg = YAML::LoadFile(config_path);
  QuadrotorEnv env1;
  EXPECT_TRUE(env1.loadParam(cfg));

  // evaluate parameters
  const int obs_dim1 = env1.getObsDim();
  const int act_dim1 = env1.getActDim();
  Matrix<OBS_DIM, OBS_DIM> Q;
  Matrix<ACT_DIM, ACT_DIM> Q_act;

  EXPECT_EQ(obs_dim1, OBS_DIM);
  EXPECT_EQ(act_dim1, ACT_DIM);

  //
  std::cout << env1 << std::endl;
}

TEST(QuadrotorEnv, ResetEnv) {
  QuadrotorEnv env;

  // randomly reset the environment
  Vector<OBS_DIM> obs;
  Vector<OBS_DIM> exp_obs;

  EXPECT_TRUE(env.reset(obs));
  env.getObs(exp_obs);
  EXPECT_TRUE(exp_obs.isApprox(obs));

  // reset the environment to the zero state
  EXPECT_TRUE(env.reset(obs, false));

  env.reset(obs);
}

TEST(QuadrotorEnv, StepEnv) {
  QuadrotorEnv env;

  Vector<ACT_DIM> act{0, 0, 0, 0};
  Vector<OBS_DIM> obs;
  Vector<OBS_DIM> next_obs;
  Vector<REW_DIM> rew;

  env.reset(obs, false);
  bool success = env.step(act, next_obs, rew);

  // check control command
  // Command cmd;
  // Vector<4> cmd;
  // EXPECT_TRUE(env.getQuadAct(cmd));
  // if (cmd.isRatesThrust()) {
  //   EXPECT_EQ(cmd.collective_thrust, -Gz);
  //   EXPECT_TRUE(cmd.omega.allFinite());
  // } else if (cmd.isSingleRotorThrusts()) {
  //   // TODO: Not tested yet.
  //   EXPECT_TRUE(cmd.thrusts.allFinite());
  // }

  // Vector<ACT_DIM> executed_act;
  // EXPECT_TRUE(env.getAct(executed_act));

  for (int i = 0; i < SIM_STEPS_N; i++) {
    success = env.step(act, next_obs, rew);
  }

  EXPECT_TRUE(success);
  // std::cout << reward << std::endl;
  // in case this failed, decrease motor_tau in the Quadrotor class.
  // EXPECT_TRUE(((next_obs - obs).norm() < 1.0));
}
