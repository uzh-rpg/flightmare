#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"
#include "flightlib/common/logger.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

using namespace flightlib;

static constexpr int OBS_DIM = 9;
static constexpr int ACT_DIM = 4;
static constexpr int SIM_STEPS_N = 20;
static constexpr Scalar TOL = 1e-3;

TEST(QuadrotorEnv, Constructor) {
  Logger logger("QuadrotorEnv");

  // create env and load configuration from a yaml file.
  std::string config_path =
    getenv("FLIGHTMARE_PATH") +
    std::string("/flightlib/configs/quadrotor_env.yaml");
  logger.info("Environment configuration path \"%s\".", config_path.c_str());
  QuadrotorEnv env0(config_path);

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
  Scalar expect_sim_dt = cfg["quadrotor_env"]["sim_dt"].as<Scalar>();
  std::vector<Scalar> Q_pos(3), Q_ori(3), Q_lin_vel(3), Q_cmd(4);
  Q_pos = cfg["rl"]["Q_pos"].as<std::vector<Scalar>>();
  Q_ori = cfg["rl"]["Q_ori"].as<std::vector<Scalar>>();
  Q_lin_vel = cfg["rl"]["Q_lin_vel"].as<std::vector<Scalar>>();
  Q_cmd = cfg["rl"]["Q_cmd"].as<std::vector<Scalar>>();
  Matrix<OBS_DIM, OBS_DIM> expect_Q =
    (Vector<OBS_DIM>() << Q_pos[0], Q_pos[1], Q_pos[2], Q_ori[0], Q_ori[1],
     Q_ori[2], Q_lin_vel[0], Q_lin_vel[1], Q_lin_vel[2])
      .finished()
      .asDiagonal();
  Matrix<ACT_DIM, ACT_DIM> expect_Q_act =
    (Vector<ACT_DIM>() << Q_cmd[0], Q_cmd[1], Q_cmd[2], Q_cmd[3])
      .finished()
      .asDiagonal();

  const int obs_dim1 = env1.getObsDim();
  const int act_dim1 = env1.getActDim();
  const Scalar sim_dt = env1.getSimTimeStep();
  Matrix<OBS_DIM, OBS_DIM> Q;
  Matrix<ACT_DIM, ACT_DIM> Q_act;
  env1.getQ(Q);
  env1.getQAct(Q_act);

  EXPECT_EQ(obs_dim1, OBS_DIM);
  EXPECT_EQ(act_dim1, ACT_DIM);
  EXPECT_EQ(expect_sim_dt, sim_dt);
  EXPECT_TRUE(Q.isApprox(expect_Q));
  EXPECT_TRUE(Q_act.isApprox(expect_Q_act));

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
  EXPECT_EQ(obs(0), 0.0);
  EXPECT_EQ(obs(1), 0.0);
  EXPECT_EQ(obs(2), 0.0);
}

TEST(QuadrotorEnv, StepEnv) {
  QuadrotorEnv env;

  Vector<ACT_DIM> act{1.0, 1.0, 1.0, 1.0};
  Vector<OBS_DIM> obs;
  Vector<OBS_DIM> next_obs;

  env.reset(obs, false);
  Scalar reward = env.step(act, next_obs);

  // check control command
  Command cmd;
  EXPECT_TRUE(env.getAct(&cmd));
  if (cmd.isRatesThrust()) {
    EXPECT_EQ(cmd.collective_thrust, -Gz);
    EXPECT_TRUE(cmd.omega.allFinite());
  } else if (cmd.isSingleRotorThrusts()) {
    // TODO: Not tested yet.
    EXPECT_TRUE(cmd.thrusts.allFinite());
  }

  Vector<ACT_DIM> executed_act;
  EXPECT_TRUE(env.getAct(executed_act));
  std::cout << "Executed Action: " << executed_act << std::endl;

  for (int i = 0; i < SIM_STEPS_N; i++) {
    reward = env.step(act, next_obs);
  }
  // in case this failed, decrease motor_tau in the Quadrotor class.
  // EXPECT_TRUE(((next_obs - obs).norm() < 1.0));

  //
  std::cout << (next_obs - obs).norm() << std::endl;
  std::cout << "Reward : " << reward << std::endl;
}
