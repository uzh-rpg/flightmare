#pragma once

// std lib
#include <stdlib.h>
#include <cmath>
#include <iostream>

// yaml cpp
#include <yaml-cpp/yaml.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/command.hpp"
#include "flightlib/common/logger.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/envs/env_base.hpp"
#include "flightlib/objects/quadrotor.hpp"

namespace flightlib {

enum CtlObsAct {
  // observations
  kObs = 0,
  //
  kObsPos = 0,
  kObsPosSize = 3,
  kObsOri = 3,
  kObsOriSize = 3,
  kObsLinVel = 6,
  kObsLinVelSize = 3,
  kObsSize = 9,
  // control actions
  kAct = 0,
  kCollectiveThrust = 0,
  kOmegaX = 1,
  kOmegaY = 2,
  kOmegaZ = 3,
  kActSize = 4,
};

class QuadrotorEnv final : public EnvBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  QuadrotorEnv();
  QuadrotorEnv(const std::string &cfg_path);
  ~QuadrotorEnv();

  // - public OpenAI-gym-style functions
  bool reset(Ref<Vector<>> obs, const bool random = true) override;
  Scalar step(const Ref<Vector<>> act, Ref<Vector<>> obs) override;

  // - public set functions
  bool loadParam(const YAML::Node &cfg);

  // - public get functions
  bool getObs(Ref<Vector<>> obs) override;
  bool getAct(Ref<Vector<>> act) const;
  bool getAct(Command *const cmd) const;
  void getQ(Ref<Matrix<CtlObsAct::kObsSize, CtlObsAct::kObsSize>> Q) const;
  void getQAct(
    Ref<Matrix<CtlObsAct::kActSize, CtlObsAct::kActSize>> Q_act) const;

  // - auxiliar functions
  bool isTerminalState(Scalar &reward) override;
  void addObjectsToUnity(std::shared_ptr<UnityBridge> bridge);

  friend std::ostream &operator<<(std::ostream &os,
                                  const QuadrotorEnv &quad_env);

 private:
  // quadrotor
  Quadrotor quadrotor_;
  QuadState quad_state_;
  Command cmd_;
  Logger logger_{"QaudrotorEnv"};

  // observations and actions (for RL)
  Vector<CtlObsAct::kObsSize> quad_obs_;
  Vector<CtlObsAct::kActSize> quad_act_;

  // reward function design (for model-free reinforcement learning)
  Matrix<CtlObsAct::kObsSize, CtlObsAct::kObsSize> Q_;
  Matrix<CtlObsAct::kActSize, CtlObsAct::kActSize> Q_act_;
  Vector<CtlObsAct::kObsSize> goal_state_;

  // action and observation normalization (for learning)
  // Vector<CtlObsAct::kActSize> act_mean_{-Gz, 0.0, 0.0, 0.0};
  // Vector<CtlObsAct::kActSize> act_std_{10.0, 6.0, 6.0, 6.0};
  Vector<CtlObsAct::kActSize> act_mean_{2.5, 2.5, 2.5, 2.5};
  Vector<CtlObsAct::kActSize> act_std_{6.0, 6.0, 6.0, 6.0};
  Vector<CtlObsAct::kObsSize> obs_mean_ = Vector<CtlObsAct::kObsSize>::Zero();
  Vector<CtlObsAct::kObsSize> obs_std_ = Vector<CtlObsAct::kObsSize>::Ones();

  YAML::Node cfg_;
  Matrix<3, 2> world_box_;
};

}  // namespace flightlib