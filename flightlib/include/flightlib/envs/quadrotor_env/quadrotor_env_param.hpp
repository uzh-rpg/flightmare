#pragma once

#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/parameter_base.hpp"
#include "flightlib/common/types.hpp"
namespace flightlib {

enum CtlObsAct {
  // observations
  kObs = 0,
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

struct QuadrotorEnvParam : ParameterBase {
  QuadrotorEnvParam();
  ~QuadrotorEnvParam();

  bool valid() override;
  bool loadParam() override;

  // reward function design (for model-free reinforcement learning)
  Matrix<CtlObsAct::kObsSize, CtlObsAct::kObsSize> Q;
  Matrix<CtlObsAct::kActSize, CtlObsAct::kActSize> Q_act;
  Vector<CtlObsAct::kObsSize> goal_state;

  // environment configuration
  SceneID scene_id;
  Scalar sim_dt;
  bool render;

  // quadrotor configuration
  Scalar mass;
  Scalar arm_length;
  Scalar kappa;  // rotor drage coeff
  Vector<3> inertia;
  bool camera;

  // action and observation normalization (for learning)
  Vector<CtlObsAct::kActSize> act_mean{-Gz, 0.0, 0.0, 0.0};
  Vector<CtlObsAct::kActSize> act_std{10.0, 6.0, 6.0, 6.0};
  Vector<CtlObsAct::kObsSize> obs_mean = Vector<CtlObsAct::kObsSize>::Zero();
  Vector<CtlObsAct::kObsSize> obs_std = Vector<CtlObsAct::kObsSize>::Ones();
};

}  // namespace flightlib
