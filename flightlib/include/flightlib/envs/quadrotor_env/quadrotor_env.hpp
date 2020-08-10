#pragma once

#include <stdlib.h>
#include <cmath>

// flightlib
#include "flightlib/common/command.hpp"
#include "flightlib/common/logger.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/envs/env_base.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env_param.hpp"
#include "flightlib/objects/quadrotor.hpp"

namespace flightlib {

class QuadrotorEnv final : public EnvBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  QuadrotorEnv();
  ~QuadrotorEnv();

  // - public OpenAI-gym-style functions
  bool reset(Ref<Vector<>> obs) override;
  Scalar step(Ref<Vector<>> act, Ref<Vector<>> obs) override;

  // - public set functions
  bool setFlightmare(bool on) override;
  bool updateParam(QuadrotorEnvParam &param);

  // - public get functions
  bool getObs(Ref<Vector<>> obs) override;

  // - auxiliar functions
  bool isTerminalState(Scalar &reward) override;

 private:
  // quadrotor
  Quadrotor quadrotor_;
  QuadState quad_state_;
  Command cmd_;
  Logger logger_{"QaudrotorEnv"};
  QuadrotorEnvParam param_;

  // auxiliary variables
  Vector<CtlObsAct::kObsSize> quad_obs_;
  Vector<CtlObsAct::kActSize> quad_act_;
};

}  // namespace flightlib