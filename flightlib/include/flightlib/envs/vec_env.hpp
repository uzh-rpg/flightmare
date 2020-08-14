#pragma once

// std
#include <memory>

// openmp
#include <omp.h>

// flightlib
#include "flightlib/common/logger.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/envs/env_base.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"

namespace flightlib {

template<typename EnvBase>
class VecEnv {
 public:
  VecEnv();
  VecEnv(const std::string& cfg_path);
  ~VecEnv();

  // - public OpenAI-gym style functions for vectorized environment
  void reset(Ref<MatrixRowMajor<>> obs);
  void step(Ref<MatrixRowMajor<>> act, Ref<MatrixRowMajor<>> obs,
            Ref<Vector<>> reward, Ref<BoolVector<>> done,
            Ref<MatrixRowMajor<>> extra_info);
  void close();

  // public set functions
  void setSeed(const int seed);

  // public get functions
  void getObs(Ref<MatrixRowMajor<>> obs);
  inline int getObsDim(void) { return obs_dim_; };
  inline int getActDim(void) { return act_dim_; };
  inline int getExtraInfoDim(void) { return extra_info_names_.size(); };
  inline int getNumOfEnvs(void) { return envs_.size(); };
  inline std::vector<std::string>& getExtraInfoNames() {
    return extra_info_names_;
  };

  // - auxiliary functions
  void isTerminalState(Ref<BoolVector<>> terminal_state);
  void testStep(Ref<MatrixRowMajor<>> act, Ref<MatrixRowMajor<>> obs,
                Ref<Vector<>> reward, Ref<BoolVector<>> done,
                Ref<MatrixRowMajor<>> extra_info);
  void curriculumUpdate();
  // flightmare (visualization)
  void connectFlightmare();
  void disconnectFlightmare();

 private:
  Logger logger_{"VecEnv"};

  // step every environment
  void perAgentStep(int agent_id, Ref<MatrixRowMajor<>> act,
                    Ref<MatrixRowMajor<>> obs, Ref<Vector<>> reward,
                    Ref<BoolVector<>> done, Ref<MatrixRowMajor<>> extra_info);
  //
  std::vector<std::unique_ptr<EnvBase>> envs_;
  std::vector<std::string> extra_info_names_;


  // auxiliar variables
  int num_envs_;
  int obs_dim_;
  int act_dim_;

  bool unity_render_;
  Matrix<> obs_dummy_;

  // yaml configurations
  YAML::Node cfg_;
};

}  // namespace flightlib
