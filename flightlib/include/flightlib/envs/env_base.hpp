//
// This is inspired by RaiGym, thanks.
//
#pragma once

// standard library
#include <unistd.h>
#include <memory>
#include <random>
#include <unordered_map>
#include <vector>

// yaml
#include <yaml-cpp/yaml.h>

// alpha gym types
#include "flightlib/common/types.hpp"

namespace flightlib {

class EnvBase {
 public:
  EnvBase();
  virtual ~EnvBase() = 0;

  // (pure virtual) public methods (has to be implemented by child classes)
  virtual bool reset(Ref<Vector<>> obs, const bool random = true) = 0;
  virtual Scalar step(const Ref<Vector<>> act, Ref<Vector<>> obs) = 0;
  virtual bool getObs(Ref<Vector<>> obs) = 0;

  // (virtual) public methods (implementations are optional.)
  virtual void curriculumUpdate();
  virtual void close();
  virtual void render();
  virtual void updateExtraInfo();
  virtual bool isTerminalState(Scalar &reward);

  // auxilirary functions
  inline void setSeed(const int seed) { std::srand(seed); };
  inline int getObsDim() { return obs_dim_; };
  inline int getActDim() { return act_dim_; };
  inline Scalar getSimTimeStep() { return sim_dt_; };
  inline int getExtraInfoDim() { return extra_info_.size(); };
  inline Scalar getMaxT() { return max_t_; };

  // public variables
  std::unordered_map<std::string, float> extra_info_;

 protected:
  // observation and action dimenstions (for Reinforcement learning)
  int obs_dim_;
  int act_dim_;

  // control time step
  Scalar sim_dt_{0.02};
  Scalar max_t_{5.0};

  // random variable generator
  std::normal_distribution<Scalar> norm_dist_{0.0, 1.0};
  std::uniform_real_distribution<Scalar> uniform_dist_{-1.0, 1.0};
  std::random_device rd_;
  std::mt19937 random_gen_{rd_()};
};

}  // namespace flightlib