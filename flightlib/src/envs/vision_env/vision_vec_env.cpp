#include "flightlib/envs/vision_env/vision_vec_env.hpp"

namespace flightlib {

template<typename EnvBaseName>
VisionVecEnv<EnvBaseName>::VisionVecEnv() {
  std::string config_path = getenv("FLIGHTMARE_PATH") +
                            std::string("/flightpy/configs/vision/config.yaml");
  cfg_ = YAML::LoadFile(config_path);
  // yaml configurations
  configEnv(cfg_);
}

template<typename EnvBaseName>
VisionVecEnv<EnvBaseName>::VisionVecEnv(const std::string& cfg,
                                        const bool from_file) {
  // load environment configuration
  if (from_file) {
    // load directly from a yaml file
    cfg_ = YAML::LoadFile(cfg);
  } else {
    // load from a string or dictionary
    cfg_ = YAML::Load(cfg);
  }
  configEnv(cfg_);
}

template<typename EnvBaseName>
VisionVecEnv<EnvBaseName>::VisionVecEnv(const YAML::Node& cfg_node) {
  cfg_ = cfg_node;
  configEnv(cfg_);
  random_reset_ = true;
}

template<typename EnvBaseName>
bool VisionVecEnv<EnvBaseName>::reset(Ref<MatrixRowMajor<>> obs) {
  if (obs.rows() != this->num_envs_ || obs.cols() != this->obs_dim_) {
    this->logger_.error(
      "Input matrix dimensions do not match with that of the environment.");
    return false;
  }

  this->receive_id_ = 0;
  for (int i = 0; i < this->num_envs_; i++) {
    this->envs_[i]->reset(obs.row(i), random_reset_);
  }

  return true;
}

template<typename EnvBaseName>
bool VisionVecEnv<EnvBaseName>::reset(Ref<MatrixRowMajor<>> obs, bool random) {
  if (obs.rows() != this->num_envs_ || obs.cols() != this->obs_dim_) {
    this->logger_.error(
      "Input matrix dimensions do not match with that of the environment.");
    return false;
  }
  random_reset_ = random;


  this->receive_id_ = 0;
  for (int i = 0; i < this->num_envs_; i++) {
    this->envs_[i]->reset(obs.row(i), random_reset_);
  }

  return true;
}

template<typename EnvBaseName>
bool VisionVecEnv<EnvBaseName>::step(Ref<MatrixRowMajor<>> act,
                                     Ref<MatrixRowMajor<>> obs,
                                     Ref<MatrixRowMajor<>> reward,
                                     Ref<BoolVector<>> done,
                                     Ref<MatrixRowMajor<>> extra_info) {
  if (act.rows() != this->num_envs_ || act.cols() != this->act_dim_ ||
      obs.rows() != this->num_envs_ || obs.cols() != this->obs_dim_ ||
      reward.rows() != this->num_envs_ || reward.cols() != this->rew_dim_ ||
      done.rows() != this->num_envs_ || done.cols() != 1 ||
      extra_info.rows() != this->num_envs_ ||
      extra_info.cols() != (long int)this->extra_info_names_.size()) {
    this->logger_.error(
      "Input matrix dimensions do not match with that of the environment.");
    std::cout << "act dim  =       [" << act.rows() << "x" << act.cols()
              << "]\n"
              << "obs dim  =       [" << obs.rows() << "x" << obs.cols()
              << "]\n"
              << "rew dim  =       [" << reward.rows() << "x" << reward.cols()
              << "]\n"
              << "done dim  =       [" << done.rows() << "x" << done.cols()
              << "]\n"
              << "extra info dim  =       [" << extra_info.rows() << "x"
              << extra_info.cols() << "]\n"
              << std::endl;
    return false;
  }

#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < this->num_envs_; i++) {
    perAgentStep(i, act, obs, reward, done, extra_info);
  }

  return true;
}

template<typename EnvBaseName>
void VisionVecEnv<EnvBaseName>::perAgentStep(int agent_id,
                                             Ref<MatrixRowMajor<>> act,
                                             Ref<MatrixRowMajor<>> obs,
                                             Ref<MatrixRowMajor<>> reward,
                                             Ref<BoolVector<>> done,
                                             Ref<MatrixRowMajor<>> extra_info) {
  // get individual rewards
  this->envs_[agent_id]->step(act.row(agent_id), obs.row(agent_id),
                              reward.row(agent_id));

  Scalar terminal_reward = 0;
  done[agent_id] = this->envs_[agent_id]->isTerminalState(terminal_reward);

  this->envs_[agent_id]->updateExtraInfo();
  for (int j = 0; j < extra_info.cols(); j++)
    extra_info(agent_id, j) =
      this->envs_[agent_id]->extra_info_[this->extra_info_names_[j]];

  if (done[agent_id]) {
    this->envs_[agent_id]->reset(obs.row(agent_id), random_reset_);
    reward(agent_id, reward.cols() - 1) = terminal_reward;
  }
}

template<typename EnvBaseName>
bool VisionVecEnv<EnvBaseName>::getQuadAct(Ref<MatrixRowMajor<>> quadact) {
  bool valid = true;
  for (int i = 0; i < this->num_envs_; i++) {
    valid &= this->envs_[i]->getQuadAct(quadact.row(i));
  }
  return valid;
}

template<typename EnvBaseName>
bool VisionVecEnv<EnvBaseName>::getQuadState(Ref<MatrixRowMajor<>> quadstate) {
  bool valid = true;
  for (int i = 0; i < this->num_envs_; i++) {
    valid &= this->envs_[i]->getQuadState(quadstate.row(i));
  }
  return valid;
}


template<typename EnvBaseName>
VisionVecEnv<EnvBaseName>::~VisionVecEnv() {}

// IMPORTANT. Otherwise:
// Linker errors because of the separation between the
// declaration and definition of the template class.
// Segmentation fault (core dumped)
template class VisionVecEnv<VisionEnv>;

}  // namespace flightlib