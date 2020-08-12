#include "flightlib/envs/vec_env.hpp"

namespace flightlib {

template<typename EnvBase>
VecEnv<EnvBase>::VecEnv()
  : VecEnv(getenv("FLIGHTMARE_PATH") +
           std::string("/flightlib/configs/quadrotor_env.yaml")) {}

template<typename EnvBase>
VecEnv<EnvBase>::VecEnv(const std::string& cfg_path) {
  // load environment configuration
  cfg_ = YAML::LoadFile(cfg_path);

  // logger
  logger_ = std::make_unique<Logger>("VecEnv");

  if (cfg_["env"]["render"]) {
    unity_render_ = cfg_["env"]["render"].as<bool>();
  }

  omp_set_num_threads(cfg_["env"]["num_threads"].as<int>());
  num_envs_ = cfg_["env"]["num_envs"].as<int>();

  // create & setup environments
  const bool render = false;
  for (int i = 0; i < num_envs_; i++) {
    envs_.push_back(std::make_unique<EnvBase>());
    // disable rendering for all environment
    envs_.back()->setFlightmare(render);
  }

  if (unity_render_) {
    // enable rendering for only one environment
    envs_[0]->setFlightmare(unity_render_);
    logger_->info("Flightmare Unity Render is enabled!");
  }

  obs_dim_ = envs_[0]->getObsDim();
  act_dim_ = envs_[0]->getActDim();

  // generate reward names
  // compute it once to get reward names. actual value is not used
  envs_[0]->updateExtraInfo();
  for (auto& re : envs_[0]->extra_info_) {
    extra_info_names_.push_back(re.first);
  }
}

template<typename EnvBase>
VecEnv<EnvBase>::~VecEnv() {}

template<typename EnvBase>
void VecEnv<EnvBase>::reset(Ref<MatrixRowMajor<>> obs) {
  for (int i = 0; i < num_envs_; i++) {
    envs_[i]->reset(obs.row(i));
  }
}

template<typename EnvBase>
void VecEnv<EnvBase>::step(Ref<MatrixRowMajor<>> act, Ref<MatrixRowMajor<>> obs,
                           Ref<Vector<>> reward, Ref<BoolVector<>> done,
                           Ref<MatrixRowMajor<>> extra_info) {
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < num_envs_; i++) {
    perAgentStep(i, act, obs, reward, done, extra_info);
  }
}

template<typename EnvBase>
void VecEnv<EnvBase>::testStep(Ref<MatrixRowMajor<>> act,
                               Ref<MatrixRowMajor<>> obs, Ref<Vector<>> reward,
                               Ref<BoolVector<>> done,
                               Ref<MatrixRowMajor<>> extra_info) {
  perAgentStep(0, act, obs, reward, done, extra_info);
  envs_[0]->getObs(obs.row(0));
}

template<typename EnvBase>
void VecEnv<EnvBase>::close() {
  for (int i = 0; i < num_envs_; i++) {
    envs_[i]->close();
  }
}

template<typename EnvBase>
void VecEnv<EnvBase>::setSeed(const int seed) {
  int seed_inc = seed;
  for (int i = 0; i < num_envs_; i++) envs_[i]->setSeed(seed_inc++);
}

template<typename EnvBase>
void VecEnv<EnvBase>::getObs(Ref<MatrixRowMajor<>> obs) {
  for (int i = 0; i < num_envs_; i++) envs_[i]->getObs(obs.row(i));
}

template<typename EnvBase>
void VecEnv<EnvBase>::perAgentStep(int agent_id, Ref<MatrixRowMajor<>> act,
                                   Ref<MatrixRowMajor<>> obs,
                                   Ref<Vector<>> reward, Ref<BoolVector<>> done,
                                   Ref<MatrixRowMajor<>> extra_info) {
  reward(agent_id) =
    envs_[agent_id]->step(act.row(agent_id), obs.row(agent_id));

  Scalar terminal_reward = 0;
  done(agent_id) = envs_[agent_id]->isTerminalState(terminal_reward);

  envs_[agent_id]->updateExtraInfo();

  for (int j = 0; j < extra_info.size(); j++)
    extra_info(agent_id, j) =
      envs_[agent_id]->extra_info_[extra_info_names_[j]];

  if (done[agent_id]) {
    envs_[agent_id]->reset(obs.row(agent_id));
    reward(agent_id) += terminal_reward;
  }
}

template<typename EnvBase>
void VecEnv<EnvBase>::isTerminalState(Ref<BoolVector<>> terminal_state) {}

template<typename EnvBase>
void VecEnv<EnvBase>::connectFlightmare(void) {}

template<typename EnvBase>
void VecEnv<EnvBase>::disconnectFlightmare(void) {}

template<typename EnvBase>
void VecEnv<EnvBase>::curriculumUpdate(void) {}

// IMPORTANT. Otherwise:
// Segmentation fault (core dumped)
template class VecEnv<QuadrotorEnv>;

}  // namespace flightlib
