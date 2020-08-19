#include "flightlib/envs/vec_env.hpp"

namespace flightlib {

template<typename EnvBase>
VecEnv<EnvBase>::VecEnv()
  : VecEnv(getenv("FLIGHTMARE_PATH") +
           std::string("/flightlib/configs/vec_env.yaml")) {}

template<typename EnvBase>
VecEnv<EnvBase>::VecEnv(const std::string& cfgs, const bool yaml_node) {
  // load environment configuration
  if (!yaml_node) {
    // load directly from a yaml file
    cfg_ = YAML::LoadFile(cfgs);
  } else {
    // load from a string or dictionary
    cfg_ = YAML::Load(cfgs);
  }

  //
  unity_render_ = cfg_["env"]["render"].as<bool>();
  seed_ = cfg_["env"]["seed"].as<int>();
  num_envs_ = cfg_["env"]["num_envs"].as<int>();
  scene_id_ = cfg_["env"]["scene_id"].as<SceneID>();

  // set threads
  omp_set_num_threads(cfg_["env"]["num_threads"].as<int>());

  // create & setup environments
  const bool render = false;
  for (int i = 0; i < num_envs_; i++) {
    envs_.push_back(std::make_unique<EnvBase>());
  }

  // set Unity
  setUnity(unity_render_);

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
  receive_id_ = 0;
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

  if (unity_render_) {
    unity_bridge_->getRender(0);
    unity_bridge_->handleOutput(unity_output_);
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
void VecEnv<EnvBase>::setUnity(bool render) {
  unity_render_ = render;
  if (unity_render_ && !unity_bridge_created_) {
    unity_bridge_ = UnityBridge::getInstance();
    unity_bridge_->initializeConnections();

    for (int i = 0; i < num_envs_; i++) {
      envs_[i]->addObjectsToUnity(unity_bridge_);
    }

    connectUnity();
    //
    unity_bridge_created_ = true;
    logger_.info("Flightmare Unity is ON!");
  }
}

template<typename EnvBase>
void VecEnv<EnvBase>::isTerminalState(Ref<BoolVector<>> terminal_state) {}

template<typename EnvBase>
void VecEnv<EnvBase>::connectUnity(void) {
  Scalar time_out_count = 0;
  Scalar sleep_useconds = 0.2 * 1e5;
  while (!unity_ready_) {
    if (unity_bridge_ != nullptr) {
      // connect unity
      unity_bridge_->setScene(scene_id_);
      unity_ready_ = unity_bridge_->connectUnity();
    }
    if (unity_ready_ || time_out_count / 1e6 > unity_connection_time_out_) {
      break;
    }
    // sleep
    usleep(0.2 * 1e5);
    // incread time out counter
    time_out_count += sleep_useconds;
  }
}

template<typename EnvBase>
void VecEnv<EnvBase>::disconnectUnity(void) {
  if (unity_bridge_ != nullptr) {
    unity_bridge_->disconnectUnity();
    unity_ready_ = false;
  } else {
    logger_.warn("Flightmare Unity Bridge is not initialized.");
  }
}

template<typename EnvBase>
void VecEnv<EnvBase>::curriculumUpdate(void) {
  for (int i = 0; i < num_envs_; i++) envs_[i]->curriculumUpdate();
}

// IMPORTANT. Otherwise:
// Segmentation fault (core dumped)
template class VecEnv<QuadrotorEnv>;

}  // namespace flightlib
