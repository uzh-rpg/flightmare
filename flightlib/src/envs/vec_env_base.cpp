#include "flightlib/envs/vec_env_base.hpp"

namespace flightlib {

template<typename EnvBaseName>
VecEnvBase<EnvBaseName>::VecEnvBase() {
  omp_set_num_threads(10);
}

template<typename EnvBaseName>
void VecEnvBase<EnvBaseName>::configEnv(const YAML::Node& cfg_node) {
  // initialization
  if (!cfg_node["unity"]["render"] || !cfg_node["simulation"]["seed"] ||
      !cfg_node["unity"]["scene_id"] || !cfg_node["simulation"]["num_envs"] ||
      !cfg_node["simulation"]["num_threads"]) {
    //
    logger_.warn("Cannot load main configurations. Using default parameters.");
    unity_render_ = false;
    seed_ = 0;
    num_envs_ = 1;
    num_threads_ = 1;
    scene_id_ = 0;
  } else {
    //
    logger_.info("Load Unity configuration.");
    unity_render_ = cfg_node["unity"]["render"].as<bool>();
    scene_id_ = cfg_node["unity"]["scene_id"].as<SceneID>();

    //
    logger_.info("Load Simulation configuration.");
    seed_ = cfg_node["simulation"]["seed"].as<int>();
    num_envs_ = cfg_node["simulation"]["num_envs"].as<int>();
    num_threads_ = cfg_node["simulation"]["num_threads"].as<int>();
  }

  // set threads
  omp_set_num_threads(num_threads_);

  // create & setup environments
  for (int env_id = 0; env_id < num_envs_; env_id++) {
    envs_.push_back(std::make_unique<EnvBaseName>(cfg_node, env_id));
  }

  // set Unity
  if (unity_render_) {
    setUnity(unity_render_);
  }

  obs_dim_ = envs_[0]->getObsDim();
  act_dim_ = envs_[0]->getActDim();
  rew_dim_ = envs_[0]->getRewDim();
  img_width_ = envs_[0]->getImgWidth();
  img_height_ = envs_[0]->getImgHeight();

  // generate reward names
  // compute it once to get reward names. actual value is not used
  envs_[0]->updateExtraInfo();
  for (auto& re : envs_[0]->extra_info_) {
    extra_info_names_.push_back(re.first);
  }
  logger_.info("%d vectorized enviromnets created. ", num_envs_);
  std::cout << "Vectorized Environment:\n"
            << "obs dim    =            [" << obs_dim_ << "]\n"
            << "act dim    =            [" << act_dim_ << "]\n"
            << "rew dim    =            [" << rew_dim_ << "]\n"
            << "einfo dim  =            [" << envs_[0]->extra_info_.size()
            << "]\n"
            << "img width  =            [" << img_width_ << "]\n"
            << "img height =            [" << img_height_ << "]\n"
            << "num_envs   =            [" << num_envs_ << "]\n"
            << "num_thread =            [" << num_threads_ << "]\n"
            << "seed       =            [" << seed_ << "]\n"
            << "scene_id   =            [" << scene_id_ << "]" << std::endl;
}

template<typename EnvBaseName>
VecEnvBase<EnvBaseName>::~VecEnvBase() {}


template<typename EnvBaseName>
bool VecEnvBase<EnvBaseName>::reset(Ref<MatrixRowMajor<>> obs) {
  if (obs.rows() != num_envs_ || obs.cols() != obs_dim_) {
    logger_.error(
      "Input matrix dimensions do not match with that of the environment.");
    return false;
  }

  receive_id_ = 0;
  for (int i = 0; i < num_envs_; i++) {
    envs_[i]->reset(obs.row(i));
  }

  return true;
}

template<typename EnvBaseName>
bool VecEnvBase<EnvBaseName>::step(Ref<MatrixRowMajor<>> act,
                                   Ref<MatrixRowMajor<>> obs,
                                   Ref<MatrixRowMajor<>> reward,
                                   Ref<BoolVector<>> done,
                                   Ref<MatrixRowMajor<>> extra_info) {
  if (act.rows() != num_envs_ || act.cols() != act_dim_ ||
      obs.rows() != num_envs_ || obs.cols() != obs_dim_ ||
      reward.rows() != num_envs_ || reward.cols() != rew_dim_ ||
      done.rows() != num_envs_ || done.cols() != 1 ||
      extra_info.rows() != num_envs_ ||
      extra_info.cols() != (long int)extra_info_names_.size()) {
    logger_.error(
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
  for (int i = 0; i < num_envs_; i++) {
    perAgentStep(i, act, obs, reward, done, extra_info);
  }

  return true;
}

template<typename EnvBaseName>
void VecEnvBase<EnvBaseName>::close() {
  for (int i = 0; i < num_envs_; i++) {
    envs_[i]->close();
  }
}

template<typename EnvBaseName>
void VecEnvBase<EnvBaseName>::setSeed(const int seed) {
  int seed_inc = seed;
  for (int i = 0; i < num_envs_; i++) envs_[i]->setSeed(seed_inc++);
}

template<typename EnvBaseName>
bool VecEnvBase<EnvBaseName>::getObs(Ref<MatrixRowMajor<>> obs) {
  bool valid_obs = true;
  for (int i = 0; i < num_envs_; i++) valid_obs &= envs_[i]->getObs(obs.row(i));
  return valid_obs;
}

template<typename EnvBaseName>
bool VecEnvBase<EnvBaseName>::getImage(Ref<ImgMatrixRowMajor<>> img,
                                       const bool rgb_image) {
  bool valid_img = true;
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < num_envs_; i++) {
    valid_img &= envs_[i]->getImage(img.row(i), rgb_image);
  }
  return valid_img;
}

template<typename EnvBaseName>
bool VecEnvBase<EnvBaseName>::getDepthImage(
  Ref<DepthImgMatrixRowMajor<>> depth_img) {
  bool valid_img = true;
  for (int i = 0; i < num_envs_; i++) {
    valid_img &= envs_[i]->getDepthImage(depth_img.row(i));
  }
  return valid_img;
}

template<typename EnvBaseName>
size_t VecEnvBase<EnvBaseName>::getEpisodeLength(void) {
  if (envs_.size() <= 0) {
    return 0;
  } else {
    return (size_t)envs_[0]->getMaxT() / envs_[0]->getSimTimeStep();
  }
}


template<typename EnvBaseName>
void VecEnvBase<EnvBaseName>::perAgentStep(int agent_id,
                                           Ref<MatrixRowMajor<>> act,
                                           Ref<MatrixRowMajor<>> obs,
                                           Ref<MatrixRowMajor<>> reward,
                                           Ref<BoolVector<>> done,
                                           Ref<MatrixRowMajor<>> extra_info) {
  // get individual rewards
  envs_[agent_id]->step(act.row(agent_id), obs.row(agent_id),
                        reward.row(agent_id));

  Scalar terminal_reward = 0;
  done[agent_id] = envs_[agent_id]->isTerminalState(terminal_reward);

  envs_[agent_id]->updateExtraInfo();
  for (int j = 0; j < extra_info.cols(); j++)
    extra_info(agent_id, j) =
      envs_[agent_id]->extra_info_[extra_info_names_[j]];

  if (done[agent_id]) {
    envs_[agent_id]->reset(obs.row(agent_id));
    reward(agent_id, reward.cols() - 1) = terminal_reward;
  }
}


template<typename EnvBaseName>
bool VecEnvBase<EnvBaseName>::setUnity(bool render) {
  unity_render_ = render;
  if (!unity_render_ || unity_bridge_ptr_ != nullptr) {
    logger_.warn(
      "Unity render is False or Flightmare Bridge has been already created. "
      "Cannot set Unity.");
    return false;
  }
  // create unity bridge
  unity_bridge_ptr_ = UnityBridge::getInstance();
  // add objects to Unity
  for (int i = 0; i < num_envs_; i++) {
    envs_[i]->addQuadrotorToUnity(unity_bridge_ptr_);
  }
  logger_.info("Flightmare Bridge created.");
  return true;
}


template<typename EnvBaseName>
bool VecEnvBase<EnvBaseName>::connectUnity(void) {
  if (unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}


template<typename EnvBaseName>
FrameID VecEnvBase<EnvBaseName>::updateUnity(const FrameID frame_id) {
  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(frame_id);
    return unity_bridge_ptr_->handleOutput(frame_id);
  } else {
    return 0;
  }
}


template<typename EnvBaseName>
void VecEnvBase<EnvBaseName>::isTerminalState(
  Ref<BoolVector<>> terminal_state) {}


template<typename EnvBaseName>
void VecEnvBase<EnvBaseName>::disconnectUnity(void) {
  if (unity_bridge_ptr_ != nullptr) {
    unity_bridge_ptr_->disconnectUnity();
    unity_ready_ = false;
  } else {
    logger_.warn("Flightmare Unity Bridge is not initialized.");
  }
}

template<typename EnvBaseName>
void VecEnvBase<EnvBaseName>::curriculumUpdate(void) {
  for (int i = 0; i < num_envs_; i++) envs_[i]->curriculumUpdate();
}

// IMPORTANT. Otherwise:
// Linker errors because of the separation between the
// declaration and definition of the template class.
// Segmentation fault (core dumped)!
template class VecEnvBase<QuadrotorEnv>;
//
template class VecEnvBase<VisionEnv>;

}  // namespace flightlib
