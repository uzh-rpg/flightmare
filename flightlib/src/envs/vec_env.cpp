#include "flightlib/envs/vec_env.hpp"

namespace flightlib {

template<typename EnvBase>
VecEnv<EnvBase>::VecEnv()
  : VecEnv(getenv("FLIGHTMARE_PATH") +
           std::string("/flightlib/configs/vec_env.yaml")) {}

template<typename EnvBase>
VecEnv<EnvBase>::VecEnv(const YAML::Node& cfg_node) : cfg_(cfg_node) {
  // initialization
  init();
}

template<typename EnvBase>
VecEnv<EnvBase>::VecEnv(const std::string& cfgs, const bool from_file) {
  // load environment configuration
  if (from_file) {
    // load directly from a yaml file
    cfg_ = YAML::LoadFile(cfgs);
  } else {
    // load from a string or dictionary
    cfg_ = YAML::Load(cfgs);
  }
  // initialization
  init();
}

template<typename EnvBase>
void VecEnv<EnvBase>::init(void) {
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

  // Give drones their start position  
  int dim_columns = 10;
  float num_cols = std::floor((float)(num_envs_)/(float)(dim_columns));
  if (num_cols==0) {
    num_cols=1;
    dim_columns = num_envs_;
  } 
  float spacing_between_drones_rows = 40;
  float spacing_between_drones_columns = 40;
  std::vector<float> start_rot = (cfg_["env"]["start_rot"]).as<std::vector<float>>();
  Vector<3> start_ori = Vector<3>(start_rot.data());

  
  Vector<3> drone_start_position;

  // Want a chessboard like pattern.
  for (int vehicle_index; vehicle_index<num_envs_; vehicle_index++) {
    // Last terms are to center the positions around zero.
    drone_start_position(0) = spacing_between_drones_columns*(ceil(vehicle_index/dim_columns)); 
    drone_start_position(1) = spacing_between_drones_rows*(vehicle_index%dim_columns);
    drone_start_position(2) = 4.0;
    envs_[vehicle_index]->setResetPose(drone_start_position, start_ori);
  }

  is_done_unity_.reserve(num_envs_); 
  is_done_unity_ = {false};

  // set Unity
  setUnity(unity_render_);

  obs_dim_ = envs_[0]->getObsDim();
  act_dim_ = envs_[0]->getActDim();
  frame_dim_ = envs_[0]->getFrameDim();

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
bool VecEnv<EnvBase>::reset(Ref<MatrixRowMajor<>> obs, Ref<DepthImageMatrixRowMajor<>> img) {
  if (obs.rows() != num_envs_ || obs.cols() != obs_dim_ ||
      img.rows() != num_envs_ || img.cols() != frame_dim_.first*frame_dim_.second ) {
    logger_.error(
      "Input matrix dimensions do not match with that of the environment.");
    return false;
  }

  receive_id_ = 0;
  for (int i = 0; i < num_envs_; i++) {
    envs_[i]->reset(obs.row(i), img.row(i), i);
  }
  return true;
}

template<typename EnvBase>
bool VecEnv<EnvBase>::step(Ref<MatrixRowMajor<>> act, Ref<MatrixRowMajor<>> obs, Ref<DepthImageMatrixRowMajor<>> img,
                           Ref<Vector<>> reward, Ref<BoolVector<>> done,
                           Ref<MatrixRowMajor<>> extra_info) {
  if (act.rows() != num_envs_ || act.cols() != act_dim_ ||
      obs.rows() != num_envs_ || obs.cols() != obs_dim_ ||
      img.rows() != num_envs_ || img.cols() != frame_dim_.first*frame_dim_.second ||
      reward.rows() != num_envs_ || reward.cols() != 1 ||
      done.rows() != num_envs_ || done.cols() != 1 ||
      extra_info.rows() != num_envs_ ||
      extra_info.cols() != extra_info_names_.size()) {
            
    std::cout<<"Frame dimension is: "<< img.rows() << " x "<< img.cols() <<" != " << num_envs_ << " x " << frame_dim_.first*frame_dim_.second*3 << std::endl;
    std::cout<<"Action dimension is: "<< act.rows() << " x "<< act.cols() <<" != " << num_envs_ << " x " << act_dim_ << std::endl;
    std::cout<<"Observation dimension is: "<< obs.rows() << " x "<< obs.cols() <<" != " << num_envs_ << " x " << obs_dim_ << std::endl;
    logger_.error(
      "Input matrix dimensions do not match with that of the environment.");
    return false;
  }

#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < num_envs_; i++) {
    perAgentStep(i, act, obs, reward, done);
  }

  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();
  }
    
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < num_envs_; i++) {
    perAgentStepUnity(i, obs, img, reward, done, extra_info);
  }
  return true;
}

template<typename EnvBase>
void VecEnv<EnvBase>::testStep(Ref<MatrixRowMajor<>> act,
                               Ref<MatrixRowMajor<>> obs ,Ref<DepthImageMatrixRowMajor<>> img, Ref<Vector<>> reward,
                               Ref<BoolVector<>> done,
                               Ref<MatrixRowMajor<>> extra_info) {
  perAgentStep(0, act, obs, reward, done);
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
void VecEnv<EnvBase>::getObs(Ref<MatrixRowMajor<>> obs, Ref<DepthImageMatrixRowMajor<>> img) {
  for (int i = 0; i < num_envs_; i++) envs_[i]->getObs(obs.row(i));
}


template<typename EnvBase>
size_t VecEnv<EnvBase>::getEpisodeLength(void) {
  if (envs_.size() <= 0) {
    return 0;
  } else {
    return (size_t)envs_[0]->getMaxT() / envs_[0]->getSimTimeStep();
  }
}

template<typename EnvBase>
void VecEnv<EnvBase>::perAgentStep(int agent_id, Ref<MatrixRowMajor<>> act,
                                   Ref<MatrixRowMajor<>> obs, Ref<Vector<>> reward, 
                                   Ref<BoolVector<>> done) {
  reward(agent_id) =
    envs_[agent_id]->step(act.row(agent_id), obs.row(agent_id));

  Scalar terminal_reward = 0;
  done(agent_id) = envs_[agent_id]->isTerminalState(terminal_reward);

  if (done[agent_id]) {
    envs_[agent_id]->resetObs(obs.row(agent_id));
    reward(agent_id) += terminal_reward;
  }
}

template<typename EnvBase>
void VecEnv<EnvBase>::perAgentStepUnity(int agent_id, Ref<MatrixRowMajor<>> obs,
                                   Ref<DepthImageMatrixRowMajor<>> img, Ref<Vector<>> reward,
                                   Ref<BoolVector<>> done, Ref<MatrixRowMajor<>> extra_info) {
  reward(agent_id) +=
    envs_[agent_id]->stepUnity(img.row(agent_id));

  Scalar terminal_reward = 0;
  is_done_unity_[agent_id] = false;
  is_done_unity_[agent_id] = envs_[agent_id]->isTerminalStateUnity(terminal_reward);
  
  envs_[agent_id]->updateExtraInfo();
  for (int j = 0; j < extra_info_names_.size(); j++)
    extra_info(agent_id, j) =
      envs_[agent_id]->extra_info_[extra_info_names_[j]];

  if (done[agent_id]) { // Done state due to dynamics, obs already reset.
    envs_[agent_id]->resetImages(img.row(agent_id));
  } else if (is_done_unity_[agent_id]) // Done state due to unity: collisions or conditions on distances.
  { 
    done[agent_id] = is_done_unity_[agent_id];
    // Full reset.
    envs_[agent_id]->reset(obs.row(agent_id), img.row(agent_id));
    reward(agent_id) += terminal_reward;
  }
}

template<typename EnvBase>
bool VecEnv<EnvBase>::setUnity(bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    // add objects to Unity
    for (int i = 0; i < num_envs_; i++) {
      envs_[i]->addObjectsToUnity(unity_bridge_ptr_);
    }
    logger_.info("Flightmare Bridge is created.");
  }
  return true;
}

template<typename EnvBase>
bool VecEnv<EnvBase>::connectUnity(void) {
  if (unity_bridge_ptr_ == nullptr) return false;
  unity_bridge_ptr_->object_density_fractions_.resize(num_envs_, 1);
  unity_bridge_ptr_->object_density_fractions_ = Eigen::MatrixXd::Zero(num_envs_, 1).cast<float>();
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

template<typename EnvBase>
void VecEnv<EnvBase>::isTerminalState(Ref<BoolVector<>> terminal_state) {}

template<typename EnvBase>
void VecEnv<EnvBase>::disconnectUnity(void) {
  if (unity_bridge_ptr_ != nullptr) {
    unity_bridge_ptr_->disconnectUnity();
    unity_ready_ = false;
  } else {
    logger_.warn("Flightmare Unity Bridge is not initialized.");
  }
}

template<typename EnvBase>
void VecEnv<EnvBase>::curriculumUpdate(void) {
  for (int i = 0; i < num_envs_; i++) envs_[i]->curriculumUpdate();
}

template<typename EnvBase>
void VecEnv<EnvBase>::setObjectsDensities(Vector<> object_density_fractions) {
    // Check on density.
    unity_bridge_ptr_->object_density_fractions_ = object_density_fractions.cwiseMin(1.0).cwiseMax(0.0);
    unity_bridge_ptr_->changed_density_ = true;
}

// template<typename EnvBase>
// std::ostream& operator<<(std::ostream& os, const VecEnv<EnvBase>& env) {
//   os.precision(3);
//   os << "Vectorized Environment:\n"
//      << "obs dim =            [" << env.obs_dim_ << "]\n"
//      << "act dim =            [" << env.act_dim_ << "]\n"
//      << "num_envs =           [" << env.num_envs_ << "]\n"
//      << "seed =               [" << env.seed_ << "]\n"
//      << "scene_id =           [" << env.scene_id_ << std::endl;
//   os.precision();
//   return os;
// }

// IMPORTANT. Otherwise:
// Segmentation fault (core dumped)
template class VecEnv<QuadrotorEnv>;

}  // namespace flightlib
