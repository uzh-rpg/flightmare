//
// This is inspired by RaiGym, thanks.
// https://raisim.com/
//
#pragma once

// std
#include <memory>

// openmp
#include <omp.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/logger.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/envs/env_base.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"
#include "flightlib/envs/vision_env/vision_env.hpp"

namespace flightlib {

template<typename EnvBaseName>
class VecEnvBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VecEnvBase();
  virtual ~VecEnvBase() = 0;

  // - public OpenAI-gym style functions for vectorized environment
  virtual void configEnv(const YAML::Node& cfgs_node);
  virtual bool reset(Ref<MatrixRowMajor<>> obs);
  virtual bool step(Ref<MatrixRowMajor<>> act, Ref<MatrixRowMajor<>> obs,
                    Ref<MatrixRowMajor<>> reward, Ref<BoolVector<>> done,
                    Ref<MatrixRowMajor<>> extra_info);
  virtual void close();

  // public set functions
  void setSeed(const int seed);

  // public get functions
  bool getObs(Ref<MatrixRowMajor<>> obs);
  bool getImage(Ref<ImgMatrixRowMajor<>> img, const bool rgb = false);
  bool getDepthImage(Ref<DepthImgMatrixRowMajor<>> img);

  size_t getEpisodeLength(void);

  // - auxiliary functions
  void isTerminalState(Ref<BoolVector<>> terminal_state);
  void curriculumUpdate();

  // flightmare (visualization)
  bool setUnity(const bool render);
  bool connectUnity();
  void disconnectUnity();
  FrameID updateUnity(const FrameID frame_id);

  // public functions
  inline int getSeed(void) const { return seed_; };
  inline SceneID getSceneID(void) const { return scene_id_; };
  inline bool getUnityRender(void) const { return unity_render_; };
  inline int getObsDim(void) const { return obs_dim_; };
  inline int getActDim(void) const { return act_dim_; };
  inline int getRewDim(void) const { return rew_dim_; };
  inline int getImgHeight(void) const { return img_height_; };
  inline int getImgWidth(void) const { return img_width_; };
  inline int getExtraInfoDim(void) const { return extra_info_names_.size(); };
  inline int getNumOfEnvs(void) const { return envs_.size(); };
  inline std::vector<std::string>& getExtraInfoNames() {
    return extra_info_names_;
  };

 protected:
  // step every environment
  virtual void perAgentStep(int agent_id, Ref<MatrixRowMajor<>> act,
                            Ref<MatrixRowMajor<>> obs,
                            Ref<MatrixRowMajor<>> reward_units,
                            Ref<BoolVector<>> done,
                            Ref<MatrixRowMajor<>> extra_info);
  // create objects
  Logger logger_{"VecEnvBase"};
  std::vector<std::unique_ptr<EnvBaseName>> envs_;
  std::vector<std::string> extra_info_names_;

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  SceneID scene_id_{UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  RenderMessage_t unity_output_;
  uint16_t receive_id_{0};

  // auxiliar variables
  int seed_, num_envs_, obs_dim_, act_dim_, rew_dim_, num_threads_;
  int img_width_, img_height_;
  Matrix<> obs_dummy_;
};

}  // namespace flightlib
