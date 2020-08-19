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

namespace flightlib {

template<typename EnvBase>
class VecEnv {
 public:
  VecEnv();
  VecEnv(const std::string& cfgs, const bool from_file = true);
  VecEnv(const YAML::Node& cfgs_node);
  ~VecEnv();

  // - public OpenAI-gym style functions for vectorized environment
  void reset(Ref<MatrixRowMajor<>> obs);
  void step(Ref<MatrixRowMajor<>> act, Ref<MatrixRowMajor<>> obs,
            Ref<Vector<>> reward, Ref<BoolVector<>> done,
            Ref<MatrixRowMajor<>> extra_info);
  void stepUnity(Ref<MatrixRowMajor<>> act, Ref<MatrixRowMajor<>> obs,
                 Ref<Vector<>> reward, Ref<BoolVector<>> done,
                 Ref<MatrixRowMajor<>> extra_info, uint64_t send_id);
  void close();

  // public set functions
  void setSeed(const int seed);

  // public get functions
  void getObs(Ref<MatrixRowMajor<>> obs);

  // - auxiliary functions
  void isTerminalState(Ref<BoolVector<>> terminal_state);
  void testStep(Ref<MatrixRowMajor<>> act, Ref<MatrixRowMajor<>> obs,
                Ref<Vector<>> reward, Ref<BoolVector<>> done,
                Ref<MatrixRowMajor<>> extra_info);
  void curriculumUpdate();

  // flightmare (visualization)
  void setUnity(bool render);
  void connectUnity();
  void disconnectUnity();

  // public functions
  inline int getSeed(void) { return seed_; };
  inline SceneID getSceneID(void) { return scene_id_; };
  inline bool getUnityRender(void) { return unity_render_; };
  inline int getObsDim(void) { return obs_dim_; };
  inline int getActDim(void) { return act_dim_; };
  inline int getExtraInfoDim(void) { return extra_info_names_.size(); };
  inline int getNumOfEnvs(void) { return envs_.size(); };
  inline std::vector<std::string>& getExtraInfoNames() {
    return extra_info_names_;
  };

 private:
  void init(void);
  // step every environment
  void perAgentStep(int agent_id, Ref<MatrixRowMajor<>> act,
                    Ref<MatrixRowMajor<>> obs, Ref<Vector<>> reward,
                    Ref<BoolVector<>> done, Ref<MatrixRowMajor<>> extra_info);

  // objects
  Logger logger_{"VecEnv"};
  std::vector<std::unique_ptr<EnvBase>> envs_;
  std::vector<std::string> extra_info_names_;

  // Flightmare (Unity3D)
  const Scalar unity_connection_time_out_{10.0};  // seconds
  bool unity_ready_{false}, unity_render_{false}, unity_bridge_created_{false};
  std::shared_ptr<UnityBridge> unity_bridge_;
  SceneID scene_id_{UnityScene::WAREHOUSE};
  RenderMessage_t unity_output_;
  uint16_t receive_id_{0};

  // auxiliar variables
  int seed_, num_envs_, obs_dim_, act_dim_;
  Matrix<> obs_dummy_;

  // yaml configurations
  YAML::Node cfg_;
};

}  // namespace flightlib
