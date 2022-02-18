
#pragma once

// std lib
#include <stdlib.h>

#include <cmath>
#include <iostream>

// yaml cpp
#include <yaml-cpp/yaml.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/command.hpp"
#include "flightlib/common/logger.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/common/utils.hpp"
#include "flightlib/envs/env_base.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

namespace flightlib {

namespace visionenv {

enum Vision : int {
  //
  kNQuadState = 25,

  // observations
  kObs = 0,
  kNObs = 15,

  // control actions
  kAct = 0,
  kNAct = 4,
};
}  // namespace visionenv

class VisionEnv final : public EnvBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VisionEnv();
  VisionEnv(const std::string &cfg_path, const int env_id);
  VisionEnv(const YAML::Node &cfg_node, const int env_id);
  ~VisionEnv();

  // - public OpenAI-gym-style functions
  bool reset(Ref<Vector<>> obs) override;
  bool reset(Ref<Vector<>> obs, bool random);
  bool step(const Ref<Vector<>> act, Ref<Vector<>> obs,
            Ref<Vector<>> reward) override;

  // - public set functions
  bool loadParam(const YAML::Node &cfg);

  // - public get functions
  bool getObs(Ref<Vector<>> obs) override;
  bool getImage(Ref<ImgVector<>> img, const bool rgb = true) override;
  bool getDepthImage(Ref<DepthImgVector<>> img) override;

  // get quadrotor states
  bool getQuadAct(Ref<Vector<>> act) const;
  bool getQuadState(Ref<Vector<>> state) const;

  // - auxiliar functions
  bool isTerminalState(Scalar &reward) override;
  bool addQuadrotorToUnity(const std::shared_ptr<UnityBridge> bridge) override;

  friend std::ostream &operator<<(std::ostream &os,
                                  const VisionEnv &vision_env);


  bool configCamera(const YAML::Node &cfg);

  // flightmare (visualization)
  bool setUnity(const bool render);
  bool connectUnity();
  void disconnectUnity();
  FrameID updateUnity(const FrameID frame_id);

  //
  inline std::vector<std::string> getRewardNames() { return reward_names_; }
  inline void setSceneID(const SceneID id) { scene_id_ = id; }
  inline std::shared_ptr<Quadrotor> getQuadrotor() { return quad_ptr_; }

 private:
  void init();
  int env_id_;
  // quadrotor
  std::shared_ptr<Quadrotor> quad_ptr_;
  QuadState quad_state_;
  Command cmd_;
  Logger logger_{"VisionEnv"};


  // Define reward for training
  Scalar dummy_coeff_;

  // observations and actions (for RL)
  Vector<visionenv::kNObs> pi_obs_;
  Vector<visionenv::kNAct> pi_act_;

  // reward function design (for model-free reinforcement learning)
  Vector<3> goal_pos_;

  // action and observation normalization (for learning)
  Vector<visionenv::kNAct> act_mean_;
  Vector<visionenv::kNAct> act_std_;
  Vector<visionenv::kNObs> obs_mean_ = Vector<visionenv::kNObs>::Zero();
  Vector<visionenv::kNObs> obs_std_ = Vector<visionenv::kNObs>::Ones();

  // robot vision
  std::shared_ptr<RGBCamera> rgb_camera_;
  cv::Mat rgb_img_, gray_img_;
  cv::Mat depth_img_;

  // auxiliary variables
  int rotor_ctrl_{true};
  bool use_camera_{false};
  YAML::Node cfg_;
  std::vector<std::string> reward_names_;
  Matrix<3, 2> world_box_;

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  SceneID scene_id_{UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  RenderMessage_t unity_output_;
  uint16_t receive_id_{0};
  Vector<3> unity_render_offset_;
};

}  // namespace flightlib