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
#include "flightlib/common/math.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/envs/env_base.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include <opencv2/core/eigen.hpp>

namespace flightlib {

namespace quadenv {

enum Ctl : int {
  // observations
  kObs = 0,
  //
  kPos = 0,
  kNPos = 3,
  kOri = 3,
  kNOri = 3,
  kLinVel = 6,
  kNLinVel = 3,
  kAngVel = 9,
  kNAngVel = 3,
  kGoal = 12,
  kNGoal = 3,
  kNObs = 15,
  // control actions
  kAct = 0,
  kNAct = 4,
  frame_height = 128,
  frame_width = 128,
  camera_FOV = 90,
};
};
class QuadrotorEnv final : public EnvBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  QuadrotorEnv();
  QuadrotorEnv(const std::string &cfg_path);
  ~QuadrotorEnv();

  // - public OpenAI-gym-style functions
  bool reset(Ref<Vector<>> obs, Ref<DepthImage<>> img, const bool random = true) override;
  bool resetObs(Ref<Vector<>> obs, const bool random = true);
  bool resetImages(Ref<DepthImage<>> img);

  void setResetPose(Vector<3> &resetPosition, Vector<3> &resetRotation);
  void setQuadstateToResetState();
  Scalar step(const Ref<Vector<>> act, Ref<Vector<>> obs) override;
  Scalar stepUnity(Ref<DepthImage<>> img);

  // - public set functions
  bool loadParam(const YAML::Node &cfg);

  // public variables
  std::unordered_map<std::string, float> extra_info_;

  // - public get functions
  bool getObs(Ref<Vector<>> obs) override;
  bool getImages(Ref<DepthImage<>> img);
  bool getDistances(Ref<Vector<>> distances);
  bool getAct(Ref<Vector<>> act) const;
  bool getAct(Command *const cmd) const;

  // - auxiliar functions
  bool isTerminalState(Scalar &reward) override;
  bool isTerminalStateUnity(Scalar &reward) override;
  void addObjectsToUnity(std::shared_ptr<UnityBridge> bridge);

  friend std::ostream &operator<<(std::ostream &os,
                                  const QuadrotorEnv &quad_env);

 private:
  // quadrotor
  std::shared_ptr<Quadrotor> quadrotor_ptr_;
  QuadState quad_state_;
  Command cmd_;
  Logger logger_{"QaudrotorEnv"};

  Vector<3> resetPosition_ = Vector<3>::Zero();
  Vector<3> resetRotation_ = Vector<3>::Zero();

  std::shared_ptr<RGBCamera> rgb_camera;
    cv::Mat img_;
  cv::Mat img_depth_;
  cv::Mat img_segm_;
  cv::Mat img_flow_;
  cv::Mat img_mask_;
  cv::Mat channels[3];
  Image_mat<quadenv::frame_height, quadenv::frame_width> img_mat_[3];
  Depth_image_mat<quadenv::frame_height, quadenv::frame_width> depth_img_mat_;
  // Define reward for training
  Scalar pos_coeff_, ori_coeff_, lin_vel_coeff_, ang_vel_coeff_, act_coeff_;

  // observations and actions (for RL)
  Vector<quadenv::kNObs> quad_obs_;
  Vector<quadenv::kNAct> quad_act_;

  // reward function design (for model-free reinforcement learning)
  Vector<quadenv::kNObs> goal_state_;

  // action and observation normalization (for learning)
  Vector<quadenv::kNAct> act_mean_;
  Vector<quadenv::kNAct> act_std_;
  Vector<quadenv::kNObs> obs_mean_ = Vector<quadenv::kNObs>::Zero();
  Vector<quadenv::kNObs> obs_std_ = Vector<quadenv::kNObs>::Ones();

  YAML::Node cfg_;
  Matrix<3, 2> world_box_;

  bool justHadCollision = false;
  int collision_step_count = 0;
};

}  // namespace flightlib