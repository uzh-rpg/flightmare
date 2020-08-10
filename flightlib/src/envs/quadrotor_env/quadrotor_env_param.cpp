#include "flightlib/envs/quadrotor_env/quadrotor_env_param.hpp"

namespace flightlib {

QuadrotorEnvParam::QuadrotorEnvParam() {
  // cost matrix
  Q = (Vector<CtlObsAct::kObsSize>() << -1e-2, -1e-2, -1e-2, -1e-2, -1e-2,
       -1e-2, -1e-3, -1e-3, -1e-3)
        .finished()
        .asDiagonal();
  Q_act = Vector<CtlObsAct::kActSize>(-1e-4, -1e-4, -1e-4, -1e-4).asDiagonal();
  goal_state = (Vector<CtlObsAct::kObsSize>() << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0)
                 .finished();

  // environment configuration
  scene_id = UnityScene::WAREHOUSE;
  sim_dt = 0.02;
  render = false;

  // quadrotor configuration
  mass = 1.0;
  arm_length = 0.2;
  kappa = 0.016;
  inertia << 0.007, 0.007, 0.012;
  camera = false;
}

QuadrotorEnvParam::~QuadrotorEnvParam() {}

bool QuadrotorEnvParam::valid(void) {
  bool valid = true;

  valid &= Q.allFinite();
  valid &= Q_act.allFinite();

  valid &= act_mean.allFinite();
  valid &= act_std.allFinite();
  valid &= obs_mean.allFinite();
  valid &= obs_std.allFinite();
  valid &= goal_state.allFinite();

  valid &= sim_dt > 0.0;
  valid &= scene_id >= 0;
  valid &= mass > 0.0;
  valid &= arm_length > 0.0;
  valid &= kappa > 0.0;
  valid &= inertia.allFinite();

  return valid;
}

bool QuadrotorEnvParam::loadParam(void) {
  if (cfg_node_["env"]) {
    // load environment configurations
    scene_id = cfg_node_["env"]["scene_id"].template as<SceneID>();
    sim_dt = cfg_node_["env"]["sim_dt"].template as<Scalar>();
    render = cfg_node_["env"]["render"].template as<bool>();
  } else {
    return false;
  }

  if (cfg_node_["quadrotor"]) {
    // load quadrotor configurations
    mass = cfg_node_["quadrotor"]["mass"].template as<Scalar>();
    arm_length = cfg_node_["quadrotor"]["arm_length"].template as<Scalar>();
    kappa = cfg_node_["quadrotor"]["rotor_drag_coeff"].template as<Scalar>();
    std::vector<Scalar> inertia_vector;
    inertia_vector =
      cfg_node_["quadrotor"]["inertia"].as<std::vector<Scalar>>();
    inertia << inertia_vector[0], inertia_vector[1], inertia_vector[2];
    camera = cfg_node_["quadrotor"]["camera"].template as<bool>();
  } else {
    return false;
  }

  if (cfg_node_["rl"]) {
    // load reinforcement learning related parameters
    std::vector<Scalar> Q_pos(3), Q_ori(3), Q_lin_vel(3), Q_cmd;
    Q_pos = cfg_node_["rl"]["Q_pos"].as<std::vector<Scalar>>();
    Q_ori = cfg_node_["rl"]["Q_ori"].as<std::vector<Scalar>>();
    Q_lin_vel = cfg_node_["rl"]["Q_lin_vel"].as<std::vector<Scalar>>();
    Q_cmd = cfg_node_["rl"]["Q_cmd"].as<std::vector<Scalar>>();

    Q = (Vector<CtlObsAct::kObsSize>() << Q_pos[0], Q_pos[1], Q_pos[2],
         Q_ori[0], Q_ori[1], Q_ori[2], Q_lin_vel[0], Q_lin_vel[1], Q_lin_vel[2])
          .finished()
          .asDiagonal();
    Q_act =
      (Vector<CtlObsAct::kActSize>() << Q_cmd[0], Q_cmd[1], Q_cmd[2], Q_cmd[3])
        .finished()
        .asDiagonal();
  } else {
    return false;
  }

  return true;
}

}  // namespace flightlib
