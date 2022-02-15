import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from ruamel.yaml import YAML
from rpg_baselines.torch.racing_utils.viz_utils import viz_rollout, \
    viz_states_inputs, viz_rew_ev
from rpg_baselines.torch.racing_utils.eval_utils import compute_stats
from scipy.spatial.transform import Rotation as R
import quaternion


def test_race_model(model, env, render, plot_data=False):

    # Perform rollout
    max_episode_steps = int(env.cfg['environment']['max_t'] /
                            env.cfg['environment']['sim_dt'])
    pos, euler, dpos, ddpos, deuler, quats = [], [], [], [], [], []
    actions = []
    target_wps, modes, missed_gates = [], [], []
    reward_components = []
    obs, done, ep_len = env.reset(), False, 0

    # Update gates in cfg
    cfg_dir = os.environ["FLIGHTMARE_PATH"] + "/flightpy/configs/racing/tracks"
    track_cfgs = \
        [YAML().load(open(os.path.join(cfg_dir, track + '.yaml'), 'r'))
         for track in env.cfg['main']['tracks']]
    render_offset = np.array(track_cfgs[0]['render_offset'])
    env.cfg['track'] = track_cfgs[0]

    # Enable rendering
    if render:
        env.connectUnity()

    while not (done or (ep_len >= max_episode_steps)):
        # Log state variables
        rot_mat = np.reshape(obs[0, :9], (3, 3))
        quat = quaternion.as_float_array(quaternion.
                                         from_rotation_matrix(rot_mat)) \
            * np.array([1.0, -1.0, -1.0, -1.0])
        euler.append(R.from_quat(quat).as_euler('zyx').tolist())
        dpos.append(obs[0, 9:12].tolist())
        quats.append(quat)
        deuler.append(obs[0, 12:15].tolist())
        ddpos.append(obs[0, 15:18].tolist())

        # Normalize observation
        obs_mean, obs_std = env.get_obs_norm()
        obs_norm = (obs - obs_mean) / obs_std

        # Get deterministic actions
        act, _ = model.predict(obs_norm, deterministic=True)

        # Simulate environment
        obs, rew, done, infos = env.step(act)
        env.render()

        # ======RGB Image=========
        rgb_img = np.reshape(
            env.getImage(rgb=True)[0], (env.img_height, env.img_width, 3))
        cv2.imshow("rgb_img", rgb_img)
        cv2.waitKey(50)

        # Log position
        x = infos[0]['extra_info']['pos_x']
        y = infos[0]['extra_info']['pos_y']
        z = infos[0]['extra_info']['pos_z']
        p = np.array([x, y, z]) - render_offset
        pos.append((p).tolist())

        # Log reward components
        reward_components.append(infos[0]['reward_components'])

        # Log actions
        actions.append(act[0, :].tolist())

        # Log additional info
        target_wps.append(infos[0]["extra_info"]["target_gate"])
        modes.append(infos[0]["extra_info"]["mode"])
        missed_gates.append(infos[0]["extra_info"]["gate_error"])

        # Increment step counter
        ep_len += 1

    pos = np.array(pos)
    dpos = np.array(dpos)
    euler = np.asarray(euler)
    deuler = np.asarray(deuler)
    target_wps = np.array(target_wps).astype(np.int)

    actions = np.asarray(actions)
    modes = np.array(modes).astype(np.int)
    missed_gates = np.array(missed_gates)
    reward_components = np.array(reward_components)
    cum_path_length = np.array(env.cum_path_length)
    rollout_data = {'pos': pos, 'dpos': dpos, 'euler': euler, 'deuler': deuler,
                    'actions': actions, 'target_wps': target_wps,
                    'modes': modes, 'missed_gates': missed_gates,
                    "reward_components": reward_components,
                    "cum_path_length": cum_path_length}

    if plot_data:
        # Compute relevant statistics from rollout data
        rollout_data = compute_stats(env.cfg, rollout_data)

        # Visualize rollout
        viz_rollout(env.cfg, rollout_data, show_projection=False, show_3d=True)

        # Visualize state and input evolution
        viz_states_inputs(env.cfg, rollout_data)

        # Visualize reward evolution
        viz_rew_ev(env.cfg, rollout_data)

        plt.show()
