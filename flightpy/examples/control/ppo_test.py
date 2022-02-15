import matplotlib.pyplot as plt
import numpy as np
import matplotlib.gridspec as gridspec
import cv2

from scipy.spatial.transform import Rotation as R

state_label = [
    "pos x",
    "pos y",
    "pos z",
    "euler x",
    "euler y",
    "euler z",
    "linear vx",
    "linear vy",
    "linear vz",
    "omega x",
    "omega y",
    "omega z",
]
act_label = ["thrust", "omgea x", "omega y", "omega z"]


def test_model(env, model, render=False, save_dir=None):
    #
    fig = plt.figure(figsize=(10, 6), tight_layout=True)
    gs = gridspec.GridSpec(5, 12)
    #
    act_min = [-0, -10, -10, -6]
    act_max = [30, 10, 10, 6]
    #
    axes_pos_xyz, axes_euler_xyz, axes_vel_xyz, axes_ang_xyz = [], [], [], []
    for i in range(3):
        axes_pos_xyz.append(fig.add_subplot(gs[0, i * 4 : (i + 1) * 4]))
        axes_euler_xyz.append(fig.add_subplot(gs[1, i * 4 : (i + 1) * 4]))
        axes_vel_xyz.append(fig.add_subplot(gs[2, i * 4 : (i + 1) * 4]))
        axes_ang_xyz.append(fig.add_subplot(gs[3, i * 4 : (i + 1) * 4]))
    #
    axes_actions = []
    for i in range(4):
        axes_actions.append(fig.add_subplot(gs[4, i * 3 : (i + 1) * 3]))
    max_ep_length = env.max_episode_steps
    num_rollouts = 5
    if render:
        env.connectUnity()
    for n_roll in range(num_rollouts):
        quad_state, quad_act = [], []
        obs, done, ep_len = env.reset(), False, 0
        while not (done or (ep_len >= max_ep_length)):
            act, _ = model.predict(obs, deterministic=True)
            obs, rew, done, info = env.step(act)
            s = env.getQuadState()
            a = env.getQuadAct()
            # a = act

            #
            env.render(0)

            # ======Gray Image=========
            # gray_img = np.reshape(
            #     env.getImage()[0], (env.img_height, env.img_width))
            # cv2.imshow("gray_img", gray_img)
            # cv2.waitKey(100)

            # ======RGB Image=========
            # rgb_img = np.reshape(
            #    env.getImage(rgb=True)[0], (env.img_height, env.img_width, 3))
            # cv2.imshow("rgb_img", rgb_img)
            # cv2.waitKey(100)

            # ======Depth Image=========
            # depth_img = np.reshape(env.getDepthImage()[
            #                        0], (env.img_height, env.img_width))
            # cv2.imshow("depth", depth_img)
            # cv2.waitKey(100)

            #
            ep_len += 1

            #
            euler = R.from_quat([s[0, 4], s[0, 5], s[0, 6], s[0, 3]]).as_euler("zyx")
            #
            s = (
                s[0, 0:3].tolist()
                + euler.tolist()
                + s[0, 7:10].tolist()
                + s[0, 10:13].tolist()
            )
            # print(s)
            quad_state.append(s)
            quad_act.append(a[0, :].tolist())
        quad_state = np.asarray(quad_state)
        quad_act = np.asarray(quad_act)
        #
        t = np.arange(0, quad_state.shape[0] - 1)

        for i in range(3):
            axes_pos_xyz[i].step(t, quad_state[:-1, i], color="C{0}".format(n_roll))
            axes_pos_xyz[i].set_title("{0}".format(state_label[i]))
            #
            axes_euler_xyz[i].step(
                t, quad_state[:-1, i + 3], color="C{0}".format(n_roll)
            )
            axes_euler_xyz[i].set_title("{0}".format(state_label[i + 3]))
            #
            axes_vel_xyz[i].step(t, quad_state[:-1, i + 6], color="C{0}".format(n_roll))
            axes_vel_xyz[i].set_title("{0}".format(state_label[i + 6]))
            axes_ang_xyz[i].step(t, quad_state[:-1, i + 9], color="C{0}".format(n_roll))
            axes_ang_xyz[i].set_title("{0}".format(state_label[i + 9]))
        for i in range(4):
            axes_actions[i].step(t, quad_act[:-1, i], color="C{0}".format(n_roll))
            axes_actions[i].set_ylim(act_min[i], act_max[i])
    #
    if render:
        env.disconnectUnity()
    #
    plt.tight_layout()
    if save_dir is not None:
        plt.savefig(save_dir)
    else:
        plt.show()
