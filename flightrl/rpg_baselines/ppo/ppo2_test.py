import matplotlib.pyplot as plt
import numpy as np
import matplotlib.gridspec as gridspec


def test_model(env, model, render=False):
    #
    fig = plt.figure(figsize=(18, 12), tight_layout=True)
    gs = gridspec.GridSpec(5, 12)
    #
    ax_x = fig.add_subplot(gs[0, 0:4])
    ax_y = fig.add_subplot(gs[0, 4:8])
    ax_z = fig.add_subplot(gs[0, 8:12])
    #
    ax_dx = fig.add_subplot(gs[1, 0:4])
    ax_dy = fig.add_subplot(gs[1, 4:8])
    ax_dz = fig.add_subplot(gs[1, 8:12])
    #
    ax_euler_x = fig.add_subplot(gs[2, 0:4])
    ax_euler_y = fig.add_subplot(gs[2, 4:8])
    ax_euler_z = fig.add_subplot(gs[2, 8:12])
    #
    ax_euler_vx = fig.add_subplot(gs[3, 0:4])
    ax_euler_vy = fig.add_subplot(gs[3, 4:8])
    ax_euler_vz = fig.add_subplot(gs[3, 8:12])
    #
    ax_action0 = fig.add_subplot(gs[4, 0:3])
    ax_action1 = fig.add_subplot(gs[4, 3:6])
    ax_action2 = fig.add_subplot(gs[4, 6:9])
    ax_action3 = fig.add_subplot(gs[4, 9:12])

    max_ep_length = env.max_episode_steps
    num_rollouts = 5
    if render:
        env.connectUnity()
    for n_roll in range(num_rollouts):
        pos, euler, dpos, deuler = [], [], [], []
        actions = []
        obs, done, ep_len = env.reset(), False, 0
        while not (done or (ep_len >= max_ep_length)):
            act, _ = model.predict(obs, deterministic=True)
            obs, rew, done, infos = env.step(act)
            #
            ep_len += 1
            #
            pos.append(obs[0, 0:3].tolist())
            dpos.append(obs[0, 6:9].tolist())
            euler.append(obs[0, 3:6].tolist())
            deuler.append(obs[0, 9:12].tolist())
            #
            actions.append(act[0, :].tolist())
        pos = np.asarray(pos)
        dpos = np.asarray(dpos)
        euler = np.asarray(euler)
        deuler = np.asarray(deuler)
        actions = np.asarray(actions)
        #
        t = np.arange(0, pos.shape[0])
        ax_x.step(t, pos[:, 0], color="C{0}".format(
            n_roll), label="trail: {0}".format(n_roll))
        ax_y.step(t, pos[:, 1], color="C{0}".format(
            n_roll), label="trail: {0}".format(n_roll))
        ax_z.step(t, pos[:, 2], color="C{0}".format(
            n_roll), label="pos [x, y, z] -- trail: {0}".format(n_roll))
        #
        ax_dx.step(t, dpos[:, 0], color="C{0}".format(
            n_roll), label="trail: {0}".format(n_roll))
        ax_dy.step(t, dpos[:, 1], color="C{0}".format(
            n_roll), label="trail: {0}".format(n_roll))
        ax_dz.step(t, dpos[:, 2], color="C{0}".format(
            n_roll), label="vel [x, y, z] -- trail: {0}".format(n_roll))
        #
        ax_euler_x.step(t, euler[:, -1], color="C{0}".format(
            n_roll), label="trail: {0}".format(n_roll))
        ax_euler_y.step(t, euler[:, 0], color="C{0}".format(
            n_roll), label="trail :{0}".format(n_roll))
        ax_euler_z.step(t, euler[:, 1], color="C{0}".format(
            n_roll), label="trail: {0}".format(n_roll))
        #
        ax_euler_vx.step(t, deuler[:, -1], color="C{0}".format(
            n_roll), label="trail: {0}".format(n_roll))
        ax_euler_vy.step(t, deuler[:, 0], color="C{0}".format(
            n_roll), label="trail :{0}".format(n_roll))
        ax_euler_vz.step(t, deuler[:, 1], color="C{0}".format(
            n_roll), label=r"$\theta$ [x, y, z] -- trail: {0}".format(n_roll))
        #
        ax_action0.step(t, actions[:, 0], color="C{0}".format(
            n_roll), label="trail: {0}".format(n_roll))
        ax_action1.step(t, actions[:, 1], color="C{0}".format(
            n_roll), label="trail: {0}".format(n_roll))
        ax_action2.step(t, actions[:, 2], color="C{0}".format(
            n_roll), label="trail: {0}".format(n_roll))
        ax_action3.step(t, actions[:, 3], color="C{0}".format(
            n_roll), label="act [0, 1, 2, 3] -- trail: {0}".format(n_roll))
    #
    if render:
        env.disconnectUnity()
    ax_z.legend()
    ax_dz.legend()
    ax_euler_z.legend()
    ax_euler_vz.legend()
    ax_action3.legend()
    #
    plt.tight_layout()
    plt.show()
