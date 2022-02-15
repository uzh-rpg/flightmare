import pandas as pd
import numpy as np

from mpl_toolkits.mplot3d import Axes3D

columns = [
    "episode_id", "done", "reward",
    "t",
    "px", "py", "pz",
    "qw", "qx", "qy", "qz",
    "vx", "vy", "vz",
    "wx", "wy", "wz",
    "ax", "ay", "az",
    "mot1", "mot2", "mot3", "mot4",
    "thrust1", "thrust2", "thrust3", "thrust4",
    "act1", "act2", "act3", "act4"
]

def traj_rollout(env, policy):
    traj_df = pd.DataFrame(columns=columns)
    max_ep_length = 1000
    obs = env.reset(random=False)
    episode_id = np.zeros(shape=(env.num_envs, 1))
    for _ in range(max_ep_length):
        act, _ = policy.predict(obs, deterministic=True)
        act = np.array(act, dtype=np.float64)
        #
        obs, rew, done, info = env.step(act)
  
        episode_id[done] += 1

        state = env.getQuadState()
        action = env.getQuadAct()

        # reshape vector
        done = done[:, np.newaxis]
        rew = rew[:, np.newaxis]

        # stack all the data
        data = np.hstack(
            (episode_id, done, rew, state, action))
        data_frame = pd.DataFrame(data=data, columns=columns)

        # append trajectory
        traj_df = pd.concat([traj_df, data_frame], axis=0, ignore_index=True)
    return traj_df
  
def plot3d_traj(ax3d, pos, vel):
    sc = ax3d.scatter(
        pos[:, 0],
        pos[:, 1],
        pos[:, 2],
        c=np.linalg.norm(vel, axis=1),
        cmap="jet",
        s=1,
        alpha=0.5,
    )
    ax3d.view_init(elev=40, azim=50)
    # 
    ax3d.set_xticks([])
    ax3d.set_yticks([])
    ax3d.set_zticks([])

    # 
    # ax3d.get_proj = lambda: np.dot(
    # Axes3D.get_proj(ax3d), np.diag([1.0, 1.0, 1.0, 1.0]))
    # zmin, zmax = ax3d.get_zlim()
    # xmin, xmax = ax3d.get_xlim()
    # ymin, ymax = ax3d.get_ylim()
    # x_f = 1
    # y_f = (ymax - ymin) / (xmax - xmin)
    # z_f = (zmax - zmin) / (xmax - xmin)
    # ax3d.set_box_aspect((x_f, y_f * 2, z_f * 2))
