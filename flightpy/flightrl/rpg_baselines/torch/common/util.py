import pandas as pd
import numpy as np

columns = [
    "env_id", "trial_id", "done", "reward",
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

def traj_rollout(env, policy, env_idx):
    traj_df = pd.DataFrame(columns=columns)
    max_ep_length = 1000
    obs = env.reset(random=False)
    trial_id = np.zeros(shape=(env.num_envs, 1))
    for _ in range(max_ep_length):
        act, _ = policy.predict(obs, deterministic=True)
        act = np.array(act, dtype=np.float64)
        #
        obs, rew, done, info = env.step(act)
  
        trial_id[done] += 1

        state = env.getQuadState()
        action = env.getQuadAct()

        # reshape vector
        done = done[:, np.newaxis]
        rew = rew[:, np.newaxis]

        # stack all the data
        data = np.hstack(
            (env_idx, trial_id, done, rew, state, action))
        data_frame = pd.DataFrame(data=data, columns=columns)

        # append trajectory
        traj_df = traj_df.append(data_frame, ignore_index=True) 
    return traj_df