#!/usr/bin/env python3
import numpy as np

def skip_first_few_episodes(env, episodes_to_skip, episode_length):
    n_roll = 0
           
    # Different rollouts.
    while n_roll < episodes_to_skip:
        obs, done, ep_len = env.reset(), False, 0
        
        while (ep_len < episode_length):
            act = np.zeros([env.num_envs,env.num_acts], dtype=np.float32)
            
            obs, rew, done, infos = env.step(act)
            ep_len += 1
            # print("n_roll : ", n_roll)
            # print("ep_len : ", ep_len)
        n_roll += 1
    # Leave a resetted env
    obs= env.reset()