#!/usr/bin/env python3
from ruamel.yaml import YAML, dump, RoundTripDumper

#
import os
import math
import argparse
import numpy as np
import time
import sys
import torch

from rpg_baselines.envs import vec_env_wrapper as wrapper
from scipy.spatial.transform import Rotation 
#
from flightgym import QuadrotorEnv_v1

def configure_random_seed(seed, env=None):
    if env is not None:
        env.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    
def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--train', type=int, default=1,
                        help="To train new model or simply test pre-trained model")
    parser.add_argument('--render', type=int, default=1,
                        help="Enable Unity Render")
    parser.add_argument('--save_dir', type=str, default=os.path.dirname(os.path.realpath(__file__)),
                        help="Directory where to save the checkpoints and training metrics")
    parser.add_argument('--seed', type=int, default=0,
                        help="Random seed")
    parser.add_argument('-w', '--weight', type=str, default='./saved/quadrotor_env.zip',
                        help='trained weight path')
    return parser

def main():
    args = parser().parse_args()
    cfg = YAML().load(open(os.environ["FLIGHTMARE_PATH"] +
                           "/flightlib/configs/vec_env.yaml", 'r'))
    cfg_quadrotor_env = YAML().load(open(os.environ["FLIGHTMARE_PATH"] +
                           "/flightlib/configs/quadrotor_env.yaml", 'r'))

    if args.render:
        cfg["env"]["render"] = "yes"
    else:
        cfg["env"]["render"] = "no"


    env = wrapper.FlightEnvVec(QuadrotorEnv_v1(dump(cfg, Dumper=RoundTripDumper), False))
    action = np.zeros([env.num_envs,env.num_acts], dtype=np.float32)
    # action += np.array([-0.01, -0.01, 0.00, 0.00])
    # action[0,0] += -0.11
    # action[0,3] += 0.1
    # action[1,0] += 0.11
    
    
    configure_random_seed(args.seed, env=env)
            
    connectedToUnity = False 
    while not connectedToUnity:
        connectedToUnity = env.connectUnity()             
        if not connectedToUnity:  
            print("Couldn't connect to unity, will try another time.")    
    
    print("env.num_envs : ", env.num_envs)

    max_ep_length = env.max_episode_steps

    if env.num_envs == 1:
        object_density_fractions = np.ones([env.num_envs], dtype=np.float32)
    else:
        object_density_fractions = np.linspace(0.0, 1.0, num=env.num_envs)

    env.set_objects_densities(object_density_fractions = object_density_fractions)   
    env.reset()
    
    time.sleep(2.5)
    
    
    done, ep_len = False, 0
        
    while not ((ep_len >= max_ep_length*10)):
        
        obs, reward, dones, infos = env.step(action)
        
        # print("distances : ", obs[0, 18:])

        ep_len += 1

if __name__ == "__main__":
    main()
