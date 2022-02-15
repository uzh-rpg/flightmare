
#!/usr/bin/env python3
from ruamel.yaml import YAML, dump, RoundTripDumper

#
import os
import glob
import math
import argparse
import numpy as np
import torch

from flightgym import QuadrotorEnv_v1
from rpg_baselines.torch.envs import vec_env_wrapper as wrapper
from rpg_baselines.torch.ppo.ppo import PPO
# from rpg_baselines.torch.ppo.ppo_test import test_model
from ppo_test import test_model


def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--loadunity', type=int, default=0,
                        help="To load unity automatically")
    parser.add_argument('--render', type=int, default=0,
                        help="Enable Unity Render")
    return parser


def main():
    args = parser().parse_args()
    cfg = YAML().load(open(os.environ["FLIGHTMARE_PATH"] +
                           "/flightpy/configs/control/config.yaml", 'r'))
    cfg["main"]["num_envs"] = 1
    cfg["main"]["num_threads"] = 1

    if args.render:
        cfg["main"]["render"] = "yes"
        if args.loadunity:
            os.system(os.environ["FLIGHTMARE_PATH"] +
                      "/flightrender/RPG_Flightmare.x86_64 &")
    else:
        cfg["main"]["render"] = "no"

    env = QuadrotorEnv_v1(dump(cfg, Dumper=RoundTripDumper), False)
    env = wrapper.FlightEnvVec(env)

    weight_dir = "./saved/PPO_11/Weights"
    save_dir = "./saved/PPO_11/Results"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    for i, weight in enumerate(sorted(glob.glob(os.path.join(weight_dir, "*.zip")))):
        print(i, weight)
        model = PPO.load(weight)
        num_iter = weight.split("/")[-1].split(".")[0]
        test_model(env, model, render=args.render,
                   save_dir=save_dir + "/{0}.png".format(num_iter))


if __name__ == "__main__":
    main()
