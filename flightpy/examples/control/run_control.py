#!/usr/bin/env python3
from ruamel.yaml import YAML, dump, RoundTripDumper

#
import os
import math
import argparse
import numpy as np
import torch


from flightgym import QuadrotorEnv_v1
from rpg_baselines.torch.envs import vec_env_wrapper as wrapper
from ppo import PPO
from ppo_test import test_model
from stable_baselines3.common.utils import get_device
from stable_baselines3.ppo.policies import MlpPolicy


def configure_random_seed(seed, env=None):
    if env is not None:
        env.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--train",
        type=int,
        default=1,
        help="To train new model or simply test pre-trained model",
    )
    parser.add_argument(
        "--loadunity", type=int, default=0, help="To load unity automatically"
    )
    parser.add_argument("--render", type=int, default=0, help="Enable Unity Render")
    parser.add_argument("--seed", type=int, default=0, help="Random seed")
    parser.add_argument("--trial", type=int, default=0, help="trained weight path")
    parser.add_argument("--iter", type=int, default=0, help="trained weight path")
    return parser


def main():
    args = parser().parse_args()
    cfg = YAML().load(
        open(
            os.environ["FLIGHTMARE_PATH"] + "/flightpy/configs/control/config.yaml", "r"
        )
    )
    if not args.train:
        cfg["main"]["num_envs"] = 1
        cfg["main"]["num_threads"] = 1

    if args.render:
        cfg["main"]["render"] = "yes"
        if args.loadunity:
            os.system(
                os.environ["FLIGHTMARE_PATH"] + "/flightrender/RPG_Flightmare.x86_64 &"
            )
    else:
        cfg["main"]["render"] = "no"

    env = QuadrotorEnv_v1(dump(cfg, Dumper=RoundTripDumper), False)
    env = wrapper.FlightEnvVec(env)

    # set random seed
    configure_random_seed(args.seed, env=env)

    #
    if args.train:
        # save the configuration and other files
        rsg_root = os.path.dirname(os.path.abspath(__file__))
        log_dir = rsg_root + "/saved"
        # saver = U.ConfigurationSaver(log_dir=log_dir)
        model = PPO(
            tensorboard_log=log_dir,
            policy="MlpPolicy",
            policy_kwargs=dict(
                activation_fn=torch.nn.ReLU,
                net_arch=[dict(pi=[128, 128], vf=[128, 128])],
                log_std_init=-0.5,
            ),
            env=env,
            use_tanh_act=True,
            gae_lambda=0.95,
            gamma=0.99,
            n_steps=250,
            ent_coef=0.0,
            vf_coef=0.5,
            max_grad_norm=0.5,
            batch_size=25000,
            clip_range=0.2,
            use_sde=False,  # don't use (gSDE), doesn't work
            env_cfg=cfg,
            verbose=1,
        )

        #
        model.learn(total_timesteps=int(50000000), log_interval=(10, 50))
    else:
        #
        weight = "./saved/PPO_{0}/Policy/iter_{1:05d}.pth".format(args.trial, args.iter)
        env_rms = "./saved/PPO_{0}/RMS/iter_{1:05d}.npz".format(args.trial, args.iter)

        device = get_device("auto")
        saved_variables = torch.load(weight, map_location=device)
        # Create policy object
        policy = MlpPolicy(**saved_variables["data"])
        #
        policy.action_net = torch.nn.Sequential(policy.action_net, torch.nn.Tanh())
        # Load weights
        policy.load_state_dict(saved_variables["state_dict"], strict=False)
        policy.to(device)

        # policy = MlpPolicy.load(weight)
        env.load_rms(env_rms)
        test_model(env, policy, render=args.render, save_dir="./traj.png")


if __name__ == "__main__":
    main()
