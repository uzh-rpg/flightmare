#!/usr/bin/env python3
from ruamel.yaml import YAML, dump, RoundTripDumper

#
import os
import math
import argparse
import numpy as np
import tensorflow as tf

#
from stable_baselines import logger

#
from rpg_baselines.common.policies import MlpPolicy
from rpg_baselines.ppo.ppo2 import PPO2
from rpg_baselines.ppo.ppo2_test import test_model
from rpg_baselines.envs import vec_env_wrapper as wrapper
import rpg_baselines.common.util as U
#
from flightgym import QuadrotorEnv_v1


def configure_random_seed(seed, env=None):
    if env is not None:
        env.seed(seed)
    np.random.seed(seed)
    tf.set_random_seed(seed)


def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--train', type=int, default=1,
                        help="To train new model or simply test pre-trained model")
    parser.add_argument('--render', type=int, default=0,
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
    if not args.train:
        cfg["env"]["num_envs"] = 1
        cfg["env"]["num_threads"] = 1

    if args.render:
        cfg["env"]["render"] = "yes"
    else:
        cfg["env"]["render"] = "no"

    env = wrapper.FlightEnvVec(QuadrotorEnv_v1(
        dump(cfg, Dumper=RoundTripDumper), False))

    # set random seed
    configure_random_seed(args.seed, env=env)

    #
    if args.train:
        # save the configuration and other files
        rsg_root = os.path.dirname(os.path.abspath(__file__))
        log_dir = rsg_root + '/saved'
        saver = U.ConfigurationSaver(log_dir=log_dir)
        model = PPO2(
            tensorboard_log=saver.data_dir,
            policy=MlpPolicy,  # check activation function
            policy_kwargs=dict(
                net_arch=[dict(pi=[128, 128], vf=[128, 128])], act_fun=tf.nn.relu),
            env=env,
            lam=0.95,
            gamma=0.99,  # lower 0.9 ~ 0.99
            # n_steps=math.floor(cfg['env']['max_time'] / cfg['env']['ctl_dt']),
            n_steps=250,
            ent_coef=0.00,
            learning_rate=3e-4,
            vf_coef=0.5,
            max_grad_norm=0.5,
            nminibatches=1,
            noptepochs=10,
            cliprange=0.2,
            verbose=1,
        )

        # tensorboard
        # Make sure that your chrome browser is already on.
        # TensorboardLauncher(saver.data_dir + '/PPO2_1')

        # PPO run
        # Originally the total timestep is 5 x 10^8
        # 10 zeros for nupdates to be 4000
        # 1000000000 is 2000 iterations and so
        # 2000000000 is 4000 iterations.
        logger.configure(folder=saver.data_dir)
        model.learn(
            total_timesteps=int(25000000),
            log_dir=saver.data_dir, logger=logger)
        model.save(saver.data_dir)

    # # Testing mode with a trained weight
    else:
        model = PPO2.load(args.weight)
        test_model(env, model, render=args.render)


if __name__ == "__main__":
    main()
