#!/usr/bin/env python3
import argparse
from ast import arg
import math
from ntpath import join
#
import os
from sqlite3 import Date
import subprocess


import numpy as np
import torch
from flightgym import VisionEnv_v1
from ruamel.yaml import YAML, RoundTripDumper, dump

from rpg_baselines.torch.envs import vec_env_wrapper as wrapper

import cv2
import shutil
import glob
import matplotlib.pyplot as plt 

from poisson_distribution import PoissonDistribution
from trajectory_generator import TrajectoryGenerator

class Dataset():

    def __init__(self, save_dir, render, max_env=1, trajectory_area=80, tree_area=100, waypoint_step=7, waypoint_time=1, num_trajectory=100, 
                    max_poisson_sampling_try=30, sim_dt=0.01):
        self.save_dir = save_dir
        self.sim_dt = sim_dt
        self.render = render
        self.cfg = YAML().load(
                open(
                     os.environ["FLIGHTMARE_PATH"] + "/flightpy/configs/vision/config.yaml", "r"
                )
        )
        if render:
            self.cfg["unity"]["render"] = "yes"
    
        # create evaluation environment
        self.cfg["simulation"]["num_envs"] = 1
        self.env = wrapper.FlightEnvVec(
            VisionEnv_v1(dump(self.cfg, Dumper=RoundTripDumper), False)
        )

        # if render:
        #     self.proc = subprocess.Popen(os.environ["FLIGHTMARE_PATH"] + "/flightrender/RPG_Flightmare.x86_64")
        
        self.trajectory_generator = TrajectoryGenerator(trajectory_area, trajectory_area, waypoint_step, waypoint_time, num_trajectory)
        self.poisson_distribution = PoissonDistribution(5, tree_area, tree_area, max_poisson_sampling_try)

        self.num_trajectory = 0
        self.max_env = max_env
        self.duration = waypoint_time * 2
        self.start_save = 2
        
    def generate_trees(self):
        trajectories = self.trajectory_generator.generate_trajectory()
        self.num_trajectory = self.trajectory_generator.get_num_trajectories()
        assert self.num_trajectory == len(trajectories)
        self.poisson_distribution.load_trajectories(trajectories)
        self.poisson_distribution.create_list()
        trees = self.poisson_distribution.generate_trees()
        return trees

    ''' todo: should clean the old trees'''
    def reset_flightmare_tree(self, trees):
        self.env.setPoissonTrees(trees)
        self.env.connectUnity()
        

    def start_collecting_data(self):
        print("-----------------Starting Collecting Data From Flightmare-------------------")
        os.makedirs(self.save_dir, exist_ok=True)

        # generate collision free trajectories and trees
        for i in range(self.max_env):
            trees = self.generate_trees()
            self.reset_flightmare_tree(trees)
            env_dir = os.path.join(self.save_dir, "environment_{:04d}".format(i))
            os.makedirs(env_dir, exist_ok=True)

            for j in range(self.num_trajectory):
                seq_dir = os.path.join(env_dir, "sequence_{:05d}".format(j))
                os.makedirs(seq_dir, exist_ok=True)
                left_image_dir = os.path.join(seq_dir, "images/left")
                right_image_dir = os.path.join(seq_dir, "images/right")
                depth_img_dir = os.path.join(seq_dir, "depth")
                os.makedirs(left_image_dir, exist_ok=True)
                os.makedirs(right_image_dir, exist_ok=True)
                os.makedirs(depth_img_dir, exist_ok=True)
                
                self.env.reset()       
                t, ep_len, frame_id = 0, 0, 0
                while (t <= self.duration):
                    state = self.trajectory_generator.get_state(j, t)
                    t += self.sim_dt
                    self.env.setQuadState(state[1:25])
                    self.env.render(ep_len)

                    
                    # ======RGB Image=========
                    img_left = self.env.getLeftImage(rgb=True) 
                    rgb_left = np.reshape(
                    img_left[0], (self.env.img_height, self.env.img_width, 3))
                    img_right = self.env.getRightImage(rgb=True) 
                    rgb_right = np.reshape(
                    img_right[0], (self.env.img_height, self.env.img_width, 3))

                    if ep_len > self.start_save:
                        cv2.imwrite(os.path.join(left_image_dir, "frame_{:010d}.png".format(frame_id)), rgb_left)
                        cv2.imwrite(os.path.join(right_image_dir,"frame_{:010d}.png".format(frame_id)), rgb_right)

                        depth_img = np.reshape(self.env.getDepthImage()[
                                        0], (self.env.img_height, self.env.img_width))
                        near = 0.02
                        far = 40
                        # depth_img = depth_img.clip(0.01, 1.0)
                        # print(np.unique(depth_img), np.mean(depth_img))
                        depth_img_save = near + depth_img  * (far - near)
                        np.save(os.path.join(depth_img_dir, "frame_{:010d}.npy".format(frame_id)), depth_img_save)

                        cv2.imshow("depth", depth_img)
                        cv2.imwrite(os.path.join(depth_img_dir, "frame_{:010d}.png".format(frame_id)), depth_img)
                        cv2.waitKey(100)

                        frame_id += 1
                        # cv2.waitKey(100)

                    # # # ======Depth Image=========
                    # depth image from Unity is not real depth, but for visualization purpose
                    # depth_img = np.reshape(self.env.getDepthImage()[
                    #                     0], (self.env.img_height, self.env.img_width))
                    # near = 0.02
                    # far = 40
                    # # depth_img = depth_img.clip(0.01, 1.0)
                    # # print(np.unique(depth_img), np.mean(depth_img))
                    # depth_img_save = near * far / (far - depth_img * (far - near))
                    # np.save(os.path.join(depth_img_dir, "frame_{:010d}.npy".format(frame_id)), depth_img_save)
                    # # print(np.unique(depth_img), np.mean(depth_img))
                    # # depth_img = depth_img * 100000
                    # # depth_img = (depth_img - np.min(depth_img)) / (np.ptp(depth_img)) * 255.0
                    # # depth_img = depth_img.astype(np.uint8)
                    # cv2.imshow("depth", depth_img)
                    # # plt.imshow(depth_img, cmap='inferno')
                    # # plt.pause(100)
                    # cv2.imwrite(os.path.join(depth_img_dir, "frame_{:010d}.png".format(frame_id)), depth_img)
                    # cv2.waitKey(100)
                    ep_len += 1
                
                self.generate_timestamps(seq_dir, frame_id)
        
        if self.render:
            self.env.disconnectUnity()
            self.proc.terminate()
    
    def generate_timestamps(self, dir, frames):
        with open(os.path.join(dir, 'timestamps.txt'), 'w') as f:
            for i in range(0, frames):
                texts = [i * self.sim_dt]
                f.write(" ".join(str(item) for item in texts))
                f.write('\n')
    




# move GT images to the SGM prepation folders and generate timestamps for generating events
def move_to_sgm(save_dir, sim_dt):
    # create folders:
    os.makedirs(os.path.join(save_dir, "sgm/gt/disparities"), exist_ok=True)
    os.makedirs(os.path.join(save_dir, "sgm/gt/disparities_vis"), exist_ok=True)

    images_left = [f for f in glob.glob(os.path.join(save_dir,'left/imgs/*.png'), recursive=False)]
    images_right = [f for f in glob.glob(os.path.join(save_dir,'right/imgs/*.png'), recursive=False)]

    # prepare timestamps for events generation
    with open(os.path.join(save_dir, 'left/timestamps.txt'), 'w') as f:
        for i in range(0, len(images_left)):
            texts = [i * sim_dt]
            f.write(" ".join(str(item) for item in texts))
            f.write('\n')

    with open(os.path.join(save_dir, 'right/timestamps.txt'), 'w') as f:
        for i in range(0, len(images_right)):
            texts = [i * sim_dt]
            f.write(" ".join(str(item) for item in texts))
            f.write('\n')
    
    # link files for SGM generation
    os.symlink(os.path.join(save_dir, 'left/imgs'), os.path.join(save_dir, 'sgm/gt/left'))
    os.symlink(os.path.join(save_dir, 'right/imgs'), os.path.join(save_dir, 'sgm/gt/right'))


if __name__ == "__main__":
    parser = argparse.ArgumentParser("""Collect Training Sequences from Flightmare""")
    parser.add_argument("--save_dir", "-s", default="", required=False)
    parser.add_argument("--render", dest="render", action="store_true")
    parser.set_defaults(render=False)
    parser.add_argument("--sim_dt", type=float, default=0.01, required=False)
    args = parser.parse_args()

    # start_data_generation(args.save_dir, args.render, args.sim_dt)
    # move_to_sgm(args.save_dir, args.sim_dt)
    dataset = Dataset("/home/chaoni/drive_data/test_depth_2", True)
    dataset.start_collecting_data()
