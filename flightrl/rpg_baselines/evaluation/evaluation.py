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
import pickle
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


#
from stable_baselines3.common import logger

#
from stable_baselines3.ppo.ppo import PPO
from rpg_baselines.ppo.ppo2_test import test_model
import rpg_baselines.common.util as U
from rpg_baselines.envs import vec_env_wrapper as wrapper
from scipy.spatial.transform import Rotation 
from utils import skip_first_few_episodes
from high_level_planner import HighLevelPlanner
#
from flightgym import QuadrotorEnv_v1

# create a class with an imposed API and put it int he same folder of this python script or specify the path relative to flightmare base folder

def configure_random_seed(seed, env=None):
    if env is not None:
        env.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    
def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, default=0,
                        help="Random seed")
    parser.add_argument('--policy_path', type=str,
                        default="")
    return parser

def main():   
    args = parser().parse_args()
    cfg_vec_env = YAML().load(open(os.environ["FLIGHTMARE_PATH"] +
                           "/flightlib/configs/vec_env.yaml", 'r'))
    
    if args.policy_path:
      policy_path = os.path.join(os.environ["FLIGHTMARE_PATH"],args.policy_path)
      sys.path.insert(1, policy_path)
      from policy_path import ObstacleAvoidanceAgent
    else:
      from obstacle_avoidance_agent import ObstacleAvoidanceAgent
            
    cfg_vec_env["env"]["render"] = "yes"
    cfg_vec_env["env"]["num_envs"] = 1
    cfg_vec_env["env"]["num_threads"] = 1    
            
    env = wrapper.FlightEnvVec(QuadrotorEnv_v1(dump(cfg_vec_env, Dumper=RoundTripDumper), False))
        
    configure_random_seed(args.seed, env=env)
            
    connectedToUnity = False 
    while not connectedToUnity:
        connectedToUnity = env.connectUnity()             
        if not connectedToUnity:  
            print("Couldn't connect to unity, will try another time.")    
            
    obstacle_avoidance_agent = ObstacleAvoidanceAgent(num_envs=env.num_envs, num_acts=env.num_acts)                    
    
    # set object density to zero 
    object_density_fractions = np.zeros([env.num_envs], dtype=np.float32)
    env.set_objects_densities(object_density_fractions = object_density_fractions)
    
    # Wait for Unity to be fully ready
    time.sleep(5.0)
    
    # Need to skip episodes at beginning because of a bug in collision detection due to slow speed at the beginning.
    skip_first_few_episodes(env=env, episodes_to_skip=3, episode_length=4)
    
    num_rollouts_per_density = 10
    num_density_values = 10
    num_rollouts = num_density_values * num_rollouts_per_density
    n_roll = 0
    max_ep_length = 5000
    
    high_level_planner = HighLevelPlanner(num_runs= num_rollouts_per_density,
                                          num_goals_per_run = 5,
                                          world_box = [-15, -15, 0, 15, 15, 8], 
                                          min_distance_between_consecutive_goals = 10,
                                          switch_goal_distance = 3)
    
    object_density_fractions_initial = np.linspace(0, 1.0, num=num_density_values, dtype=np.float32)
    object_density_fractions_different_episodes = np.repeat(object_density_fractions_initial, num_rollouts_per_density)
    object_density_fractions_different_episodes[(num_rollouts_per_density-1):-1] = object_density_fractions_different_episodes[num_rollouts_per_density:]
    env.set_objects_densities(object_density_fractions = object_density_fractions_initial[0].reshape(1, -1))
    
    episodes_terminal_goal_number = np.zeros([num_rollouts], dtype=np.float32)
    episodes_survived = np.zeros([num_rollouts], dtype=np.float32)
    episodes_lengths = np.zeros([num_rollouts], dtype=np.float32)
    
    while n_roll < num_rollouts:
        drone_pos, drone_vel, euler, deuler, goal_pos, reward_instant = [], [], [], [], [], []
        actions = []     
        done, done_from_high_level_planner, ep_len = False, False, 0
        if (n_roll == 0):
            obs = env.reset()
            images = env.get_images()
        # get current goal position 
        drone_pos = obs[0, :3]
        current_goal , _, _ = high_level_planner.get_current_goal(drone_position=drone_pos, num_run=int(n_roll%num_rollouts_per_density))
            
        # Single episode until termination.
        while not (done or done_from_high_level_planner or (ep_len >= max_ep_length)):
            actions = obstacle_avoidance_agent.getActions(obs, done, images, current_goal)

            if ep_len == 5:
                env.set_objects_densities(object_density_fractions= object_density_fractions_different_episodes[n_roll].reshape(1, -1)) 
            
            obs, rew, done, infos = env.step(actions)
            images = env.get_images()  
            ep_len += 1
        
            drone_pos = obs[0, :3]
            current_goal , done_from_high_level_planner, high_level_planner_goal_reached_number = high_level_planner.get_current_goal(drone_position=drone_pos, num_run=int(n_roll%num_rollouts_per_density))      
            
            
            if done:
                episodes_terminal_goal_number[n_roll] = high_level_planner_goal_reached_number
                high_level_planner.to_next_run()      
                    
            if done_from_high_level_planner:
                obs = env.reset()
                images = env.get_images()
                current_goal , _ , _ = high_level_planner.get_current_goal(drone_position=drone_pos, num_run=int(n_roll%num_rollouts_per_density))      
                episodes_terminal_goal_number[n_roll] = high_level_planner_goal_reached_number
                episodes_survived[n_roll] = 1
                
        episodes_lengths[n_roll] = ep_len
        n_roll = n_roll + 1
        
        
        
    total_success_rate =  np.mean(episodes_survived)
    
    txt_file_path = "./evaluation_result.txt"
    txt_file = open(txt_file_path, "w") 
    
    txt_file.write("total_success_rate : \t")
    txt_file.writelines(str(total_success_rate))
    
    for density_idx in range(num_density_values):
        range_start = density_idx*num_rollouts_per_density
        range_end = density_idx*num_rollouts_per_density + num_rollouts_per_density
        txt_file.write("\ndensity_value                  : \t")
        txt_file.writelines(str(object_density_fractions_initial[density_idx]))   
        success_rate =  np.mean(episodes_survived[range_start:range_end])
        txt_file.write("\nsuccess_rate_this_density      : \t")
        txt_file.writelines(str(success_rate))
        txt_file.write("\nepisodes_results               : \t")
        [txt_file.writelines(str(episodes_survived[n_roll]) + "\t\t")                         for n_roll in range(range_start,range_end)]
        txt_file.write("\nepisodes_lengths               : \t")
        [txt_file.writelines(str("{:.7f}".format(episodes_lengths[n_roll]))[:9] + "\t")      for n_roll in range(range_start,range_end)]
        
        txt_file.write("\nnumber_goal_reached            : \t")
        [txt_file.writelines(str("{:.7f}".format(episodes_terminal_goal_number[n_roll]))[:9] + "\t") for n_roll in range(range_start,range_end)]
            
    txt_file.close()
    
    pickle_file_path = os.path.join("./evaluation_result.pickle")
    # Save rollout info in a pickle file for latter use.
    data_dict = {
        'num_density_values'                : num_density_values,
        'object_density_fractions'          : object_density_fractions_initial.tolist(),
        'num_rollouts_per_density'          : num_rollouts_per_density,
        'num_rollouts'                      : num_rollouts,
        'object_density_fractions_initial'  : object_density_fractions_initial.tolist(),
        'total_success_rate'                : total_success_rate.tolist(),
        'episodes_terminal_goal_number'     : episodes_terminal_goal_number.tolist(),
        'episodes_results'                  : episodes_survived.tolist()
    }

    pickle_file = open(pickle_file_path, "wb")
    pickle.dump(data_dict, pickle_file)
    pickle_file.close()
    
    graphs_x_axis = object_density_fractions_initial
   
    # survival_on_density
    episodes_results = episodes_survived

    # mean_goal_reached_on_density
    episodes_terminal_goal_number = np.asarray(episodes_terminal_goal_number, dtype=np.float32)
    
    survival_on_density_y_axis                      = np.zeros([num_density_values], dtype=np.float32)
    mean_goal_reached_on_density_y_axis             = np.zeros([num_density_values], dtype=np.float32)
    
    range_start = 0
    range_end = 0
    
    for density_idx in range(num_density_values):
        range_start = density_idx*num_rollouts_per_density 
        range_end = density_idx*num_rollouts_per_density + num_rollouts_per_density 
        
        if range_end!=num_density_values*num_density_values:
            survival_on_density_y_axis[density_idx]                      = np.mean(episodes_results[range_start:range_end])      
            mean_goal_reached_on_density_y_axis[density_idx]             = np.mean(episodes_terminal_goal_number[range_start:range_end])
        else:
            survival_on_density_y_axis[density_idx]                      = np.mean(episodes_results[range_start:])
            mean_goal_reached_on_density_y_axis[density_idx]             = np.mean(episodes_terminal_goal_number[range_start:])
    matplotlib.use('Agg')    
    fig = plt.figure(figsize=(18, 12), tight_layout=True)
    gs = gridspec.GridSpec(nrows=2, ncols=2)
    survival_on_density                       = fig.add_subplot(gs[0, 0])
    mean_goal_reached_on_density              = fig.add_subplot(gs[0, 1])

    survival_on_density.plot(graphs_x_axis, survival_on_density_y_axis, 'bo-')
    survival_on_density.set_title('Survival rate on density')
    survival_on_density.set_xlabel('Object density')
    survival_on_density.set_ylabel('Survival rate')
    survival_on_density.grid()
    survival_on_density.set_ylim((0,1.1))
    
    mean_goal_reached_on_density.plot(graphs_x_axis, mean_goal_reached_on_density_y_axis, 'co-')
    mean_goal_reached_on_density.set_title('Mean number of goals reached on density')
    mean_goal_reached_on_density.set_xlabel('Object density')
    mean_goal_reached_on_density.set_ylabel('Mean number of goal reached')
    mean_goal_reached_on_density.grid()
    mean_goal_reached_on_density.set_ylim((0, 1.1*np.max(mean_goal_reached_on_density_y_axis)))
       
    plt.tight_layout()
    plt.savefig("./evaluation_result_graphs.png")
    # plt.show()
        

if __name__ == "__main__":
    main()
