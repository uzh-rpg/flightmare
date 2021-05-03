#!/usr/bin/env python3

import numpy as np
import math
from typing import List

class HighLevelPlanner():
  def __init__(self, 
               num_runs: int = 10,
               num_goals_per_run: int = 5, 
               world_box: List[float] = [-15, -15, 0, 15, 15, 8], 
               min_distance_between_consecutive_goals: int = 15,
               switch_goal_distance: float = 3.0,
               ):
    self.num_runs = num_runs
    self.switch_goal_distance = switch_goal_distance 
    self.goals_matrix = np.zeros([num_runs, num_goals_per_run, 3], dtype=np.float32)
    self.velocities_matrix = np.zeros([num_runs, num_goals_per_run], dtype=np.float32)
    np.random.seed(0)
    self.world_box = np.asarray(world_box, dtype=np.float32)
    self.world_box[:3] = self.world_box[:3] + 1.0 
    self.world_box[3:] = self.world_box[3:] - 1.0
    # Need bigger world box than actual size so that some goals are outside.
    self.world_box_dim_factors = np.zeros([3], dtype=np.float32)
    self.world_box_dim_factors[0] = (self.world_box[3] - self.world_box[0]) / 2.0
    self.world_box_dim_factors[1] = (self.world_box[4] - self.world_box[1]) / 2.0
    self.world_box_dim_factors[2] = (self.world_box[5] - self.world_box[2])
    self.distance_to_avoid_corners = np.sqrt(self.world_box_dim_factors[0]**2 + self.world_box_dim_factors[1]**2 + (self.world_box_dim_factors[2]/2)**2)*0.9 
    
    self.num_goals_per_run = num_goals_per_run
    self.internal_state = 0 # Will be used to keep track of goal numbers 
    self.goal_reached_number = 0
    
    self.populate_goals_matrix(num_runs, num_goals_per_run, min_distance_between_consecutive_goals)
        
  def populate_goals_matrix(self, num_runs, num_goals_per_run, min_distance_between_consecutive_goals):
    for run_idx in range(num_runs):
      for goal_idx in range(num_goals_per_run):
        good_goal = False
        while not good_goal:
          random_goal_position = self.draw_random_position()
          
          if (goal_idx==0): # if first goal of episode we only care is far from center [0, 0, 0]
            goals_relative_position = random_goal_position
          else: # else we care two consecutive goals are not too close to each other
            goals_relative_position = random_goal_position - self.goals_matrix[run_idx, goal_idx-1, :]
          if(np.linalg.norm(goals_relative_position) > min_distance_between_consecutive_goals and np.linalg.norm(goals_relative_position) < self.distance_to_avoid_corners): 
            good_goal = True
            self.goals_matrix[run_idx, goal_idx, :] = random_goal_position
    
  def draw_random_position(self):
    random_values = np.random.rand(3)
    random_values[0:2] = random_values[0:2]*2 - 1 # Uniform distribution from -1 and 1
    return np.multiply(random_values, self.world_box_dim_factors)  
  
  def get_current_goal(self, drone_position, num_run):
    goal_relative_position = self.goals_matrix[num_run, self.internal_state, :] - drone_position
    goal_relative_position_norm = np.linalg.norm(goal_relative_position)
    # print("Goal distance: ", goal_relative_position_norm)
    done = False
    
    self.goal_reached_number = self.internal_state
    if(goal_relative_position_norm < self.switch_goal_distance):
      if(self.internal_state == 4): # arrived to last goal of episode
        done = True
        self.internal_state = 0
        self.goal_reached_number = 5
        safe_num_run = (num_run + 1)*(num_run<(self.num_runs-1)) + num_run*(num_run==(self.num_runs-1))
        return self.goals_matrix[safe_num_run, self.internal_state, :], done, float(self.goal_reached_number)
        
      self.internal_state += 1

    return self.goals_matrix[num_run, self.internal_state, :], done, float(self.goal_reached_number)
  
  def to_next_run(self):
    self.internal_state = 0
  
  def reset(self):
    self.internal_state = 0
    
