#!/usr/bin/env python3

import numpy as np
import math
from typing import List

class ObstacleAvoidanceAgent():
  def __init__(self, 
               num_envs : int = 1,
               num_acts : int = 4,
               ):
    self._num_envs = num_envs
    self._num_acts = num_acts
    
    # initialization
        
  def getActions(self, obs, done, images):
    action = np.zeros([self._num_envs,self._num_acts], dtype=np.float32)
    action[0,0] += -0.01
    return action