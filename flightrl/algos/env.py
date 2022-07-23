import numpy as np
from gym import spaces
# from stable_baselines.common.vec_env import VecEnv
from rpg_baselines.envs.vec_env_wrapper import FlightEnvVec
# from flightgym import QuadrotorEnv_v1

class DroneReachTarget(FlightEnvVec):
    def __init__(self, impl):
        super().__init__(impl)
        self.num_obs += 6
        print("new observations = ",self.num_obs, self.num_acts, self.num_envs)
        
        self._observation_space = spaces.Box(
            np.ones(self.num_obs) * -np.Inf,
            np.ones(self.num_obs) * np.Inf, dtype=np.float32)
        self._action_space = spaces.Box(
            low=np.ones(self.num_acts) * -1.,
            high=np.ones(self.num_acts) * 1.,
            dtype=np.float32)

        self._observation = np.zeros([self.num_envs, self.num_obs],
                                     dtype=np.float32)
        
        self._extraInfoNames = self.wrapper.getExtraInfoNames()
        self._extraInfo = np.zeros([self.num_envs,
                                    len(self._extraInfoNames)], dtype=np.float32)
        self.rewards = [[] for _ in range(self.num_envs)]

        self.max_episode_steps = 300