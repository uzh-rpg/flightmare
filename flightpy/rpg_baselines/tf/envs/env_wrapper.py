#!/usr/bin/env python3
import gym
import numpy as np
import time


class EnvWrapper(gym.Env):
    def __init__(self, env):
        self.env = env
        self.env.init()
        self.num_obs = env.getObsDim()
        self.num_act = env.getActDim()

        self._observation_space = gym.spaces.Box(
            np.ones(self.num_obs) * -np.Inf,
            np.ones(self.num_obs) * np.Inf,
            dtype=np.float32)
        # the actions are eventually constrained by the action space.
        self._action_space = gym.spaces.Box(
            low=np.ones(self.num_act) * -1.,
            high=np.ones(self.num_act) * 1.,
            dtype=np.float32)
        self.observation = np.zeros(self.num_obs, dtype=np.float32)
        self.reward = np.float32(0.0)
        self.done = False

        gym.Env.__init__(self)
        #
        self._max_episode_steps = 300

    def seed(self, seed=None):
        self.env.setSeed(seed)

    def step(self, action):
        self.reward = self.env.step(action, self.observation)
        terminal_reward = 0.0
        self.done = self.env.isTerminalState(terminal_reward)
        return self.observation.copy(), self.reward, \
            self.done, [dict(reward_run=self.reward, reward_ctrl=0.0)]

    def reset(self):
        self.reward = 0.0
        self.env.reset(self.observation)
        return self.observation.copy()

    def reset_and_update_info(self):
        return self.reset(),

    def obs(self):
        self.env.getObs(self.observation)
        return self.observation

    def close(self,):
        return True

    def getQuadState(self,):
        quad_state = np.zeros(10, dtype=np.float32)
        self.env.getQuadState(quad_state)
        quad_correct = np.zeros(10, dtype=np.float32)
        quad_correct[0:3] = quad_state[0:3]
        quad_correct[3] = quad_state[9]
        quad_correct[4] = quad_state[6]
        quad_correct[5] = quad_state[7]
        quad_correct[6] = quad_state[8]
        quad_correct[7:10] = quad_state[3:6]
        return quad_correct

    def getGateState(self,):
        gate_state = np.zeros(9, dtype=np.float32)
        self.env.getGateState(gate_state)
        return gate_state

    def connectUnity(self):
        self.env.connectUnity()

    def disconnectUnity(self):
        self.env.disconnectUnity()

    @property
    def observation_space(self):
        return self._observation_space

    @property
    def action_space(self):
        return self._action_space

    @property
    def max_episode_steps(self):
        return self._max_episode_steps

# def main():
#   import os
#   from ruamel.yaml import YAML, dump, RoundTripDumper
#   cfg_path = os.path.abspath("../configs/env.yaml")
#   cfg = YAML().load(open(cfg_path, 'r'))
#   #
#   env = DynamicGate_v0(dump(cfg["env"], Dumper=RoundTripDumper))
#   env = EnvWrapper(env)

#   obs = env.reset()

#   obs = env.obs()
#   print(obs)

#   for i in range(10000):
#     act = np.array([10.0, 0.0, 0.0, 0.0])
#     next_obs, rew, done, _ = env.step(act)
#     time.sleep(0.01)

# if __name__ == "__main__":
#   main()
