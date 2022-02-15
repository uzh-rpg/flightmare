import os
import pickle
from copy import deepcopy
from typing import Any, Callable, List, Optional, Sequence, Type, Union

import gym
import numpy as np
from gym import spaces
from numpy.core.fromnumeric import shape
from stable_baselines3.common.running_mean_std import RunningMeanStd
from stable_baselines3.common.vec_env.base_vec_env import (VecEnv,
                                                           VecEnvIndices,
                                                           VecEnvObs,
                                                           VecEnvStepReturn)
from stable_baselines3.common.vec_env.util import (copy_obs_dict, dict_to_obs,
                                                   obs_space_info)


class FlightEnv(object):
    #
    def __init__(self, impl):
        self.wrapper = impl
        self.obs_dim = self.wrapper.getObsDim()
        self.act_dim = self.wrapper.getActDim()
        self.rew_dim = self.wrapper.getRewDim()
        self.num_gates = self.wrapper.getNumGates()
        self._observation_space = spaces.Box(
            np.ones(self.obs_dim) * -np.Inf,
            np.ones(self.obs_dim) * np.Inf,
            dtype=np.float64,
        )
        self._action_space = spaces.Box(
            low=np.ones(self.act_dim) * -1.0,
            high=np.ones(self.act_dim) * 1.0,
            dtype=np.float64,
        )
        #
        self._observation = np.zeros([self.obs_dim], dtype=np.float64)
        #
        self._reward_components = np.zeros([self.rew_dim, 1], dtype=np.float64)
        self._done = False

        self._quadstate = np.zeros([44], dtype=np.float64)
        self._quadact = np.zeros([4], dtype=np.float64)
        self._gatepose = np.zeros([7 * self.num_gates, 1], dtype=np.float64)
        self._gatecorners = np.zeros([24 * self.num_gates, 1], dtype=np.float64)
        self._nextgatecorners = np.zeros([24, 1], dtype=np.float64)
        self._frontgatecorner = np.zeros([12, 1], dtype=np.float64)
        self._quadact = np.zeros([4], dtype=np.float64)
        self._flightmodes = np.zeros([1], dtype=np.float64)

        #  state normalization
        self.obs_rms = RunningMeanStd(shape=[1, self.obs_dim])
        self.obs_rms_new = RunningMeanStd(shape=[1, self.obs_dim])

    def step(self, action):
        done = self.wrapper.step(action, self._observation, self._reward_components)
        # print(done)
        obs_u = np.reshape(self._observation, (1, self.obs_dim))
        obs = self.normalize_obs(obs_u)
        return obs, obs_u, done

    def reset(self, random=True):
        self._reward_components = np.zeros([self.rew_dim, 1], dtype=np.float64)
        self.wrapper.reset(self._observation, random)
        # reshape, because Eigen is colum-major and numpy is row major
        obs = np.reshape(self._observation, (1, self.obs_dim))
        #
        obs = self.normalize_obs(obs)
        return obs

    def getObs(self):
        self.wrapper.getObs(self._observation)
        return self.normalize_obs(self._observation)

    def get_obs_norm(self):
        return self.obs_rms.mean, self.obs_rms.var

    def getProgress(self):
        return self._reward_components[:, 0]

    def getNumGates(self):
        return self.wrapper.getNumGates()

    def _normalize_obs(self, obs: np.ndarray, obs_rms: RunningMeanStd) -> np.ndarray:
        return (obs - obs_rms.mean) / np.sqrt(obs_rms.var + 1e-8)

    def normalize_obs(self, obs: np.ndarray) -> np.ndarray:
        """
        Normalize observations using this VecNormalize's observations statistics.
        Calling this method does not update statistics.
        """
        # Avoid modifying by reference the original object
        # obs_ = deepcopy(obs)
        obs_ = self._normalize_obs(obs, self.obs_rms).astype(np.float64)
        return obs_

    def connectUnity(self):
        self.wrapper.connectUnity()

    def disconnectUnity(self):
        self.wrapper.disconnectUnity()

    def render(self, mode="human"):
        return self.wrapper.updateUnity()

    def setQuadState(self, quadstate, t=0.0):
        self.wrapper.setQuadState(quadstate)

    def setQuadAct(self, quadact, t=0.0):
        self.wrapper.setQuadAct(quadact)

    def setGateID(self, gate_id):
        self.wrapper.setGateID(gate_id)

    def getQuadState(self):
        self.wrapper.getQuadState(self._quadstate)
        return self._quadstate.copy()


    def getQuadAct(self):
        self.wrapper.getQuadAct(self._quadact)
        return self._quadact.copy()


    @property
    def observation_space(self):
        return self._observation_space

    @property
    def action_space(self):
        return self._action_space

    def load_rms(self, data_dir) -> None:
        self.mean, self.var = None, None
        np_file = np.load(data_dir)
        #
        self.mean = np_file["mean"]
        self.var = np_file["var"]
        #
        self.obs_rms.mean = np.mean(self.mean, axis=0)
        self.obs_rms.var = np.mean(self.var, axis=0)
