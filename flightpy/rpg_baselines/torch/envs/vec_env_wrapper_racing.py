import numpy as np
from gym import spaces
from stable_baselines3.common.vec_env import VecEnv
from ruamel.yaml import YAML
import os
from collections import deque
import random
import gym
from gym import spaces
from typing import Any, Callable, List, Optional, Sequence, Type, Union
from stable_baselines3.common.vec_env.base_vec_env import VecEnv, VecEnvIndices, VecEnvObs, VecEnvStepReturn

TRACK_CONFIG_DIR = os.environ["FLIGHTMARE_PATH"] + \
    "/flightpy/configs/racing/tracks"


class FlightEnvVec(VecEnv):
    def __init__(self, impl, cfg):

        # Set config
        self.cfg = cfg
        self.track_cfgs = \
            [YAML().load(open(os.path.join(TRACK_CONFIG_DIR,
                                           track + '.yaml'), 'r'))
             for track in cfg['main']['tracks']]

        # Set wrapper object -> PyBind object
        self.wrapper = impl

        self.dt = self.cfg['environment']['sim_dt']

        ##########################
        # Observation and Action #
        ##########################

        self.num_obs = impl.getObsDim()
        self.num_acts = impl.getActDim()
        self.img_width = self.wrapper.getImgWidth()
        self.img_height = self.wrapper.getImgHeight()

        # Get base class parameters
        self._observation_space = spaces.Box(
            np.ones(self.num_obs) * -np.Inf,
            np.ones(self.num_obs) * np.Inf, dtype=np.float32)

        self._action_space = spaces.Box(
            low=np.ones(self.num_acts) * -1.,
            high=np.ones(self.num_acts) * 1.,
            dtype=np.float32)

        ##############
        # Containers #
        ##############

        self.rew_dim = self.wrapper.getRewDim()
        self.num_tracks = self.wrapper.getTracksN()
        self.num_segments = np.zeros(self.num_tracks, dtype=np.float32)
        self.wrapper.getSegmentsN(self.num_segments)
        self.num_segments_total = int(np.sum(self.num_segments))
        self.num_segments = self.num_segments.astype(np.int)
        self.cum_path_length = None
        self.reward_components_names = self.wrapper.getRewComponentsNames()

        self._rgb_img_obs = np.zeros(
            [self.num_envs, self.img_width * self.img_height * 3], dtype=np.uint8)
        self._gray_img_obs = np.zeros(
            [self.num_envs, self.img_width * self.img_height], dtype=np.uint8)

        # observation normalization
        self.norm_obs = self.cfg['environment']['norm_obs']
        self.obs_mean = np.zeros(self.num_obs, dtype=np.float32)
        self.obs_std = np.ones(self.num_obs, dtype=np.float32)

        # Data containers (step function)
        self._observation = np.zeros([self.num_envs, self.num_obs],
                                     dtype=np.float32)
        self._reward = np.zeros(self.num_envs, dtype=np.float32)
        self._done = np.zeros(self.num_envs, dtype=np.bool)
        self._extraInfoNames = self.wrapper.getExtraInfoNames()
        self._extraInfo = np.zeros([self.num_envs,
                                    len(self._extraInfoNames)],
                                   dtype=np.float32)

        # Data containers (tensorboard logging)
        self.thrusts = np.zeros([self.num_envs, self.num_acts],
                                dtype=np.float32)
        self.success_ratios = np.zeros([self.num_envs, self.num_segments_total],
                                       dtype=np.float32)
        self._reward_components = np.zeros([self.num_envs, self.rew_dim],
                                           dtype=np.float32)
        self.rewards = [[] for _ in range(self.num_envs)]
        self.lap_rewards = None  # cumulative reward of successful laps
        self._lap_success_buffer = deque(maxlen=100)

        ###################
        # Reward Function #
        ###################

        rew_cfg = self.cfg['rewards']
        self.omega_max = np.array(cfg['quadrotor_dynamics']['omega_max'])
        rew_coefs = np.array(rew_cfg['coefs_init'])
        self.rew_coefs = rew_coefs.astype(np.float32)
        max_pen_coefs = np.array([self.action_space.high[0] -
                                  self.action_space.low[0]] +
                                 cfg['quadrotor_dynamics']['omega_max'])
        self.max_pen_coefs = max_pen_coefs.astype(np.float32)
        self.rew_coefs_init = self.rew_coefs.copy()

        ##############
        # Evaluation #
        ##############

        self.eval_crash_rate = 1.0
        self.best_lap = np.full(self.num_tracks, np.inf)
        self.save_best = False
        self.best_crash_ratio = np.full(self.num_tracks, 1.0)
        self.best_avg_lap = np.full(self.num_tracks, np.inf)
        self.save_robust = False

    def seed(self, seed=0):
        self.wrapper.setSeed(seed)

    def step(self, action):

        # Perform single environment step
        self.wrapper.step(action, self._observation,
                          self._reward_components, self._done, self._extraInfo)

        # Multiply reward components by coefficients
        self._reward_components[:, 0] *= self.rew_coefs[0]
        self._reward_components[:, 1] *= self.rew_coefs[1]
        self._reward_components[:, 2] *= self.rew_coefs[2]
        self._reward_components[:, 4] *= self.rew_coefs[4]
        self._reward_components[:, 5] *= self.rew_coefs[5]
        self._reward_components[:, 6] *= self.rew_coefs[6]
        self._reward_components[:, 7] *= self.rew_coefs[7]
        zero_mask = np.abs(self._reward_components[:, 3]) >= self.rew_coefs[3]
        self._reward_components[:, 3] += self.rew_coefs[3]
        self._reward_components[:, 3] *= zero_mask

        # Get scalar reward by summing already weighted reward components
        # NOTE: Raw components only returned for visualization purposes
        self._reward = np.sum(self._reward_components, axis=1)

        # Collect extra information from environment
        if len(self._extraInfoNames) is not 0:
            info = [{'extra_info': {
                self._extraInfoNames[j]: self._extraInfo[i, j]
                for j in range(0, len(self._extraInfoNames))
            }} for i in range(self.num_envs)]
        else:
            info = [{} for _ in range(self.num_envs)]

        # Add reward components to information
        for i, info_dict in enumerate(info):
            info_dict.update({'reward_components':
                              self._reward_components[i].copy()})

        # Collect reward information after successful lap
        lap_times = np.array([inf['extra_info']['lap_time'] for
                              inf in info])
        lap_envs = np.where(lap_times > 0)[0]
        self.lap_rewards = np.array([sum(self.rewards[i]) for i in lap_envs])

        # Get indicator of successful or failed lap attempt
        modes = np.array([inf['extra_info']['mode'] for
                          inf in info])
        n_success = np.sum(np.logical_and(self._done, lap_times > 0))
        n_fail = np.sum(np.logical_and(self._done, modes > 1))
        result = [1]*n_success + [0]*n_fail
        random.shuffle(result)
        self._lap_success_buffer.extend(result)

        # Collect information from all environments at episode end
        for i in range(self.num_envs):
            self.rewards[i].append(self._reward[i])
            if self._done[i]:
                # Get episode length and accumulated reward
                eprew = sum(self.rewards[i])
                eplen = len(self.rewards[i])

                # Get environment mode and target waypoint at final step
                mode = int(self._extraInfo[i,
                                           self._extraInfoNames.index('mode')])
                target_gate = \
                    int(self._extraInfo[i, self._extraInfoNames
                                        .index('target_gate')])

                track_id = \
                    int(self._extraInfo[i, self._extraInfoNames
                                        .index('track_id')])

                # Construct information dict about full episodes
                epinfo = {"r": eprew, "l": eplen, "target_gate": target_gate,
                          "mode": mode, "track_id": track_id}
                info[i]['episode'] = epinfo

                # Clear rewards after terminated episode
                self.rewards[i].clear()

        return self._observation.copy(), self._reward.copy(), \
            self._done.copy(), info.copy()

    def env_method(self, method_name: str, *method_args, indices: VecEnvIndices = None, **method_kwargs) -> List[Any]:
        """Call instance methods of vectorized environments."""
        target_envs = self._get_target_envs(indices)
        return [getattr(env_i, method_name)(*method_args, **method_kwargs) for env_i in target_envs]

    def env_is_wrapped(self, wrapper_class: Type[gym.Wrapper], indices: VecEnvIndices = None) -> List[bool]:
        """Check if worker environments are wrapped with a given wrapper"""
        target_envs = self._get_target_envs(indices)
        # Import here to avoid a circular import
        from stable_baselines3.common import env_util
        return [env_util.is_wrapped(env_i, wrapper_class) for env_i in target_envs]

    def _get_target_envs(self, indices: VecEnvIndices) -> List[gym.Env]:
        indices = self._get_indices(indices)
        return [self.envs[i] for i in indices]

    def sample_actions(self):
        actions = []
        for i in range(self.num_envs):
            action = self.action_space.sample().tolist()
            actions.append(action)
        return np.asarray(actions, dtype=np.float32)

    def reset(self):
        self._reward = np.zeros(self.num_envs, dtype=np.float32)
        self.wrapper.reset(self._observation)
        self.cum_path_length = self.wrapper.getCumPathLength()
        return self._observation.copy()

    def reset_and_update_info(self):
        return self.reset(), self._update_epi_info()

    def get_thrusts(self):
        self.wrapper.getThrusts(self.thrusts)
        return self.thrusts.copy()

    def get_success_ratios(self):
        self.wrapper.getSuccessRatios(self.success_ratios)

    def get_obs_norm(self):
        """
        Observation Stats computed on 1000 episodes on Sample_Universal tracks
        """
        if self.norm_obs:
            if self.cfg['environment']['obs_type'] == 0:
                # Spherical observation with single gate
                self.obs_mean = np.array([0.0, 0.0, 0.0,
                                          1.07011459e-01, -2.47773334e-02, - 1.85360839e-02, 2.51834515e-02, 4.97261903e-02, -
                                          9.06418170e-01, 1.07947802e-02,  9.26252975e-02, 1.71195511e-01,
                                          2.16440502e+01, 1.57523139e+00, 4.42573793e-01,
                                          1.56897334e-01, 2.96680140e-02, - 3.68501257e-02,
                                          4.26438660e+01, 1.55373473e+00, - 2.71635039e-04,
                                          6.31064434e+00, 1.56819367e+00, 3.95790566e-01,
                                          5.84836904e-03])
                self.obs_std = np.array([1.0, 1.0, 1.0,
                                         0.65923946, 0.67997511, 0.30102705, 0.28505248, 0.28686752, 0.10835089, 0.68697547, 0.66608064, 0.21537794,
                                         3.72837596, 0.12523772, 1.73681841,
                                         2.52205183, 5.08121603, 0.48151847,
                                         14.5214218, 0.34217968, 1.8115009,
                                         3.75226511, 0.11976621, 1.7519917,
                                         0.51595242])
                # Cartesian observation with two gates
                # self.obs_mean = np.array([0.0, 0.0, 0.0,
                #                           1.20321364e-01, - 3.99321160e-02, 3.14818905e-01, 5.00718357e-02, 9.15861160e-02, - 8.56695652e-01, 9.10456185e-03, 1.01671484e-01, 1.67833907e-01,
                #                           2.63248775e-01, 5.31829158e+00, - 1.46861101e-01,
                #                           1.26314752e-01, 4.57233827e-02, - 1.49748777e-02,
                #                           1.29848990e-01, - 1.86704021e+00,  8.03409934e-02,
                #                           9.75442508e-02, 1.40615197e+00, - 1.48750018e-02, 1.12112101e+01, 2.38218201e-02, 1.05243169e-02,
                #                           - 1.10701254e-03, - 3.69734824e-03])
                # self.obs_std = np.array([1.0, 1.0, 1.0,
                #                          0.63043715, 0.64384149, 0.26985201, 0.35200189, 0.33701013, 0.13297337, 0.6793836, 0.67198091, 0.21972388,
                #                          15.22073655, 15.00404513, 2.68547184,
                #                          2.94988804, 4.83163756, 0.61709752,
                #                          30.88432713, 30.43385818, 11.94339087,
                #                          5.09003777, 5.1061406, 0.87027306, 3.56689216, 6.53559402, 1.37165265,
                #                          0.54447845, 0.57861471])
            elif self.cfg['environment']['obs_type'] == 1:
                self.obs_mean = np.array([  # 0.0, 0.0, 0.0,
                    1.07011459e-01, -2.47773334e-02, - 1.85360839e-02, 2.51834515e-02, 4.97261903e-02, - \
                    9.06418170e-01, 1.07947802e-02,  9.26252975e-02, 1.71195511e-01,
                    2.16440502e+01, 1.57523139e+00, 4.42573793e-01,
                    1.56897334e-01, 2.96680140e-02, - 3.68501257e-02,
                    4.26438660e+01, 1.55373473e+00, - 2.71635039e-04,
                    6.31064434e+00, 1.56819367e+00, 3.95790566e-01, 1.33197424e+01, 1.57048723e+00, - 1.37570336e-03,
                    5.84836904e-03, 7.96373830e-03])
                self.obs_std = np.array([  # 1.0, 1.0, 1.0,
                    0.65923946, 0.67997511, 0.30102705, 0.28505248, 0.28686752, 0.10835089, 0.68697547, 0.66608064, 0.21537794,
                    3.72837596, 0.12523772, 1.73681841,
                    2.52205183, 5.08121603, 0.48151847,
                    14.5214218, 0.34217968, 1.8115009,
                    3.75226511, 0.11976621, 1.7519917, 2.60184969, 0.10980724, 0.57087237,
                    0.51595242, 0.57504733])
            elif self.cfg['environment']['obs_type'] == 2:
                self.obs_mean = np.array([0.0, 0.0, 0.0,
                                          1.07011459e-01, -2.47773334e-02, - 1.85360839e-02, 2.51834515e-02, 4.97261903e-02, -
                                          9.06418170e-01, 1.07947802e-02,  9.26252975e-02, 1.71195511e-01,
                                          2.16440502e+01, 1.57523139e+00, 4.42573793e-01,
                                          1.56897334e-01, 2.96680140e-02, - 3.68501257e-02,
                                          4.26438660e+01, 1.55373473e+00, - 2.71635039e-04,
                                          6.31064434e+00, 1.56819367e+00, 3.95790566e-01, 1.33197424e+01, 1.57048723e+00, -
                                          1.37570336e-03, 1.33197424e+01, 1.57048723e+00, - 1.37570336e-03,
                                          5.84836904e-03, 7.96373830e-03, 7.96373830e-03])
                self.obs_std = np.array([1.0, 1.0, 1.0,
                                         0.65923946, 0.67997511, 0.30102705, 0.28505248, 0.28686752, 0.10835089, 0.68697547, 0.66608064, 0.21537794,
                                         3.72837596, 0.12523772, 1.73681841,
                                         2.52205183, 5.08121603, 0.48151847,
                                         14.5214218, 0.34217968, 1.8115009,
                                         3.75226511, 0.11976621, 1.7519917, 2.60184969, 0.10980724, 0.57087237, 2.60184969, 0.10980724, 0.57087237,
                                         0.51595242, 0.57504733, 0.57504733])
            else:
                print('No suitable mean provided.')
                exit()

        return self.obs_mean.copy(), self.obs_std.copy()

    def _update_epi_info(self):
        info = [{} for _ in range(self.num_envs)]

        for i in range(self.num_envs):
            eprew = sum(self.rewards[i])
            eplen = len(self.rewards[i])
            epinfo = {"r": eprew, "l": eplen}
            info[i]['episode'] = epinfo
            self.rewards[i].clear()
        return info

    def getImage(self, rgb=False):
        if (rgb):
            self.wrapper.getImage(self._rgb_img_obs, True)
            return self._rgb_img_obs.copy()
        else:
            self.wrapper.getImage(self._gray_img_obs, False)
            return self._gray_img_obs.copy()

    def render(self, mode='human'):
        return self.wrapper.updateUnity()

    def close(self):
        self.wrapper.close()

    def connectUnity(self):
        self.wrapper.connectUnity()

    def disconnectUnity(self):
        self.wrapper.disconnectUnity()

    @property
    def num_envs(self):
        return self.wrapper.getNumOfEnvs()

    @property
    def observation_space(self):
        return self._observation_space

    @property
    def action_space(self):
        return self._action_space

    @property
    def extra_info_names(self):
        return self._extraInfoNames

    def start_recording_video(self, file_name):
        raise RuntimeError('This method is not implemented')

    def stop_recording_video(self):
        raise RuntimeError('This method is not implemented')

    def curriculum_callback(self):
        self.wrapper.curriculumUpdate()

    def step_async(self, actions: np.ndarray):
        raise RuntimeError('This method is not implemented')

    def step_wait(self):
        raise RuntimeError('This method is not implemented')

    def get_attr(self, attr_name, indices=None):
        """
        Return attribute from vectorized environment.
        :param attr_name: (str) The name of the attribute whose value to return
        :param indices: (list,int) Indices of envs to get attribute from
        :return: (list) List of values of 'attr_name' in all environments
        """
        raise RuntimeError('This method is not implemented')

    def set_attr(self, attr_name, value, indices=None):
        """
        Set attribute inside vectorized environments.
        :param attr_name: (str) The name of attribute to assign new value
        :param value: (obj) Value to assign to `attr_name`
        :param indices: (list,int) Indices of envs to assign value
        :return: (NoneType)
        """
        raise RuntimeError('This method is not implemented')

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        """
        Call instance methods of vectorized environments.
        :param method_name: (str) The name of the environment method to invoke.
        :param indices: (list,int) Indices of envs whose method to call
        :param method_args: (tuple) Any positional arguments to provide in the call
        :param method_kwargs: (dict) Any keyword arguments to provide in the call
        :return: (list) List of items returned by the environment's method call
        """
        raise RuntimeError('This method is not implemented')
