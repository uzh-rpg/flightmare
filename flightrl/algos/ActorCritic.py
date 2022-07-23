import torch
import torch.nn as nn
import numpy as np
import gym
import os
from torch.distributions.normal import Normal
from torch.distributions.multivariate_normal import MultivariateNormal
from torch.nn import MSELoss
from typing import List

from ruamel.yaml import YAML, dump, RoundTripDumper
from rpg_baselines.envs import vec_env_wrapper as wrapper
from rpg_baselines.envs.vec_env_wrapper import FlightEnvVec
from flightgym import QuadrotorEnv_v1


############### Hyperparameters ##############
MAX_EPISODE_STEPS = 300
NUM_EPISODES = 1000
NUM_ENV = 1
GAMMA = 0.99
LR = 1e-4
##############################################


class ActorCritic(nn.Module):
    def __init__(self):
        super(ActorCritic,self).__init__()
        self.linear_stack = nn.Sequential(
            nn.Linear(12,64),
            nn.ReLU(),
            nn.Linear(64,32),
            nn.ReLU(),
        )
        self.critic = nn.Sequential(nn.Linear(32,1))
        self.actor = nn.Sequential(
            nn.Linear(32,8),
            nn.Tanh(),
        )

    def forward(self,x):
        x = self.linear_stack(x)
        actor = self.actor(x)
        critic = self.critic(x)
        return actor, critic

class ActorCriticAgent():
    def __init__(self, env: FlightEnvVec, render: bool):
        # what is self.env.num_envs for?
        self.env = env
        self.env.max_episode_steps = MAX_EPISODE_STEPS
        self.gamma = GAMMA
        
        self.model = ActorCritic()
        self.state = torch.from_numpy(self.env.reset())

        self.optimizer = torch.optim.Adam(self.model.parameters(),lr=LR)

        if render:
            env.connectUnity()

    def train(self):
        probs, value = self.model(self.state)
        # probs is tensor of dim (num_envs,8)
        action = np.zeros((self.env.num_envs,4),dtype=np.float32)   

        # print(probs[0][4:].size())
        distribList:List[MultivariateNormal] = []
        for i in range(self.env.num_envs):
            # print(probs[i][:4].size(),torch.diag(probs[i][4:]).size())
            # for j in range(4):
            #     m = Normal(probs[i][j],torch.max(probs[i][j+4],torch.tensor(0.01)))
            #     action[i][j] = m.sample()
            #     action[i][j] = np.maximum(-1.0,action[i][j])
            #     action[i][j] = np.minimum(1.0,action[i][j])
            cov = torch.abs(probs[i][4:])*torch.eye(4)
            m = MultivariateNormal(probs[i][:4],cov)
            action[i] = m.sample()
            distribList.append(m)
        # print(action)

        nextState, reward, done, _ = self.env.step(action)
        _, newValue = self.model(torch.from_numpy(nextState))

        newValue = torch.from_numpy(done).unsqueeze(dim=0).T*newValue
        errorValue = torch.from_numpy(reward).unsqueeze(dim=1) + self.gamma*newValue - value
        # print(errorValue.shape, reward.shape, done.shape, newValue.shape, value.shape)
        actorLoss = 0
        for i in range(self.env.num_envs):
            actorLoss += - distribList[i].log_prob(torch.from_numpy(action[i]))*errorValue[i]
        mseObj = MSELoss()
        criticLoss = mseObj(torch.from_numpy(reward).unsqueeze(dim=1) + self.gamma*newValue, value)
        loss = actorLoss+criticLoss
        # print(nextState.shape)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        self.state = torch.from_numpy(nextState)
        for i in range(self.env.num_envs):
            if(done[i] == True):
                self.state = torch.from_numpy(self.env.reset())
                return True, reward
        return False, reward
        
        

if __name__ == "__main__":
    cfg = YAML().load(open(os.environ["FLIGHTMARE_PATH"] +
                           "/flightlib/configs/vec_env.yaml", 'r'))
    # cfg = ordereddict([('env', ordereddict([('seed', 1), ('scene_id', 0), ('num_envs', 100), ('num_threads', 10), ('render', 'no')]))])

    cfg["env"]["num_envs"] = NUM_ENV
    cfg["env"]["num_threads"] = 1

    cfg["env"]["render"] = "yes"

    env = wrapper.FlightEnvVec(QuadrotorEnv_v1(
        dump(cfg, Dumper=RoundTripDumper), False))
    
    agent = ActorCriticAgent(env, True)
    for i in range(NUM_EPISODES):
        done = False
        ep_rew = 0
        while (not done):
            done, reward = agent.train()
            ep_rew += float(np.sum(reward))/NUM_ENV
        print("Episode: ", i, " , Reward: ", ep_rew)