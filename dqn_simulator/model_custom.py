#! /usr/bin/env python
#-*- coding:utf-8 -*-

#! /usr/bin/env python
#-*- coding:utf-8 -*-

import random
import numpy as np
from collections import deque

import torch
import torch.optim as optim
import torch.nn as nn
import torch.nn.functional as F

class DQN(nn.Module):
    def __init__(self, state_size, stack_frame, action_size):
        super(DQN, self).__init__()
        input_size = state_size*stack_frame

        self.fc1 = nn.Linear(input_size, 512)
        self.fc2 = nn.Linear(512, 512)
        self.fc3 = nn.Linear(512, action_size)

    def forward(self, x):
        '''
        input : tensor with size (batch_size, feature_num * stack_num)
        output : tensor with size (batch_size, action_num)
        '''
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x

class DQNAgent(object):
    def __init__(self, params, training, load_path=None):

        self.training = training
        self.set_parameters(params)

        self.model = DQN(self.state_size, self.stack_frame, self.action_size)
        self.obs_set = deque(maxlen=self.skip_frame*self.stack_frame)

        if load_path is not None:
            self.model.load_state_dict(torch.load(load_path))
            print("Model is loaded from %s"%load_path)

        if training:
            self.target_model = DQN(self.state_size, self.stack_frame, self.action_size)
            self.optimizer = optim.Adam(self.model.parameters(), lr=self.lr)
            self.memory = deque(maxlen=self.mem_maxlen)
            self.eps = self.eps_init
            self.update_target()

    def set_parameters(self, params):
        self.state_size = params['state_size']
        self.action_size = params['action_size']
        self.lr = params['lr']
        self.discount_factor = params['discount_factor']
        self.batch_size = params['batch_size']
        self.mem_maxlen = params['mem_maxlen']
        self.eps_init = params['eps_init']
        self.eps_min = params['eps_min']
        self.skip_frame = params['skip_frame']
        self.stack_frame = params['stack_frame']

    def get_action(self, state):
        if self.training:
            if self.eps > np.random.rand():
                return np.random.randint(0, self.action_size)
            else:
                with torch.no_grad():
                    Q = self.model(torch.FloatTensor(state).unsqueeze(0))
                    return np.argmax(Q.detach().numpy())
        else:
            with torch.no_grad():
                Q = self.model(torch.FloatTensor(state).unsqueeze(0))
                return np.argmax(Q.detach().numpy())
    
    def skip_stack_frame(self, obs):
        self.obs_set.append(obs)
        state = np.zeros([self.state_size*self.stack_frame])
        for i in range(self.stack_frame):
            state[self.state_size*i : self.state_size*(i+1)] = self.obs_set[-1 -(self.skip_frame*i)]
        return state

    def append_sample(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def update_target(self):
        self.target_model.load_state_dict(self.model.state_dict())

    def train_model(self):

        batch = random.sample(self.memory, self.batch_size)

        state_batch = torch.FloatTensor(np.stack([b[0] for b in batch], axis=0))
        action_batch = torch.FloatTensor(np.stack([b[1] for b in batch], axis=0))
        reward_batch = torch.FloatTensor(np.stack([b[2] for b in batch], axis=0))
        next_state_batch = torch.FloatTensor(np.stack([b[3] for b in batch], axis=0))
        done_batch = torch.FloatTensor(np.stack([b[4] for b in batch], axis=0))

        eye = torch.eye(self.action_size)
        one_hot_action = eye[action_batch.view(-1).long()]
        q = (self.model(state_batch) * one_hot_action).sum(1)

        with torch.no_grad():
            #max_Q = torch.max(q).item()
            next_q = self.target_model(next_state_batch)
            target_q = reward_batch + next_q.max(1).values * (self.discount_factor*(1-done_batch))

        loss = F.mse_loss(q, target_q)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        return loss.item()  #, max_Q