#! /usr/bin/env python
# -*- coding:utf-8 -*-

import torch
import torch.nn as nn
import numpy as np

class DQNModel(nn.Module):
    def __init__(self, input_size, output_size):
        super(DQNModel, self).__init__()
        self.seq = nn.Sequential(
          nn.Linear(input_size, 512),
          nn.ReLU(),
          nn.Linear(512, 512),
          nn.ReLU(),
          nn.Linear(512, output_size)
        )

    def forward(self, input):
        return self.seq(input)

class DQN(object):
    def __init__(self, sensorlength = 70.0):

        # settings
        self.action_dict = {
            0 : [(-15, 45)],
            1 : [(0, 45)],
            2 : [(15, 45)],
            3 : [(0, -35)]*31 + [(-50, 35)]*62,
            4 : [(0, -35)]*31 + [(50, 35)]*62}

        self.load_path = '/home/nvidia/automonous_rally/src/rally/src/dqn_weight.pth'
        self.model = DQNModel(5, 5)
        self.model.load_state_dict(torch.load(self.load_path))
        self.model.eval()

        # action related
        self.need_action = True
        self.action_list = None
        self.idx = 0

        # sensor calibration
        self.sensorlength = sensorlength
        self.offset = [10.0, 0, 0, 0, 10.0]

    def control(self, ultrasonic):

        if self.need_action:
            self.need_action = False
            state = np.minimum(1.0, (ultrasonic[:5]-self.offset)/self.sensorlength)
            with torch.no_grad():
                Q = self.model(torch.FloatTensor(state).unsqueeze(0))
                action = np.argmax(Q.detach().numpy())
            self.action_list = self.action_dict[action]
            self.idx = 0

        angle, speed = self.action_list[self.idx]
        self.idx += 1

        if self.idx == len(self.action_list):
            self.need_action = True

        return angle, speed
