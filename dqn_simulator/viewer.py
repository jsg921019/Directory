#!/usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np
import cv2
from assets import Car, Map
from model_custom  import DQNAgent


#################################


def action2msg(action):
    return action*50.0 - 50.0

def msg2rad(msg):
    return np.radians(msg * -3.0 /5.0)


##################################

# DQN model parameters

param_dict = {
    'state_size' : 5,
    'action_size' : 3,

    'lr' : 0.00001,
    'discount_factor' : 0.98,

    'batch_size' : 32,
    'mem_maxlen' : 10000,

    'eps_init' : 0.03,
    'eps_min' : 0.03,

    'skip_frame' : 1,
    'stack_frame' : 1
    }

###################################

map = Map('map.png')
car = Car(map)
agent = DQNAgent(param_dict, training=False, load_path='save/sample.pth')
agent.model.train()

car.reset()
obs = car.measure_distance(max_dist=200)

for i in range(agent.skip_frame * agent.stack_frame):
    agent.obs_set.append(obs)

state = agent.skip_stack_frame(obs)
done = False
step = 0

while not done:

    step += 1

    action = agent.get_action(state)
    angle = action2msg(action)
    steer_angle = msg2rad(angle)

    car.update(steer_angle, speed=50, dt=0.1)

    next_obs = car.measure_distance(max_dist=200)
    next_state = agent.skip_stack_frame(next_obs)

    done = car.check_collision()
    state = next_state

    finished = car.check_goal()
    if finished:
        car.reset()
        obs = car.measure_distance(max_dist=200)

        for i in range(agent.skip_frame * agent.stack_frame):
            agent.obs_set.append(obs)

        state = agent.skip_stack_frame(obs)

    if step >= 10000:
        done = True

    frame = map.img.copy()
    car.draw(frame)
    cv2.putText(frame, "step : " + str(step), (20, 50), cv2.FONT_HERSHEY_DUPLEX, 0.8, 2, 2)
    cv2.imshow('viewer', frame)
    if cv2.waitKey(100) == ord(' '):
        break

cv2.destroyAllWindows()