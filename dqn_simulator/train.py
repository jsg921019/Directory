#! /usr/bin/env python
# -*- coding:utf-8 -*-

########## Imports ############

import cv2
import numpy as np
from assets import Map, Car
from my_reward import Reward
from recorder import Recorder
from visual import visualize
from model_custom import DQNAgent



########## Parameters ##########

# DQN model parameters

param_dict = {
    'state_size' : 5,
    'action_size' : 3,

    'lr' : 0.0005,
    'discount_factor' : 0.98,

    'batch_size' : 32,
    'mem_maxlen' : 10000,

    'eps_init' : 1.0,
    'eps_min' : 0.05,

    'skip_frame' : 1,
    'stack_frame' : 1
    }

# training parameters

map_file = 'map.png'
start_train_step = 3000
max_episode = 1000000
target_update_cycle = 1000
report_cycle = 10
autosave_cycle = 100
end_training_loop = 10
eps_decrease_rate = 1.0 / 50000


########## Conversion ###########

def action2msg(action):
    return action*50.0 - 50.0

def msg2rad(msg):
    return np.radians(msg * -3.0 / 5.0)



########## Instances ############

map = Map(map_file)
car = Car(map)
agent = DQNAgent(param_dict, training=True, load_path=None)
agent.model.train()
recorder = Recorder(car)
reward_policy = Reward(map=map)
visual = visualize(port=8888)
visual.chart_init()



########## Main #################

step_tot, score_tot, max_score = 0, 0.0, -float('inf')
prev_step_tot, prev_score_tot = 0, 0.0
episode = 0

end = False
done = True

while not end:

    episode += 1
    loop, step, score = 0, 0, 0.0
    cache = []
    done = False

    car.reset()
    cache.append(car.get_pose())

    obs = car.measure_distance(max_dist=200.0)

    for i in range(agent.skip_frame * agent.stack_frame):
        agent.obs_set.append(obs)
    state = agent.skip_stack_frame(obs)

    while not done:

        step += 1
        step_tot += 1

        action = agent.get_action(state)
        msg_angle = action2msg(action)
        steer_angle = msg2rad(msg_angle)

        car.update(steer_angle, speed=50, dt=0.1)
        cache.append(car.get_pose())

        next_obs = car.measure_distance(max_dist=200.0)
        next_state = agent.skip_stack_frame(next_obs)

        done = car.check_collision()
        reward = reward_policy.give_reward(car.x, car.y, done)
        agent.append_sample(state, action, reward, next_state, done)

        score += reward
        state = next_state

        finished_loop = car.check_goal()

        if finished_loop :
            loop += 1
            if loop >= end_training_loop:
                print('End Training!')
                recorder.save(agent.model, 'final')
                recorder.render(cache, 'final')
                end = True
                break
            else:
                car.reset()
                cache.append(car.get_pose())
                obs = car.measure_distance(max_dist=200.0)
                for i in range(agent.skip_frame * agent.stack_frame):
                    agent.obs_set.append(obs)
                state = agent.skip_stack_frame(obs)

        if step_tot >= start_train_step:

            loss = agent.train_model()

            if agent.eps > agent.eps_min:
                agent.eps -= eps_decrease_rate

            if step_tot % target_update_cycle == 0:
                agent.update_target()

    score_tot += score

    if  score > max_score:
        print("new winner with reward %f"%score)
        max_score = score
        recorder.save(agent.model, episode)
        recorder.render(cache, 'episode_%d_step_%d'%(episode, step))

    if episode % autosave_cycle == 0:
        recorder.save(agent.model, 'autosave')

    if episode % report_cycle == 0 :
        print("episode: {}, mean steps: {}, mean rewards: {}, epsilon: {:.1f}%".format(episode, (step_tot-prev_step_tot)/10.0, (score_tot-prev_score_tot)/10.0, agent.eps*100))
        visual.learning_curve_update(episode, (step_tot-prev_step_tot)/report_cycle)
        visual.reward_update(episode, (score_tot-prev_score_tot)/report_cycle)
        prev_step_tot, prev_score_tot = step_tot, score_tot