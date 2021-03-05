#!/usr/bin/env python

from visdom import Visdom
from visdom.server import start_server, download_scripts, DEFAULT_ENV_PATH, DEFAULT_HOSTNAME
from multiprocessing import Process
import numpy as np
import time

class visualize:
    def __init__(self, port=8097):
        download_scripts()
        self.port = port
        hostname = DEFAULT_HOSTNAME
        base_url = ""
        env_path = DEFAULT_ENV_PATH
        readonly = False
        user_credential = None
        use_frontend_client_polling = False

        self.p = Process(target=start_server, args=(self.port, hostname, base_url, env_path, readonly, None, user_credential, use_frontend_client_polling,))
        self.p.start()
        time.sleep(2)

    def chart_init(self):
        self.viz = Visdom(port=self.port)
        self.line_window = self.viz.line(Y=np.array([0]), X=np.array([0]), opts=dict(title = "dqn - Reward Graph", xlabel='Episode', ylabel='Reward'))
        #self.scatter_window = self.viz.scatter([[0,0]], opts=dict(markersize=3, title = "dqn - death position", xlabel='dead position X', ylabel='dead position Y'))
        self.Learning_curve = self.viz.line(Y=np.array([0]), X=np.array([0]), opts=dict(title = "dqn - Step Graph", xlabel='Episode', ylabel='Steps'))
        #self.loss_graph = self.viz.line(Y=np.array([0]), X=np.array([0]), opts=dict(title = "dqn - Loss Graph", xlabel='Episode', ylabel='Loss value'))

    # def dead_position_update(self, dead_position):
    #     self.viz.scatter([[dead_position[0], -dead_position[1]]],win=self.scatter_window, update='append')

    def reward_update(self, episode, total_reward):
        self.viz.line(Y=np.array([total_reward]), X=np.array([episode]), win=self.line_window, update='append')

    def learning_curve_update(self, episode, steps):
        self.viz.line(Y=np.array([steps]), X=np.array([episode]), win=self.Learning_curve, update='append')

    # def loss_graph_update(self, episode, loss):
    #     self.viz.line(Y=np.array([float(loss)]), X=np.array([episode]), win=self.loss_graph, update='append')

    def exit_ready(self):
        self.p.kill()
        self.p.close()
        self.viz.close()

