#!/usr/bin/env python
#-*- coding:utf-8 -*-

import numpy as np
from collections import deque

class Filter(object):
    def __init__(self, maxlen=3):
        self.queue = deque(maxlen=maxlen)

    def filter(self, ultrasonic):
        self.queue.append(ultrasonic)
        return np.median(self.queue, axis=0)