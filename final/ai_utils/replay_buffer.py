##
## implementation of the replay buffer memory,
## which stores previous interactions
##

from collections import deque

import random
import numpy as np

class replay_buffer:
    def __init__(self, batch_size, seed=1337, size=10000):
        self.size = size
        self.batch_size = batch_size

        self.count = 0
        self.buffer = deque()

        random.seed(seed)

    def get_batch(self):
        batch = np.array(random.sample(self.buffer, \
            min(self.count, self.batch_size)))

        s     = np.array([b[0] for b in batch])
        a     = np.array([b[1] for b in batch])
        r     = np.array([b[2] for b in batch])
        t     = np.array([b[3] for b in batch])
        new_s = np.array([b[4] for b in batch])

        return s, a, r, t, new_s

    def store(self, s, a, r, new_s, done):
        exp = (s, a, r, new_s, done)

        if self.count < self.size:
            self.count += 1
        else:
            self.buffer.popleft()

        self.buffer.append(exp)

    def clear(self):
        self.buffer.clear()
        self.count = 0
