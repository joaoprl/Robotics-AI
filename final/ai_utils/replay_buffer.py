##
## implementation of the replay buffer memory,
## which stores previous interactions
##

from collections import deque

import random
import numpy as np

class replay_buffer:
    def __init__(self, size, batch_size):
        self.size = size
        self.batch_size = batch_size

        self.count = 0
        self.buffer = deque()

    def get_batch(self):
        sample = np.array(random.sample(self.buffer, \
            min(self.count, self.batch_size)))

        return tuple([t[0] for t in sample.T])

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
