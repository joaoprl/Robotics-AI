##
## implementation of Ornstein-Uhlenbeck process for noise
##

import random
import numpy as np

class ou:
    def noise(self, x, mu=0., theta=.15, sigma=.2):
        return theta * (mu - x) + sigma * np.random.randn(1)
