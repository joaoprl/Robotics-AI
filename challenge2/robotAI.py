import math

V_LINEAR = 1
V_ANGULAR = 0.75
MARGIN_FOR_ERROR = 0.0001
MARGIN_FOR_OBSTACLES = 0.4

class robotAI:
    def __init__(self, p3dx):
        self.verbose = True
        self.p3dx = p3dx

        if self.verbose:
            print('> AI started')

    ## updates AI states
    def tick(self):
        pass
        
