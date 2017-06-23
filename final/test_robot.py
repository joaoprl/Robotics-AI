import random
from time import time
from robbie import Robbie
from simulator import Simulator

TIMEOUT = 10
SIMULATOR_PORT = 25000

def test_robot():
    # connect to vrep simulator
    sim = Simulator("127.0.0.1", SIMULATOR_PORT)
    sim.connect()

    # get robbie instance and reset it
    robbie = Robbie(sim, "Robbie")
    robbie.reset_robot()

    # total time of this generation
    start_time = time()

    # update loop
    while True:
        # send actions to robot
        actions = [random.randrange(-1, 1) for _ in range(8)]
        new_state, reward, done = robbie.act(actions)
        print(new_state + [reward] + [done])

        # get generation execution time
        gen_time = time() - start_time

        # reset robot if finished simulation or timed out
        if done or gen_time >= TIMEOUT:
            robbie.reset_robot()
            start_time = time()

    # disconnect from simulator
    sim.disconnect()

if __name__ == "__main__":
    test_robot()
