from time import time
from robbie import Robbie, Action
from simulator import Simulator

TIMEOUT = 60
SIMULATOR_PORT = 25000

def main():
    # connect to vrep simulator
    sim = Simulator("127.0.0.1", SIMULATOR_PORT)
    sim.connect()

    # get robbie instance and reset it
    robbie = Robbie(sim, "Robbie")
    robbie.reset_robot()

    # delta time variable
    tick_time = 0

    # total time of this generation
    gen_time = 0

    # update loop
    while True:
        # start counting loop time
        start_time = time()

        # update robot
        robbie.update(tick_time)

        # send actions to robot
        actions = [Action.get_random_action() for _ in range(8)]
        new_state, reward, done = robbie.act(actions)
        print new_state + [reward] + [done]

        # get delta time
        tick_time = time() - start_time
        gen_time += tick_time

        # reset robot if finished simulation or timed out
        if done or gen_time >= TIMEOUT:
            robbie.reset_robot()
            gen_time = 0
            tick_time = 0

    # disconnect from simulator
    sim.disconnect()

if __name__ == "__main__":
    main()
