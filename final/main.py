from time import time
from robbie import Robbie, Action
from simulator import Simulator

SIMULATOR_PORT = 25000

def main():
    # connect to vrep simulator
    sim = Simulator("127.0.0.1", SIMULATOR_PORT)
    sim.connect()

    # get robbie instance
    robbie = Robbie(sim, "Robbie")

    # delta time variable
    tick_time = 0

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

        # reset robot if finished simulation
        if done:
            robbie.reset_robot()

        # get delta time
        tick_time = time() - start_time

    # disconnect from simulator
    sim.disconnect()

if __name__ == "__main__":
    main()
