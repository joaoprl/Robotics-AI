from ddpg import ddpg
from robbie import Robbie
from simulator import Simulator

# simulator constants
SIMULATOR_PORT = 25000

# training constants
MAX_EPISODES = 10
MAX_STEPS = 10000

def train_robot():
    # connect to vrep simulator
    sim = Simulator("127.0.0.1", SIMULATOR_PORT)
    sim.connect()

    # get robbie instance and reset it
    robbie = Robbie(sim, "Robbie")
    robbie.reset_robot()

    # start AI
    states_dim, actions_dim = robbie.get_dimensions()
    robbie_ai = ddpg(robbie, states_dim, actions_dim)

    # train robot
    robbie_ai.train(MAX_EPISODES, MAX_STEPS)

    # run robot one time after training
    robbie_ai.run(1, MAX_STEPS)

    # disconnect from simulator
    sim.disconnect()

if __name__ == "__main__":
    train_robot()
