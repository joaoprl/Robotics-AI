from robot import robot
from simulator import *

def main():
    sim = simulator("127.0.0.1", 25000)
    sim.connect()

    p3dx = robot(sim, "Pioneer_p3dx")
    p3dx.update()

    sim.disconnect()

if __name__ == "__main__":
    main()