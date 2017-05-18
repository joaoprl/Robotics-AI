#
#   P2!
#  Implements odometry and fuzzy behavior for p3dx.
#

from time import time
from simulator import simulator
from robot import robot
from fuzzy import fuzzy
from harry_plotter import harry_plotter

def main():
    sim = simulator("127.0.0.1", 25000)
    sim.connect()

    p3dx = robot(sim, "Pioneer_p3dx", False, True)
    ai = fuzzy(p3dx)
    hp = harry_plotter(p3dx, ai)

    tick_time = 0

    while True:
        start_time = time()
        p3dx.update(tick_time)
        hp.update()
        ai.tick()
        tick_time = time() - start_time

    sim.disconnect()

if __name__ == "__main__":
    main()
