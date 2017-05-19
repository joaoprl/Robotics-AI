from robot import robot
from robotAI import robotAI
from simulator import *
from harry_plotter import harry_plotter

def main():
    plot = False

    sim = simulator("127.0.0.1", 25000)
    sim.connect()

    p3dx = robot(sim, "Pioneer_p3dx")
    p3dxAI = robotAI(p3dx)

    if plot:
        h = harry_plotter(p3dx, p3dxAI)

    while True:
        p3dx.update()
        if plot:
            h.update()
        p3dxAI.tick()

    sim.disconnect()

if __name__ == "__main__":
    main()
