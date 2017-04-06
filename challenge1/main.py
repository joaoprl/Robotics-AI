from robot import robot
from robotAI import robotAI
from simulator import *
from harry_plotter import harry_plotter

def main():
    sim = simulator("127.0.0.1", 25000)
    sim.connect()

    p3dx = robot(sim, "Pioneer_p3dx")
    p3dxAI = robotAI(p3dx)

    h = harry_plotter(p3dx)

    while True:
        p3dx.update()
        # p3dx.print_pose()
        # p3dxAI.tick()
        #p3dx.print_pose()
        h.update()
        p3dx.print_pose()

    sim.disconnect()

if __name__ == "__main__":
    main()
