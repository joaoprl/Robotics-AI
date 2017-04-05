from robot import robot
from simulator import *

def main():
    sim = simulator("127.0.0.1", 25000)
    sim.connect()

    p3dx = robot(sim, "Pioneer_p3dx")
    p3dx.update()
    # p3dx.move(-2, -2)
    p3dx.move(2, 2)

    while(True):
        p3dx.update()
        print(p3dx.sonar_readings)

    sim.disconnect()

if __name__ == "__main__":
    main()
