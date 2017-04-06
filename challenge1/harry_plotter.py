from robot import robot
import matplotlib.pyplot as plt

class harry_plotter:
    def __init__(self, rob):
        self.rob = rob
        plt.ion()

    def update(self):
        plt.clf()

        # initialize graph
        plt.title('Sensors')
        plt.ylabel('Y')
        plt.xlabel('X')
        plt.xlim(-1.5,1.5)                # try commenting this out...
        plt.ylim(-1.5,1.5)
        plt.grid(False)

        # plot sensors
        sensors = self.rob.get_rel_sonar_positions()
        for p in sensors:
            plt.scatter(p[0], p[1], 1, c='y')

        # plot robot center
        plt.scatter(0,0,1,c='black')

        # plot sensors values
        detected = self.rob.get_rel_sonar_readings()
        for p in detected:
            plt.scatter(p[0], p[1], 1, c='r')
        plt.pause(0.001)
