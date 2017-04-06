from robot import robot
import matplotlib.pyplot as plt
import math

class harry_plotter:
    def __init__(self, rob, robAI):
        self.rob = rob
        self.robAI = robAI
        plt.ion()

    def update(self):
        self.plotMap()

        # self.plotSensors()

    def plotSensors(self):
        plt.clf()
        # initialize graph
        plt.title('Sensors')
        plt.ylabel('Y')
        plt.xlabel('X')
        plt.xlim(-1.5,1.5)
        plt.ylim(-1.5,1.5)
        plt.grid(False)

        # plot sensors
        sensors = self.rob.get_rel_sonar_positions()
        for p in sensors:
            plt.scatter(p[0], p[1], 1, c='y')

        # plot robot center
        robotColor = 'black'
        if self.robAI.check_stuck:
            robotColor = 'yellow'
        plt.scatter(0,0,1,c=robotColor)

        # plot detected positions
        detected = self.rob.get_rel_sonar_readings()
        for p in detected:
            plt.scatter(p[0], p[1], 1, c='r')

        # Update shown plot
        plt.pause(0.000000001)

    def plotMap(self):
        # plt.clf()
        # initialize graph
        plt.title('Obstacles Chamber')
        plt.ylabel('Y')
        plt.xlabel('X')
        plt.xlim(-1000,1000)
        plt.ylim(-1000,1000)
        plt.grid(False)

        # plot robot
        robotColor = 'red'
        if self.robAI.check_stuck:
            robotColor = 'yellow'
        plt.scatter(int(100 * self.rob.position[0]), int(100 * self.rob.position[1]), 1, c=robotColor)

        # plot detected positions
        detected = self.rob.get_rel_sonar_readings()
        for p in detected:
            x = p[0]
            y = p[1]

            # rotate
            th = self.rob.orientation[2]
            x = math.cos(th) * x - math.sin(th) * y
            y = math.sin(th) * x + math.cos(th) * y

            # translate
            x = x + self.rob.position[0]
            y = y + self.rob.position[1]

            # zoom in and round to integer
            x = int(100 * x)
            y = int(100 * y)

            # plot
            plt.scatter(x, y, 1, c='black')

        # Update shown plot
        plt.pause(0.000000001)
