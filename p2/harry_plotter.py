from robot import robot
import matplotlib.pyplot as plt
import math

class harry_plotter:
    def __init__(self, p3dx, ai, is_map=True):
        self.p3dx = p3dx
        self.ai = ai
        self.is_map = is_map
        plt.ion()

    def update(self):
        if self.plot_map:
            self.plot_map()
        else:
            self.plot_sensors()

    def plot_sensors(self):
        plt.clf()
        # initialize graph
        plt.title('Sensors')
        plt.ylabel('')
        plt.xlabel('')
        plt.xlim(-1.5, 1.5)
        plt.ylim(-1.5, 1.5)
        plt.grid(False)

        ## This plot is rotate 90 degrees!

        # plot sensors
        sensors = self.p3dx.get_rel_sonar_positions()
        for p in sensors:
            plt.scatter(-p[1], p[0], 1, c='y')

        # plot robot center
        robotColor = 'black'
        #if self.ai.check_stuck:
        #    robotColor = 'yellow'
        plt.scatter(0, 0, 1, c=robotColor)

        # plot detected positions
        detected = self.p3dx.get_rel_sonar_readings()
        for p in detected:
            plt.scatter(-p[1], p[0], 1, c='r')

        # Update shown plot
        plt.pause(0.000000001)

    def plot_map(self):
        # initialize graph
        plt.title('Obstacles Chamber')
        plt.ylabel('Y')
        plt.xlabel('X')
        plt.xlim(-1000, 1000)
        plt.ylim(-1000, 1000)
        plt.grid(False)

        # plot robot
        robotColor = 'red'
        #if self.ai.check_stuck:
        #    robotColor = 'yellow'
        plt.scatter(int(100 * self.p3dx.position[0]), int(100 * self.p3dx.position[1]), 1, c=robotColor)

        # plot detected positions
        detected = self.p3dx.get_rel_sonar_readings()
        for p in detected:
            # rotate
            th = self.p3dx.orientation[2]
            x = math.cos(th) * p[0] - math.sin(th) * p[1]
            y = math.sin(th) * p[0] + math.cos(th) * p[1]

            # translate
            x = x + self.p3dx.position[0]
            y = y + self.p3dx.position[1]

            # zoom in and round to integer
            x = int(100 * x)
            y = int(100 * y)

            # plot
            plt.scatter(x, y, 1, c='black')

        # Update shown plot
        plt.pause(0.000000001)
