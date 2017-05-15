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
        for point in sensors:
            plt.scatter(-point[1], point[0], 1, c='y')

        # plot robot center
        #robot_color = 'black'
        #if self.ai.check_stuck:
        #    robot_color = 'red'
        plt.scatter(0, 0, 1, c='black')

        # plot detected positions
        detected = self.p3dx.get_rel_sonar_readings()
        for point in detected:
            plt.scatter(-point[1], point[0], 1, c='r')

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
        #robot_color = 'blue'
        #if self.ai.check_stuck:
        #    robot_color = 'red'
        true_pose = self.p3dx.true_pose
        world_pose = self.p3dx.get_world_pose()
        plt.scatter(int(100 * true_pose[0]), int(100 * true_pose[1]), 1, c='blue')
        plt.scatter(int(100 * world_pose[0]), int(100 * world_pose[1]), 1, c='green')

        # plot detected positions
        detected = self.p3dx.get_rel_sonar_readings()
        for point in detected:
            # rotate
            angle = self.p3dx.pose[2]
            xpos = math.cos(angle) * point[0] - math.sin(angle) * point[1]
            ypos = math.sin(angle) * point[0] + math.cos(angle) * point[1]

            # translate
            xpos = xpos + world_pose[0]
            ypos = ypos + world_pose[1]

            # zoom in and round to integer
            xpos = int(100 * xpos)
            ypos = int(100 * ypos)

            # plot
            plt.scatter(xpos, ypos, 1, c='black')

        # Update shown plot
        plt.pause(0.000000001)
