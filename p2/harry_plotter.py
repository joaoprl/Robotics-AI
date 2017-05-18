import math
import numpy as np
import matplotlib.pyplot as plt
from robot import robot

class harry_plotter:
    def __init__(self, p3dx, ai, is_map=True):
        self.p3dx = p3dx
        self.ai = ai
        self.is_map = is_map

        # plot variables
        self.xdata = []
        self.ydata = []
        self.colors = []
        self.sizes = []
        self.new_xdata = []
        self.new_ydata = []
        self.new_colors = []
        self.new_sizes = []
        self.count = 0
        self.fig, self.ax = plt.subplots()
        self.sc = self.ax.scatter(self.xdata, self.ydata)
        self.background = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        self.fig.canvas.mpl_connect('resize_event', self.redraw_all)

        # init plot
        plt.ion()
        plt.show()

        # different plot dependaple variables
        if self.is_map:
            self.ax.set_title('Obstacles Chamber')
            self.ax.set_ylabel('Y')
            self.ax.set_xlabel('X')
            self.ax.set_xlim(-1000, 1000)
            self.ax.set_ylim(-1000, 1000)
            self.ax.grid(False)
        else:
            self.ax.set_title('Sensors')
            self.ax.set_ylabel('')
            self.ax.set_xlabel('')
            self.ax.set_xlim(-1.5, 1.5)
            self.ax.set_ylim(-1.5, 1.5)
            self.ax.grid(False)

    def add_point(self, x, y, w, c):
        self.count += 1
        self.new_xdata.append(x)
        self.new_ydata.append(y)
        self.new_sizes.append(w)
        self.new_colors.append(c)

    def update_points(self):
        self.xdata.extend(self.new_xdata)
        self.ydata.extend(self.new_ydata)
        self.sizes.extend(self.new_sizes)
        self.colors.extend(self.new_colors)

    def clear_plot(self):
        self.ax.clf()

    def redraw_all(self):
        self.sc.set_offsets(np.column_stack((self.xdata, self.ydata)))
        self.sc.set_sizes(self.sizes)
        self.sc.set_color(self.colors)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def redraw_plot(self):
        self.sc.set_offsets(np.column_stack((self.new_xdata, self.new_ydata)))
        self.sc.set_sizes(self.new_sizes)
        self.sc.set_color(self.new_colors)
        self.background = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        self.fig.canvas.restore_region(self.background)
        self.ax.draw_artist(self.sc)
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()

    def update(self):
        if self.plot_map:
            self.plot_map()
        else:
            self.plot_sensors()
        self.update_points()
        self.redraw_plot()

    def plot_sensors(self):
        self.clear_plot()

        # add sensors
        sensors = self.p3dx.get_rel_sonar_positions()
        for point in sensors:
            self.add_point(-point[1], point[0], 1, 'y')

        # add robot center
        if self.ai.is_stuck:
            self.add_point(0, 0, 3, 'red')
        else:
            self.add_point(0, 0, 2, 'black')

        # add detected positions
        detected = self.p3dx.get_rel_sonar_readings()
        for point in detected:
            self.add_point(-point[1], point[0], 2, 'blue')

    def plot_map(self):
        # add robot positions
        world_pose = self.p3dx.get_world_pose()
        if self.ai.is_stuck:
            self.add_point(int(100 * world_pose[0]), int(100 * world_pose[1]), 2, 'red')
        else:
            self.add_point(int(100 * world_pose[0]), int(100 * world_pose[1]), 1, 'green')

        # add ground truth position
        true_pose = self.p3dx.true_pose
        self.add_point(int(100 * true_pose[0]), int(100 * true_pose[1]), 1, 'blue')

        # add detected positions
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

            # add points
            self.add_point(xpos, ypos, 0.75, 'black')
