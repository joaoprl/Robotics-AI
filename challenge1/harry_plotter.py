from robot import robot
import matplotlib.pyplot as plt
import math

class harry_plotter:
    def __init__(self, rob):
        self.rob = rob
        self.map = {}
        plt.ion()

    def update(self):
        # plt.clf()

        # initialize graph
        plt.title('Sensors')
        plt.ylabel('Y')
        plt.xlabel('X')
        plt.xlim(-1000,1000)
        plt.ylim(-1000,1000)
        plt.grid(False)

        # plot sensors
        sensors = self.rob.getRelSonarPositions()
        for p in sensors:
            plt.scatter(p[0], p[1], 1, c='y')

        # plot robot center
        plt.scatter(0,0,1,c='black')

        # plot sensors values
        detected = self.rob.getRelSonarReadings()
        flag = True
        for p in detected:
            # plt.scatter(p[0], p[1], 1, c='r')

            # point = (int(p[0] * 100), int(p[1] * 100))
            # if point in self.map:
            #     self.map[point] += 1
            # else:
            #     self.map[point] = 1

            # print(self.rob.orientation[2])

            # rotate
            th = self.rob.orientation[2]
            x = math.cos(th) * p[0] - math.sin(th) * p[1]
            y = math.sin(th) * p[0] + math.cos(th) * p[1]

            # translate
            if flag:
                print(x, y)
                flag = False
            x = x + self.rob.position[0]

            y = y + self.rob.position[1]

            # zoom in and aproximate
            x = int(100 * x)
            y = int(100 * y)

            # draw
            plt.scatter(x, y, 1, c='black')
        plt.pause(0.000000001)
