import math
import numpy as np
from simulator import *
from reference_system import reference_system

# name and values definitions
WHEEL_DISTANCE = 0.381
WHEEL_RADIUS = 0.0975
ROBOT_LENGTH = 0.455

LEFT_WHEEL = "_leftWheel"
RIGHT_WHEEL = "_rightWheel"

LEFT_MOTOR = "_leftMotor"
RIGHT_MOTOR = "_rightMotor"

# name and total of sensors
SONAR_SENSOR = "_ultrasonicSensor"
NUM_SONARS = 16
SONAR_ANGLES = [90, 50, 30, 10, -10, -30, -50, -90, -90, -130, -150, -170, 170, 150, 130, 90]
SONAR_ANGLES_PLOT = [i * math.pi / 180 for i in SONAR_ANGLES]
SONAR_RADIUS = 0.0975
SONAR_HEIGHT = 0.21

# robot class definition
class robot:
    def __init__(self, sim, name, use_ground_truth=False):
        self.sim = sim                          # simulation environment
        self.name = name                        # robot's name
        self.handle = self.sim.get_handle(name) # robot's id handle
        self.use_ground_truth = use_ground_truth

        self.L = WHEEL_DISTANCE
        self.R = WHEEL_RADIUS

        # get id handles of wheels
        self.wheel_handle = [self.sim.get_handle(self.name + RIGHT_WHEEL),
                             self.sim.get_handle(self.name + LEFT_WHEEL)]

        # get id handles of motors
        self.motor_handle = [self.sim.get_handle(self.name + RIGHT_MOTOR),
                             self.sim.get_handle(self.name + LEFT_MOTOR)]

        # connect to each of sonar sensor and set its readings
        self.sonar_handle = [self.sim.get_handle(self.name + SONAR_SENSOR + str(i + 1)) \
                                for i in range(NUM_SONARS)]
        self.sonar_readings = [0] * NUM_SONARS

        # init each sonar sensor
        for i in range(NUM_SONARS):
            state, coord = self.sim.init_prox_sensor(self.sonar_handle[i])
            if state > 0:
                self.sonar_readings[i] = coord[2]
            else:
                self.sonar_readings[i] = ERROR

        # get encoders for each motor and initialize it
        self.wheel_angle = np.array([self.sim.get_joint_position(i) for i in self.motor_handle])
        self.last_wheel_angle = np.array([i for i in self.wheel_angle])

        # calculate sonar sensors relative position
        self.sonars_rel_pos = []
        for i in range(NUM_SONARS):
            self.sonars_rel_pos.append([math.cos(SONAR_ANGLES_PLOT[i]) * SONAR_RADIUS, \
                                    math.sin(SONAR_ANGLES_PLOT[i]) * SONAR_RADIUS, \
                                    SONAR_HEIGHT])

        # declare pose and velocity variables
        self.true_pose = np.zeros(3)
        self.pose = np.zeros(3)
        self.last_pose = np.zeros(3)
        self.true_v_linear = self.true_v_angular = 0
        self.v_linear = self.v_angular = 0

        # update values and dicard first batch
        self.update()
        self.update()

        # initialize our pose and velocity arrays
        if self.use_ground_truth:
            self.pose = np.copy(self.true_pose)
            self.last_pose = np.copy(self.pose)

        # create reference system
        self.reference_system = reference_system(self.true_pose)

    ## general update for our robot
    def update(self, tick_time=0):
        self.sim.update()
        self.update_sensors()
        self.update_ground_truth()
        if tick_time > 0:
            self.update_pose(tick_time)

    ## update our sensors
    def update_sensors(self):
        for i in range(NUM_SONARS):
            state, coord = self.sim.read_prox_sensor(self.sonar_handle[i])

            if state > 0:
                self.sonar_readings[i] = coord[2]
            else:
                self.sonar_readings[i] = ERROR

        self.last_wheel_angle = np.array([i for i in self.wheel_angle])
        self.wheel_angle = np.array([self.sim.get_joint_position(i) for i in self.motor_handle])

    ## update pose for our robot
    def update_pose(self, tick_time):
        # save last pose
        self.last_pose = np.copy(self.pose)

        # update pose with ground truth
        if self.use_ground_truth:
            self.last_pose = np.copy(self.pose)
            self.pose = np.copy(self.true_pose)
            self.v_linear = self.true_v_linear
            self.v_angular = self.true_v_angular
        # update pose with odometry
        else:
            # determine delta angular position of wheels
            wheel_delta_theta = np.zeros(2)
            for i in range(2):
                delta = self.wheel_angle[i] - self.last_wheel_angle[i]
                if abs(delta) < math.pi:
                    wheel_delta_theta[i] = delta
                elif delta < 0:
                    wheel_delta_theta[i] = delta + 2 * math.pi
                else:
                    wheel_delta_theta[i] = delta - 2 * math.pi

            # calculate wheel velocity
            wheel_v_angular = wheel_delta_theta / tick_time
            wheel_v_linear = self.R * wheel_v_angular

            # calculate robot velocity
            self.v_linear = (wheel_v_linear[0] + wheel_v_linear[1]) / 2
            self.v_angular = (wheel_v_linear[0] - wheel_v_linear[1]) / self.L

            #composed_v_angular = wheel_v_linear / self.L
            #turn_radius = self.v_linear / self.v_angular

            # calculate difference in pose
            delta_s = self.v_linear * tick_time
            delta_theta = self.v_angular * tick_time
            new_theta = self.pose[2] + delta_theta / 2
            delta_pose = np.array([delta_s * math.cos(new_theta), delta_s * math.sin(new_theta), delta_theta])

            # save last pose and update current one
            self.last_pose = self.pose
            self.pose = self.pose + delta_pose

        # update pose with odometry
        self.pose = np.copy(self.pose)

    def update_ground_truth(self):
        position = self.sim.get_position(self.handle)
        orientation = self.sim.get_orientation(self.handle)
        self.true_pose = np.array([position[0], position[1], orientation[2]])
        v_linear, v_angular = self.sim.get_velocity(self.handle)
        self.true_v_linear = math.sqrt(math.pow(v_linear[0], 2) + math.pow(v_linear[1], 2))
        self.true_v_angular = v_angular[2]

    def move(self, v_left, v_right):
        self.sim.set_joint_target_velocity(self.motor_handle[0], v_right)
        self.sim.set_joint_target_velocity(self.motor_handle[1], v_left)

    ## stop all motors
    def stop(self):
        for motor in self.motor_handle:
            self.sim.set_joint_target_velocity(motor, 0)

    ## drive our robot
    def drive(self, v_linear, v_angular):
        v_left = self.v_L(v_linear, v_angular)
        v_right = self.v_R(v_linear, v_angular)
        self.move(v_left, v_right)

    ## get left motor velocity
    def v_L(self, v_linear, v_angular):
        return 2*v_linear - (self.L*v_angular) / (2*self.R)

    ## get right motor velocity
    def v_R(self, v_linear, v_angular):
        return 2*v_linear + (self.L*v_angular) / (2*self.R)

    ## get sonar reading relative to the robot zero
    def get_rel_sonar_readings(self):
        rel_sonars = []
        for i in range(NUM_SONARS):
            if self.sonar_readings[i] != -1:
                rel_sonars.append([self.sonar_readings[i] * math.cos(SONAR_ANGLES_PLOT[i]) + self.sonars_rel_pos[i][0], \
                                   self.sonar_readings[i] * math.sin(SONAR_ANGLES_PLOT[i]) + self.sonars_rel_pos[i][1],\
                                   self.sonars_rel_pos[i][2], i])
        return rel_sonars

    def get_rel_sonar_positions(self):
        return self.sonars_rel_pos

    def get_sonar_radius(self):
        return SONAR_RADIUS

    def get_sonar_angles(self):
        return SONAR_ANGLES

    def get_robot_width(self):
        return WHEEL_DISTANCE

    def get_robot_length(self):
        return ROBOT_LENGTH

    def get_local_pose(self):
        if self.use_ground_truth:
            return self.reference_system.to_local_coord(self.pose)
        else:
            return self.pose

    def get_world_pose(self):
        if self.use_ground_truth:
            return self.pose
        else:
            return self.reference_system.to_world_coord(self.pose)

    ### [debug] robot position, orientation and velocity
    def print_pose(self):
        print('[' + str(self.pose[0]) + ', ' + str(self.pose[1]) + ', ' +\
                str(self.pose[2]) + ', ' +\
                str(self.v_linear) + ', ' + str(self.v_angular) + ']')

    ### [debug] sonar readings
    def print_sonars(self):
        print(self.sonar_readings)
