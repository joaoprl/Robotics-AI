from simulator import *

# name and values definitions
WHEEL_DISTANCE = 0.381
WHEEL_RADIUS = 0.0975

LEFT_HANDLE = "_leftWheel"
RIGHT_HANDLE = "_rightWheel"

LEFT_MOTOR = "_leftMotor"
RIGHT_MOTOR = "_rightMotor"

# name and total of sensors
SONAR_SENSOR = "_ultrasonicSensor"
NUM_SONARS = 16
SONAR_ANGLES = [90, 50, 30, 10, -10, -30, -50, -90, -90, -130, -150, -170, 170, 150, 130, 90]

# robot class definition
class robot:
    def __init__(self, simulator, name):
        self.sim = simulator                    # simulation environment
        self.name = name                        # robot's name
        self.handle = self.sim.get_handle(name) # robot's id handle

        self.L = WHEEL_DISTANCE
        self.R = WHEEL_RADIUS

        # get id handles of encoders
        self.encoder_handle = [self.sim.get_handle(self.name + LEFT_HANDLE), \
                                    self.sim.get_handle(self.name + RIGHT_HANDLE)]

        # get id handles of motors and set their velocity
        self.motor_handle = [self.sim.get_handle(self.name + LEFT_MOTOR), \
                                self.sim.get_handle(self.name + RIGHT_MOTOR)]
        self.velocity = [1, 1]

        # connect to each of sonar sensor and set its readings
        self.sonar_handle = [self.sim.get_handle(self.name + SONAR_SENSOR + str(i)) \
                                for i in range(NUM_SONARS)]
        self.sonar_readings = [0] * NUM_SONARS

        # init each sonar sensor
        for i in range(NUM_SONARS):
            state, coord = self.sim.init_prox_sensor(self.sonar_handle[i])
            if state > 0:
                self.sonar_readings[i] = coord[2]
            else:
                self.sonar_readings[i] = ERROR

        # get current position and orientation
        self.position = self.sim.get_position(self.handle)
        self.orientation = self.sim.get_orientation(self.handle)

        # initialize our last position
        self.last_position = [i for i in self.position]

        # get encoders for each motorand initialize it
        self.encoders = [self.sim.get_joint_position(i) for i in self.motor_handle]
        self.last_encoder = [i for i in self.encoders]

    ## general update for our robot
    def update(self):
        self.sim.update()
        self.update_sensors()
        self.update_pose()

    ## update our sensors
    def update_sensors(self):
        for i in range(NUM_SONARS):
            state, coord = self.sim.read_prox_sensor(self.sonar_handle[i])

            if state > 0:
                self.sonar_readings[i] = coord[2]
            else:
                self.sonar_readings[i] = ERROR

        self.last_encoder = [i for i in self.encoders]
        self.encoders = [self.sim.get_joint_position(i) for i in self.motor_handle]

    ## update pose for our robot
    def update_pose(self):
        self.last_position = [i for i in self.position]

        self.position = self.sim.get_position(self.handle)
        self.orientation = self.sim.get_orientation(self.handle)

    def move(self, v_left, v_right):
        self.sim.set_joint_target_velocity(self.motor_handle[0], v_left)
        self.sim.set_joint_target_velocity(self.motor_handle[1], v_right)

    ## stop all motors
    def stop(self):
        for motor in self.motor_handle:
            self.sim.set_joint_target_velocity(motor, 0)

    ## drive our robot
    def drive(self, v_linear, v_angular):
        self.sim.set_joint_target_velocity(self.motor_handle[0], \
                                        self.v_L(v_linear, v_angular))
        self.sim.set_joint_target_velocity(self.motor_handle[1], \
                                        self.v_R(v_linear, v_angular))

    ## get left motor velocity
    def v_L(self, v_linear, v_angular):
        return (2*v_linear + self.L*v_angular) / (2*self.R)

    ## get right motor velocity
    def v_R(self, v_linear, v_angular):
        return (2*v_linear - self.L*v_angular) / (2*self.R)

    ### debug
    def print_pose(self):
        print ('[' + str(self.position[0]) + ', ' + str(self.position[1]) + ', ' + \
                str(self.orientation[2]) + ']')
