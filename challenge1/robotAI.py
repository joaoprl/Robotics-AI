import math

V_LINEAR = 1
V_ANGULAR = 0.75
MARGIN_FOR_ERROR = 0.0001
MARGIN_FOR_OBSTACLES = 0.4

class robotAI:
    def __init__(self, p3dx):
        self.p3dx = p3dx
        self.goal = self.stop
        self.state = self.stop
        self.state_first_run = True
        self.obstacle_forward = False
        self.obstacle_backward = False
        self.obstacle_left = False
        self.obstacle_right = False
        print('> AI started')
        self.change_state(self.stop)

    ## updates AI states
    def tick(self):
        self.update_obstacles()
        self.check_stuck()
        self.state()

    ## changes to new state and makes sure the first run is differentiated
    def change_state(self, new_state):
        self.state = new_state
        print('> Switched to state \'' + self.get_state_name() + '\'')
        self.state_first_run = True
        new_state()
        self.state_first_run = False

    def get_state_name(self):
        return str(self.state.__name__)

    ## checks with robot has stopped (or almost stopped)
    def check_stopped(self, is_exact = False):
        if is_exact:
            return self.p3dx.last_position == self.p3dx.position
        else:
            return abs(self.p3dx.last_position[0] - self.p3dx.position[0]) < MARGIN_FOR_ERROR and \
                   abs(self.p3dx.last_position[1] - self.p3dx.position[1]) < MARGIN_FOR_ERROR

    ## checks if robot is stuck while trying to move
    def check_stuck(self):
        if self.check_stopped():
            if self.state == self.move_forward or self.state == self.move_backward:
                print('> Robot possibly stuck at [' + str(self.p3dx.position[0]) + ', ' +\
                      str(self.p3dx.last_position[0]) + ', ' + str(self.p3dx.orientation[2]) + ']')
                return True
            else:
                self.change_state(self.move_forward)
        return False

    # sets obstacle boolean variables depending on sonar readings
    def update_obstacles(self):
        self.obstacle_forward = False
        self.obstacle_backward = False
        self.obstacle_left = False
        self.obstacle_right = False

        robot_half_length = self.p3dx.get_robot_length() / 2
        robot_half_width = self.p3dx.get_robot_width() / 2
        sonar_angles = self.p3dx.get_sonar_angles()
        obstacle_positions = self.p3dx.get_rel_sonar_readings()

        for position in obstacle_positions:
            magnitude = math.sqrt(math.pow(position[0], 2) + math.pow(position[1], 2))
            if magnitude > MARGIN_FOR_OBSTACLES:
                continue

            angle = sonar_angles[position[3]]
            angle_radians = math.radians(angle)
            vector_component_y = math.fabs(magnitude * math.cos(angle_radians))
            vector_component_x = math.fabs(magnitude * math.sin(angle_radians))

            if vector_component_y < robot_half_length:
                if angle > 0:
                    self.obstacle_left = True
                else:
                    self.obstacle_right = True

            if vector_component_x < robot_half_width:
                if math.fabs(angle) >= 90:
                    self.obstacle_backward = True
                else:
                    self.obstacle_forward = True

    ## state: stop robot
    def stop(self):
        if self.state_first_run:
            self.p3dx.stop()
        if self.check_stopped():
            self.change_state(self.move_forward)

    ## state: move robot forward
    def move_forward(self):
        if self.state_first_run:
            self.p3dx.drive(V_LINEAR, 0)
        if self.obstacle_forward:
            if not self.obstacle_right:
                self.change_state(self.rotate_right)
            else:
                self.change_state(self.rotate_left)

    ## state: move robot backward
    def move_backward(self):
        if self.state_first_run:
            self.p3dx.drive(-V_LINEAR, 0)
        if self.obstacle_backward:
            if not self.obstacle_right:
                self.change_state(self.rotate_left)
            else:
                self.change_state(self.rotate_right)

    ## state: rotate robot left
    def rotate_left(self):
        if self.state_first_run:
            self.p3dx.drive(0, V_ANGULAR)
        if not self.obstacle_forward:
            self.change_state(self.move_forward)

    ## state: rotate robot right
    def rotate_right(self):
        if self.state_first_run:
            self.p3dx.drive(0, -V_ANGULAR)
        if not self.obstacle_forward:
            self.change_state(self.move_forward)

    ## [debug] ai states and detected obstacle booleans
    def print_ai_state(self):
        print('[ state: ' + self.get_state_name() +\
              ', front: ' + str(self.obstacle_forward) + ', back: ' + str(self.obstacle_backward) +\
              ', left: ' + str(self.obstacle_left) + ', right: ' + str(self.obstacle_right) + ']')
