import math

V_LINEAR = 1
V_ANGULAR = 0.75
MARGIN_FOR_ERROR = 0.0001
MARGIN_FOR_OBSTACLES = 0.4

class robotAI:
    def __init__(self, p3dx):
        self.p3dx = p3dx
        self.goal = None
        self.state = None
        self.goal_first_run = False
        self.state_first_run = False
        self.changed_state = False
        self.flip_movement = False
        self.obstacle_forward = False
        self.obstacle_backward = False
        self.obstacle_left = False
        self.obstacle_right = False
        print('> AI started')
        self.change_goal(self.goal_explore)

    ## updates AI states
    def tick(self):
        self.update_obstacles()
        self.check_stuck()
        self.goal()

    ## changes to new goal and makes sure the first run is differentiated
    def change_goal(self, new_goal, new_state = None):
        if new_goal != self.goal:
            self.goal = new_goal
            print('> Switched to goal \'' + self.get_goal_name() + '\'')
            self.goal_first_run = True
        new_goal(new_state)
        self.goal_first_run = False

    ## changes to new state and makes sure the first run is differentiated
    def change_state(self, new_state):
        if new_state != self.state:
            if self.changed_state:
                print('> State change queued for next tick')
                self.changed_state = False
                return
            self.state = new_state
            print('> Switched to state \'' + self.get_state_name() + '\'')
            self.state_first_run = True
        self.changed_state = True
        new_state()
        self.state_first_run = False

    ## returns name of the function of current goal
    def get_goal_name(self):
        return str(self.goal.__name__).replace('goal_', '')

    ## returns name of the function of current state
    def get_state_name(self):
        return str(self.state.__name__).replace('state_', '')

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
            if self.state == self.state_move_forward or self.state == self.state_move_backward:
                print('> Robot possibly stuck at [' + str(self.p3dx.position[0]) + ', ' +\
                      str(self.p3dx.last_position[0]) + ', ' + str(self.p3dx.orientation[2]) + ']')
                self.change_goal(self.goal_break_free)
                return True
            else:
                self.change_state(self.state_move_forward)
        return False

    ## sets obstacle boolean variables depending on sonar readings
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
            vector_component_y = abs(magnitude * math.cos(angle_radians))
            vector_component_x = abs(magnitude * math.sin(angle_radians))

            if vector_component_y < robot_half_length:
                if angle > 0:
                    self.obstacle_left = True
                else:
                    self.obstacle_right = True

            if vector_component_x < robot_half_width:
                if abs(angle) >= 90:
                    self.obstacle_backward = True
                else:
                    self.obstacle_forward = True

    ## chooses which direction to rotate
    def get_next_rotation_state(self):
        if not self.obstacle_right:
            return self.state_rotate_right
        else:
            return self.state_rotate_left

    ## flips movement
    def set_flipped_movement(self, flip_movement):
        self.flip_movement = flip_movement

    ## goal: explore until find another wall
    def goal_explore(self, new_state = None):
        # start exploration by moving forward
        if self.goal_first_run:
            if new_state != None:
                self.change_state(new_state)
            else:
                self.change_state(self.state_move_forward)
        # if a wall is found, start following it
        if self.state == self.state_move_forward and self.obstacle_forward:
            self.change_goal(self.goal_follow_wall)
        # if it isn't found, continue exploring
        else:
            self.change_state(self.state)

    ## goal: follow wall
    def goal_follow_wall(self, new_state=None):
        # start following by moving forward
        if self.goal_first_run:
            if new_state != None:
                self.change_state(new_state)
            else:
                self.change_state(self.state_move_forward)
        # if no wall is being followed anymore, start exploring
        if self.state == self.state_move_forward and not \
                (self.obstacle_forward or self.obstacle_left or self.obstacle_right):
            # TODO: make robot follow the wall
            self.change_goal(self.goal_explore)
        # if it is, continue following
        else:
            self.change_state(self.state)

    ## goal: try to get unstuck
    def goal_break_free(self, new_state=None):
        if self.goal_first_run:
            if new_state != None:
                self.change_state(new_state)
            # if there's no obstacle in the back, try to move backward
            elif self.state == self.state_move_forward and not self.obstacle_backward:
                self.change_state(self.state_move_backward)
            # if there's no obstacle in the front, try to move forward
            elif self.state == self.state_move_backward and not self.obstacle_forward:
                self.change_state(self.state_move_forward)
            # if nothing works, try rotating
            else:
                self.change_state(self.get_next_rotation_state())
        # if a wall is found, start following it
        if self.state == self.state_move_forward and self.obstacle_forward:
            self.change_goal(self.goal_follow_wall)
        # if a wall is found on the back, start following it, but rotates first
        elif self.state == self.state_move_backward and self.obstacle_backward:
            self.set_flipped_movement(True)
            self.change_goal(self.goal_follow_wall, self.get_next_rotation_state())
        # if it isn't found, continue escaping
        else:
            self.change_state(self.state)

    ## state: stop robot
    def state_stop(self):
        if self.state_first_run:
            self.p3dx.stop()
        if self.check_stopped():
            self.change_state(self.state_move_forward)

    ## state: move robot forward
    def state_move_forward(self):
        if self.state_first_run:
            self.p3dx.drive(V_LINEAR, 0)
        if self.obstacle_forward:
            self.change_state(self.get_next_rotation_state())

    ## state: move robot backward
    def state_move_backward(self):
        if self.state_first_run:
            self.p3dx.drive(-V_LINEAR, 0)
        if self.obstacle_backward:
            self.change_state(self.get_next_rotation_state())

    ## state: rotate robot left
    def state_rotate_left(self):
        if self.state_first_run:
            self.p3dx.drive(0, V_ANGULAR)
        if self.flip_movement:
            if not self.obstacle_backward:
                self.change_state(self.state_move_forward)
                self.set_flipped_movement(False)
        elif not self.obstacle_forward:
            self.change_state(self.state_move_forward)

    ## state: rotate robot right
    def state_rotate_right(self):
        if self.state_first_run:
            self.p3dx.drive(0, -V_ANGULAR)
        if self.flip_movement:
            if not self.obstacle_backward:
                self.change_state(self.state_move_forward)
                self.set_flipped_movement(False)
        elif not self.obstacle_forward:
            self.change_state(self.state_move_forward)

    ## [debug] ai states and detected obstacle booleans
    def print_ai_state(self):
        print('[ goal: ' + self.get_goal_name() + ', state: ' + self.get_state_name() +\
              ', front: ' + str(self.obstacle_forward) + ', back: ' + str(self.obstacle_backward) +\
              ', left: ' + str(self.obstacle_left) + ', right: ' + str(self.obstacle_right) + ']')
