import copy
import math
from time import time

# remote API script
REMOTE_API_OBJ = 'RemoteAPI'
REMOTE_API_FUNC = 'resetSimulation'

# robot constants
STUCK_MARGIN = 1e-2
STUCK_TIMEOUT = 10
FALL_HEIGHT = 7e-2

# robot joints names
TAIL_JOINT = "tailJoint"
LEG_TOP_JOINT = "robbieLegJoint1"
LEG_MIDDLE_JOINT = "robbieLegJoint2"
LEG_BOTTOM_JOINT = "robbieLegJoint3"
FOOT_TIP = "robbieFootTip"
FOOT_TARGET = "robbieFootTarget"
LEG_JOINT_SUFFIX = ["", "#0", "#1", "#2"]

# reward values
FORWARD_REWARD = 200 # reward for getting far
CONTINUOUS_REWARD = 0 # reward for having same speed as last frame
BACKWARDS_PENALTY = -.5 # penalty for going backwards
ROTATION_PENALTY = -.25 # penalty for getting off track
STUCK_PENALTY = 0 # penalty for getting stuck
FALL_PENALTY = -10 # penalty for falling down
STOP_PENALTY = -5 # penalty for not moving

# state and action contants
STATES_DIM = 36
ACTIONS_DIM = 8

# action values
MAX_SPEED = 0.5 # max speed of feet
MIN_LIMITS = [0, -2e-2, -1e-2] # min relative position of each foot
MAX_LIMITS = [0, 2e-2, 2e-2] # max relative position of each foot

class Robbie(object):
    def __init__(self, sim, name):
        self.sim = sim                          # simulation environment
        self.name = name                        # robot's name
        self.handle = self.sim.get_handle(name) # robot's id handle

        # last tick time
        self.last_tick = time()

        # id handles
        self.foot_tips = []
        self.foot_targets = []
        self.robot_joints = []

        # get handles of leg joints and foot summies
        for suffix in LEG_JOINT_SUFFIX:
            self.foot_tips += [self.sim.get_handle(FOOT_TIP + suffix)]
            self.foot_targets += [self.sim.get_handle(FOOT_TARGET + suffix)]
            self.robot_joints += [self.sim.get_handle(LEG_TOP_JOINT + suffix),
                                  self.sim.get_handle(LEG_MIDDLE_JOINT + suffix),
                                  self.sim.get_handle(LEG_BOTTOM_JOINT + suffix)]

        # get handle for tail joint
        self.robot_joints += [self.sim.get_handle(TAIL_JOINT)]

        # declare pose, position and speed variables
        self.position = [0] * 3
        self.orientation = [0] * 3
        self.tips_position = [0] * len(self.foot_tips)
        self.tips_speed = [0] * (2 * len(self.foot_tips))
        self.joints_position = [0] * len(self.foot_targets)

        # relative positions
        self.tips_rel_position = [0] * len(self.foot_tips)
        self.init_rel_position = [0] * len(self.tips_rel_position)
        self.max_positions = [0] * len(self.tips_rel_position)
        self.min_positions = [0] * len(self.tips_rel_position)

        # last frame variables
        self.last_position = [0] * 3
        self.last_orientation = [0] * 3
        self.last_speed = [0] * len(self.tips_speed)

        # stuck and fallen check variables
        self.is_stuck = False
        self.has_stopped = False
        self.stuck_position = [0] * 3
        self.stuck_time = 0
        self.has_fallen = False

        # initial update
        self.pre_update()

    ## reset robot on the scene
    def reset_robot(self):
        # reset server through script
        self.sim.execute_script(REMOTE_API_OBJ, REMOTE_API_FUNC)
        self.sim.disconnect()
        self.sim.connect()

        # reset variables
        self.last_speed = [0] * len(self.tips_speed)
        self.is_stuck = False
        self.has_stopped = False
        self.stuck_position = [0] * 3
        self.stuck_time = 0

        # initial update
        self.pre_update()

    # first update to be run
    def pre_update(self):
        self.sim.update()
        self.update_pose(True)
        self.update_sensors(True)
        self.sim.update()
        self.update_pose(False)
        self.update_sensors(False)
        self.calculate_limits()

    ## main update
    def update(self):
        # get tick delta time
        now_tick = time()
        tick_time = now_tick - self.last_tick
        self.last_tick = now_tick

        # update robot feet position
        self.move_feet(tick_time)

        # update simulator after rotations
        self.sim.update()
        self.update_pose(False)
        self.update_sensors(False)
        self.check_stuck(tick_time)
        self.check_fallen()

    ## update pose
    def update_pose(self, first_time):
        self.last_position = copy.copy(self.position)
        self.last_orientation = copy.copy(self.orientation)
        self.position = self.sim.get_position(self.handle, first_time)
        self.orientation = self.sim.get_orientation(self.handle, first_time)

    ## update sensors
    def update_sensors(self, first_time):
        self.joints_position = [self.sim.get_joint_position(i, first_time) for i in self.robot_joints]
        self.tips_position = [self.sim.get_position(i, first_time) for i in self.foot_tips]
        self.tips_rel_position = [self.sim.get_position(i, first_time, True) for i in self.foot_targets]

    ## move robot feet targets
    def move_feet(self, tick_time):
        for i, foot_target in enumerate(self.foot_targets):
            index = i * 2
            tick_move = MAX_SPEED * tick_time

            # calculate wanted values
            target_delta = [0] * 3
            target_delta[0] = 0
            target_delta[1] = self.tips_speed[index] * tick_move
            target_delta[2] = self.tips_speed[index + 1] * tick_move

            # clamp values
            new_rel_position = [a + b for a, b in zip(self.tips_rel_position[i], target_delta)]
            for j, _ in enumerate(new_rel_position):
                new_rel_position[j] = min(new_rel_position[j], self.max_positions[i][j])
                new_rel_position[j] = max(new_rel_position[j], self.min_positions[i][j])

            self.sim.set_position(foot_target, new_rel_position, True)

    ## return robot current state
    def get_state(self):
        state = []
        for tip_position in self.tips_position:
            relative_position = [a - b for a, b in zip(self.position, tip_position)]
            state += relative_position  # 12 states (4 feet tips position 3 axis)
        state += self.tips_speed        # 8 states (4 feet targets speed 2 axis)
        state += self.joints_position   # 13 states (passive joints position)
        state += self.orientation       # 3 states (robot orientation 3 axis)
        return state                    # total: 36 states

    ## return current state reward
    def get_reward(self):
        # start with neutral reward
        reward = 0

        # get position and orientation diff
        diff_position = [a - b for a, b in zip(self.position, self.last_position)]
        diff_orientation = [a - b for a, b in zip(self.orientation, self.last_orientation)]

        # calculate distance
        distance = math.sqrt(math.pow(diff_position[0], 2) + math.pow(diff_position[1], 2))

        # calculate diff angle
        diff_angle = diff_orientation[2]
        if diff_angle > math.pi:
            diff_angle -= 2 * math.pi
        elif diff_angle < -math.pi:
            diff_angle += 2 * math.pi
        diff_angle_deg = abs(diff_angle) * 180 / math.pi

        # calculate direction
        last_angle = self.last_orientation[2]
        angle_vector = [-math.sin(last_angle), math.cos(last_angle), 0]
        dot_product = angle_vector[0] * diff_position[0] + angle_vector[1] * diff_position[1]
        direction = math.copysign(1, dot_product)

        # calculate if targets have same speed than last frame
        same_speeds = [math.copysign(1, a) == math.copysign(1, b) for a, b in zip(self.tips_speed, self.last_speed)]

        # reward for getting far or penalty for going backwards
        if direction == 1:
            reward += distance * FORWARD_REWARD
        else:
            reward += distance * BACKWARDS_PENALTY

        # penalty for getting off track
        reward += diff_angle_deg * ROTATION_PENALTY

        # reward for having same speed as last frame
        for same_speed in same_speeds:
            if same_speed:
                reward += CONTINUOUS_REWARD

        # penalty for getting stuck
        if self.is_stuck:
            reward += STUCK_PENALTY

        # penalty for falling down
        if self.has_fallen:
            reward += FALL_PENALTY

        # penalty for not moving
        if self.has_stopped:
            reward += STOP_PENALTY

        return reward

    ## check if robot didn't move for some time
    def check_stuck(self, tick_time):
        is_close = True
        for i in range(3):
            diff_pos = abs(self.stuck_position[i] - self.position[i])
            if diff_pos >= STUCK_MARGIN:
                is_close = False
                break
        if is_close:
            self.stuck_time += tick_time
            self.has_stopped = True
            self.is_stuck = self.stuck_time >= STUCK_TIMEOUT
        else:
            self.stuck_time = 0
            self.stuck_position = self.position
            self.has_stopped = False
            self.is_stuck = False

    ## check if robot has fallen
    def check_fallen(self):
        self.has_fallen = self.position[2] < FALL_HEIGHT

    ## calculate min and max position for each foot
    def calculate_limits(self):
        self.init_rel_position = copy.copy(self.tips_rel_position)
        for i, rel_position in enumerate(self.init_rel_position):
            self.max_positions[i] = [a + b for a, b in zip(rel_position, MAX_LIMITS)]
            self.min_positions[i] = [a + b for a, b in zip(rel_position, MIN_LIMITS)]

    ## exectute actions on robot
    def act(self, actions):
        # perform actions
        self.last_speed = copy.copy(self.tips_speed)
        for i, action in enumerate(actions):
            self.tips_speed[i] = action * MAX_SPEED

        # update robot on simulator
        self.update()

        # check if should finish
        done = self.is_stuck or self.has_fallen

        # return new state
        return self.get_state(), self.get_reward(), done

    @staticmethod
    ## return states and actions dimensions
    def get_dimensions():
        return STATES_DIM, ACTIONS_DIM

    ### [debug] robot pose
    def print_pose(self):
        print(self.position + self.orientation)

    ### [debug] robot state
    def print_state(self):
        print(self.get_state())
