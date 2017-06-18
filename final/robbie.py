import random

# robot joints names
TAIL_JOINT = "tailJoint"
LEG_TOP_JOINT = "robbieLegJoint1"
LEG_MIDDLE_JOINT = "robbieLegJoint2"
LEG_BOTTOM_JOINT = "robbieLegJoint3"
LEG_JOINT_SUFFIX = ["", "#0", "#1", "#2"]

# reward values
FORWARD_REWARD = 1
SIDEWAYS_PENALTY = -5

# action values
ROTATION_SPEED = 1

# enum for actions
class Action(object):
    cc_rotation = 1
    cw_rotation = 2
    no_rotation = 3

    @staticmethod
    def get_random_action():
        options = [Action.cc_rotation, Action.cw_rotation, Action.no_rotation]
        return random.choice(options)

class Robbie(object):
    def __init__(self, sim, name):
        self.sim = sim                          # simulation environment
        self.name = name                        # robot's name
        self.handle = self.sim.get_handle(name) # robot's id handle

        # id handles
        self.active_joints = []
        self.passive_joints = []

        # get handles of leg joints
        for suffix in LEG_JOINT_SUFFIX:
            self.active_joints += [self.sim.get_handle(LEG_TOP_JOINT + suffix),
                                   self.sim.get_handle(LEG_BOTTOM_JOINT + suffix)]
            self.passive_joints += [self.sim.get_handle(LEG_MIDDLE_JOINT + suffix)]

        # get handle for tail joint
        self.passive_joints += [self.sim.get_handle(TAIL_JOINT)]

        # declare pose, position and speed variables
        self.position = [0] * 3
        self.orientation = [0] * 3
        self.active_pos = [0] * len(self.active_joints)
        self.active_speed = [0] * len(self.active_joints)
        self.passive_pos = [0] * len(self.passive_joints)

        # initial updates
        self.update()
        self.update()

        # save initial values to reset scene
        self.init_position = self.position
        self.init_orientation = self.orientation
        self.init_active_pos = self.active_pos
        self.init_passive_pos = self.passive_pos

    ## reset robot on the scene
    def reset_robot(self):
        self.sim.pause()
        self.sim.set_position(self.handle, self.init_position)
        self.sim.set_orientation(self.handle, self.init_orientation)
        for index, active_joint in enumerate(self.active_joints):
            self.sim.set_joint_position(active_joint, self.init_active_pos[index])
        for index, passive_joint in enumerate(self.passive_joints):
            self.sim.set_joint_position(passive_joint, self.init_passive_pos[index])
        self.active_speed = [0] * len(self.active_joints)
        self.sim.resume()

    ## main update
    def update(self, tick_time=0):
        self.sim.update()
        self.update_pose()
        self.update_sensors()
        self.rotate_joints(tick_time)

    ## update pose
    def update_pose(self):
        self.position = self.sim.get_position(self.handle)
        self.orientation = self.sim.get_orientation(self.handle)

    ## update sensors
    def update_sensors(self):
        self.active_pos = [self.sim.get_joint_position(i) for i in self.active_joints]
        self.passive_pos = [self.sim.get_joint_position(i) for i in self.passive_joints]

    ## rotate active joints based on speed
    def rotate_joints(self, tick_time):
        for index, active_joint in enumerate(self.active_joints):
            new_pos = self.active_pos[index] + self.active_speed[index] * tick_time
            self.sim.set_joint_position(active_joint, new_pos)

    ## return robot current state
    def get_state(self):
        state = self.active_joints
        state += self.passive_joints
        state += [self.position[0]]
        state += self.orientation
        state += self.active_speed
        return state

    ## return current state reward
    def get_reward(self):
        # start with neutral reward
        reward = 0

        # reward for getting far
        reward += self.position[0] * FORWARD_REWARD

        # penalty for getting off track
        reward += abs(self.position[1]) * SIDEWAYS_PENALTY

        return reward

    ## check if robot didn't move for some time
    def is_stuck(self):
        return False

    ## exectute actions on robot
    def act(self, actions):
        for index, action in enumerate(actions):
            if action == Action.cc_rotation:
                self.active_speed[index] = ROTATION_SPEED
            elif action == Action.cw_rotation:
                self.active_speed[index] = -ROTATION_SPEED
            elif action == Action.no_rotation:
                self.active_speed[index] = 0

        # return new state
        return self.get_state(), self.get_reward(), self.is_stuck()

    ### [debug] robot pose
    def print_pose(self):
        print self.position + self.orientation

    ### [debug] robot state
    def print_state(self):
        print self.get_state()
