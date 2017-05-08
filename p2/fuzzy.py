# -*- coding: utf-8 -*-

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from matplotlib import pyplot as plt

V_LINEAR = 1
V_ANGULAR = .75
MARGIN_FOR_ERROR = .0001
MARGIN_FOR_OBSTACLES = .4

class fuzzy:
    def __init__(self, robot):
        self.robot = robot
        self.bake_space()
        self.bake_rules()

        self.vmax = .45
        self.vmin = .05

    ## updates AI states
    def tick(self):
        # self.orientation_rule.input['perception_angle'] = 0.2
        # self.orientation_rule.input['perception'] = 0.2
        # self.orientation_rule.compute()

        # orientation = self.orientation_rule.output['turn']

        return

    ###
    ### fuzzy space
    ###
    ## build our fuzzy rules
    def bake_space(self):
        ## 
        ## inputs
        ##
        angles_space = np.arange(-180, 180, 1)
        general_space = np.arange(0, 1, .1)

        ## rules for perception angle
        self.perception_angle = ctrl.Antecedent(angles_space, 'perception angle $\\alpha$')
        self.set_perception_angle(self.perception_angle)

        ## rules for general perception
        self.perception = ctrl.Antecedent(general_space, 'general perception $p$')
        self.set_perception(self.perception)

        ## rules for general change
        self.perception_change = ctrl.Antecedent(general_space, 'perception change $p*$')
        self.set_perception_change(self.perception_change)

        ## 
        ## outputs
        ##
        orientation_space = np.arange(-30, 30, 1)
        steer_space = np.arange(-100, 100, 1)
        acc_space = np.arange(-.4, .4, .1)

        ## rules for turning
        self.turn = ctrl.Antecedent(orientation_space, 'turn $\\phi[\\deg/s]$')
        self.set_turn(self.turn)

        ## rules for steer
        self.steer = ctrl.Antecedent(steer_space, 'steer $\\Psi[\\deg/s]$')
        self.set_steer(self.steer)

        ## rules for acceleration
        self.acceleration = ctrl.Antecedent(acc_space, 'acceleration $v[m/s^2]$')
        self.set_acceleration(self.acceleration)

    ## 
    ## inputs
    ##
    def set_perception_angle(self, fuzzy):
        fuzzy['right_back'] = fuzz.trimf(fuzzy.universe, [-210, -135, -60])
        fuzzy['right_front'] = fuzz.trimf(fuzzy.universe, [-120, -45, 30])
        fuzzy['left_front'] = fuzz.trimf(fuzzy.universe, [-30, 45, 120])
        fuzzy['left_back'] = fuzz.trimf(fuzzy.universe, [60, 135, 210])

    def set_perception(self, fuzzy):
        fuzzy['very_low'] = fuzz.trapmf(fuzzy.universe, [0, 0, .2, .4])
        fuzzy['low'] = fuzz.trimf(fuzzy.universe, [.2, .4, .6])
        fuzzy['medium'] = fuzz.trimf(fuzzy.universe, [.3, .5, .7])
        fuzzy['high'] = fuzz.trimf(fuzzy.universe, [.4, .6, .8])
        fuzzy['very_high'] = fuzz.trapmf(fuzzy.universe, [.6, .8, 1, 1])

    def set_perception_change(self, fuzzy):
        fuzzy['zero'] = fuzz.trimf(fuzzy.universe, [0, 0, .3])
        fuzzy['low'] = fuzz.trimf(fuzzy.universe, [0, .2, .5])
        fuzzy['high'] = fuzz.trapmf(fuzzy.universe, [.2, .7, 1, 1])

    ## 
    ## outputs
    ##
    def set_turn(self, fuzzy):
        fuzzy['right'] = fuzz.trimf(fuzzy.universe, [-30, -30, -25])
        fuzzy['little_right'] = fuzz.trimf(fuzzy.universe, [-7.5, -5, -2.5])
        fuzzy['little_left'] = fuzz.trimf(fuzzy.universe, [2.5, 5, 7.5])
        fuzzy['left'] = fuzz.trimf(fuzzy.universe, [25, 30, 30])

    def set_steer(self, fuzzy):
        fuzzy['hard_right'] = fuzz.trapmf(fuzzy.universe, [-100, -100, -60, -30])
        fuzzy['right'] = fuzz.trimf(fuzzy.universe, [-60, -30, 0])
        fuzzy['center'] = fuzz.trimf(fuzzy.universe, [-30, 0, 30])
        fuzzy['left'] = fuzz.trimf(fuzzy.universe, [0, 30, 60])
        fuzzy['hard_left'] = fuzz.trapmf(fuzzy.universe, [30, 60, 100, 100])

    def set_acceleration(self, fuzzy):
        fuzzy['em_brake'] = fuzz.trimf(fuzzy.universe, [-4, -.4, -.35])
        fuzzy['brake'] = fuzz.trapmf(fuzzy.universe, [-.4, -.3, -.1, 0])
        fuzzy['zero'] = fuzz.trimf(fuzzy.universe, [-.15, 0, .15])
        fuzzy['positive'] = fuzz.trapmf(fuzzy.universe, [0, .1, .4, .4])

    ###
    ### graphics
    ###
    ## display our current rules
    def display_space(self):
        self.perception_angle.view()
        plt.show()

        self.perception.view()
        plt.show()

        self.perception_change.view()
        plt.show()

        self.turn.view()
        plt.show()

        self.steer.view()
        plt.show()

        self.acceleration.view()
        plt.show()

    ###
    ### rules
    ###
    ## prepare our rules
    def bake_rules(self):
        self.set_orientation_rule()
        self.set_directional_rule()
        self.set_speed_rule()

    ## rule  base  controlling the orientation of the robot
    ## via turning speed 
    ##   α | RB | RF | LF | LB | RB    | LF    |
    ## p   |    |    |    |    | or RF | or LF |
    ## --  | LL | LR | LL | LR | --    | --    |
    ## VL  | -- | -- | -- | -- | L     | R     |
    def set_orientation_rule(self):
        rule1 = ctrl.Rule(self.perception_angle['right_back'], \
                            self.turn['little_left'])
        rule2 = ctrl.Rule(self.perception_angle['right_front'], \
                            self.turn['little_right']) 
        rule3 = ctrl.Rule(self.perception_angle['left_front'], \
                            self.turn['little_left'])
        rule4 = ctrl.Rule(self.perception_angle['left_back'], \
                            self.turn['little_right']) 

        rule5 = ctrl.Rule((self.perception_angle['right_back'] | \
                            self.perception_angle['right_front']) & 
                            self.perception['very_low'], \
                            self.turn['left']) 
        rule6 = ctrl.Rule((self.perception_angle['left_back'] | \
                            self.perception_angle['left_front']) & 
                            self.perception['very_low'], \
                            self.turn['right'])

        orientation_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, \
                                               rule4, rule5, rule6])
        self.orientation = ctrl.ControlSystemSimulation(orientation_ctrl)

    ## rule  base  controlling the robot direction of movement 
    ## via steering speed 
    ##   p | VL | L  | M  | H  | VH |
    ## α   |    |    |    |    |    |
    ## RB  | HR | HR | R  | R  | C  |
    ## RF  | C  | L  | L  | HL | HL |
    ## LF  | C  | R  | R  | HR | HR |
    ## LB  | HL | HL | L  | L  | C  |
    def set_directional_rule(self):
        rule11 = ctrl.Rule(self.perception['very_low'] & \
                            self.perception_angle['right_back'], \
                            self.steer['hard_right'])
        rule12 = ctrl.Rule(self.perception['low'] & \
                            self.perception_angle['right_back'], \
                            self.steer['hard_right'])
        rule13 = ctrl.Rule(self.perception['medium'] & \
                            self.perception_angle['right_back'], \
                            self.steer['right'])
        rule14 = ctrl.Rule(self.perception['high'] & \
                            self.perception_angle['right_back'], \
                            self.steer['right'])
        rule15 = ctrl.Rule(self.perception['very_high'] & \
                            self.perception_angle['right_back'], \
                            self.steer['center'])

        rule21 = ctrl.Rule(self.perception['very_low'] & \
                            self.perception_angle['right_front'], \
                            self.steer['center'])
        rule22 = ctrl.Rule(self.perception['low'] & \
                            self.perception_angle['right_front'], \
                            self.steer['left'])
        rule23 = ctrl.Rule(self.perception['medium'] & \
                            self.perception_angle['right_front'], \
                            self.steer['left'])
        rule24 = ctrl.Rule(self.perception['high'] & \
                            self.perception_angle['right_front'], \
                            self.steer['hard_left'])
        rule25 = ctrl.Rule(self.perception['very_high'] & \
                            self.perception_angle['right_front'], \
                            self.steer['hard_left'])

        rule31 = ctrl.Rule(self.perception['very_low'] & \
                            self.perception_angle['left_front'], \
                            self.steer['center'])
        rule32 = ctrl.Rule(self.perception['low'] & \
                            self.perception_angle['left_front'], \
                            self.steer['right'])
        rule33 = ctrl.Rule(self.perception['medium'] & \
                            self.perception_angle['left_front'], \
                            self.steer['right'])
        rule34 = ctrl.Rule(self.perception['high'] & \
                            self.perception_angle['left_front'], \
                            self.steer['hard_right'])
        rule35 = ctrl.Rule(self.perception['very_high'] & \
                            self.perception_angle['left_front'], \
                            self.steer['hard_right'])

        rule41 = ctrl.Rule(self.perception['very_low'] & \
                            self.perception_angle['left_back'], \
                            self.steer['hard_left'])
        rule42 = ctrl.Rule(self.perception['low'] & \
                            self.perception_angle['left_back'], \
                            self.steer['hard_left'])
        rule43 = ctrl.Rule(self.perception['medium'] & \
                            self.perception_angle['left_back'], \
                            self.steer['left'])
        rule44 = ctrl.Rule(self.perception['high'] & \
                            self.perception_angle['left_back'], \
                            self.steer['left'])
        rule45 = ctrl.Rule(self.perception['very_high'] & \
                            self.perception_angle['left_back'], \
                            self.steer['center'])

        directional_ctrl = \
            ctrl.ControlSystem([rule11, rule12, rule13, rule14, rule15, \
                                rule21, rule22, rule23, rule24, rule25, \
                                rule31, rule32, rule33, rule34, rule35, \
                                rule41, rule42, rule43, rule44, rule45])
        self.directional = ctrl.ControlSystemSimulation(directional_ctrl)

    ## rule base controlling  the  speed  of the robot 
    ## via acceleration 
    ##   p | VL    | L    | ME | -- |
    ## p*  | or VH | or H |    |    |
    ## ZE  | ZE    | P    | P  | -- |
    ## L   | EB    | B    | Z  | -- |
    ## H   | --    | --   | -- | EB |
    def set_speed_rule(self):
        rule11 = ctrl.Rule((self.perception['very_low'] | \
                            self.perception['very_high']) & \
                            self.perception_change['zero'], \
                            self.acceleration['zero'])
        rule12 = ctrl.Rule((self.perception['low'] | \
                            self.perception['high']) & \
                            self.perception_change['zero'], \
                            self.acceleration['positive'])
        rule13 = ctrl.Rule(self.perception['medium'] & \
                            self.perception_change['zero'], \
                            self.acceleration['positive'])

        rule21 = ctrl.Rule((self.perception['very_low'] | \
                            self.perception['very_high']) & \
                            self.perception_change['low'], \
                            self.acceleration['em_brake'])
        rule22 = ctrl.Rule((self.perception['low'] | \
                            self.perception['high']) & \
                            self.perception_change['low'], \
                            self.acceleration['brake'])
        rule23 = ctrl.Rule(self.perception['medium'] & \
                            self.perception_change['low'], \
                            self.acceleration['zero'])

        rule31 = ctrl.Rule(self.perception_change['high'], \
                            self.acceleration['em_brake'])

        speed_ctrl = ctrl.ControlSystem([rule11, rule12, rule13, \
                                         rule21, rule22, rule23, \
                                         rule31])
        self.speed = ctrl.ControlSystemSimulation(speed_ctrl)

if __name__ == "__main__":
    ai = fuzzy("robot")
    ai.display_space()
