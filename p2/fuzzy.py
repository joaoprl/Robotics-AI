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

    ## updates AI states
    def tick(self):
        return

    ## build our fuzzy rules
    def prepare_space(self):
        ### 
        ### inputs
        ###
        angles_space = np.arange(-180, 180, 1)
        general_space = np.arange(0, 1, .1)

        ## rules for perception angle
        self.perception_angle = ctrl.Antecedent(angles_space, 'perception angle $\\alpha$')
        self.perception_angle['right_back'] = fuzz.trimf(angles_space, [-210, -135, -60])
        self.perception_angle['right_front'] = fuzz.trimf(angles_space, [-120, -45, 30])
        self.perception_angle['left_front'] = fuzz.trimf(angles_space, [-30, 45, 120])
        self.perception_angle['left_back'] = fuzz.trimf(angles_space, [60, 135, 210])

        ## rules for general perception
        self.perception = ctrl.Antecedent(general_space, 'general perception $p$')
        self.perception['very_low'] = fuzz.trapmf(general_space, [0, 0, .2, .4])
        self.perception['low'] = fuzz.trimf(general_space, [.2, .4, .6])
        self.perception['medium'] = fuzz.trimf(general_space, [.3, .5, .7])
        self.perception['high'] = fuzz.trimf(general_space, [.4, .6, .8])
        self.perception['very_high'] = fuzz.trapmf(general_space, [.6, .8, 1, 1])

        ## rules for general change
        self.perception_change = ctrl.Antecedent(general_space, 'perception change $p*$')
        self.perception_change['zero'] = fuzz.trimf(general_space, [0, 0, .3])
        self.perception_change['low'] = fuzz.trimf(general_space, [0, .2, .5])
        self.perception_change['high'] = fuzz.trapmf(general_space, [.2, .7, 1, 1])

        ### 
        ### outputs
        ###
        orientation_space = np.arange(-30, 30, 1)
        steer_space = np.arange(-100, 100, 1)
        acc_space = np.arange(-.4, .4, .1)

        ## rules for turning
        self.turn = ctrl.Antecedent(orientation_space, 'turn $\\phi[\\deg/s]$')
        self.turn['right'] = fuzz.trimf(orientation_space, [-30, -30, -25])
        self.turn['little_right'] = fuzz.trimf(orientation_space, [-7.5, -5, -2.5])
        self.turn['little_left'] = fuzz.trimf(orientation_space, [2.5, 5, 7.5])
        self.turn['left'] = fuzz.trimf(orientation_space, [25, 30, 30])

        ## rules for steer
        self.steer = ctrl.Antecedent(steer_space, 'steer $\\Psi[\\deg/s]$')
        self.steer['hard_right'] = fuzz.trapmf(steer_space, [-100, -100, -60, -30])
        self.steer['right'] = fuzz.trimf(steer_space, [-60, -30, 0])
        self.steer['center'] = fuzz.trimf(steer_space, [-30, 0, 30])
        self.steer['left'] = fuzz.trimf(steer_space, [0, 30, 60])
        self.steer['hard_left'] = fuzz.trapmf(steer_space, [30, 60, 100, 100])

        ## rules for accelaration
        self.accelaration = ctrl.Antecedent(acc_space, 'accelaration $v[m/s^2]$')
        self.accelaration['em_brake'] = fuzz.trimf(acc_space, [-4, -.4, -.35])
        self.accelaration['brake'] = fuzz.trapmf(acc_space, [-.4, -.3, -.1, 0])
        self.accelaration['zero'] = fuzz.trimf(acc_space, [-.15, 0, .15])
        self.accelaration['positive'] = fuzz.trapmf(acc_space, [0, .1, .4, .4])

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

        self.accelaration.view()
        plt.show()

    def prepare_rules(self):
        orientation_rule = ctrl.Rule(self.perception_angle['right_back'], \
            turn['little_left'])

if __name__ == "__main__":
    ai = fuzzy("robot")
    ai.prepare_space()
    ai.display_space()
