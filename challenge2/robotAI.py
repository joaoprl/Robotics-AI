import math
from robot import *

MAX_SPEED = 0.7
UNCERT = 0.2

class robotAI:
    def __init__(self, p3dx):
        self.verbose = True
        self.p3dx = p3dx

        if self.verbose:
            print('> AI started')

        self.too_far = 0.8
        self.too_close = 0.2

        self.dists = [-1] * len(SONAR_ANGLES_PLOT)

        self.fuzzy_rwall = 1
        self.fuzzy_lwall = 1
        self.fuzzy_fwall = 1
        self.fuzzy_bwall = 1

        self.lfuzzy_rwall = self.fuzzy_rwall
        self.lfuzzy_lwall = self.fuzzy_lwall
        self.lfuzzy_fwall = self.fuzzy_fwall
        self.lfuzzy_bwall = self.fuzzy_bwall

    ## updates AI states
    def tick(self):
        self.use_sensors()
        self.move()

    def use_sensors(self):
        right_wall = []
        left_wall = []
        front_wall = []
        back_wall = []
        for i in range (0, len(SONAR_ANGLES_PLOT)):
            p = SONAR_ANGLES_PLOT[i]
            d = self.p3dx.sonar_readings[i]
            self.dists[i] = d

            if math.sin(p) < 0:
                if d != -1:
                    right_wall += [(-math.sin(p), d)]
            if math.sin(p) > 0:
                if d != -1:
                    left_wall += [(math.sin(p), d)]
            if math.cos(p) > 0:
                if d != -1:
                    front_wall += [(math.cos(p), d)]
            if math.cos(p) < 0:
                if d != -1:
                    back_wall += [(-math.cos(p), d)]

        self.lfuzzy_rwall = self.fuzzy_rwall
        self.lfuzzy_lwall = self.fuzzy_lwall
        self.lfuzzy_fwall = self.fuzzy_fwall
        self.lfuzzy_bwall = self.fuzzy_bwall

        self.fuzzy_rwall = 0
        sum_p = 0
        for p, d in right_wall:
            if d > self.too_far:
                d = self.too_far
            if d < self.too_close:
                d = self.too_close

            d = (d - self.too_close) / (self.too_far - self.too_close)
            sum_p += p
            self.fuzzy_rwall += p * d

        if sum_p > 0:
            self.fuzzy_rwall /= sum_p

        self.fuzzy_lwall = 0
        sum_p = 0
        for p, d in left_wall:
            if d > self.too_far:
                d = self.too_far
            if d < self.too_close:
                d = self.too_close

            d = (d - self.too_close) / (self.too_far - self.too_close)
            sum_p += p
            self.fuzzy_lwall += p * d

        if sum_p > 0:
            self.fuzzy_lwall /= sum_p

        self.fuzzy_fwall = 0
        sum_p = 0
        for p, d in front_wall:
            if d > self.too_far:
                d = self.too_far
            if d < self.too_close:
                d = self.too_close

            d = (d - self.too_close) / (self.too_far - self.too_close)
            sum_p += p
            self.fuzzy_fwall += p * d

        if sum_p > 0:
            self.fuzzy_fwall /= sum_p

        self.fuzzy_bwall = 0
        sum_p = 0
        for p, d in back_wall:
            if d > self.too_far:
                d = self.too_far
            if d < self.too_close:
                d = self.too_close

            d = (d - self.too_close) / (self.too_far - self.too_close)
            sum_p += p
            self.fuzzy_bwall += p * d

        if sum_p > 0:
            self.fuzzy_bwall /= sum_p

        # print("front wall:", self.fuzzy_fwall)
        # print("back wall:", self.fuzzy_bwall)
        # print("left wall:", self.fuzzy_lwall)
        # print("right wall:", self.fuzzy_rwall)

    def move(self):
        afastando = self.fuzzy_rwall - self.lfuzzy_rwall

        while abs(afastando) < 1 and afastando != 0:
            afastando *= 10
        afastando /= 10

        rotate_left = (1 - afastando) * (1 - self.fuzzy_rwall)
        rotate_right = (1 + afastando) * self.fuzzy_rwall


        afastando = self.fuzzy_lwall - self.lfuzzy_lwall

        while abs(afastando) < 1 and afastando != 0:
            afastando *= 10
        afastando /= 10

        rotate_right = max(rotate_right, (-afastando) * (1 - self.fuzzy_lwall))
        rotate_left = rotate_left
        print(rotate_left, rotate_right)

        v_l = rotate_right + MAX_SPEED
        v_r = rotate_left + MAX_SPEED

        self.p3dx.move(v_l, v_r)
        # print(v_l, v_r, self.fuzzy_rwall)
