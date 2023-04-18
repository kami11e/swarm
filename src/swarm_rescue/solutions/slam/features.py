import numpy as np
import math
from fractions import Fraction
from scipy.odr import *

# https://github.com/startgram/SlamNavDrone/tree/9b8b015e24730c0d65cd3608c5ed2dab3e7de5b9/SLAM

Landmarks = []


class featuresDetection:
    def __init__(self):
        self.EPSILON = 10
        self.DELTA = 501
        self.SNUM = 6
        self.PMIN = 20
        self.GMAX = 20
        self.SEED_SEGMENTS = []
        self.LINE_SEGMENTS = []
        self.LASERPOINTS = []
        self.LINE_PARAMS = None
        self.NP = len(self.LASERPOINTS) - 1
        self.LMIN = 20 # minimum len of a segment
        self.LR = 0 # real len of a line seg
        self.PR = 0 # num of lidar pts in a line seg
        self.FEATURES = []

    # euclidian distance from a pt to another
    def dist_point2point(self, point1, point2):
        Px = (point1[0] - point2[0]) ** 2
        Py = (point1[1] - point2[1]) ** 2
        return math.sqrt(Px + Py)

    # dist from a pt to a line (line in form (a,b,c))
    def dist_point2line(self, params, point):
        A, B, C = params
        distance = abs(A * point[0] + B * point[1] + C) / math.sqrt(A ** 2 + B ** 2)
        return distance

    # extract 2 pts from a line eq under the slope intercepts form
    def line_2points(self, m, b, x=5, x2=2000):
        y = m * x + b
        y2 = m * x2 + b
        return [(x, y), (x2, y2)]

    # general form -> slope intercept
    def lineForm_G2SI(self, A, B, C):
        m = -A / B
        B = -C / B
        return m, B

    # slope intercept -> general form
    def lineForm_Si2G(self, m, B):
        A, B, C = -m, 1, -B
        if A < 0:
            A, B, C = -A, -B, -C
        den_a = Fraction(A).limit_denominator(1000).as_integer_ratio()[1]
        den_c = Fraction(C).limit_denominator(1000).as_integer_ratio()[1]

        gcd = np.gcd(den_a, den_c)
        lcm = den_a * den_c / gcd

        A = A * lcm
        B = B * lcm
        C = C * lcm
        return A, B, C

    def line_intersect_general(self, params1, params2):
        a1, b1, c1 = params1
        a2, b2, c2 = params2
        x = (c1 * b2 - b1 * c2) / (b1 * a2 - a1 * b2)
        y = (a1 * c2 - a2 * c1) / (b1 * a2 - a1 * b2)
        return x, y

    def points_2line(self, point1, point2):
        m, b = 0, 0
        if point2[0] == point1[0]:
            pass
        else:
            m = (point2[1] - point1[1]) / (point2[0] - point1[0])
            b = point2[1] - m * point2[0]
        return m, b

    def projection_point2line(self, point, m, b):
        x, y = point
        m2 = -1 / m
        c2 = y - m2 * x
        intersection_x = - (b - c2) / (m - m2)
        intersection_y = m2 * intersection_x + c2
        return intersection_x, intersection_y

    def AD2pos(self, distance, angle, robot_position):
        x = distance * math.cos(angle) + robot_position[0]
        y = -distance * math.sin(angle) + robot_position[1]
        return (int(x), int(y))

    def laser_points_set(self, data, angles, robot_pos):
        self.LASERPOINTS = []
        if len(data)==0:
            pass
        else:
            for i in range(len(data)):
                coordinates = self.AD2pos(data[i], angles[i], robot_pos)
                self.LASERPOINTS.append([coordinates, angles[i]])
        self.NP = len(self.LASERPOINTS) - 1

    def liner_func(self, p, x):
        m, b = p
        return m * x + b

    def odr_fit(self, laser_points):
        x = np.array([i[0][0] for i in laser_points])
        y = np.array([i[0][1] for i in laser_points])

        linear_model = Model(self.liner_func)

        data = RealData(x, y)

        odr_model = ODR(data, linear_model, beta0=[0., 0.])

        out = odr_model.run()
        m, b = out.beta
        return m, b

    def predictPoint(self, line_params, sensed_point, robotpos):
        m, b = self.points_2line(robotpos, sensed_point)
        params1 = self.lineForm_Si2G(m, b)
        predx, predy = self.line_intersect_general(params1, line_params)
        return predx, predy

    def seed_segment_detection(self, robot_position, break_point_ind):
        flag = True
        self.NP = max(0, self.NP)
        self.SEED_SEGMENTS = []
        for i in range(break_point_ind, (self.NP - self.PMIN)):
            predicted_points_to_draw = []
            j = i + self.SNUM
            m, c = self.odr_fit(self.LASERPOINTS[i:j])

            params = self.lineForm_Si2G(m, c)

            for k in range(i, j):
                predicted_point = self.predictPoint(params, self.LASERPOINTS[k][0], robot_position)
                predicted_points_to_draw.append(predicted_point)
                d1 = self.dist_point2point(predicted_point, self.LASERPOINTS[k][0])

                if d1 > self.DELTA:
                    flag = False
                    break

                d2 = self.dist_point2line(params, predicted_point)

                if d2 > self.EPSILON:
                    flag = False
                    break
            if flag:
                self.LINE_PARAMS = params
                return [self.LASERPOINTS[i:j], predicted_points_to_draw, (i, j)]

    def seed_segment_growing(self, indices, break_point):
        line_eq = self.LINE_PARAMS
        i, j = indices
        PB, PF = max(break_point, i - 1), min(j + 1, len(self.LASERPOINTS) - 1)

        while self.dist_point2line(line_eq, self.LASERPOINTS[PF][0]) < self.EPSILON:
            if PF > self.NP - 1:
                break
            else:
                m, b = self.odr_fit(self.LASERPOINTS[PB:PF])
                line_eq = self.lineForm_Si2G(m, b)

                POINT = self.LASERPOINTS[PF][0]
            PF = PF + 1
            NEXTPOINT = self.LASERPOINTS[PF][0]
            if self.dist_point2point(POINT, NEXTPOINT) > self.GMAX:
                break
        PF = PF - 1

        while self.dist_point2line(line_eq, self.LASERPOINTS[PB][0]):
            if PB < break_point:
                break
            else:
                m, b = self.odr_fit(self.LASERPOINTS[PB:PF])
                line_eq = self.lineForm_Si2G(m, b)
                POINT = self.LASERPOINTS[PB][0]

            PB = PB - 1
            NEXTPOINT = self.LASERPOINTS[PB][0]
            if self.dist_point2point(POINT, NEXTPOINT) > self.GMAX:
                break

        PB = PB + 1

        LR = self.dist_point2point(self.LASERPOINTS[PB][0], self.LASERPOINTS[PF][0])
        PR = len(self.LASERPOINTS[PB:PF])

        if (LR >= self.LMIN) and (PR >= self.PMIN):
            self.LINE_PARAMS = line_eq
            m, b = self.lineForm_G2SI(line_eq[0], line_eq[1], line_eq[2])
            self.two_points = self.line_2points(m, b)
            self.LINE_SEGMENTS.append((self.LASERPOINTS[PB + 1][0], self.LASERPOINTS[PF - 1][0]))
            return [self.LASERPOINTS[PB:PF], self.two_points,
                    (self.LASERPOINTS[PB + 1][0], self.LASERPOINTS[PF - 1][0]), PF, line_eq, (m, b)]
        else:
            return False

    def lineFeats2point(self):
        new_rep = []

        for feature in self.FEATURES:
            projection = self.projection_point2line((0, 0), feature[0][0], feature[0][1])
            new_rep.append([feature[0], feature[1], projection])

        return new_rep


def landmark_association(landmarks):
    thresh = 10
    for l in landmarks:
        flag = False
        for i, Landmark in enumerate(Landmarks):
            dist = featuresDetection.dist_point2point(l[2], Landmark[2])
            if dist < thresh:
                if not is_overlap(l[1], Landmark[1]):
                    continue
                else:
                    Landmarks.pop(i)
                    Landmarks.insert(i, l)
                    flag = True

                    break
        if not flag:
            Landmarks.append(l)


def is_overlap(seg1, seg2):
    length1 = featuresDetection.dist_point2point(seg1[0], seg1[1])
    length2 = featuresDetection.dist_point2point(seg2[0], seg2[1])
    center1 = ((seg1[0][0] + seg1[1][0]) / 2, (seg1[0][1] + seg1[1][1]) / 2)
    center2 = ((seg2[0][0] + seg2[1][0]) / 2, (seg2[0][1] + seg2[1][1]) / 2)
    dist = featuresDetection.dist_point2point(center1, center2)
    if dist > (length1 + length2) / 2:
        return False
    else:
        return True