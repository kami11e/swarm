"""
Simple random controller
The Drone will move forward and turn for a random angle when an obstacle is hit
"""
import math
import random
from typing import Optional

import numpy as np

from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.utils.misc_data import MiscData
from spg_overlay.utils.utils import normalize_angle


class MyDroneSolution5(DroneAbstract):
    def __init__(self,
                 identifier: Optional[int] = None,
                 misc_data: Optional[MiscData] = None,
                 **kwargs):
        super().__init__(identifier=identifier,
                         misc_data=misc_data,
                         should_display_lidar=False,
                         **kwargs)
        self.counterStraight = 0
        self.angleStopTurning = random.uniform(-math.pi, math.pi)
        self.distStopStraight = random.uniform(10, 50)
        self.isTurning = False
        self.isFineTurning = False
        self.newTurn = False
        self.alongWall = False
        self.angleStopTurning = 0
        self.lastRightDist = 0
        self.lastLeftDist = 0

    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass

    def process_touch_sensor(self):
        """
        Returns True if the drone hits an obstacle
        """
        if self.touch().get_sensor_values() is None:
            return False

        touched = False
        detection = max(self.touch().get_sensor_values())

        if detection > 0.5:
            touched = True

        return touched

    def process_lidar_sensor(self, lidar):

        values = lidar.get_sensor_values()
        ray_angles = lidar.ray_angles
        size = lidar.resolution
        min_dist = 1000
        if size != 0:
            far_angle_raw = ray_angles[np.argmax(values)]
            min_dist = min(values)
            near_angle_raw = ray_angles[np.argmin(values)]

        far_angle = far_angle_raw
        # If far_angle_raw is small then far_angle = 0
        if abs(far_angle) < 1 / 180 * np.pi:
            far_angle = 0.0

        near_angle = near_angle_raw
        far_angle = normalize_angle(far_angle)

        touched = self.process_touch_sensor()
        print(min_dist, near_angle, far_angle_raw, touched)
        if min_dist < 20:
            print("--------------------------")
            self.flyAlongWall = True
            return {"forward": 1.0 if abs(near_angle) >= 1.0 else 0.0,
                    "lateral": 0.0,
                    "rotation": 1.0 if abs(near_angle) < 1.0 else 0.0,
                    "grasper": 0}
        else:
            if self.flyAlongWall is True:
                self.flyAlongWall = False
                self.angleStopTurning = math.pi
                return {"forward": 0.0,
                        "lateral": 0.0,
                        "rotation": 1.0,
                        "grasper": 0}
            print("&*********************************")
            return None

    def control(self):
        """
        The Drone will move forward and turn for a random angle when an obstacle is hit
        """
        angular_vel_controller = 1.0

        detection_semantic = self.semantic().get_sensor_values()
        if detection_semantic is not None:
            print(detection_semantic)

        measured_angle = 0
        if self.measured_compass_angle() is not None:
            measured_angle = self.measured_compass_angle()
            if len(measured_angle) == 0:
                self.angleStopTurning = -measured_angle
                return {"forward": -1.0,
                        "lateral": 0.0,
                        "rotation": 0,
                        "grasper": 0}

        angle = normalize_angle(measured_angle)

        values = self.lidar().get_sensor_values()
        ray_angles = self.lidar().ray_angles
        size = self.lidar().resolution
        min_dist = 1000
        if size != 0:
            min_dist = min(values)
            near_angle_raw = ray_angles[np.argmin(values)]
            right_dist = values[45]
            left_dist = values[135]
            front_dist = values[90]

        if self.lastRightDist == 0:
            self.lastRightDist = right_dist
        if self.lastLeftDist == 0:
            self.lastLeftDist = left_dist

        near_angle = near_angle_raw
        diff_angle = measured_angle - near_angle

        if min_dist < 40 and front_dist < 50 and self.newTurn is False:
            if measured_angle < math.pi / 4 and measured_angle > -math.pi / 4:
                self.angleStopTurning = math.pi / 2
            elif measured_angle > math.pi / 4 and measured_angle < math.pi * 3 / 4:
                self.angleStopTurning = math.pi
            elif measured_angle > math.pi * 3 / 4 or measured_angle <  - math.pi * 3 / 4:
                self.angleStopTurning = math.pi * 3 / 2
            else:
                self.angleStopTurning = 0
            self.newTurn = True
            self.alongWall = True
            print(self.angleStopTurning)

        if right_dist - self.lastRightDist > 100 and self.newTurn is False:
            print("*******************************++++++++++++++++++++++++++++++++++++++++")
            if measured_angle < math.pi / 4 and measured_angle > -math.pi / 4:
                self.angleStopTurning = -math.pi / 2
            elif measured_angle > math.pi / 4 and measured_angle < math.pi * 3 / 4:
                self.angleStopTurning = 0
            elif measured_angle > math.pi * 3 / 4 or measured_angle <  - math.pi * 3 / 4:
                self.angleStopTurning = math.pi / 2
            else:
                self.angleStopTurning = -math.pi
                angular_vel_controller = -1
            self.newTurn = True

        if left_dist - self.lastLeftDist > 100 and self.newTurn is False:
            print("*******************************!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            angular_vel_controller = -1
            if measured_angle < math.pi / 4 and measured_angle > -math.pi / 4:
                self.angleStopTurning = -math.pi / 2
            elif measured_angle > math.pi / 4 and measured_angle < math.pi * 3 / 4:
                angular_vel_controller = -1
                self.angleStopTurning = 0
            elif measured_angle > math.pi * 3 / 4 or measured_angle <  - math.pi * 3 / 4:
                self.angleStopTurning = math.pi / 2
            else:
                self.angleStopTurning = math.pi
            self.newTurn = True

        self.lastRightDist = right_dist
        self.lastLeftDist = left_dist

        print(">>>", near_angle, measured_angle, diff_angle, min_dist)

        # command = self.process_lidar_sensor(self.lidar())
        # if command is not None:
        #     return command

        diff_angle = normalize_angle(self.angleStopTurning - measured_angle)
        if abs(diff_angle) > 0.2:
            self.isTurning = True
            # print("<<<<<<<<<<<<<<<<<<<<<", self.isTurning)
        elif abs(diff_angle) > 0.1:
            self.isFineTurning = True
        else:
            self.isFineTurning = False
            self.isTurning = False
            self.newTurn = False
            # print(">>>>>>>>>>>>>>>>", self.isTurning)

        rotation = 0.5 if self.isTurning else 0.1 if self.isFineTurning else 0.0

        return {"forward": 0.0 if self.isTurning else 1.0,
                "lateral": 0.0,
                "rotation": rotation * angular_vel_controller,
                "grasper": 0}
