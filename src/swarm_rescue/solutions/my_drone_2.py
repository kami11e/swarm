import math
import numpy as np
import pandas as pd
from abc import abstractmethod
from enum import IntEnum
from typing import Optional

from spg.agent.agent import Agent
from spg.agent.communicator.communicator import Communicator
from spg.agent.interactor import GraspMagnet

from spg_overlay.entities.drone_base import DroneBase
from spg_overlay.entities.drone_distance_sensors import DroneLidar, DroneTouch, DroneSemanticSensor
from spg_overlay.entities.drone_sensors import DroneGPS, DroneCompass, DroneOdometer
from spg_overlay.utils.misc_data import MiscData

import matplotlib.pyplot as plt

from spg_overlay.utils.utils import normalize_angle

from spg_overlay.entities.drone_abstract import DroneAbstract
import random


class MyDrone2(DroneAbstract):
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
        self.exploredMap = {}
        self.maxX = 0
        self.maxY = 0
        self.minX = 0
        self.minY = 0
        self.exploredWall = []

    def define_message_for_all(self):
        """
        This function is mandatory in the class you have to create that will inherit from this class.
        You should return want you want to send to all nearest drone.
        For example:
            def define_message_for_all(self):
                msg_data = (
                    self.identifier, (self.measured_gps_position(), self.measured_compass_angle()))
                return msg_data
        """
        pass


    def process_lidar_sensor(self, the_lidar_sensor):
        values = the_lidar_sensor.get_sensor_values()
        angular = self.measured_compass_angle()[0]*180/math.pi
        xx, yy = self.measured_gps_position()
        # x0, y0 = self.true_position()
        # print(xx,",",yy,"|",x0,",", y0)
        da = 2
        values_ = [values[i]-values[i-1] for i in range(0,181)]
        for i in range(0, 181):
            ang = angular + (i-90)*da 
            if(values[i]<200 ):
                if(values_[i]*values_[(i+1)%180]>-5):
                    
                    x = xx + values[i]*math.cos(ang*math.pi/180.)
                    y = yy + values[i]*math.sin(ang*math.pi/180.)
                    x = round(x/5)
                    y = round(y/5)
                    self.exploredMap[(x,y)] = 1
                    self.maxX = max(x, self.maxX)
                    self.maxY = max(y, self.maxY)
                    self.minX = min(x, self.minX)
                    self.minY = min(y, self.minY)

            

    def display_explored_map(self):
        if len(self.exploredMap)>0:   
            plt.figure(10)
            plt.cla()
            draw = np.zeros(((self.maxX-self.minX+1), (self.maxY-self.minY+1)))
            for (x,y) in self.exploredMap:
                draw[x-self.minX][y-self.minY] = 1
            plt.pcolormesh(np.arange(-1,self.maxX-self.minX+1, 1), np.arange(-1,self.maxY-self.minY+1), draw.T)

            # plt.figure(self.SensorType.LIDAR)
            # plt.cla()
            # plt.axis([-math.pi, math.pi, 0, self.lidar().max_range])
            # plt.plot(self.lidar().ray_angles, self.lidar().get_sensor_values(), "g.:")
            # plt.grid(True)
            # plt.draw()
            # plt.pause(0.001)
            plt.ion()
            plt.draw()
            plt.pause(0.001)
    def points2walls(self):
        plt.figure(11)
        plt.cla()
        pList = list(zip(*self.exploredMap.keys()))
        xList = list(pList[0])
        yList = list(pList[1])
        binsX = int((self.maxX-self.minY)/4)
        # xRes = pd.DataFrame(xList).hist(bins=binsX)
        plt.hist(xList, bins=binsX)
        # print(xRes)
        plt.ion()
        plt.draw()
        plt.pause(0.001)



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
        


    def control(self):
        """
        This function is mandatory in the class you have to create that will inherit from this class.
        This function should return a command which is a dict with values for the actuators.
        For example:
        command = {"forward": 1.0,
                   "lateral": 0.0,
                   "rotation": -1.0,
                   "grasper": 0}
        """
        self.process_lidar_sensor(self.lidar())
        # self.display_explored_map()
        self.points2walls()
        """
        The Drone will move forward and turn for a random angle when an obstacle is hit
        """
        command_straight = {"forward": 1.0,
                            "lateral": 0.0,
                            "rotation": 0.0,
                            "grasper": 0}

        command_turn = {"forward": 0.0,
                        "lateral": 0.0,
                        "rotation": 1.0,
                        "grasper": 0}

        touched = self.process_touch_sensor()

        self.counterStraight += 1

        if touched and not self.isTurning and self.counterStraight > self.distStopStraight:
            self.isTurning = True
            self.angleStopTurning = random.uniform(-math.pi, math.pi)

        measured_angle = 0
        if self.measured_compass_angle() is not None:
            measured_angle = self.measured_compass_angle()

        diff_angle = normalize_angle(self.angleStopTurning - measured_angle)
        if self.isTurning and abs(diff_angle) < 0.2:
            self.isTurning = False
            self.counterStraight = 0
            self.distStopStraight = random.uniform(10, 50)

        if self.isTurning:
            return command_turn
        else:
            return command_straight

        pass
