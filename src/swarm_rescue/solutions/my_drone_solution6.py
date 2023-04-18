"""
Simple random controller
The Drone will move forward and turn for a random angle when an obstacle is hit
"""
import math
import random
import numpy as np

from enum import Enum
from typing import Optional

from collections import deque

from spg_overlay.entities.drone_distance_sensors import DroneSemanticSensor
from spg_overlay.entities.rescue_center import RescueCenter, wounded_rescue_center_collision
from spg_overlay.entities.wounded_person import WoundedPerson
from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.gui_map.closed_playground import ClosedPlayground
from spg_overlay.utils.misc_data import MiscData
from spg_overlay.utils.utils import normalize_angle


class Graph(object):

    def __init__(self):
        self.__graph_dict = {}

    def add_vertex(self, vertex):
        if vertex not in self.__graph_dict:
            self.__graph_dict[vertex] = []
            return True
        else:
            return False

    def add_edge(self, edge):
        vertex1, vertex2 = edge
        for x, y in [(vertex1, vertex2), (vertex2, vertex1)]:
            if x in self.__graph_dict:
                self.__graph_dict[x].append(y)
            else:
                self.__graph_dict[x] = [y]


class Vertex(object):
    def __init__(self, position, angle=0, type=1):
        """
        type=1 -> path node
        type=0 -> resuce center
        """
        self.__position = position
        x, y = position
        self.__key = round(x - 10), round(y - 10), round(x+10), round(y+10)
        self.__type = type
        self.__angle = angle

    def get_position(self):
        return self.__position

    def get_key(self):
        return self.__key
    
    def get_angle(self):
        return self.__angle

    def __hash__(self):
        return hash(self.get_key)

    def __eq__(self, other):
        return self.get_key == other.get_key()

    def __ne__(self, other):
        return not(self == other)

    def __str__(self):
        type = "Rescue Center" if self.__type == 0 else "Node"
        return f'{type}: {self.__angle}, {self.__position}'

class MyDroneSolution6(DroneAbstract):

    class Direction(Enum):
        NORTH = 0
        EAST = 1
        SOUTH = 2
        WEST = 3

    class Activity(Enum):
        """
        All the states of the drone as a state machine
        """
        SEARCHING_WOUNDED = 1
        GRASPING_WOUNDED = 2
        SEARCHING_RESCUE_CENTER = 3
        DROPPING_AT_RESCUE_CENTER = 4

    def __init__(self,
                 identifier: Optional[int] = None,
                 misc_data: Optional[MiscData] = None,
                 **kwargs):
        super().__init__(identifier=identifier,
                         misc_data=misc_data,
                         should_display_lidar=False,
                         **kwargs)
        self.state = self.Activity.SEARCHING_WOUNDED
        self.counterStraight = 0
        self.angleStopTurning = random.uniform(-math.pi, math.pi)
        self.distStopStraight = random.uniform(10, 50)
        self.isTurning = False
        self.isBacking = False
        self.alongRightSideWall = True
        self.alongRightSideCount = 0
        self.rightSideWallDists = deque([], 3)
        self.detectedRightSideWall = False
        self.detectedRightSideCount = 0
        self.forwardSpeed = 1.0
        self.forwardAccel = 0.1
        self.lateral = 0.0
        self.rescueCenterPosition = None
        self.lastPathPosition = None
        self.firstDetectedWounded = False
        self.graph = Graph()

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

    def is_number_within(self, value, min_value, max_value):
        return value > min_value and value < max_value

    def get_head_direction(self, angle):
        alpha = normalize_angle(angle)

        if self.is_number_within(alpha, math.pi/4, math.pi*3/4):
            return self.Direction.NORTH
        elif self.is_number_within(alpha, -math.pi/4, math.pi/4):
            return self.Direction.EAST
        elif self.is_number_within(alpha, -math.pi*3/4, -math.pi/4):
            return self.Direction.SOUTH
        else:
            return self.Direction.WEST

    def get_rotation_speed(self, measured_angle, position):

        diff_angle = normalize_angle(self.angleStopTurning - measured_angle)
        if diff_angle > 0:
            angular_vel_controller = 1
        elif diff_angle < 0:
            angular_vel_controller = -1
        else:
            angular_vel_controller = 0

        if self.isTurning is False:
            if self.alongRightSideCount > 10:
                if abs(diff_angle) > 0.3:
                    self.isTurning = True
                    return 0.2 * angular_vel_controller, False
            return 0.0, False

        if self.isTurning and abs(diff_angle) < 0.1:
            self.isTurning = False
            self.forwardSpeed = 0.0
            self.forwardAccel = 0.025
            if self.isBacking is False:
                # no need to add this node in backing mode
                currentPathPosition = Vertex(position, measured_angle)
                if self.graph.add_vertex(currentPathPosition):
                    self.graph.add_edge((currentPathPosition, self.lastPathPosition))
                self.lastPathPosition = currentPathPosition
            return 0.0, True
        elif self.isTurning and abs(diff_angle) < 0.2:
            return 0.3 * angular_vel_controller, False
        elif self.isTurning and abs(diff_angle) < 0.4:
            return 0.7 * angular_vel_controller, False
        else:
            return 1.0 * angular_vel_controller, False

    def get_forward_speed(self, dist):
        if self.isTurning:
            return 0.0
        elif self.isBacking:
            return -1
        else:
            self.forwardSpeed += self.forwardAccel
            if self.forwardSpeed > 1.0:
                self.forwardSpeed = 1.0
                self.forwardAccel = 0.0
            if dist < 100:
                return 0.4
            else:
                return self.forwardSpeed

    def turn_left(self, measured_angle):
        self.isTurning = True
        direction = self.get_head_direction(measured_angle)
        if direction is self.Direction.NORTH:
            self.angleStopTurning = math.pi
        elif direction is self.Direction.EAST:
            self.angleStopTurning = math.pi/2
        elif direction is self.Direction.SOUTH:
            self.angleStopTurning = 0
        else:
            self.angleStopTurning = -math.pi/2

    def turn_right(self, measured_angle):
        self.isTurning = True
        direction = self.get_head_direction(measured_angle)
        if direction is self.Direction.NORTH:
            self.angleStopTurning = 0
        elif direction is self.Direction.EAST:
            self.angleStopTurning = -math.pi/2
        elif direction is self.Direction.SOUTH:
            self.angleStopTurning = -math.pi
        else:
            self.angleStopTurning = math.pi/2

    def turn_back(self, measured_angle):
        self.isTurning = True
        direction = self.get_head_direction(measured_angle)
        if direction is self.Direction.NORTH:
            self.angleStopTurning = -math.pi/2
        elif direction is self.Direction.EAST:
            self.angleStopTurning = -math.pi
        elif direction is self.Direction.SOUTH:
            self.angleStopTurning = math.pi/2
        else:
            self.angleStopTurning = 0

    def process_semantic_sensor(self, the_semantic_sensor, position):

        detection_semantic = the_semantic_sensor.get_sensor_values()

        if self.rescueCenterPosition is None \
                and detection_semantic is not None:
            for data in detection_semantic:
                if data.entity_type == DroneSemanticSensor.TypeEntity.RESCUE_CENTER:
                    x = position[0] + data.distance * math.cos(data.angle)
                    y = position[1] + data.distance * math.sin(data.angle)
                    self.rescueCenterPosition = Vertex((x, y), 0)
                    self.graph.add_vertex(self.rescueCenterPosition)
                    self.graph.add_edge(
                        (self.rescueCenterPosition, self.lastPathPosition))

        found_wounded = False
        wounded_position = None
        best_angle = 1000
        if (self.state is self.Activity.SEARCHING_WOUNDED
            or self.state is self.Activity.GRASPING_WOUNDED) \
                and detection_semantic is not None:
            scores = []
            for data in detection_semantic:
                # If the wounded person detected is held by nobody
                if data.entity_type == DroneSemanticSensor.TypeEntity.WOUNDED_PERSON and not data.grasped:
                    found_wounded = True
                    x = position[0] + data.distance * math.cos(data.angle)
                    y = position[1] + data.distance * math.sin(data.angle)
                    wounded_position = (x, y)
                    v = (data.angle * data.angle) + \
                        (data.distance * data.distance / 10 ** 5)
                    scores.append((v, data.angle, data.distance))
            # Select the best one among wounded persons detected
            best_score = 10000
            for score in scores:
                if score[0] < best_score:
                    best_score = score[0]
                    best_angle = score[1]
            if found_wounded is True and self.firstDetectedWounded is False:
                # log the position where it found the wounded
                self.firstDetectedWounded = True
                lastPosition = Vertex((position), 0)
                if self.graph.add_vertex(lastPosition):
                    self.graph.add_edge((lastPosition, self.lastPathPosition))
                    self.lastPathPosition = lastPosition

        command = {"forward": 0.5,
                   "lateral": 0.0,
                   "rotation": 0.0}
        angular_vel_controller_max = 1.0
        if wounded_position:
            # simple P controller
            # The robot will turn until best_angle is 0
            kp = 2.0
            a = kp * best_angle
            a = min(a, 1.0)
            a = max(a, -1.0)
            command["rotation"] = a * angular_vel_controller_max

            # reduce speed if we need to turn a lot
            if abs(a) > 0.5:
                command["forward"] = 0.2

        return found_wounded, command

    def control(self):
        """
        The Drone will move forward and turn for a random angle when an obstacle is hit
        """
        command = {"forward": 0.0,
                   "lateral": 0.0,
                   "rotation": 0.0,
                   "grasper": 0}

        measured_angle = 0
        if self.measured_compass_angle() is not None:
            measured_angle = self.measured_compass_angle()

        position = None
        if self.measured_gps_position() is not None:
            position = self.measured_gps_position()
            if self.lastPathPosition is None:
                self.lastPathPosition = Vertex(position, measured_angle)
                self.graph.add_vertex(self.lastPathPosition)

        # found_wounded, command = self.process_semantic_sensor(
        #     self.semantic(), position)

        # #############
        # # TRANSITIONS OF THE STATE MACHINE
        # #############

        # if self.state is self.Activity.SEARCHING_WOUNDED and found_wounded:
        #     self.state = self.Activity.GRASPING_WOUNDED

        # elif self.state is self.Activity.GRASPING_WOUNDED and self.base.grasper.grasped_entities:
        #     self.state = self.Activity.SEARCHING_RESCUE_CENTER

        # elif self.state is self.Activity.GRASPING_WOUNDED and not found_wounded:
        #     self.state = self.Activity.SEARCHING_WOUNDED

        # elif self.state is self.Activity.SEARCHING_RESCUE_CENTER and self.rescueCenterPosition is not None:
        #     self.state = self.Activity.DROPPING_AT_RESCUE_CENTER

        # elif self.state is self.Activity.DROPPING_AT_RESCUE_CENTER and not self.base.grasper.grasped_entities:
        #     self.state = self.Activity.SEARCHING_WOUNDED

        # elif self.state is self.Activity.DROPPING_AT_RESCUE_CENTER and self.rescueCenterPosition is not None:
        #     self.state = self.Activity.SEARCHING_RESCUE_CENTER


        # grasper = 0
        # ##########
        # # COMMANDS FOR EACH STATE
        # # Searching randomly, but when a rescue center or wounded person is detected, we use a special command
        # ##########
        # if self.state is self.Activity.SEARCHING_WOUNDED:
        #     pass

        # elif self.state is self.Activity.GRASPING_WOUNDED:
        #     command["grasper"] = 1
        #     grasper = 1

        # elif self.state is self.Activity.SEARCHING_RESCUE_CENTER:
        #     grasper = 1
        #     if self.isTurning is False:
        #         if self.isBacking is False:
        #             lastPosition = self.lastPathPosition.get_position()
        #             deltaY = lastPosition[1] - position[1]
        #             deltaX = lastPosition[0] - position[0]
        #             self.angleStopTurning = math.atan2(deltaY, deltaX)
        #             self.isTurning = True
        #             self.isBacking = True
        #         else:
        #             lastPosition = self.lastPathPosition.get_position()
        #             deltaY = lastPosition[1] - position[1]
        #             deltaX = lastPosition[0] - position[0]
        #             targetAngle = math.atan2(deltaY, deltaX)
        #             print(deltaX, deltaY, targetAngle)
        #             if abs(deltaX) < 10 and abs(deltaY) < 5:
        #                 print("++++++++++++++++++++++++++")

        # elif self.state is self.Activity.DROPPING_AT_RESCUE_CENTER:
        #     grasper = 1
        #     command["grasper"] = 1

        # if found_wounded:
        #     return command

        values = self.lidar().get_sensor_values()
        ray_angles = self.lidar().ray_angles
        size = self.lidar().resolution

        min_dist = 1000
        if size != 0:
            far_angle_raw = ray_angles[np.argmax(values)]
            min_dist = min(values)
            near_angle_raw = ray_angles[np.argmin(values)]
            right_dist = np.average(values[44:47])
            left_dist = np.average(values[134:137])
            head_dist = np.average(values[89:92])
            tail_dist = np.average([values[0], values[180]])

            forward = 0.5 if head_dist < 120 else 1.0

            if head_dist < 30 and self.isTurning is False:
                self.turn_left(measured_angle)

            self.lateral = 0.0
            if self.isTurning is False and self.alongRightSideCount > 3:
                average_right_dist = np.average(self.rightSideWallDists)
                if right_dist > 50:
                    self.turn_right(measured_angle)
                elif right_dist < 20:
                    self.lateral = 0.2

            if right_dist < 40 or head_dist < 40:
                self.alongRightSideCount += 1
                self.rightSideWallDists.append(min(right_dist, head_dist))
            else:
                self.alongRightSideCount = 0

            print(self.alongRightSideCount)

        if self.isTurning is False and self.detectedRightSideCount > 0 and self.detectedRightSideCount < 10:
            if right_dist > 100:
                self.turn_right(measured_angle)

        if right_dist < 100:
            self.detectedRightSideCount += 1
            print("---------------------------------",
                  self.detectedRightSideCount)
        else:
            self.detectedRightSideCount = 0

        # check if no exit in space ahead
        # if self.isTurning is False and head_dist < 50:
        #     no_exit_flag = True
        #     for i in range(30, 150):
        #         if values[i] > 150:
        #             no_exit_flag = False
        #     if no_exit_flag is True:
        #         self.turn_back(measured_angle)
        #         print("+++++++++++++++++++++!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

        rotation, turnEnd = self.get_rotation_speed(measured_angle, position)

        forward = self.get_forward_speed(head_dist)

        grasper = 0
        return {"forward": forward,
                "lateral": self.lateral,
                "rotation": rotation,
                "grasper": grasper}
