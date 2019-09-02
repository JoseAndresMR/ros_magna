#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# ----------------------------------------------------------------------------------------------------------------------
# ROS-MAGNA
# ----------------------------------------------------------------------------------------------------------------------
# The MIT License (MIT)

# Copyright (c) 2016 GRVC University of Seville

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
# Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
# OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# ----------------------------------------------------------------------------------------------------------------------

"""
Created on Mon Feb 21 2018

@author: josmilrom
"""

import sys
import rospy
import std_msgs.msg
import time
import math
import numpy as np
import tf, tf2_ros
import json
import copy
import random
import rospkg
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from xml.dom import minidom
from gazebo_msgs.srv import DeleteModel,SpawnModel
from visualization_msgs.msg import Marker
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray, TorusArray, PolygonArray
from jsk_recognition_msgs.msg import Torus as jsk_Torus
from sympy import Point3D, Line3D, Segment3D
from sympy import Point as Point2D
from sympy import Polygon as Polygon2D
import xml.etree.ElementTree

from magna.srv import *
from PoseElements import *


class GenericGeometry:
    def __init__(self,geometry_def,parent_name,parent_prefix,transforms_list):

        self.name = parent_prefix + '_' + geometry_def["name"]
        self.prefix = parent_prefix + '_' + geometry_def["prefix"]
        self.shape = geometry_def["shape"]
        self.origin = geometry_def["origin"]
        self.color = geometry_def["color"]
        self.alpha = geometry_def["alpha"]
        self.id = geometry_def["id"]
        self.dimensions = geometry_def["dimensions"]
        self.parent_name = parent_name

        self.transforms_auxiliar_list = copy.deepcopy(transforms_list)
        # print("all",len(self.transforms_auxiliar_list))
        self.transforms_list = copy.deepcopy(transforms_list)
        # print("persistent",len(self.transforms_list))

        self.n_obs = 0
        self.obs_pose_list = []
        self.obs_list = []
        self.obs_transforms_list = []
        self.obs_shape_list = []

        self.obstacles_dicc = {}
        self.fsp_dicc = {"List": [], "Matrix": [], "Path" : {}}

        tfbroadcaster = StaticTfBroadcaster(self.name,self.parent_name,self.origin,self.transforms_auxiliar_list)
        self.transforms_auxiliar_list = tfbroadcaster.getTransforms()
        # print("all",len(self.transforms_auxiliar_list))
        self.transforms_list.append(tfbroadcaster.getTransforms()[-1])
        # print("persistent",len(self.transforms_list))

        time.sleep(0.05)
        

    def MakeRvizMarker(self):

        self.rviz_marker_def = {"shape" : self.shape}
        self.rviz_marker_def["origin"] = self.origin
        self.rviz_marker_def["parent_name"] = self.parent_name
        self.rviz_marker_def["name"] = self.name
        self.rviz_marker_def["id"] = self.id
        self.rviz_marker_def["scale"] = self.dimensions

        color = self.color
        color.append(self.alpha)
        self.rviz_marker_def["color"] = color

        self.rviz_marker = RvizMarker(self.rviz_marker_def)

    def EraseRvizMarker(self):

        self.rviz_marker.Erase()

    def MakeRvizPolygonArray(self):

        self.rviz_polygon_array_def = {"parent_name" : self.parent_name}
        self.rviz_polygon_array_def["name"] = self.name
        self.rviz_polygon_array_def["id"] = self.id

        self.rviz_polygon_array_def["polygon_array_poses"] = self.polygon_array_poses

        self.rviz_polyghon_array = RvizPolygonArray(self.rviz_polygon_array_def)

    def MakeRvizTrousArray(self):

        self.rviz_torus_array_def = {"parent_name" : self.parent_name}
        self.rviz_torus_array_def["name"] = self.name
        self.rviz_torus_array_def["id"] = self.id

        self.rviz_torus_array_def["torus_array_poses"] = [[False,self.PoseFromArray(self.origin),self.dimensions[0]/2,self.dimensions[2]/2]]

        self.rviz_polyghon_array = RvizTorusArray(self.rviz_torus_array_def)

    def EraseRvizPolygonArray(self):

        self.rviz_polyghon_array.Erase()

    def getTransforms(self):
        # print("persistent",len(self.transforms_list))
        return self.transforms_list

    def getObstacles(self,indexes):

        indexes = [int(x) for x in indexes]

        return self.obstacles[indexes[0]][indexes[1]][indexes[2]]

    def getFSPGlobalPosefromMatrix(self,indexes):

        indexes = [int(x) for x in indexes]

        return [self.fsp_dicc["Matrix"][indexes[0]][indexes[1]][indexes[2]].global_pose]

    # def getFSPGlobalPosefromList(self):

    #     pose_def = {"use": "poses","quantity": 1}
    #     self.GeneratePosesSetRandom(pose_def)
    #     return self.fsp_dicc["List"][-1].global_pose

    def genFSPRandomGlobalPoseList(self,quantity):
        pose_def = {"use": "poses","quantity": quantity}
        self.GeneratePosesSetRandom(pose_def)
        return self.getFSPGlobalPosefromList(self.fsp_dicc["List"][-quantity:])


    def getFSPGlobalPosefromPath(self,path_name):

        return self.getFSPGlobalPosefromList(self.fsp_dicc["Path"][path_name])

    def getFSPGlobalPosefromList(self,fsp_list):

        global_path = []
        for pose in fsp_list:
            global_path.append(pose.global_pose)

        return global_path
    
    def getFSPGlobalPosefromCoordinates(self,coordinates):

        return [self.GenerateFreeSpacePosesFromCoordinates(coordinates).global_pose]

    def RawDensityMatrix(self,poses_set_def):
 
        # Initializes a tensor with random values and shaped as the obstacle tube
        random_raw_matrix = np.random.rand(poses_set_def["dimensions"][0],poses_set_def["dimensions"][1],poses_set_def["dimensions"][2])
    
        # If the random value is lower than the density, set its obstacle tensor's position at True
        selected_positions_matrix = random_raw_matrix<=poses_set_def["density"]

        return selected_positions_matrix

    def GenerateObstacleFromMatrix(self,selected_positions_matrix,poses_matrix,shapes,dimensions):

        obs_list = copy.deepcopy(selected_positions_matrix.tolist())

        for i in np.arange(selected_positions_matrix.shape[0]):
            for j in np.arange(selected_positions_matrix.shape[1]):
                for k in np.arange(selected_positions_matrix.shape[2]):
                    # For every obstacled that has been created
                    if selected_positions_matrix[i,j,k] == True:

                        shape = random.choice(shapes)

                        self.obs_shape_list.append(shape)       # Add its shape to the list

                        # Add the obstacle object to the list of obstacles. During its initialization are already spawned
                        obs_list[i][j][k] = Obstacle('{0}_{1}_{2}'.format(i,j,k),\
                                                            shape,\
                                                            dimensions,\
                                                            poses_matrix[i][j][k],\
                                                            self.name,\
                                                            self.prefix,\
                                                            self.transforms_auxiliar_list)

                        self.transforms_auxiliar_list = obs_list[i][j][k].getTransforms()
                        # print("all",len(self.transforms_auxiliar_list))

                        # self.obs_pose_list.append([np.asarray(self.obs_list.point),np.asarray(self.obs_list.quaternion)])

                        # Add also its single list pose to the list of single list poses
                        self.obs_pose_list.append([[float(obs_list[i][j][k].global_pose.position.x),float(obs_list[i][j][k].global_pose.position.y),float(obs_list[i][j][k].global_pose.position.z)],[0,0,0,1]])

                        self.n_obs=self.n_obs+1     # Actualize the counter of created obstacles

        return obs_list

    def GenerateObstacleFromPosesList(self,selected_positions,shapes,dimensions):

        obs_list = []

        for i,pose in enumerate(selected_positions):
            shape = random.choice(shapes)
            self.obs_shape_list.append(shape)

            # Add the obstacle object to the list of obstacles. During its initialization are already spawned
            obs_list.append(Obstacle(str(i),\
                                    shape,\
                                    dimensions,\
                                    pose,\
                                    self.name,\
                                    self.prefix,\
                                    self.transforms_auxiliar_list))
            self.obs_pose_list.append([[float(obs_list[-1].global_pose.position.x),float(obs_list[-1].global_pose.position.y),float(obs_list[-1].global_pose.position.z)],[0,0,0,1]])

            self.n_obs=self.n_obs+1     # Actualize the counter of created obstacles

        return obs_list

    def GenerateFreeSpacePosesFromMatrix(self,selected_positions_matrix,poses_matrix,make_persistent = True):

        fsposes_list = copy.deepcopy(selected_positions_matrix.tolist())

        for i in np.arange(selected_positions_matrix.shape[0]):
            for j in np.arange(selected_positions_matrix.shape[1]):
                for k in np.arange(selected_positions_matrix.shape[2]):

                    # Add the obstacle object to the list of obstacles. During its initialization are already spawned
                    fsposes_list[i][j][k] = FreeSpacePose('{0}_{1}_{2}'.format(i,j,k),\
                                                        poses_matrix[i][j][k],\
                                                        self.name,\
                                                        self.prefix,\
                                                        self.transforms_auxiliar_list)

                    self.transforms_auxiliar_list = fsposes_list[i][j][k].getTransforms()
                    # print("all",len(self.transforms_auxiliar_list))
                    if make_persistent == True:
                        self.transforms_list.append(fsposes_list[i][j][k].getTransforms()[-1])
                        # print("persistent",len(self.transforms_list))

        return fsposes_list

    def GenerateFreeSpacePosesFromPosesList(self,selected_positions,make_persistent = True):

        if self.fsp_dicc["List"] != []:
            init_i = len(self.fsp_dicc["List"])-1
        else:
            init_i = 0

        fsp_list = []

        for i,pose in enumerate(selected_positions):

            fsp_list.append(FreeSpacePose(str(init_i+i),  pose, self.name, self.prefix, self.transforms_auxiliar_list))

            self.transforms_auxiliar_list = fsp_list[-1].getTransforms()
            # print("all",len(self.transforms_auxiliar_list))
            if make_persistent == True:
                self.transforms_list.append(fsp_list[-1].getTransforms()[-1])
                # print("persistent",len(self.transforms_list))


        return fsp_list

    def GenerateFreeSpacePosesFromCoordinates(self,coordinates):

        return FreeSpacePose(np.random.randint(100),self.PoseFromArray(coordinates),self.name,self.prefix,self.transforms_auxiliar_list)


    def GeneratePosesSetDimensionMatrix(self,poses_set_def,make_persistent = True):

        if poses_set_def["matrix_type"] == "dimension": 

            selected_positions_matrix = self.RawDensityMatrix(poses_set_def)

            poses_matrix = self.PosesDimensionMatrix(poses_set_def,selected_positions_matrix)

        elif poses_set_def["matrix_type"] == "distance": 

            poses_matrix, selected_positions_matrix = self.PosesDistanceMatrix(poses_set_def)


        if poses_set_def["use"] == "obstacles":

            self.obstacles = self.GenerateObstacleFromMatrix(selected_positions_matrix,poses_matrix,poses_set_def["obstacles_shape"],poses_set_def["obstacles_dimensions"])

        elif poses_set_def["use"] == "poses":

            self.fsp_dicc["Matrix"] = self.GenerateFreeSpacePosesFromMatrix(selected_positions_matrix,poses_matrix,make_persistent)

    def GeneratePosesSetRandom(self,poses_set_def):

        if "orienation" in poses_set_def.keys():
            random_poses = self.RandomPoses(poses_set_def["quantity"],poses_set_def["orientation"])
        else:
            random_poses = self.RandomPoses(poses_set_def["quantity"])

        if poses_set_def["use"] == "obstacles":

            self.obstacles = self.GenerateObstacleFromPosesList(random_poses,poses_set_def["obstacles_shape"],poses_set_def["obstacles_dimensions"])

        elif poses_set_def["use"] == "poses":
            self.fsp_dicc["List"] = self.fsp_dicc["List"] + self.GenerateFreeSpacePosesFromPosesList(random_poses)

    def GeneratePosesSetCoordinates(self,poses_set_def):

        poses_list = [self.PoseFromArray(coordinates) for coordinates in poses_set_def["coordinates"]]

        # for coordinates in poses_set_def["coordinates"]:

        #     poses_list.append(self.PoseFromArray(coordinates))
            
        if poses_set_def["use"] == "obstacles":

            self.obstacles = self.GenerateObstacleFromPosesList(poses_list,poses_set_def["obstacles_shape"],poses_set_def["obstacles_dimensions"])

        elif poses_set_def["use"] == "poses":

           self.fsp_dicc["List"] =  self.fsp_dicc["List"] + self.GenerateFreeSpacePosesFromPosesList(poses_list)

        return poses_list

    def GeneratePosesSetZigZag(self,poses_set_def):

        poses = self.ZigZagOnPerimeter(self.base_vertexes_pose_list,poses_set_def["height"],poses_set_def["sweep_angle"],poses_set_def["spacing"],poses_set_def["margins"],poses_set_def["initial_sense"])

        if poses_set_def["use"] == "obstacles":

            self.obstacles = self.GenerateObstacleFromPosesList(poses,poses_set_def["obstacles_shape"],poses_set_def["obstacles_dimensions"])

        elif poses_set_def["use"] == "poses":

            fsp_poses_list = self.GenerateFreeSpacePosesFromPosesList(poses)

            self.fsp_dicc["List"] =  self.fsp_dicc["List"] + fsp_poses_list

            self.fsp_dicc["Path"]["Zigzag"] = fsp_poses_list


    def GenerateRandomDimensionalValues(self, limits):
        return np.random.uniform(limits["lower"],limits["upper"])


    def PoseFromArray(self,Array):
        quat = tf.transformations.quaternion_from_euler(Array[1][0],Array[1][1],Array[1][2])

        return Pose(Point(Array[0][0],Array[0][1],Array[0][2]),Quaternion(quat[0],quat[1],quat[2],quat[3]))

    def ArrayFromPose(self,pose):

        euler = tf.transformations.euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])

        return [[pose.position.x,pose.position.y,pose.position.z],[euler[0],euler[1],euler[2]]]


class Cube(GenericGeometry):
    def __init__(self,geometry_def,parent_name,parent_prefix,transforms_list):
        GenericGeometry.__init__(self,geometry_def,parent_name,parent_prefix,transforms_list)

        self.MakeRvizMarker()

        if "poses_sets" in geometry_def.keys():
            for poses_set_def in geometry_def["poses_sets"]:
                if poses_set_def["type"] == "coordinates":
                    self.GeneratePosesSetCoordinates(poses_set_def)

                elif poses_set_def["type"] == "matrix":
                    self.GeneratePosesSetDimensionMatrix(poses_set_def)

                elif poses_set_def["type"] == "random":
                    self.GeneratePosesSetRandom(poses_set_def)

                elif poses_set_def["type"] == "zigzag":
                    self.GeneratePosesSetZigZag(poses_set_def)


    def PosesDimensionMatrix(self,poses_set_def,selected_positions_matrix):

        # Create a list shaped as the obstacle tube
        poses_matrix = copy.deepcopy(selected_positions_matrix.tolist())       ### HACERLO MAS FACIL SIN PASAR selected_positions_matrix

        # self.obstacles_matrix[obstacle_positions] = Obstacle(np.arange(obstacle_positions.sum),"sphere",list(obstacle_positions)*2,[0,0,0,1])            SIMPLIFICAR ASÏ
        # For every position in the three dimesions
        for i in np.arange(poses_set_def["dimensions"][0]):
            for j in np.arange(poses_set_def["dimensions"][1]):
                for k in np.arange(poses_set_def["dimensions"][2]):

                    pose = self.PoseFromArray([[-self.dimensions[0]/2+(0.5+i)*self.dimensions[0]/poses_set_def["dimensions"][0],\
                                                -self.dimensions[1]/2+(0.5+j)*self.dimensions[1]/poses_set_def["dimensions"][1],\
                                                -self.dimensions[2]/2+(0.5+k)*self.dimensions[2]/poses_set_def["dimensions"][2]],
                                                poses_set_def["poses_orientation"]])

                    poses_matrix[i][j][k] = pose

        return poses_matrix

    def RandomPoses(self,quantity = 1,orientation = [0,0,0]):
        random_poses = []

        for i in range(quantity):

            half_dimensions = np.array(self.dimensions)/2

            limits = {}
            limits["lower"] = - half_dimensions
            limits["upper"] = half_dimensions

            random_position = self.GenerateRandomDimensionalValues(limits)

            if "orientation" not in orientation:
                limits["lower"] = [0,0,0]
                limits["upper"] = [2*np.pi,2*np.pi,2*np.pi]

                random_orientation = self.GenerateRandomDimensionalValues(limits)
            random_poses.append(self.PoseFromArray([[random_position[0],random_position[1],random_position[2]],
                                                    [random_orientation[0],random_orientation[1],random_orientation[2]]]))

        return random_poses

        



    # def GeneratePosesSetZigZag(self,poses_set_def):

    #     matrix_definition = {"type" : "matrix", "use" : "poses",
    #                         "dimensions": poses_set_def["dimensions"], "density": 1,
    #                         "orientation": poses_set_def["orientation"]}

    #     self.GeneratePosesSetDimensionMatrix(matrix_definition,False)

    #     zigzag_path = []

    #     for i in np.arange(matrix_definition["dimensions"][0]):
    #         for j in np.arange(matrix_definition["dimensions"][1]):
    #             for k in np.arange(matrix_definition["dimensions"][2]):

    #                 if i%2 == 1:

    #                     J = matrix_definition["dimensions"][1]-1 - j

    #                 else:
    #                     J=j

    #                 zigzag_path.append(self.fsp_dicc["Matrix"][i][J][k])

    #     self.fsp_dicc["List"] = zigzag_path


class Sphere(GenericGeometry):
    def __init__(self,geometry_def,parent_name,parent_prefix,transforms_list):
        GenericGeometry.__init__(self,geometry_def,parent_name,parent_prefix,transforms_list)

        self.MakeRvizMarker()

        if "poses_sets" in geometry_def.keys():
            for poses_set_def in geometry_def["poses_sets"]:
                if poses_set_def["type"] == "coordinates":
                    self.GeneratePosesSetCoordinates(poses_set_def)

                elif poses_set_def["type"] == "matrix":
                    self.GeneratePosesSetDimensionMatrix(poses_set_def)

                elif poses_set_def["type"] == "random":
                    self.GeneratePosesSetRandom(poses_set_def)

                elif poses_set_def["type"] == "zigzag":
                    self.GeneratePosesSetZigZag(poses_set_def)

    def PosesDimensionMatrix(self,poses_set_def,selected_positions_matrix):

        # Create a list shaped as the obstacle tube
        poses_matrix = copy.deepcopy(selected_positions_matrix.tolist())       ### HACERLO MAS FACIL SIN PASAR selected_positions_matrix

        # self.obstacles_matrix[obstacle_positions] = Obstacle(np.arange(obstacle_positions.sum),"sphere",list(obstacle_positions)*2,[0,0,0,1])            SIMPLIFICAR ASÏ
        # For every position in the three dimesions
        for i in np.arange(poses_set_def["dimensions"][0]):
            for j in np.arange(poses_set_def["dimensions"][1]):
                for k in np.arange(poses_set_def["dimensions"][2]):

                    angle1 = (0.5+i)*(np.pi)/poses_set_def["dimensions"][0]
                    angle2 = (0.5+j)*(np.pi)/poses_set_def["dimensions"][1]
                    radius = (0.5+k)*self.dimensions[2]/poses_set_def["dimensions"][2]-self.dimensions[2]/2      # Esfera de radio [,,R]


                    pose = self.PoseFromArray([[np.cos(angle1)*np.sin(angle2)*radius,\
                                                np.sin(angle1)*np.sin(angle2)*radius,\
                                                np.cos(angle2)*radius],
                                                poses_set_def["poses_orientation"]])

                    poses_matrix[i][j][k] = pose

        return poses_matrix

    def RandomPoses(self,quantity = 1,orientation = [0,0,0]):

        random_poses = []

        for i in range(quantity):

            limits = {}
            limits["lower"] = [0,0,0]
            limits["upper"] = [2*np.pi,np.pi,self.dimensions[2]]

            [angle1,angle2,radius] = self.GenerateRandomDimensionalValues(limits)

            if "orientation" not in orientation:
                limits["lower"] = [0,0,0]
                limits["upper"] = [2*np.pi,2*np.pi,2*np.pi]

                random_orientation = self.GenerateRandomDimensionalValues(limits)
            random_poses.append(self.PoseFromArray([[np.cos(angle1)*np.sin(angle2)*radius, np.sin(angle1)*np.sin(angle2)*radius, np.cos(angle2)*radius],
                                                    [random_orientation[0],random_orientation[1],random_orientation[2]]]))

        return random_poses


class Cylinder(GenericGeometry):
    def __init__(self,geometry_def,parent_name,parent_prefix,transforms_list):
        GenericGeometry.__init__(self,geometry_def,parent_name,parent_prefix,transforms_list)

        self.MakeRvizMarker()

        if "poses_sets" in geometry_def.keys():
            for poses_set_def in geometry_def["poses_sets"]:
                if poses_set_def["type"] == "coordinates":
                    self.GeneratePosesSetCoordinates(poses_set_def)

                elif poses_set_def["type"] == "matrix":
                    self.GeneratePosesSetDimensionMatrix(poses_set_def)

                elif poses_set_def["type"] == "random":
                    self.GeneratePosesSetRandom(poses_set_def)

                elif poses_set_def["type"] == "zigzag":
                    self.GeneratePosesSetZigZag(poses_set_def)

    def PosesDimensionMatrix(self,poses_set_def,selected_positions_matrix):

        # Create a list shaped as the obstacle tube
        poses_matrix = copy.deepcopy(selected_positions_matrix.tolist())       ### HACERLO MAS FACIL SIN PASAR selected_positions_matrix

        # self.obstacles_matrix[obstacle_positions] = Obstacle(np.arange(obstacle_positions.sum),"sphere",list(obstacle_positions)*2,[0,0,0,1])            SIMPLIFICAR ASÏ
        # For every position in the three dimesions
        for i in np.arange(poses_set_def["dimensions"][0]):
            for j in np.arange(poses_set_def["dimensions"][1]):
                for k in np.arange(poses_set_def["dimensions"][2]):

                    angle1 = (0.5+i)*(np.pi)/poses_set_def["dimensions"][0]
                    radius = (0.5+j)*self.dimensions[1]/poses_set_def["dimensions"][1]-self.dimensions[1]/2      # Esfera de radio [,,R]
                    height = -self.dimensions[2]/2+(0.5+k)*self.dimensions[2]/poses_set_def["dimensions"][2]


                    pose = self.PoseFromArray([[np.cos(angle1)*radius,\
                                                np.sin(angle1)*radius,\
                                                height],
                                                poses_set_def["poses_orientation"]])

                    poses_matrix[i][j][k] = pose

        return poses_matrix


    def RandomPoses(self,quantity = 1,orientation = [0,0,0]):

        random_poses = []

        for i in range(quantity):

            limits = {}
            limits["lower"] = [0,0,-self.dimensions[2]/2]
            limits["upper"] = [2*np.pi,self.dimensions[1]/2,self.dimensions[2]/2]

            [angle1,radius,height] = self.GenerateRandomDimensionalValues(limits)

            if "orientation" not in orientation:
                limits["lower"] = [0,0,0]
                limits["upper"] = [2*np.pi,2*np.pi,2*np.pi]

                random_orientation = self.GenerateRandomDimensionalValues(limits)
            random_poses.append(self.PoseFromArray([[np.cos(angle1)*radius, np.sin(angle1)*radius, height],
                                                    [random_orientation[0],random_orientation[1],random_orientation[2]]]))

        return random_poses


class Torus(GenericGeometry):
    def __init__(self,geometry_def,parent_name,parent_prefix,transforms_list):
        GenericGeometry.__init__(self,geometry_def,parent_name,parent_prefix,transforms_list)

        self.MakeRvizTrousArray()

        if "poses_sets" in geometry_def.keys():
            for poses_set_def in geometry_def["poses_sets"]:
                if poses_set_def["type"] == "coordinates":
                    self.GeneratePosesSetCoordinates(poses_set_def)

                elif poses_set_def["type"] == "matrix":
                    self.GeneratePosesSetDimensionMatrix(poses_set_def)

                elif poses_set_def["type"] == "random":
                    self.GeneratePosesSetRandom(poses_set_def)

                elif poses_set_def["type"] == "zigzag":
                    self.GeneratePosesSetZigZag(poses_set_def)

    def TBDPosesDimensionMatrix(self,poses_set_def,selected_positions_matrix):

        # Create a list shaped as the obstacle tube
        poses_matrix = copy.deepcopy(selected_positions_matrix.tolist())       ### HACERLO MAS FACIL SIN PASAR selected_positions_matrix

        # self.obstacles_matrix[obstacle_positions] = Obstacle(np.arange(obstacle_positions.sum),"sphere",list(obstacle_positions)*2,[0,0,0,1])            SIMPLIFICAR ASÏ
        # For every position in the three dimesions
        for i in np.arange(poses_set_def["dimensions"][0]):
            for j in np.arange(poses_set_def["dimensions"][1]):
                for k in np.arange(poses_set_def["dimensions"][2]):

                    angle1 = (0.5+i)*(np.pi)/poses_set_def["dimensions"][0]
                    radius = (0.5+j)*self.dimensions[1]/poses_set_def["dimensions"][1]-self.dimensions[1]/2      # Esfera de radio [,,R]
                    height = -self.dimensions[2]/2+(0.5+k)*self.dimensions[2]/poses_set_def["dimensions"][2]


                    pose = self.PoseFromArray([[np.cos(angle1)*radius,\
                                                np.sin(angle1)*radius,\
                                                height],
                                                poses_set_def["poses_orientation"]])

                    poses_matrix[i][j][k] = pose

        return poses_matrix


    def RandomPoses(self,quantity = 1,orientation = [0,0,0]):

        random_poses = []

        for i in range(quantity):

            limits = {}
            limits["lower"] = [0,0,-self.dimensions[2]/2]
            limits["upper"] = [2*np.pi,np.pi,self.dimensions[2]/2]

            [angular1,angular2,linear] = self.GenerateRandomDimensionalValues(limits)

            if "orientation" not in orientation:
                limits["lower"] = [0,0,0]
                limits["upper"] = [2*np.pi,2*np.pi,2*np.pi]

                random_orientation = self.GenerateRandomDimensionalValues(limits)

            random_poses.append(self.PoseFromArray([[np.cos(angular1)*(self.dimensions[0]/2+np.cos(angular2)*linear),
                                                     np.sin(angular1)*(self.dimensions[0]/2+np.cos(angular2)*linear),
                                                     np.sin(angular2)*linear],
                                                    [random_orientation[0],random_orientation[1],random_orientation[2]]]))

        return random_poses


class Prism(GenericGeometry):
    def __init__(self,geometry_def,parent_name,parent_prefix,transforms_list):
        GenericGeometry.__init__(self,geometry_def,parent_name,parent_prefix,transforms_list)

        self.TransformVertexes()
        self.ClusterVertexesOnPolygons()

        self.MakeRvizPolygonArray()

        if "poses_sets" in geometry_def.keys():
            for poses_set_def in geometry_def["poses_sets"]:
                if poses_set_def["type"] == "coordinates":
                    self.GeneratePosesSetCoordinates(poses_set_def)

                elif poses_set_def["type"] == "matrix":
                    self.GeneratePosesSetDimensionMatrix(poses_set_def)

                elif poses_set_def["type"] == "random":
                    self.GeneratePosesSetRandom(poses_set_def)

                elif poses_set_def["type"] == "zigzag":
                    self.GeneratePosesSetZigZag(poses_set_def)

        # del(self.transforms_auxiliar_list)

    def TransformVertexes(self):
        n_vertexes = len(self.dimensions[1])
        self.vertexes_tf_list = []
        self.base_vertexes_pose_list = []
        for i,x_y in enumerate(self.dimensions[1]):

            self.vertexes_tf_list.append(StaticTfBroadcaster("{0}_prism_vertex_{1}".format(self.name,i),self.name,self.PoseFromArray([[x_y[0],x_y[1],0.0],[0,0,0,1]]),self.transforms_auxiliar_list))
            self.base_vertexes_pose_list.append(self.vertexes_tf_list[-1].pose)
            # self.transforms_auxiliar_list = self.vertexes_tf_list[-1].getTransforms()
            # print("all",len(self.transforms_auxiliar_list))

            self.vertexes_tf_list.append(StaticTfBroadcaster("{0}_prism_vertex_{1}".format(self.name,i+n_vertexes),self.name,self.PoseFromArray([[x_y[0],x_y[1],self.dimensions[0]],[0,0,0,1]]),self.transforms_auxiliar_list))
            # self.transforms_auxiliar_list = self.vertexes_tf_list[-1].getTransforms()
            # print("all",len(self.transforms_auxiliar_list))

    def ClusterVertexesOnPolygons(self):

        n_vertexes = len(self.vertexes_tf_list)
        self.polygon_array_poses = []

        # Lateral faces
        for polygon_index in range(0,n_vertexes,2):
            polygon_poses = []

            for vertex_index in [0,2,3,1]:
                if polygon_index == n_vertexes-2 and vertex_index>=2:
                    vertex_index = vertex_index - n_vertexes
                polygon_poses.append(self.vertexes_tf_list[polygon_index+vertex_index].getGlobalPose())

            self.polygon_array_poses.append(polygon_poses)

        # Bottom face
        polygon_poses = []
        for vertex_index in range(n_vertexes-2,-2,-2):
            polygon_poses.append(self.vertexes_tf_list[vertex_index].getGlobalPose())
        self.polygon_array_poses.append(polygon_poses)

        # Top face
        polygon_poses = []
        for vertex_index in range(1,n_vertexes,2):
            polygon_poses.append(self.vertexes_tf_list[vertex_index].getGlobalPose())
        self.polygon_array_poses.append(polygon_poses)


    def PosesDimensionMatrix(self,poses_set_def,selected_positions_matrix):
        
        auxiliar_reference_tf, square_limits, center_tf = self.DefineSquareOverPerimeter(self.TranslatePosesList(self.base_vertexes_pose_list,[0,0,self.dimensions[0]/2]),poses_set_def["set_orientation"][2])

        square_dimensions = [square_limits[0][1]-square_limits[0][0],square_limits[1][1]-square_limits[1][0],self.dimensions[0]]

        # Create a list shaped as the obstacle tube
        poses_matrix = copy.deepcopy(selected_positions_matrix.tolist())

        vertexes_point2d_list = []
        for pose in self.base_vertexes_pose_list:
            vertexes_point2d_list.append(Point2D(pose.position.x,pose.position.y))

        polygon = Polygon2D(*vertexes_point2d_list)

        # For every position in the three dimesions
        for i in np.arange(poses_set_def["dimensions"][0]):
            for j in np.arange(poses_set_def["dimensions"][1]):
                for k in np.arange(poses_set_def["dimensions"][2]):

                    pose = self.PoseFromArray([[-square_dimensions[0]/2+(0.5+i)*square_dimensions[0]/poses_set_def["dimensions"][0],\
                                                -square_dimensions[1]/2+(0.5+j)*square_dimensions[1]/poses_set_def["dimensions"][1],\
                                                -square_dimensions[2]/2+(0.5+k)*square_dimensions[2]/poses_set_def["dimensions"][2]],
                                                poses_set_def["poses_orientation"]])

                    auxiliar_tf = StaticTfBroadcaster("aux",center_tf.name,pose,self.transforms_auxiliar_list)
                    _,trans,rot = auxiliar_tf.LookUpTransformFromFrame(self.name)

                    aux_pose = self.PoseFromArray([trans,[0,0,0]])
                    pose.position = aux_pose.position

                    poses_matrix[i][j][k] = pose
                    
                    is_inside = polygon.encloses_point(Point2D(pose.position.x,pose.position.y))

                    if not is_inside:
                        selected_positions_matrix[i][j][k] = False

        return poses_matrix

    
    def PosesDistanceMatrix(self,poses_set_def):

        auxiliar_reference_tf, square_limits, center_tf = self.DefineSquareOverPerimeter(self.TranslatePosesList(self.base_vertexes_pose_list,[0,0,self.dimensions[0]/2]),poses_set_def["set_orientation"][2])

        aux_cube_dimension = [square_limits[0][1]-square_limits[0][0],square_limits[1][1]-square_limits[1][0],self.dimensions[0]]

        range_x = np.arange(0.2-aux_cube_dimension[0]/2,aux_cube_dimension[0]/2,poses_set_def["dimensions"][0])
        range_y = np.arange(0.2-aux_cube_dimension[1]/2,aux_cube_dimension[1]/2,poses_set_def["dimensions"][1])
        range_z = np.arange(0.0,1.0,self.dimensions[0])

        selected_positions_matrix = self.RawDensityMatrix({"density":poses_set_def["density"],
                                                            "dimensions" : [len(range_x),len(range_y),len(range_z)]})

        # Create a list shaped as the obstacle tube
        poses_matrix = copy.deepcopy(selected_positions_matrix.tolist())

        vertexes_point2d_list = []
        for pose in self.base_vertexes_pose_list:
            vertexes_point2d_list.append(Point2D(pose.position.x,pose.position.y))

        polygon = Polygon2D(*vertexes_point2d_list)

        # For every position in the three dimesions
        for i,I in enumerate(range_x):
            for j,J in enumerate(range_y):
                for k,K in enumerate(range_z):

                    pose = self.PoseFromArray([[I,J,K], poses_set_def["poses_orientation"]])

                    auxiliar_tf = StaticTfBroadcaster("aux",center_tf.name,pose,self.transforms_auxiliar_list)
                    _,trans,rot = auxiliar_tf.LookUpTransformFromFrame(self.name)

                    aux_pose = self.PoseFromArray([trans,[0,0,0]])
                    pose.position = aux_pose.position
                    
                    poses_matrix[i][j][k] = pose
                    
                    is_inside = polygon.encloses_point(Point2D(pose.position.x,pose.position.y))

                    if not is_inside:
                        selected_positions_matrix[i][j][k] = False

        return poses_matrix, selected_positions_matrix

    def TranslatePosesList(self,poses_list,direction):

        translated_poses_list = []
        for pose in poses_list:
            pose_array = self.ArrayFromPose(pose)
            pose_array[0][0] += direction[0]
            pose_array[0][1] += direction[1]
            pose_array[0][2] += direction[2]

            translated_poses_list.append(self.PoseFromArray(pose_array))

        return translated_poses_list

    def DefineSquareOverPerimeter(self,perimeter_vertexes_poses_list,sweep_angle):

        # create auxiliar tf with desired orientation over first vertex
        first_vertex_position = perimeter_vertexes_poses_list[0].position

        auxiliar_reference_pose = self.PoseFromArray([[first_vertex_position.x,first_vertex_position.y,first_vertex_position.z],[0,0,sweep_angle]])

        auxiliar_reference_tf = StaticTfBroadcaster("{0}_auxiliar_reference".format(self.name),self.name,auxiliar_reference_pose,self.transforms_auxiliar_list)
        self.transforms_auxiliar_list = auxiliar_reference_tf.getTransforms()
        # print("all",len(self.transforms_auxiliar_list))

        # create frames for every perimeter axes and fill x,y distances
        perimeter_vertexes_tf_list = []
        distances_x = []
        distances_y = []
        for i,vertex_pose in enumerate(perimeter_vertexes_poses_list):
            perimeter_vertexes_tf_list.append(StaticTfBroadcaster("{0}_perimeter_vertex_{1}".format(self.name,i),self.name,vertex_pose,self.transforms_auxiliar_list))

            self.transforms_auxiliar_list = perimeter_vertexes_tf_list[-1].getTransforms()
            # print("all",len(self.transforms_auxiliar_list))
            
            _,trans,rot = perimeter_vertexes_tf_list[-1].LookUpTransformFromFrame("{0}_auxiliar_reference".format(self.name))

            distances_x.append(trans[0])
            distances_y.append(trans[1])
        
        square_limits = [[min(distances_x),max(distances_x)],[min(distances_y),max(distances_y)]]

        center_pose = self.PoseFromArray([[square_limits[0][0]+(square_limits[0][1]-square_limits[0][0])/2,
                                           square_limits[1][0]+(square_limits[1][1]-square_limits[1][0])/2,
                                           0.0],[0.0,0.0,0.0]])
        
        center_tf = StaticTfBroadcaster("{0}_center".format(self.name),"{0}_auxiliar_reference".format(self.name),center_pose,self.transforms_auxiliar_list)
        self.transforms_auxiliar_list = center_tf.getTransforms()
        # print("all",len(self.transforms_auxiliar_list))

        return auxiliar_reference_tf, square_limits, center_tf



    def DefineParallelSegmentsOverPerimeter(self,perimeter_vertexes_poses_list,sweep_angle,line_spacing):

        auxiliar_reference_tf, square_limits, center_tf = self.DefineSquareOverPerimeter(perimeter_vertexes_poses_list,sweep_angle)

        # build segments
        segments_list = []
        for segment_y_distance in np.arange(square_limits[1][0],square_limits[1][1],line_spacing[1]):

            pose1_auxiliar = self.PoseFromArray([[square_limits[0][0],segment_y_distance,0.0],[0,0,0]])
            auxiliar_tf = StaticTfBroadcaster("{0}_auxiliar_1".format(self.name),"{0}_auxiliar_reference".format(self.name),pose1_auxiliar,self.transforms_auxiliar_list)
            _,trans,rot = auxiliar_tf.LookUpTransformFromFrame(self.name)
            p1 = Point3D(trans[0],trans[1],trans[2])

            pose2_auxiliar = self.PoseFromArray([[square_limits[0][1],segment_y_distance,0.0],[0,0,0]])
            auxiliar_tf = StaticTfBroadcaster("{0}_auxiliar_2".format(self.name),"{0}_auxiliar_reference".format(self.name),pose2_auxiliar,self.transforms_auxiliar_list)
            _,trans,rot = auxiliar_tf.LookUpTransformFromFrame(self.name)
            p2 = Point3D(trans[0],trans[1],trans[2])

            segments_list.append(Segment3D(p1,p2))
        
        return segments_list

    def SortedIntersectionsOfTwoSetOfSegments(self,set1,set2):

        # Extract cuts that set2 makes on set1 for each segment in set2
        intersection_tf_matrix = []
        i=0
        for segment2 in set2:
            segment_intersection_tf_list = []
            for segment1 in set1:
                segment_intersection_point3D = segment2.intersection(segment1)
                if segment_intersection_point3D:
                    segment_intersection_tf = StaticTfBroadcaster(self.name+"zigzag_inter"+str(i),self.name,self.Point3D2Pose(segment_intersection_point3D[0]),self.transforms_auxiliar_list)
                    self.transforms_auxiliar_list = segment_intersection_tf.getTransforms()
                    # print("all",len(self.transforms_auxiliar_list))
                    segment_intersection_tf_list.append(segment_intersection_tf)
                    i += 1

            intersection_tf_matrix.append(segment_intersection_tf_list)

        # Sort intersection groups by distance

        sorted_intersection_tf_matrix = []
        for i,intersections_tf_array in enumerate(intersection_tf_matrix):
            
            distances_list = []
            origin = Point3D(set2[i].points[0][0],set2[i].points[0][1],set2[i].points[0][2])
            for intersection_tf in intersections_tf_array:
                _,trans,_ = intersection_tf.LookUpTransformFromFrame(self.name)
                point = Point3D(trans[0],trans[1],trans[2])
                distances_list.append(origin.distance(point))

            sorted_intersection_tf_array = []
            for j in np.argsort(distances_list):
                sorted_intersection_tf_array.append(intersections_tf_array[j])

            sorted_intersection_tf_matrix.append(sorted_intersection_tf_array)

        return sorted_intersection_tf_matrix

    
    def ZigZagPosesFromIntersectionsTfMatrix(self,sorted_intersection_tf_matrix,initial_sense):

        if initial_sense == "right":
            aux_i = [0,-1]
        elif initial_sense == "left":
            aux_i = [-1,0]

        poses_list = []
        for row_tf_list in sorted_intersection_tf_matrix:
            if row_tf_list:
                poses_list.append(row_tf_list[aux_i[0]].pose)
                poses_list.append(row_tf_list[aux_i[1]].pose)

                if aux_i == [-1,0]:
                    aux_i = [0,-1]
                elif aux_i == [0,-1]:
                    aux_i = [-1,0]

        return poses_list

    
    def Point3D2Pose(self,point3D):

        return self.PoseFromArray([[point3D[0],point3D[1],point3D[2]],[0,0,0]])

    def Pose2Point3D(self,pose):

        return Point3D(pose.position.x, pose.position.y, pose.position.z)

    def SegmentizeListOfPoses(self,poses_list,open):

        n_vertexes = len(poses_list)
        segments_list = []
        for i,vertex_pose in enumerate(poses_list[:-1]):

            segments_list.append(Segment3D(self.Pose2Point3D(poses_list[i]),self.Pose2Point3D(poses_list[i+1])))

        if open == False:
            segments_list.append(Segment3D(self.Pose2Point3D(poses_list[-1]),self.Pose2Point3D(poses_list[0])))

        return segments_list

    def ZigZagOnPerimeter(self,base_vertexes_pose_list,height,sweep_angle,spacing,margins,initial_sense):

        parallel_segments = self.DefineParallelSegmentsOverPerimeter(self.TranslatePosesList(base_vertexes_pose_list,[0,0,height]),sweep_angle,spacing)

        sorted_intersection_tf_matrix = self.SortedIntersectionsOfTwoSetOfSegments(self.SegmentizeListOfPoses(self.TranslatePosesList(base_vertexes_pose_list,[0,0,height]),False),
                                            parallel_segments)

        poses = self.ZigZagPosesFromIntersectionsTfMatrix(sorted_intersection_tf_matrix,initial_sense)

        return poses



