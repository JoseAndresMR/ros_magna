#!/usr/bin/env python2
# -*- coding: utf-8 -*-

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
from uav_abstraction_layer.srv import *
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from xml.dom import minidom
from gazebo_msgs.srv import DeleteModel,SpawnModel
from visualization_msgs.msg import Marker
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray, Torus, TorusArray, PolygonArray
from sympy import Point3D, Line3D, Segment3D
from sympy import Point as Point2D
from sympy import Polygon as Polygon2D
import xml.etree.ElementTree


# from Brain import *

class Worlds(object):
    def __init__(self,ID):
        self.ID = int(ID)

        self.GettingWorldDefinition()       # Global parameters inizialization.
        self.home_path = rospkg.RosPack().get_path('pydag')[:-5]

        print "creating world",ID

        world_def_path = "{0}/Code/JSONs/Worlds/{1}/{2}.json"\
                            .format(self.home_path,self.world_name,self.subworld_name)

        with open(world_def_path) as f:
            self.world_def = json.load(f)

        self.world_definition["world_boundaries"] = self.world_def["scenario"]["world_boundaries"]
        rospy.set_param('world_definition', self.world_definition)

        if self.mission_name != 'basic_movement':

            # Save world definition params
            self.N_uav = self.world_definition['N_uav']
            self.N_obs = self.world_definition['N_obs']
            self.path_length = self.world_definition['path_length']         # In the future would be interesting to be able to decide this for every path generatoiion

        self.obstacleGeneratorGeneric()        # Decide position and spawn the obstacles

    def obstacleGeneratorGeneric(self): 

        self.scenario_def = self.world_def["scenario"]


        # Initialization of lists
        self.n_obs = 0
        self.obs_list = []      # List with the objects that deal with an obstacle
        self.obs_shape_list = []         # List with the shapes of the objects
        self.obs_pose_list = []         # List with the poses of the objects
        self.transforms_list = []       # List of obstacles TF
        self.markers_list = []

        self.volumes = {}

        for volume_def in self.scenario_def["volumes"]:

            self.volumes[volume_def["name"]] = Volume(volume_def,self.transforms_list)
            self.transforms_list = self.volumes[volume_def["name"]].getTransforms()
            if self.volumes[volume_def["name"]].obs_pose_list != []:
                [self.obs_pose_list.append(obs) for obs in self.volumes[volume_def["name"]].obs_pose_list]

        self.world_definition["obs_shape"] = []
        self.world_definition["obs_pose_list"] = self.obs_pose_list
        rospy.set_param('world_definition', self.world_definition)      # Actualize the ROS param

    def getFSPoseGlobal(self,params):

        if params[2] == "matrix":
            return self.volumes[params[0]].getGeometry(params[1]).getFPGlobalPosefromMatrix(params[3])

        elif params[2] == "random":

            return self.volumes[params[0]].getGeometry(params[1]).getFPGlobalPosefromList()

        elif params[2] == "path":
            return self.volumes[params[0]].getGeometry(params[1]).getFPGlobalPosefromPath()

        elif params[2] == "coordinates":
            return self.volumes[params[0]].getGeometry(params[1]).getFPGlobalPosefromCoordinates(params[3])

    
    def GettingWorldDefinition(self):
        self.world_definition = rospy.get_param('world_definition')
        self.mission_name = self.world_definition['mission']
        self.submission_name = self.world_definition['submission']
        self.world_name = self.world_definition['world']
        self.subworld_name = self.world_definition['subworld']
        self.n_simulation = self.world_definition['n_simulation']
        self.N_uav = self.world_definition['N_uav']
        self.N_obs = self.world_definition['N_obs']
        self.n_dataset = self.world_definition['n_dataset']
        self.path_length = self.world_definition['path_length']
        self.solver_algorithm = self.world_definition['solver_algorithm']
        self.smach_view = self.world_definition['smach_view']
        self.depth_camera_use = self.world_definition['depth_camera_use']








class Volume(object):
    def __init__(self,volume_def,transforms_list):
        self.name = volume_def["name"]
        self.prefix = volume_def["prefix"]
        self.permits = volume_def["permits"]

        self.origin = volume_def["origin"]

        self.transforms_list = transforms_list
        print("on volume",len(self.transforms_list))
        self.obs_pose_list = []
        self.fsp_list = []

        tfbroadcaster = StaticTfBroadcaster(self.name,"map",self.origin,self.transforms_list)
        self.transforms_list = tfbroadcaster.getTransforms()
        print("on volume",len(self.transforms_list))

        self.geometry_classes = {"cube" : Cube, "sphere" : Sphere, "cylinder" : Cylinder,
                                 "prism": Prism}

        self.geometries = {}

        for geometry_def in volume_def["geometries"]:

            self.geometries[geometry_def["name"]] = self.geometry_classes[geometry_def["shape"]](geometry_def,self.name,self.prefix,self.transforms_list)
            # self.transforms_list = self.geometries[geometry_def["name"]].getTransforms()
            print("on volume",len(self.transforms_list))

            if self.geometries[geometry_def["name"]].obs_pose_list != []:
                [self.obs_pose_list.append(obs) for obs in self.geometries[geometry_def["name"]].obs_pose_list]

    def getFPGlobalPosefromCoordinates(self,coordinates):

        return FreeSpacePose(np.random.randint(1000),self.PoseFromArray(coordinates),self.name,self.prefix,self.transforms_list).global_pose

                
    def getTransforms(self):

        return self.transforms_list

    def getGeometry(self,geometry_name):

        if geometry_name != "volume":
            return self.geometries[geometry_name]
        else:
            return self

    def PoseFromArray(self,Array):
        quat = tf.transformations.quaternion_from_euler(Array[1][0],Array[1][1],Array[1][2])

        return Pose(Point(Array[0][0],Array[0][1],Array[0][2]),Quaternion(quat[0],quat[1],quat[2],quat[3]))

    def ArrayFromPose(self,pose):

        euler = tf.transformations.euler_from_quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orienation.w)

        return [[pose.position.x,pose.position.y,pose.position.z],[euler[0],euler[1],euler[2]]]
        







        
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

        tfbroadcaster = StaticTfBroadcaster(self.name,self.parent_name,self.origin,self.transforms_auxiliar_list)
        self.transforms_auxiliar_list = tfbroadcaster.getTransforms()
        # print("all",len(self.transforms_auxiliar_list))
        self.transforms_list.append(tfbroadcaster.getTransforms()[-1])
        # print("persistent",len(self.transforms_list))

        time.sleep(1)
        

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

        self.rviz_polython_array = RvizPolygonArray(self.rviz_polygon_array_def)

    def EraseRvizPolygonArray(self):

        self.rviz_polython_array.Erase()

    def getTransforms(self):
        # print("persistent",len(self.transforms_list))
        return self.transforms_list

    def getObstacles(self,indexes):

        return self.obstacles[indexes[0]][indexes[1]][indexes[2]]

    def getFPGlobalPosefromMatrix(self,indexes):

        return self.poses[indexes[0]][indexes[1]][indexes[2]].global_pose

    def getFPGlobalPosefromList(self):

        pose_def = {"use": "poses","quantity": 1}
        self.GeneratePosesSetRandom(pose_def)

        return self.poses[0].global_pose

    def getFPGlobalPosefromPath(self):

        global_path = []
        for pose in self.path:
            global_path.append(pose.global_pose)

        return global_path
    
    def getFPGlobalPosefromCoordinates(self,coordinates):

        return self.GenerateFreeSpacePosesFromCoordinates(coordinates).global_pose

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

        fsposes_list = []

        for i,pose in enumerate(selected_positions):

            fsposes_list.append(FreeSpacePose(str(i),  pose, self.name, self.prefix, self.transforms_auxiliar_list))

            self.transforms_auxiliar_list = fsposes_list[-1].getTransforms()
            # print("all",len(self.transforms_auxiliar_list))
            if make_persistent == True:
                self.transforms_list.append(fsposes_list[-1].getTransforms()[-1])
                # print("persistent",len(self.transforms_list))


        return fsposes_list

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

            self.poses = self.GenerateFreeSpacePosesFromMatrix(selected_positions_matrix,poses_matrix,make_persistent)

    def GeneratePosesSetRandom(self,poses_set_def):

        if "orienation" in poses_set_def.keys():
            random_poses = self.RandomPoses(poses_set_def["quantity"],poses_set_def["orientation"])
        else:
            random_poses = self.RandomPoses(poses_set_def["quantity"])

        if poses_set_def["use"] == "obstacles":

            self.obstacles = self.GenerateObstacleFromPosesList(random_poses,poses_set_def["obstacles_shape"],poses_set_def["obstacles_dimensions"])

        elif poses_set_def["use"] == "poses":

            self.poses = self.GenerateFreeSpacePosesFromPosesList(random_poses)

    def GeneratePosesSetZigZag(self,poses_set_def):

        poses = self.ZigZagOnPerimeter(self.base_vertexes_pose_list,poses_set_def["height"],poses_set_def["sweep_angle"],poses_set_def["spacing"],poses_set_def["margins"],poses_set_def["initial_sense"])

        if poses_set_def["use"] == "obstacles":

            self.obstacles = self.GenerateObstacleFromPosesList(poses,poses_set_def["obstacles_shape"],poses_set_def["obstacles_dimensions"])

        elif poses_set_def["use"] == "poses":

            self.path = self.GenerateFreeSpacePosesFromPosesList(poses)


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
                if poses_set_def["type"] == "matrix":
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



    def GeneratePosesSetZigZag(self,poses_set_def):

        matrix_definition = {"type" : "matrix", "use" : "poses",
                            "dimensions": poses_set_def["dimensions"], "density": 1,
                            "orientation": poses_set_def["orientation"]}

        self.GeneratePosesSetDimensionMatrix(matrix_definition,False)

        zigzag_path = []

        for i in np.arange(matrix_definition["dimensions"][0]):
            for j in np.arange(matrix_definition["dimensions"][1]):
                for k in np.arange(matrix_definition["dimensions"][2]):

                    if i%2 == 1:

                        J = matrix_definition["dimensions"][1]-1 - j

                    else:
                        J=j

                    zigzag_path.append(self.poses[i][J][k])

        self.path = zigzag_path





class Sphere(GenericGeometry):
    def __init__(self,geometry_def,parent_name,parent_prefix,transforms_list):
        GenericGeometry.__init__(self,geometry_def,parent_name,parent_prefix,transforms_list)

        self.MakeRvizMarker()

        if "poses_sets" in geometry_def.keys():
            for poses_set_def in geometry_def["poses_sets"]:
                if poses_set_def["type"] == "matrix":
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

                    angle1 = (0.5+i)*(2*np.pi)/poses_set_def["dimensions"][0]
                    angle2 = (0.5+j)*(np.pi)/poses_set_def["dimensions"][1]
                    radius = (0.5+k)*self.dimensions[2]/poses_set_def["dimensions"][2]/2      # Esfera de radio [,,R]


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
                if poses_set_def["type"] == "matrix":
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

                    angle1 = (0.5+i)*(2*np.pi)/poses_set_def["dimensions"][0]
                    radius = (0.5+j)*self.dimensions[1]/poses_set_def["dimensions"][1]/2      # Esfera de radio [,,R]
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
            limits["lower"] = [0,0,0]
            limits["upper"] = [2*np.pi,self.dimensions[1],self.dimensions[2]]

            [angle1,radius,height] = self.GenerateRandomDimensionalValues(limits)

            if "orientation" not in orientation:
                limits["lower"] = [0,0,0]
                limits["upper"] = [2*np.pi,2*np.pi,2*np.pi]

                random_orientation = self.GenerateRandomDimensionalValues(limits)
            random_poses.append(self.PoseFromArray([[np.cos(angle1)*radius, np.sin(angle1)*radius, height],
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
                if poses_set_def["type"] == "matrix":
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

        # # Bottom face
        # polygon_poses = []
        # for vertex_index in range(n_vertexes-2,-2,-2):
        #     polygon_poses.append(self.vertexes_tf_list[vertex_index].getGlobalPose())
        # self.polygon_array_poses.append(polygon_poses)

        # # Top face
        # polygon_poses = []
        # for vertex_index in range(1,n_vertexes,2):
        #     polygon_poses.append(self.vertexes_tf_list[vertex_index].getGlobalPose())
        # self.polygon_array_poses.append(polygon_poses)


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

        


# Class to deal with one single obstacle
class Obstacle(object):
    def __init__(self,ID,shape,dimensions,pose,parent_name,parent_prefix,transforms_list):

        # Save params from args
        self.ID = ID
        self.name = parent_prefix + '_' + 'obs_{0}'.format(self.ID)
        self.parent_name = parent_name
        self.pose = pose
        self.home_path = rospkg.RosPack().get_path('pydag')[:-5]

        basic_path = "{0}/Code/gz_models/".format(self.home_path)
        product_xml_path_dict = {"cube":basic_path + "{0}.sdf".format("cube"),\
                                "cylinder":basic_path + "{0}.sdf".format("cylinder"),\
                                "sphere":basic_path + "{0}.sdf".format("sphere"),\
                                "brick":basic_path + "{0}.sdf".format("brick"),\
                                }

        et = xml.etree.ElementTree.parse(product_xml_path_dict[shape])
        root = et.getroot()

        if shape == "cube":
            root[0][1][0][0][0][0].text = '{0} {1} {2}'.format(dimensions[0],dimensions[1],dimensions[2])
            root[0][1][1][0][0][0].text = '{0} {1} {2}'.format(dimensions[0],dimensions[1],dimensions[2])
        elif shape == "sphere":
            root[0][1][0][0][0][0].text = str(dimensions[0])
            root[0][1][1][0][0][0].text = str(dimensions[0])

        et.write(product_xml_path_dict[shape])

        # Definition of the dictionary with the paths where every type of obstacle is stores. In the future should be packed
        self.product_xml = open(product_xml_path_dict[shape],"r").read()

        self.rviz_marker_def = {"shape" : shape}
        self.rviz_marker_def["origin"] = [[0,0,0],[0,0,0]]
        self.rviz_marker_def["parent_name"] = str(self.name)
        self.rviz_marker_def["name"] = self.name
        self.rviz_marker_def["id"] = 1
        self.rviz_marker_def["scale"] = dimensions

        color = [0,0,0]
        color.append(0)
        self.rviz_marker_def["color"] = [0,0,0,1]
           
        # Get ROS params
        self.world_definition = rospy.get_param('world_definition')
        self.N_obs = self.world_definition['N_obs']

        # Start service proxis to spawn and delete obstacles
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

        self.transforms_list = transforms_list

        tfbroadcaster = StaticTfBroadcaster(self.name,self.parent_name,self.pose,self.transforms_list)

        self.transforms_list = tfbroadcaster.getTransforms()
        
        self.global_pose = tfbroadcaster.getGlobalPose()

        # self.Spawner()      # Spawn obstacle with defined data

        self.MakeRvizMarker()

        # self.transforms_list = self.transforms_list[:-1]
        # tfbroadcaster = StaticTfBroadcaster(self.name,self.parent_name,self.pose,self.transforms_list)


    # Function to query the service to spawn an obstacle
    def Spawner(self):
        try:
            rospy.wait_for_service('gazebo/spawn_sdf_model')
            response = self.spawn_model(self.name, self.product_xml, "", self.global_pose, "world")
            time.sleep(0.01)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in spawn_sdf_model"

    # Function to query the service to erase an obstacle
    def Erase(self):
        rospy.wait_for_service('gazebo/delete_model')
        try:
            delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
            item_name = self.name
            response = self.delete_model(item_name)
            time.sleep(0.01)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in delete_model obstacle"

    def getTransforms(self):
        return self.transforms_list

    def getGlobalPose(self):
        return self.global_pose

    def MakeRvizMarker(self):

        self.marker = RvizMarker(self.rviz_marker_def)







class FreeSpacePose(object):
    def __init__(self,ID,pose,parent_name,parent_prefix,transforms_list):

        # Save params from args
        self.ID = ID
        self.name = parent_prefix + '_' + 'pos_{0}'.format(self.ID)
        self.parent_name = parent_name
        self.pose = pose

        self.transforms_list = transforms_list

        tfbroadcaster = StaticTfBroadcaster(self.name,self.parent_name,self.pose,self.transforms_list)

        self.transforms_list = tfbroadcaster.getTransforms()
        
        self.global_pose = tfbroadcaster.getGlobalPose()

    def getTransforms(self):
        return self.transforms_list

    def getGlobalPose(self):
        return self.global_pose






class RvizMarker(object):
    def __init__(self,rviz_marker_def):
        self.rviz_marker_def = rviz_marker_def

        self.world_definition = rospy.get_param('world_definition')

        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size = 1)

        self.Spawner()

    def Spawner(self):

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.rviz_marker_def["parent_name"]
        marker.ns = str(self.rviz_marker_def["name"])
        marker.id = self.rviz_marker_def["id"]

        if self.rviz_marker_def["shape"] == "arrow":
            marker.type = 0
        elif self.rviz_marker_def["shape"] == "cube":
            marker.type = 1
        elif self.rviz_marker_def["shape"] == "sphere":
            marker.type = 2
        elif self.rviz_marker_def["shape"] == "cylinder":
            marker.type = 3

        marker.action = 0
        euler = self.rviz_marker_def["origin"][1]
        quaternion = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
        marker.pose = Pose(Point(self.rviz_marker_def["origin"][0][0],self.rviz_marker_def["origin"][0][1],self.rviz_marker_def["origin"][0][2]),
                        Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
        marker.scale = Point(self.rviz_marker_def["scale"][0],self.rviz_marker_def["scale"][1],self.rviz_marker_def["scale"][2])
        marker.color = ColorRGBA(self.rviz_marker_def["color"][0],self.rviz_marker_def["color"][1],self.rviz_marker_def["color"][2],self.rviz_marker_def["color"][3])
        # marker.lifetime = 0# rospy.Duration(0)
        
        self.marker = marker
        t = 0
        while not rospy.is_shutdown() and t < 10:
            self.marker_pub.publish(self.marker)
            t = t+1
            time.sleep(0.05)

    def Erase(self):
        self.marker.action = 2

        self.marker_pub.publish(self.marker)

    def Actualize(self,posestamped):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose = posestamped.pose
        self.marker_pub.publish(self.marker)





class RvizPolygonArray(object):
    def __init__(self,rviz_polygon_array_def):
        self.rviz_polygon_array_def = rviz_polygon_array_def

        self.polygon_array_pub = rospy.Publisher('/visualization_polygon_array/{}'.format(rviz_polygon_array_def["name"]), PolygonArray, queue_size = 1)

        self.Spawner()

    def Spawner(self):

        polygon_array = PolygonArray()
        polygon_array.header.stamp = rospy.Time.now()
        polygon_array.header.frame_id = "map"

        for i,polygon_poses in enumerate(self.rviz_polygon_array_def["polygon_array_poses"]):
            polygon = PolygonStamped()
            polygon.header.stamp = rospy.Time.now()
            polygon.header.frame_id = "map"

            for pose in polygon_poses:
                polygon.polygon.points.append(Point32(pose.position.x,pose.position.y,pose.position.z))
                polygon_array.labels.append(1)
                polygon_array.likelihood.append(1.0)

            polygon_array.polygons.append(polygon)

            # polygon_array.labels = self.rviz_polygon_array_def["labels"]#[i]
            # polygon_array.likelihood = self.rviz_polygon_array_def["likelihood"]#[i]
        
        # polygon_array.labels = [1,1,1,1,1,1]
        # polygon_array.likelihood = [1.0,1.0,1.0,1.0,1.0,1.0]

        self.polygon_array = polygon_array

        t = 0
        while not rospy.is_shutdown() and t < 10:
            self.polygon_array_pub.publish(self.polygon_array)
            t = t+1
            time.sleep(0.05)

    def Erase(self):
        pass

    def Actualize(self,posestamped):
        pass






class StaticTfBroadcaster(object):
    def __init__(self,name,parent_name = "map",poses = [],transforms_list = []):

        self.pose = poses
        self.name = name
        self.transforms_list = copy.deepcopy(transforms_list)

        self.lookup_transform_listener = tf.TransformListener()

        transformStamped = geometry_msgs.msg.TransformStamped()
        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = parent_name

        if type(poses) == list:
            poses = np.array(poses)

            if len(poses.shape) == 2: 
                transformStamped.child_frame_id = name
                transformStamped.transform.translation = Point(poses[0][0],poses[0][1],poses[0][2])
                quat = tf.transformations.quaternion_from_euler(poses[1][0],poses[1][1],poses[1][2])
                transformStamped.transform.rotation = Quaternion(quat[0],quat[1],quat[2],quat[3])   

                self.transforms_list.append(copy.deepcopy(transformStamped))

            elif len(poses.shape) == 3: 
                identity = 0
                for i in range(poses.shape[0]):
                    for j in range(poses.shape[1]):
                        for k in range(poses.shape[2]):
                            transformStamped.child_frame_id = name + '_{0}_{1}_{2}'.format(i,j,k)
                            transformStamped.transform.translation = poses[i][j][k].position
                            transformStamped.transform.rotation = poses[i][j][k].orientation  

                            identity + 1

                            self.transforms_list.append(copy.deepcopy(transformStamped))

        elif type(poses) == type(Pose()):
            transformStamped.child_frame_id = name
            transformStamped.transform.translation = poses.position
            transformStamped.transform.rotation = poses.orientation  

            self.transforms_list.append(copy.deepcopy(transformStamped))

        self.Broadcast()


    def Broadcast(self):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        broadcaster.sendTransform(self.transforms_list)
        time.sleep(0.05)

    def getTransforms(self):
        return self.transforms_list

    def LookUpTransformFromFrame(self,frame):
        
        exit_flag = False
        while not(rospy.is_shutdown()) and exit_flag == False:
            try:
                trans,rot = self.lookup_transform_listener.lookupTransform(frame,self.name, rospy.Time.now())
                exit_flag = True
            except:
                time.sleep(0.2)

        pose = Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))

        return pose,trans,rot

    def getGlobalPose(self):

        self.global_pose,_,_ = self.LookUpTransformFromFrame("map")

        return self.global_pose



class DynamicTfBroadcaster(object):
    def __init__(self,name,parent_name = "map",poses = [],transforms_list = []):

        self.pose = poses
        self.name = name
        self.transforms_list = copy.deepcopy(transforms_list)

        self.lookup_transform_listener = tf.TransformListener()

        transformStamped = geometry_msgs.msg.TransformStamped()
        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = parent_name

        if type(poses) == list:
            poses = np.array(poses)

            if len(poses.shape) == 2: 
                transformStamped.child_frame_id = name
                transformStamped.transform.translation = Point(poses[0][0],poses[0][1],poses[0][2])
                quat = tf.transformations.quaternion_from_euler(poses[1][0],poses[1][1],poses[1][2])
                transformStamped.transform.rotation = Quaternion(quat[0],quat[1],quat[2],quat[3])   

                self.transforms_list.append(copy.deepcopy(transformStamped))

            elif len(poses.shape) == 3: 
                identity = 0
                for i in range(poses.shape[0]):
                    for j in range(poses.shape[1]):
                        for k in range(poses.shape[2]):
                            transformStamped.child_frame_id = name + '_{0}_{1}_{2}'.format(i,j,k)
                            transformStamped.transform.translation = poses[i][j][k].position
                            transformStamped.transform.rotation = poses[i][j][k].orientation  

                            identity + 1

                            self.transforms_list.append(copy.deepcopy(transformStamped))

        elif type(poses) == type(Pose()):
            transformStamped.child_frame_id = name
            transformStamped.transform.translation = poses.position
            transformStamped.transform.rotation = poses.orientation  

            self.transforms_list.append(copy.deepcopy(transformStamped))

        self.Broadcast()


    def Broadcast(self):
        broadcaster = tf2_ros.TransformBroadcaster()
        broadcaster.sendTransform(self.transforms_list)
        time.sleep(0.05)

    def getTransforms(self):
        return self.transforms_list

    def LookUpTransformFromFrame(self,frame):
        
        exit_flag = False
        while not(rospy.is_shutdown()) and exit_flag == False:
            try:
                trans,rot = self.lookup_transform_listener.lookupTransform(frame,self.name, rospy.Time.now())
                exit_flag = True
            except:
                time.sleep(0.2)

        pose = Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))

        return pose,trans,rot

    def getGlobalPose(self):

        self.global_pose,_,_ = self.LookUpTransformFromFrame("map")

        return self.global_pose


