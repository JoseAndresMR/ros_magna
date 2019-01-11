#!/usr/bin/env python2
# -*- coding: utf-8 -*-
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
from uav_abstraction_layer.srv import *
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from xml.dom import minidom
from gazebo_msgs.srv import DeleteModel,SpawnModel
from visualization_msgs.msg import Marker
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray, Torus, TorusArray
import xml.etree.ElementTree


# from Brain import *

class Worlds(object):
    def __init__(self,ID):
        self.ID = int(ID)

        self.GettingWorldDefinition()       # Global parameters inizialization.
        print "creating world",ID

        world_def_path = "/home/{0}/catkin_ws/src/pydag/Code/Worlds/{1}.json"\
                            .format(self.home_path,self.world_type)

        with open(world_def_path) as f:
            self.world_def = json.load(f)

        if self.mission != 'basic_movement':

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
    
    def GettingWorldDefinition(self):
        self.world_definition = rospy.get_param('world_definition')
        self.mission = self.world_definition['mission']
        self.world_type = self.world_definition['type']
        self.n_simulation = self.world_definition['n_simulation']
        self.N_uav = self.world_definition['N_uav']
        self.N_obs = self.world_definition['N_obs']
        self.obs_tube = self.world_definition['obs_tube']
        self.n_dataset = self.world_definition['n_dataset']
        self.uav_models = self.world_definition['uav_models']
        self.path_length = self.world_definition['path_length']
        self.home_path = self.world_definition['home_path']
        self.solver_algorithm = self.world_definition['solver_algorithm']
        self.smach_view = self.world_definition['smach_view']
        self.depth_camera_use = self.world_definition['depth_camera_use']

# Class to deal with one single obstacle
class Obstacle(object):
    def __init__(self,ID,shape,dimensions,pose,parent_name,parent_prefix,transforms_list):

        # Save params from args
        self.ID = ID
        self.name = parent_prefix + '_' + 'obs_{0}'.format(self.ID)
        self.parent_name = parent_name
        self.pose = pose
        self.home_path = rospy.get_param('world_definition/home_path')

        basic_path = "/home/{0}/catkin_ws/src/pydag/Code/gz_models/".format(self.home_path)
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

        et.write(product_xml_path_dict[shape])

        # Definition of the dictionary with the paths where every type of obstacle is stores. In the future should be packed
        self.product_xml = open(product_xml_path_dict[shape],"r").read()

        self.marker_def = {"shape" : shape}
        self.marker_def["origin"] = [[0,0,0],[0,0,0]]
        self.marker_def["parent_name"] = str(self.name)
        self.marker_def["name"] = self.name
        self.marker_def["id"] = 1
        self.marker_def["scale"] = dimensions

        color = [0,0,0]
        color.append(0)
        self.marker_def["color"] = [0,0,0,1]
           
        # Get ROS params
        self.world_definition = rospy.get_param('world_definition')
        self.N_obs = self.world_definition['N_obs']

        # Start service proxis to spawn and delete obstacles
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

        self.transforms_list = transforms_list

        tfbroadcaster = TfBroadcaster(self.name,self.parent_name,self.pose,self.transforms_list)
        tfbroadcaster.Broadcast()

        self.transforms_list = tfbroadcaster.getTransforms()
        listener = tf.TransformListener()
        
        trans,rot = listener.lookupTransform('map',self.name, rospy.Time.now())

        self.global_pose = Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))

        self.Spawner()      # Spawn obstacle with defined data

        self.MakeMarker()


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

    def MakeMarker(self):

        self.marker = RvizMarker(self.marker_def)

class FreeSpacePose(object):
    def __init__(self,ID,pose,parent_name,parent_prefix,transforms_list):

        # Save params from args
        self.ID = ID
        self.name = parent_prefix + '_' + 'pos_{0}'.format(self.ID)
        self.parent_name = parent_name
        self.pose = pose

        self.transforms_list = transforms_list

        tfbroadcaster = TfBroadcaster(self.name,self.parent_name,self.pose,self.transforms_list)
        tfbroadcaster.Broadcast()

        self.transforms_list = tfbroadcaster.getTransforms()
        listener = tf.TransformListener()
        
        trans,rot = listener.lookupTransform('map',self.name, rospy.Time.now())

        self.global_pose = Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))


    def getTransforms(self):
        return self.transforms_list

    def getGlobalPose(self):
        return self.global_pose


class RvizMarker(object):
    def __init__(self,marker_def):
        self.marker_def = marker_def

        self.world_definition = rospy.get_param('world_definition')

        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size = 1)

        self.Spawner()

    def Spawner(self):

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.marker_def["parent_name"]
        marker.ns = str(self.marker_def["name"])
        marker.id = self.marker_def["id"]

        if self.marker_def["shape"] == "arrow":
            marker.type = 0
        elif self.marker_def["shape"] == "cube":
            marker.type = 1
        elif self.marker_def["shape"] == "sphere":
            marker.type = 2
        elif self.marker_def["shape"] == "cylinder":
            marker.type = 3

        marker.action = 0
        euler = self.marker_def["origin"][1]
        quaternion = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
        marker.pose = Pose(Point(self.marker_def["origin"][0][0],self.marker_def["origin"][0][1],self.marker_def["origin"][0][2]),
                        Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
        marker.scale = Point(self.marker_def["scale"][0],self.marker_def["scale"][1],self.marker_def["scale"][2])
        marker.color = ColorRGBA(self.marker_def["color"][0],self.marker_def["color"][1],self.marker_def["color"][2],self.marker_def["color"][3])
        # marker.lifetime = 0# rospy.Duration(0)
        
        self.marker = marker
        t = 0
        while t < 10:
            self.marker_pub.publish(self.marker)
            t = t+1
            time.sleep(0.1)


    def Erase(self):
        self.marker.action = 2

        self.marker_pub.publish(self.marker)

    def Actualize(self,posestamped):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose = posestamped.pose
        self.marker_pub.publish(self.marker)

class Volume(object):
    def __init__(self,volume_def,transforms_list):
        self.name = volume_def["name"]
        self.prefix = volume_def["prefix"]
        self.permits = volume_def["permits"]

        self.origin = volume_def["origin"]

        self.transforms_list = transforms_list
        self.obs_pose_list = []

        tfbroadcaster = TfBroadcaster(self.name,"map",self.origin,self.transforms_list)
        tfbroadcaster.Broadcast()
        self.transforms_list = tfbroadcaster.getTransforms()


        self.geometry_classes = {"cube" : Cube, "sphere" : Sphere, "cylinder" : Cylinder}

        self.geometries = {}

        for geometry_def in volume_def["geometries"]:

            self.geometries[geometry_def["name"]] = self.geometry_classes[geometry_def["shape"]](geometry_def,self.name,self.prefix,self.transforms_list)
            self.transforms_list = self.geometries[geometry_def["name"]].getTransforms()

            if self.geometries[geometry_def["name"]].obs_pose_list != []:
                [self.obs_pose_list.append(obs) for obs in self.geometries[geometry_def["name"]].obs_pose_list]
                
    def getTransforms(self):

        return self.transforms_list

    def getGeometry(self,geometry_name):

        return self.geometries[geometry_name]
        
class GenericGeometry:
    def __init__(self,geometry_def,parent_name,parent_prefix,transforms_list):

        self.name = parent_prefix + '_' + geometry_def["name"]
        self.prefix = parent_prefix + '_' + geometry_def["prefix"]
        self.shape = geometry_def["shape"]
        self.origin = geometry_def["origin"]
        self.color = geometry_def["color"]
        self.alpha = geometry_def["alpha"]
        self.id = geometry_def["id"]
        self.parent_name = parent_name

        self.transforms_list = transforms_list

        self.n_obs = 0
        self.obs_pose_list = []
        self.obs_list = []
        self.obs_transforms_list = []
        self.obs_shape_list = []

        tfbroadcaster = TfBroadcaster(self.name,self.parent_name,self.origin,self.transforms_list)
        tfbroadcaster.Broadcast()
        self.transforms_list = tfbroadcaster.getTransforms()

        time.sleep(1)
        
        self.marker_def = {"shape" : self.shape}
        self.marker_def["origin"] = self.origin
        self.marker_def["parent_name"] = parent_name
        self.marker_def["name"] = self.name
        self.marker_def["id"] = self.id

        color = geometry_def["color"]
        color.append(self.alpha)
        self.marker_def["color"] = color

        
    def MakeMarker(self):

        self.marker = RvizMarker(self.marker_def)

    def EraseMarker(self):

        self.marker.Erase()

    def getTransforms(self):

        return self.transforms_list

    def getObstacles(self,indexes):

        return self.obstacles[indexes[0]][indexes[1]][indexes[2]]

    def getFPGlobalPosefromMatrix(self,indexes):

        return self.poses[indexes[0]][indexes[1]][indexes[2]].global_pose

    def getFPGlobalPosefromList(self):

        pose_def = {"use": "poses","quantity": 1}
        self.GenerateRandomPoses(pose_def)

        return self.poses[0].global_pose

    def getFPGlobalPosefromPath(self):

        global_path = []
        for pose in self.path:
            global_path.append(pose.global_pose)

        return global_path

    def RawDensityMatrix(self,obstacles_def):

        # Initializes a tensor with random values and shaped as the obstacle tube
        random_raw_matrix = np.random.rand(obstacles_def["dimensions"][0],obstacles_def["dimensions"][1],obstacles_def["dimensions"][2])

        # If the random value is lower than the density, set its obstacle tensor's position at True
        selected_positions_matrix = random_raw_matrix<=obstacles_def["density"]

        return selected_positions_matrix

    def GenerateObstacleFromDensityMatrix(self,selected_positions_matrix,poses_matrix,shapes,dimensions):

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
                                                            self.transforms_list)

                        self.transforms_list = obs_list[i][j][k].getTransforms()

                        # self.obs_pose_list.append([np.asarray(self.obs_list.point),np.asarray(self.obs_list.quaternion)])

                        # Add also its single list pose to the list of single list poses
                        self.obs_pose_list.append([[float(obs_list[i][j][k].global_pose.position.x),float(obs_list[i][j][k].global_pose.position.y),float(obs_list[i][j][k].global_pose.position.z)],[0,0,0,0]])

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
                                    self.transforms_list))
            self.obs_pose_list.append([[float(obs_list[-1].global_pose.position.x),float(obs_list[-1].global_pose.position.y),float(obs_list[-1].global_pose.position.z)],[0,0,0,0]])

            self.n_obs=self.n_obs+1     # Actualize the counter of created obstacles

        return obs_list

    def GenerateFreeSpacePosesFromDensityMatrix(self,selected_positions_matrix,poses_matrix):

        fsposes_list = copy.deepcopy(selected_positions_matrix.tolist())

        for i in np.arange(selected_positions_matrix.shape[0]):
            for j in np.arange(selected_positions_matrix.shape[1]):
                for k in np.arange(selected_positions_matrix.shape[2]):

                    # Add the obstacle object to the list of obstacles. During its initialization are already spawned
                    fsposes_list[i][j][k] = FreeSpacePose('{0}_{1}_{2}'.format(i,j,k),\
                                                        poses_matrix[i][j][k],\
                                                        self.name,\
                                                        self.prefix,\
                                                        self.transforms_list)

                    self.transforms_list = fsposes_list[i][j][k].getTransforms()

        return fsposes_list

    def GenerateFreeSpacePosesFromPosesList(self,selected_positions):

        fsposes_list = []

        for i,pose in enumerate(selected_positions):

            fsposes_list.append(FreeSpacePose(str(i),\
                                                pose,\
                                                self.name,\
                                                self.prefix,\
                                                self.transforms_list))

            self.transforms_list = fsposes_list[-1].getTransforms()

        return fsposes_list


    def GenerateDivisionMatrix(self,obstacles_def):

        selected_positions_matrix = self.RawDensityMatrix(obstacles_def)

        poses_matrix = self.PosesDensityMatrix(obstacles_def,selected_positions_matrix)

        if obstacles_def["use"] == "obstacles":

            self.obstacles = self.GenerateObstacleFromDensityMatrix(selected_positions_matrix,poses_matrix,obstacles_def["obstacles_shape"],obstacles_def["obstacles_dimensions"])

        elif obstacles_def["use"] == "poses":

            self.poses = self.GenerateFreeSpacePosesFromDensityMatrix(selected_positions_matrix,poses_matrix)

    def GenerateRandomPoses(self,obstacles_def):

        if "orienation" in obstacles_def.keys():
            random_poses = self.RandomPoses(obstacles_def["quantity"],obstacles_def["orientation"])
        else:
            random_poses = self.RandomPoses(obstacles_def["quantity"])

        if obstacles_def["use"] == "obstacles":

            self.obstacles = self.GenerateObstacleFromPosesList(random_poses,obstacles_def["obstacles_shape"],obstacles_def["obstacles_dimensions"])

        elif obstacles_def["use"] == "poses":

            self.poses = self.GenerateFreeSpacePosesFromPosesList(random_poses)


    def GenerateRandomDimensionalValues(self, limits):
        return np.random.uniform(limits["lower"],limits["upper"])


    def PoseFromArray(self,Array):
        quat = tf.transformations.quaternion_from_euler(Array[1][0],Array[1][1],Array[1][2])

        return Pose(Point(Array[0][0],Array[0][1],Array[0][2]),Quaternion(quat[0],quat[1],quat[2],quat[3]))

    def GenerateZigZag(self,obstacles_def):

        matrix_definition = {"type" : "matrix", "use" : "poses",
                            "dimensions": obstacles_def["dimensions"], "density": 1,
                            "orientation": obstacles_def["orientation"]}

        self.GenerateDivisionMatrix(matrix_definition)

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

class Cube(GenericGeometry):
    def __init__(self,geometry_def,parent_name,parent_prefix,transforms_list):
        GenericGeometry.__init__(self,geometry_def,parent_name,parent_prefix,transforms_list)

        self.dimensions = geometry_def["dimensions"]

        self.marker_def["scale"] = self.dimensions

        self.MakeMarker()

        if "divisions" in geometry_def.keys():
            for obstacles_def in geometry_def["divisions"]:
                if obstacles_def["type"] == "matrix":
                    self.GenerateDivisionMatrix(obstacles_def)

                elif obstacles_def["type"] == "random":
                    self.GenerateRandomPoses(obstacles_def)

                elif obstacles_def["type"] == "path":
                    self.GenerateZigZag(obstacles_def)

    def PosesDensityMatrix(self,obstacles_def,selected_positions_matrix):

        # Create a list shaped as the obstacle tube
        poses_matrix = copy.deepcopy(selected_positions_matrix.tolist())       ### HACERLO MAS FACIL SIN PASAR selected_positions_matrix

        # self.obstacles_matrix[obstacle_positions] = Obstacle(np.arange(obstacle_positions.sum),"sphere",list(obstacle_positions)*2,[0,0,0,0])            SIMPLIFICAR ASÏ
        # For every position in the three dimesions
        for i in np.arange(obstacles_def["dimensions"][0]):
            for j in np.arange(obstacles_def["dimensions"][1]):
                for k in np.arange(obstacles_def["dimensions"][2]):

                    pose = self.PoseFromArray([[-self.dimensions[0]/2+(0.5+i)*self.dimensions[0]/obstacles_def["dimensions"][0],\
                                                -self.dimensions[1]/2+(0.5+j)*self.dimensions[1]/obstacles_def["dimensions"][1],\
                                                -self.dimensions[2]/2+(0.5+k)*self.dimensions[2]/obstacles_def["dimensions"][2]],
                                                obstacles_def["orientation"]])

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


class Sphere(GenericGeometry):
    def __init__(self,geometry_def,parent_name,parent_prefix,transforms_list):
        GenericGeometry.__init__(self,geometry_def,parent_name,parent_prefix,transforms_list)

        self.dimensions = geometry_def["dimensions"]

        self.marker_def["scale"] = self.dimensions

        self.MakeMarker()

        if "divisions" in geometry_def.keys():
            for obstacles_def in geometry_def["divisions"]:
                if obstacles_def["type"] == "matrix":
                    self.GenerateDivisionMatrix(obstacles_def)

                elif obstacles_def["type"] == "random":
                    self.GenerateRandomPoses(obstacles_def)

                elif obstacles_def["type"] == "path":
                    self.GenerateZigZag(obstacles_def)

    def PosesDensityMatrix(self,obstacles_def,selected_positions_matrix):

        # Create a list shaped as the obstacle tube
        poses_matrix = copy.deepcopy(selected_positions_matrix.tolist())       ### HACERLO MAS FACIL SIN PASAR selected_positions_matrix

        # self.obstacles_matrix[obstacle_positions] = Obstacle(np.arange(obstacle_positions.sum),"sphere",list(obstacle_positions)*2,[0,0,0,0])            SIMPLIFICAR ASÏ
        # For every position in the three dimesions
        for i in np.arange(obstacles_def["dimensions"][0]):
            for j in np.arange(obstacles_def["dimensions"][1]):
                for k in np.arange(obstacles_def["dimensions"][2]):

                    angle1 = (0.5+i)*(2*np.pi)/obstacles_def["dimensions"][0]
                    angle2 = (0.5+j)*(np.pi)/obstacles_def["dimensions"][1]
                    radius = (0.5+k)*self.dimensions[2]/obstacles_def["dimensions"][2]/2      # Esfera de radio [,,R]


                    pose = self.PoseFromArray([[np.cos(angle1)*np.sin(angle2)*radius,\
                                                np.sin(angle1)*np.sin(angle2)*radius,\
                                                np.cos(angle2)*radius],
                                                obstacles_def["orientation"]])

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

        self.dimensions = geometry_def["dimensions"]

        self.marker_def["scale"] = self.dimensions

        self.MakeMarker()

        if "divisions" in geometry_def.keys():
            for obstacles_def in geometry_def["divisions"]:
                if obstacles_def["type"] == "matrix":
                    self.GenerateDivisionMatrix(obstacles_def)

                elif obstacles_def["type"] == "random":
                    self.GenerateRandomPoses(obstacles_def)

                elif obstacles_def["type"] == "path":
                    self.GenerateZigZag(obstacles_def)

    def PosesDensityMatrix(self,obstacles_def,selected_positions_matrix):

        # Create a list shaped as the obstacle tube
        poses_matrix = copy.deepcopy(selected_positions_matrix.tolist())       ### HACERLO MAS FACIL SIN PASAR selected_positions_matrix

        # self.obstacles_matrix[obstacle_positions] = Obstacle(np.arange(obstacle_positions.sum),"sphere",list(obstacle_positions)*2,[0,0,0,0])            SIMPLIFICAR ASÏ
        # For every position in the three dimesions
        for i in np.arange(obstacles_def["dimensions"][0]):
            for j in np.arange(obstacles_def["dimensions"][1]):
                for k in np.arange(obstacles_def["dimensions"][2]):

                    angle1 = (0.5+i)*(2*np.pi)/obstacles_def["dimensions"][0]
                    radius = (0.5+j)*self.dimensions[1]/obstacles_def["dimensions"][1]/2      # Esfera de radio [,,R]
                    height = -self.dimensions[2]/2+(0.5+k)*self.dimensions[2]/obstacles_def["dimensions"][2]


                    pose = self.PoseFromArray([[np.cos(angle1)*radius,\
                                                np.sin(angle1)*radius,\
                                                height],
                                                obstacles_def["orientation"]])

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


class TfBroadcaster(object):
    def __init__(self,name,parent_name = "map",poses = [],transforms_list = []):

        self.transforms_list = transforms_list

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

                self.transforms_list.append(transformStamped)

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

        elif type (poses) == type(Pose()):
            transformStamped.child_frame_id = name
            transformStamped.transform.translation = poses.position
            transformStamped.transform.rotation = poses.orientation  

            self.transforms_list.append(transformStamped)


    def Broadcast(self):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        broadcaster.sendTransform(self.transforms_list)
        time.sleep(0.5)

    def getTransforms(self):
        return self.transforms_list
