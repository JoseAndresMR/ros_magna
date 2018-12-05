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
from uav_abstraction_layer.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from xml.dom import minidom
from gazebo_msgs.srv import DeleteModel,SpawnModel
import copy

from Ground_Station import *
from Brain import *

class Worlds(object):
    def __init__(self,ID):
        self.ID = int(ID)

        self.GettingWorldDefinition()       # Global parameters inizialization.
        print "creating world",ID

        if self.mission != 'basic_movement':

            # Save world definition params
            self.N_uav = self.world_definition['N_uav']
            self.N_obs = self.world_definition['N_obs']
            self.path_length = self.world_definition['path_length']         # In the future would be interesting to be able to decide this for every path generatoiion

        self.obstacleGenerator()        # Decide position and spawn the obstacles

    # Decide the position and shape and spawn in Gazebo the obstacles
    def obstacleGenerator(self):

        # Definition of the dictionary with the paths where every type of obstacle is stores. In the future should be packed
        product_xml_dict = {"cube":open("/home/{1}/catkin_ws/src/pydag/Code/gz_models/{0}.sdf".format("cube",self.home_path),"r").read(),\
                            "cylinder":open("/home/{1}/catkin_ws/src/pydag/Code/gz_models/{0}.sdf".format("cylinder",self.home_path),"r").read(),\
                            "sphere":open("/home/{1}/catkin_ws/src/pydag/Code/gz_models/{0}.sdf".format("sphere",self.home_path),"r").read(),\
                            "brick":open("/home/{1}/catkin_ws/src/pydag/Code/gz_models/{0}.sdf".format("brick",self.home_path),"r").read(),\
                            }

        # Initialization of lists
        self.n_obs = 0
        self.obs_list = []      # List with the objects that deal with an obstacle
        self.obs_shape_list = []         # List with the shapes of the objects
        self.obs_pose_list = []         # List with the poses of the objects
        self.obs_transforms_list = []       # List of obstacles TF

        ### Creation of obstacles depending on the world type

        if self.world_type == 1:

            # Set scenario parameters
            self.obs_zone_rad = 3 #4
            self.neutral_zone_width = 1 #2
            self.rw_uavs_zone_width = 1
            self.fw_uavs_zone_width = 150
            self.goal_uncertainty = np.pi/4
            self.obs_zone_lower_heigth = 1
            self.obs_zone_upper_heigth = 5

            # For every obstacle, choose randomly a position inside the determined zone
            for self.n_obs in np.arange(self.N_obs):
                # Set randomizer parameters
                obs_mu = 0
                obs_sigma = self.obs_zone_rad*0.5     #### AJUSTAR SIGMA

                # Randomly define the pose
                obs_pose = Pose(Point(np.random.normal(obs_mu,obs_sigma,1),\
                                        np.random.normal(obs_mu,obs_sigma,1),\
                                        np.random.uniform(self.obs_zone_lower_heigth,self.obs_zone_upper_heigth,1)),\
                                Quaternion(0,0,0,0))

                shape = "sphere"        # Impose shpere as shape for first world type

                self.obs_shape_list.append(shape)       # Add to the whole obstacles shape list

                # Add to the whole obstacles object list as are also spawned on int initialization
                self.obs_list.append(Obstacle(self.n_obs,\
                                                    product_xml_dict[shape],\
                                                    obs_pose,\
                                                    self.obs_transforms_list))

                # Add to the wholes poses list
                self.obs_pose_list.append([float(obs_pose.position.x),float(obs_pose.position.y),float(obs_pose.position.z)])

        elif self.world_type == 2:

            # Set scenario parameters
            self.dbo=[10,3.5,3.5]       # Define the distance between objects of the tube of objects
            self.obstacle_density = 0.25     # Defines the probability that in a point of the tube there is an obstacle or not

            # Initializes a tensor with random values and shaped as the obstacle tube
            self.obstacles_raw_matrix = np.random.rand(self.obs_tube[0],self.obs_tube[1],self.obs_tube[2])

            # If the random value is lower than the density, set its obstacle tensor's position at True
            self.obstacles_positions = self.obstacles_raw_matrix<=self.obstacle_density

            # If the random value is bigger than the density, set its obstacle tensor's position at False
            self.empty_positions = self.obstacles_raw_matrix>self.obstacle_density

            # Create a list shaped as the obstacle tube
            self.poses_matrix = self.obstacles_raw_matrix.tolist()

            # self.obstacles_matrix[obstacle_positions] = Obstacle(np.arange(obstacle_positions.sum),"sphere",list(obstacle_positions)*2,[0,0,0,0])            SIMPLIFICAR ASÏ
            # For every position in the three dimesions
            for i in np.arange(self.obstacles_raw_matrix.shape[0]):       ##### O AL MENOS PONER ESTO EN UNA SOLA LINEA
                for k in np.arange(self.obstacles_raw_matrix.shape[2]):
                    for j in np.arange(self.obstacles_raw_matrix.shape[1]):

                        # Randomize orientation or not
                        if self.world_type == 2:
                            obs_orientation = Quaternion(0,0,0,0)
                        elif self.world_type == 4:      # Change
                            obs_orientation = Quaternion(np.random.rand(1),np.random.rand(1),np.random.rand(1),np.random.rand(1))

                        # Create a matrix with the poses of the obstacles in the positions in which has been created one
                        self.poses_matrix[i][j][k]=Pose(Point(i*self.dbo[0],\
                                                              -(-self.dbo[1]/2.0+self.dbo[1]*self.obstacles_raw_matrix.shape[1])/2.0+j*self.dbo[1],\
                                                              (k+0.5)*self.dbo[2]),\
                                                        obs_orientation)

                        # For every obstacled that has been created
                        if self.obstacles_positions[i,j,k] == True:

                            # Randomize shape or not
                            if self.world_type == 2:
                                shape = "brick"
                            elif self.world_type == 4:  # Change
                                shape = self.randomizeShape()

                            self.obs_shape_list.append(shape)       # Add its shape to the list

                            # Add the obstacle object to the list of obstacles. During its initialization are already spawned
                            self.obs_list.append(Obstacle(self.n_obs,\
                                                                product_xml_dict[shape],\
                                                                self.poses_matrix[i][j][k],\
                                                                self.obs_transforms_list))

                            # self.obs_pose_list.append([np.asarray(self.obs_list.point),np.asarray(self.obs_list.quaternion)])

                            # Add also its single list pose to the list of single list poses
                            self.obs_pose_list.append([float(self.poses_matrix[i][j][k].position.x),float(self.poses_matrix[i][j][k].position.y),float(self.poses_matrix[i][j][k].position.z)])

                            self.n_obs=self.n_obs+1     # Actualize the counter of created obstacles

            self.world_definition["N_obs"] = self.n_obs     # Actualize the final number of obstacles to the world definition so it will be a ROS param

        # Actualize new obstacles information to the world definition so it will be a ROS param
        self.world_definition["obs_shape"] = self.obs_shape_list
        self.world_definition["obs_pose_list"] = self.obs_pose_list
        rospy.set_param('world_definition', self.world_definition)      # Actualize the ROS param

        print "world", self.ID, "created"

    # Generate a new adequate path for each world
    def PathGenerator(self, uav_model):

        if self.world_type == 1:

            # Think on cylindric coordinates. The base circle is centered in [0,0,0]
            # Definition of a random radius, angle and height for the first waypoint
            uav_rad = np.random.uniform(self.obs_zone_rad + self.neutral_zone_width,\
                                        self.obs_zone_rad + self.neutral_zone_width+self.rw_uavs_zone_width,\
                                        1)
            uav_ang = np.random.uniform(0,2*np.pi,1)
            uav_heigth = np.random.uniform(self.obs_zone_lower_heigth,self.obs_zone_upper_heigth,1)

            # Transformation on it into a pose
            self.path_poses_list=[Pose(Point(np.asscalar(uav_rad*np.cos(uav_ang)),\
                                            np.asscalar(uav_rad*np.sin(uav_ang)),\
                                            np.asscalar(uav_heigth)),\
                                    Quaternion(0,0,0,0))]
        
            if uav_model == "plane":
                initial_yaw_euler = math.atan2(self.path_poses_list[0].position.y,self.path_poses_list[0].position.x)
                initial_yaw_quaternion = tf.transformations.quaternion_from_euler(0,0,initial_yaw_euler)
                self.path_poses_list[0].orientation = Quaternion(initial_yaw_quaternion[0],initial_yaw_quaternion[1],initial_yaw_quaternion[2],initial_yaw_quaternion[3])


            # For next waypoints, define also random radius, angle and height.
            # But adding pi and some uncertainty to make the drone pass central part of the cylinder
            for n_WP in np.arange(self.path_length-1):

                if uav_model == "typhoon_h480":
                    uav_rad = np.random.uniform(self.obs_zone_rad + self.neutral_zone_width,\
                                            self.obs_zone_rad + self.neutral_zone_width+self.rw_uavs_zone_width,\
                                            1)
                    uav_ang = uav_ang + np.pi + np.random.uniform(-self.goal_uncertainty/2,self.goal_uncertainty/2,1)
                    uav_heigth = np.random.uniform(self.obs_zone_lower_heigth,self.obs_zone_upper_heigth,1)
                    self.path_poses_list.append(Pose(Point(uav_rad*np.cos(uav_ang),\
                                                            uav_rad*np.sin(uav_ang),\
                                                            uav_heigth),\
                                                        Quaternion(0,0,0,0)))

                elif uav_model == "plane":
                    uav_rad = np.random.uniform(self.obs_zone_rad + self.neutral_zone_width,\
                                            self.obs_zone_rad + self.neutral_zone_width+self.fw_uavs_zone_width,\
                                            1)
                    uav_ang = uav_ang + np.pi + np.random.uniform(-self.goal_uncertainty/2,self.goal_uncertainty/2,1)
                    uav_heigth = np.random.uniform(self.obs_zone_lower_heigth,self.obs_zone_upper_heigth,1)
                    self.path_poses_list.append(Pose(Point(uav_rad*np.cos(uav_ang),\
                                                            uav_rad*np.sin(uav_ang),\
                                                            uav_heigth),\
                                                        Quaternion(0,0,0,0)))


        elif self.world_type == 2:


            # for every waypoint, posision random points at one edge of the tube every each time
            self.path_poses_list = []
            for n_WP in np.arange(self.path_length):
                if n_WP % 2 == 1:
                    Point_x = -10 + self.obstacles_raw_matrix.shape[0]*self.dbo[0] + 10
                elif n_WP % 2 == 0:
                    Point_x = -10

                # new_pose = Pose(Point(Point_x,
                #                       float((np.random.rand(1)-0.5) * self.obstacles_raw_matrix.shape[1]),
                #                       1+self.dbo[2]*self.obstacles_raw_matrix.shape[2]/2.0+(np.random.rand(1)-0.5)*self.obstacles_raw_matrix.shape[2]),
                #                 Quaternion(0, 0, 0, 0))

                new_pose = Pose(Point(Point_x,
                                      0,
                                      self.dbo[2]*self.obstacles_raw_matrix.shape[2]/2),
                                Quaternion(0, 0, 0, 0))

                self.path_poses_list.append(new_pose)

        return self.path_poses_list

    # Function to all every saved obstacle and call its erase method
    def eraseAllObstacles(self):
        for obs in self.obs_list:
            obs.Erase()

    # Function to randomize shapes over an equal normal probability
    def randomizeShape(self):

        data=np.random.rand(1)
        if 0<=data and data<=1.0/3:
            shape ="sphere"
        elif 1.0/3<data and data<=2.0/3:
            shape ="cylinder"
        else:
            shape ="cube"

        return shape

    # Function to get Global ROS parameters
    def GettingWorldDefinition(self):
        self.world_definition = rospy.get_param('world_definition')
        self.mission = self.world_definition['mission']
        self.world_type = self.world_definition['type']
        self.home_path = self.world_definition['home_path']
        self.obs_tube = self.world_definition['obs_tube']

# Class to deal with one single obstacle
class Obstacle(object):
    def __init__(self,ID,product_xml,pose,obstacle_transform_list):

        # Save params from args
        self.ID = ID
        self.product_xml = product_xml
        self.pose = pose

        # Get ROS params
        self.world_definition = rospy.get_param('world_definition')
        self.N_obs = self.world_definition['N_obs']

        # Start service proxis to spawn and delete obstacles
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

        self.obstacle_transform_list = obstacle_transform_list

        self.Spawner()      # Spawn obstacle with defined data
        self.Tf2TransformUnifier()

        # Last obstacle, broadcasts the all the obstacles
        if self.ID == self.N_obs-1:
            self.Tf2UnifiedBroadcaster()

    # Function to query the service to spawn an obstacle
    def Spawner(self):
        try:
            rospy.wait_for_service('gazebo/spawn_sdf_model')
            item_name = 'obstacle_{0}'.format(self.ID)
            item_pose = self.pose
            response = self.spawn_model(item_name, self.product_xml, "", item_pose, "world")

            time.sleep(0.01)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in spawn_sdf_model"

    # Function to create a TF from a Pose
    def Tf2TransformUnifier(self):
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "obs_{}".format(self.ID+1)

        static_transformStamped.transform.translation = self.pose.position

        quat = tf.transformations.quaternion_from_euler(0,0,0)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]


        self.obstacle_transform_list.append(static_transformStamped)

    # Broadcast all transformations in the list
    def Tf2UnifiedBroadcaster(self):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        broadcaster.sendTransform(self.obstacle_transform_list)

    # Function to query the service to erase an obstacle
    def Erase(self):
        rospy.wait_for_service('gazebo/delete_model')
        try:
            delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
            item_name = 'obstacle_{0}'.format(self.ID)
            response = self.delete_model(item_name)
            time.sleep(0.01)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in delete_model obstacle"
