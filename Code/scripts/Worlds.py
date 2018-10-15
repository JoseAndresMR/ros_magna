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
from cv_bridge import CvBridge, CvBridgeError
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
        self.GettingWorldDefinition()
        print "creating world",ID
        if self.project == "gauss":
            self.N_uav = self.world_definition['N_uav']
            self.N_obs = self.world_definition['N_obs']
            self.path_length = self.world_definition['path_length']
        
        self.obstacleGenerator()


    def obstacleGenerator(self):
        product_xml_dict = {"cube":open("/home/{1}/catkin_ws/src/Firmware/Tools/sitl_gazebo/models/JA_models/{0}.sdf".format("cube",self.home_path),"r").read(),\
                            "cylinder":open("/home/{1}/catkin_ws/src/Firmware/Tools/sitl_gazebo/models/JA_models/{0}.sdf".format("cylinder",self.home_path),"r").read(),\
                            "sphere":open("/home/{1}/catkin_ws/src/Firmware/Tools/sitl_gazebo/models/JA_models/{0}.sdf".format("sphere",self.home_path),"r").read(),\
                            "brick":open("/home/{1}/catkin_ws/src/Firmware/Tools/sitl_gazebo/models/JA_models/{0}.sdf".format("brick",self.home_path),"r").read(),\
                            }

        self.n_obs = 0
        self.obs_list = []
        self.obs_shape_list = []
        self.obs_pose_list = []
        self.obs_transforms_list = []


        if self.project == "dcdaa":
            self.dbo=[5,1.5,1.5]
            self.obstacle_density = 0.7
            self.obstacles_raw_matrix = np.random.rand(2,3,3)
            self.obstacles_positions = self.obstacles_raw_matrix<=self.obstacle_density
            self.empty_positions = self.obstacles_raw_matrix>self.obstacle_density
            self.poses_matrix = self.obstacles_raw_matrix.tolist()

            # self.obstacles_matrix[obstacle_positions] = Obstacle(np.arange(obstacle_positions.sum),"sphere",list(obstacle_positions)*2,[0,0,0,0])            SIMPLIFICAR ASÏ
            for i in np.arange(self.obstacles_raw_matrix.shape[0]):       ##### O AL MENOS PONER ESTO EN UNA SOLA LINEA
                for k in np.arange(self.obstacles_raw_matrix.shape[2]):
                    for j in np.arange(self.obstacles_raw_matrix.shape[1]):
                        if self.world_type == 1:
                            obs_orientation = Quaternion(0,0,0,0)
                        elif self.world_type == 2:
                            obs_orientation = Quaternion(np.random.rand(1),np.random.rand(1),np.random.rand(1),np.random.rand(1))
                            
                        self.poses_matrix[i][j][k]=Pose(Point(-10+self.dbo[0]*1.5+i*self.dbo[0],\
                                                              -(self.dbo[1]*self.obstacles_raw_matrix.shape[1])/2.0+j*self.dbo[1],\
                                                              1+k*self.dbo[2]),\
                                                        obs_orientation)
                        if self.obstacles_positions[i,j,k] == True:
                            shape = self.randomizeShape()
                            self.obs_shape_list.append(shape)
                            self.obs_list.append(Obstacle(self.n_obs,\
                                                                product_xml_dict[shape],\
                                                                self.poses_matrix[i][j][k],\
                                                                self.obs_transforms_list))              
                            # self.obs_pose_list.append([np.asarray(self.obs_list.point),np.asarray(self.obs_list.quaternion)])       
                            self.obs_pose_list.append([float(self.poses_matrix[i][j][k].position.x),float(self.poses_matrix[i][j][k].position.y),float(self.poses_matrix[i][j][k].position.z)])
                            self.n_obs=self.n_obs+1
            self.world_definition["N_obs"] = self.n_obs

        elif self.project == "gauss":
            if self.world_type == 1:
                self.obs_zone_rad = 3 #4
                self.neutral_zone_width = 1 #2
                self.uavs_zone_width = 1
                self.goal_uncertainty = np.pi/4
                self.obs_zone_lower_heigth = 1
                self.obs_zone_upper_heigth = 5

                for self.n_obs in np.arange(self.N_obs):
                    obs_mu = 0
                    obs_sigma = self.obs_zone_rad*0.5     #### AJUSTAR SIGMA
                    obs_pose = Pose(Point(np.random.normal(obs_mu,obs_sigma,1),\
                                            np.random.normal(obs_mu,obs_sigma,1),\
                                            np.random.uniform(self.obs_zone_lower_heigth,self.obs_zone_upper_heigth,1)),\
                                    Quaternion(0,0,0,0))

                    shape = "sphere"
                    self.obs_shape_list.append(shape)
                    self.obs_list.append(Obstacle(self.n_obs,\
                                                        product_xml_dict[shape],\
                                                        obs_pose,\
                                                        self.obs_transforms_list))
                    self.obs_pose_list.append([float(obs_pose.position.x),float(obs_pose.position.y),float(obs_pose.position.z)])
            
            if self.world_type == 2:
                self.dbo=[5,1.5,1.5]
                self.obstacle_density = 0.7
                self.obstacles_raw_matrix = np.random.rand(2,3,3)
                self.obstacles_positions = self.obstacles_raw_matrix<=self.obstacle_density
                self.empty_positions = self.obstacles_raw_matrix>self.obstacle_density
                self.poses_matrix = self.obstacles_raw_matrix.tolist()

                # self.obstacles_matrix[obstacle_positions] = Obstacle(np.arange(obstacle_positions.sum),"sphere",list(obstacle_positions)*2,[0,0,0,0])            SIMPLIFICAR ASÏ
                for i in np.arange(self.obstacles_raw_matrix.shape[0]):       ##### O AL MENOS PONER ESTO EN UNA SOLA LINEA
                    for k in np.arange(self.obstacles_raw_matrix.shape[2]):
                        for j in np.arange(self.obstacles_raw_matrix.shape[1]):
                            if self.world_type == 1:
                                obs_orientation = Quaternion(0,0,0,0)
                            elif self.world_type == 2:
                                obs_orientation = Quaternion(np.random.rand(1),np.random.rand(1),np.random.rand(1),np.random.rand(1))
                                
                            self.poses_matrix[i][j][k]=Pose(Point(-10+self.dbo[0]*1.5+i*self.dbo[0],\
                                                                -(self.dbo[1]*self.obstacles_raw_matrix.shape[1])/2.0+j*self.dbo[1],\
                                                                1+k*self.dbo[2]),\
                                                            obs_orientation)
                            if self.obstacles_positions[i,j,k] == True:
                                shape = self.randomizeShape()
                                self.obs_shape_list.append(shape)
                                self.obs_list.append(Obstacle(self.n_obs,\
                                                                    product_xml_dict[shape],\
                                                                    self.poses_matrix[i][j][k],\
                                                                    self.obs_transforms_list))              
                                # self.obs_pose_list.append([np.asarray(self.obs_list.point),np.asarray(self.obs_list.quaternion)])       
                                self.obs_pose_list.append([float(self.poses_matrix[i][j][k].position.x),float(self.poses_matrix[i][j][k].position.y),float(self.poses_matrix[i][j][k].position.z)])
                                self.n_obs=self.n_obs+1
                self.world_definition["N_obs"] = self.n_obs
                    

        self.world_definition["obs_shape"] = self.obs_shape_list
        self.world_definition["obs_pose_list"] = self.obs_pose_list
        rospy.set_param('world_definition', self.world_definition)

        print "world", self.ID, "created"

    def PathGenerator(self):
        # if self.project == "dcdaa":
        #     goal_pose = Pose(Point(-10+self.dbo[0]*3+self.obstacles_raw_matrix.shape[0]*self.dbo[0],\
        #                            (np.random.rand(1)-0.5)*self.obstacles_raw_matrix.shape[1],\
        #                            1+self.dbo[2]*self.obstacles_raw_matrix.shape[2]/2.0+(np.random.rand(1)-0.5)*self.obstacles_raw_matrix.shape[2]),\
        #                      Quaternion(0,0,0,0))
        #     self.path_poses_list=[Pose(Point(-10+self.dbo[0]*0.5,\
        #                                      np.asscalar((np.random.rand(1)-0.5)*self.obstacles_raw_matrix.shape[1]),\
        #                                      np.asscalar(1+self.dbo[2]*self.obstacles_raw_matrix.shape[2]/2.0+(np.random.rand(1)-0.5)*self.obstacles_raw_matrix.shape[2])),\
        #                                Quaternion(0,0,0,0))]
        #     for i in np.arange(self.obstacles_raw_matrix.shape[0]):
        #         max_distance = 10000
        #         for j in np.arange(self.obstacles_raw_matrix.shape[1]):
        #             for k in np.arange(self.obstacles_raw_matrix.shape[2]):
        #                 if self.empty_positions[i,j,k] == True:
        #                     current_distance = DistanceBetweenPoses(self.path_poses_list[i],self.poses_matrix[i][j][k])\
        #                                     +DistanceBetweenPoses(self.poses_matrix[i][j][k],goal_pose)
        #                     if current_distance<max_distance:
        #                         max_distance = current_distance
        #                         selected_pose = self.poses_matrix[i][j][k]

        #         security_selected_pose_1 = copy.deepcopy(selected_pose)
        #         security_selected_pose_2 = copy.deepcopy(selected_pose)
        #         security_selected_pose_1.position.x = selected_pose.position.x - self.dbo[0]*1/3.0
        #         security_selected_pose_2.position.x = selected_pose.position.x + self.dbo[0]*1/3.0
        #         self.path_poses_list.append(security_selected_pose_1)
        #         self.path_poses_list.append(security_selected_pose_2)

        #     self.path_poses_list.append(goal_pose)

        if self.project == "dcdaa":
            goal_pose = Pose(Point(-10+self.dbo[0]*3+self.obstacles_raw_matrix.shape[0]*self.dbo[0],\
                                   (np.random.rand(1)-0.5)*self.obstacles_raw_matrix.shape[1],\
                                   1+self.dbo[2]*self.obstacles_raw_matrix.shape[2]/2.0+(np.random.rand(1)-0.5)*self.obstacles_raw_matrix.shape[2]),\
                             Quaternion(0,0,0,0))
            self.path_poses_list=[Pose(Point(-10+self.dbo[0]*0.5,\
                                             np.asscalar((np.random.rand(1)-0.5)*self.obstacles_raw_matrix.shape[1]),\
                                             np.asscalar(1+self.dbo[2]*self.obstacles_raw_matrix.shape[2]/2.0+(np.random.rand(1)-0.5)*self.obstacles_raw_matrix.shape[2])),\
                                       Quaternion(0,0,0,0))]
            # for i in np.arange(self.obstacles_raw_matrix.shape[0]):
            #     max_distance = 10000
            #     for j in np.arange(self.obstacles_raw_matrix.shape[1]):
            #         for k in np.arange(self.obstacles_raw_matrix.shape[2]):
            #             if self.empty_positions[i,j,k] == True:
            #                 current_distance = DistanceBetweenPoses(self.path_poses_list[i],self.poses_matrix[i][j][k])\
            #                                 +DistanceBetweenPoses(self.poses_matrix[i][j][k],goal_pose)
            #                 if current_distance<max_distance:
            #                     max_distance = current_distance
            #                     selected_pose = self.poses_matrix[i][j][k]
            #     self.path_poses_list.append(security_selected_pose_1)
            #     self.path_poses_list.append(security_selected_pose_2)

            self.path_poses_list.append(goal_pose)

        elif self.project == "gauss":
            if self.world_type == 1:
                uav_rad = np.random.uniform(self.obs_zone_rad + self.neutral_zone_width,\
                                            self.obs_zone_rad + self.neutral_zone_width+self.uavs_zone_width,\
                                            1)
                uav_ang = np.random.uniform(0,2*np.pi,1)
                uav_heigth = np.random.uniform(self.obs_zone_lower_heigth,self.obs_zone_upper_heigth,1)
                self.path_poses_list=[Pose(Point(np.asscalar(uav_rad*np.cos(uav_ang)),\
                                                np.asscalar(uav_rad*np.sin(uav_ang)),\
                                                np.asscalar(uav_heigth)),\
                                        Quaternion(0,0,0,0))]

                for n_WP in np.arange(self.path_length-1):
                    uav_rad = np.random.uniform(self.obs_zone_rad + self.neutral_zone_width,\
                                            self.obs_zone_rad + self.neutral_zone_width+self.uavs_zone_width,\
                                            1)
                    uav_ang = uav_ang + np.pi + np.random.uniform(-self.goal_uncertainty/2,self.goal_uncertainty/2,1)
                    uav_heigth = np.random.uniform(self.obs_zone_lower_heigth,self.obs_zone_upper_heigth,1)
                    self.path_poses_list.append(Pose(Point(uav_rad*np.cos(uav_ang),\
                                                            uav_rad*np.sin(uav_ang),\
                                                            uav_heigth),\
                                                        Quaternion(0,0,0,0)))

            if self.world_type == 2:
                self.path_poses_list=[Pose(Point(-10+self.dbo[0]*0.5,\
                                                np.asscalar((np.random.rand(1)-0.5)*self.obstacles_raw_matrix.shape[1]),\
                                                np.asscalar(1+self.dbo[2]*self.obstacles_raw_matrix.shape[2]/2.0+(np.random.rand(1)-0.5)*self.obstacles_raw_matrix.shape[2])),\
                                        Quaternion(0,0,0,0))]
                for n_WP in np.arange(self.path_length-1):
                    if n_WP % 2 == 1:
                        Point_x = -10+self.dbo[0]*3+self.obstacles_raw_matrix.shape[0]*self.dbo[0]
                    elif n_WP % 2 == 0:
                        Point_x = -10+self.dbo[0]*0.5

                    new_pose = Pose(Point(Point_x,\
                                      (np.random.rand(1)-0.5)*self.obstacles_raw_matrix.shape[1],\
                                      1+self.dbo[2]*self.obstacles_raw_matrix.shape[2]/2.0+(np.random.rand(1)-0.5)*self.obstacles_raw_matrix.shape[2]),\
                                Quaternion(0,0,0,0))
                    self.path_poses_list.append(new_pose)

        return self.path_poses_list

    def GettingWorldDefinition(self):
        self.world_definition = rospy.get_param('world_definition')
        self.project = self.world_definition['project']
        self.world_type = self.world_definition['type']
        self.home_path = self.world_definition['home_path']
        self.solver_algorithm = self.world_definition['solver_algorithm']

    def eraseAllObstacles(self):
        for obs in self.obs_list:
            obs.Erase()
    
    def randomizeShape(self):
        if self.world_type == 1:
            shape ="brick"

        elif self.world_type == 2:
            data=np.random.rand(1)
            if 0<=data and data<=1.0/3:
                shape ="sphere"
            elif 1.0/3<data and data<=2.0/3:
                shape ="cylinder"
            else:
                shape ="cube"
        return shape

class Obstacle(object):
    def __init__(self,ID,product_xml,pose,obstacle_transform_list):
        self.ID = ID
        self.product_xml = product_xml
        self.pose = pose
        self.world_definition = rospy.get_param('world_definition')
        self.N_obs = self.world_definition['N_obs']
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.obstacle_transform_list = obstacle_transform_list

        self.Spawner()
        self.Tf2TransformUnifier()

        if self.ID == self.N_obs-1:
            self.Tf2UnifiedBroadcaster()

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
        
    def Tf2UnifiedBroadcaster(self):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        broadcaster.sendTransform(self.obstacle_transform_list)

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


def DistanceBetweenPoses(Pose1,Pose2):
    distance=np.sqrt((Pose1.position.x-Pose2.position.x)**2+\
                     (Pose1.position.y-Pose2.position.y)**2+\
                     (Pose1.position.z-Pose2.position.z)**2)
    return distance