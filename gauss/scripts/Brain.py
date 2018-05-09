#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 19 17:03:12 2018

@author: josmilrom
"""
import sys
import rospy
import std_msgs.msg
import time
import math
import numpy as np
import tf
import rvo2
import time
from cv_bridge import CvBridge, CvBridgeError
from uav_abstraction_layer.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *


class Brain(object):
    def __init__(self,ID):
        self.ID = ID
        print "brain",ID,"alive"

        self.world_definition = rospy.get_param('world_definition')
        self.project = self.world_definition['project']
        self.world_type = self.world_definition['type']
        self.n_simulation = self.world_definition['n_simulation']
        self.N_uav = self.world_definition['N_uav']
        self.N_obs = self.world_definition['N_obs']
        self.n_dataset = self.world_definition['n_dataset']
        self.solver_algorithm = self.world_definition['solver_algorithm']
        self.obs_pose_list_simple = self.world_definition['obs_pose_list_simple']

    def Guidance(self,actual_uav_pose_list,actual_uav_vel_list, goal_WP_pose):
        self.actual_uav_pose_list = actual_uav_pose_list
        self.actual_uav_vel_list = actual_uav_vel_list
        self.goal_WP_pose = goal_WP_pose

        if self.solver_algorithm == "simple":
            return self.SimpleGuidance()

        elif self.solver_algorithm == "neural_network":
            return self.NeuralNetwork()

        elif self.solver_algorithm == "orca":
            return self.ORCA()

    def NeuralNetwork(self):
        return new_velocity_twist

    def ORCA(self):

        start = time.time()
        sim = rvo2.PyRVOSimulator(0.11, # float timeStep,
                                  2,   # float neighborDist
                                  2,    # size_t maxNeighbors
                                  100,    # float timeHorizon
                                  5,    # float timeHorizonObst
                                  0.5,     # float radius
                                  2)     # float maxSpeed

        agent_list = []
        for n_uas in np.arange(self.N_uav):
            agent_list.append(sim.addAgent((self.actual_uav_pose_list[n_uas].position.x, self.actual_uav_pose_list[n_uas].position.y)))
            
            # tuple pos, neighborDist=None,
            # maxNeighbors=None, timeHorizon=None,
            # timeHorizonObst=None, radius=None, maxSpeed=None,
            # velocity=None

        # Obstacles
        for n_obs in np.arange(self.N_obs):
            obs_reduced_list = []
            obs_raduis = 1
            obs_pose = self.obs_pose_list_simple[n_obs]
            circle_division = 16
            for n in np.arange(circle_division):
                obs_reduced_list.append((obs_pose[0]+obs_raduis*np.cos(2*np.pi*n/circle_division),obs_pose[1]+obs_raduis*np.sin(2*np.pi*n/circle_division)))
            sim.addObstacle(obs_reduced_list)

        sim.processObstacles()

        for n_uas in np.arange(self.N_uav):
            if n_uas == self.ID-1:
                prefered_velocity = self.SimpleGuidance()
                sim.setAgentPrefVelocity(agent_list[n_uas],(prefered_velocity.linear.x,prefered_velocity.linear.y))
            else:
                sim.setAgentPrefVelocity(agent_list[n_uas],(self.actual_uav_vel_list[n_uas].linear.x,self.actual_uav_vel_list[n_uas].linear.y))

        sim.doStep()
        selected_velocity = sim.getAgentVelocity(self.ID-1)
        new_velocity_twist = Twist(Vector3(0,0,0),Vector3(0,0,0))
        new_velocity_twist.linear.x = selected_velocity[0]
        new_velocity_twist.linear.y = selected_velocity[1]

        finish = time.time()
        # print finish - start

        return new_velocity_twist

    def SimpleGuidance(self):
        relative_WP_linear=Vector3(self.goal_WP_pose.position.x-self.actual_uav_pose_list[self.ID-1].position.x,\
                                self.goal_WP_pose.position.y-self.actual_uav_pose_list[self.ID-1].position.y,\
                                self.goal_WP_pose.position.z-self.actual_uav_pose_list[self.ID-1].position.z)
        relative_WP_pose_degrees=Pose(relative_WP_linear,\
                                Vector3(np.arctan2(relative_WP_linear.z,relative_WP_linear.y),\
                                np.arctan2(relative_WP_linear.x,relative_WP_linear.z),\
                                np.arctan2(relative_WP_linear.y,relative_WP_linear.x)))  #### COMPROBAR ANGULOS

        orientation_list = [self.actual_uav_pose_list[self.ID-1].orientation.x, self.actual_uav_pose_list[self.ID-1].orientation.y, self.actual_uav_pose_list[self.ID-1].orientation.z, self.actual_uav_pose_list[self.ID-1].orientation.w]
        euler = tf.transformations.euler_from_quaternion(orientation_list)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        new_velocity_twist = Twist(relative_WP_pose_degrees.position,\
                                   Vector3(0,\
                                   0,\
                                   relative_WP_pose_degrees.orientation.z-yaw))

        new_velocity_twist.linear.x = self.UpperLowerThresholds(new_velocity_twist.linear.x,2)
        new_velocity_twist.linear.y = self.UpperLowerThresholds(new_velocity_twist.linear.y,2)
        new_velocity_twist.angular.z = self.UpperLowerThresholds(new_velocity_twist.angular.z,0.5)

        return new_velocity_twist

    def Hover(self):
        new_velocity_twist = Twist(Vector3(0,0,0),Vector3(0,0,0))

        return new_velocity_twist

    def UpperLowerThresholds(self,value,threshold):
        if value > threshold:
            value = threshold
        elif value < -threshold:
            value = -threshold
        return value