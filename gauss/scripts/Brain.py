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
        self.GettingWorldDefinition()

        if self.solver_algorithm == "neural_network":
            pass

    def Guidance(self,uavs_list, goal_WP_pose):
        self.uavs_list = uavs_list
        self.goal_WP_pose = goal_WP_pose

        if self.solver_algorithm == "simple":
            return self.SimpleGuidance()

        elif self.solver_algorithm == "neural_network":
            return self.NeuralNetwork()

        elif self.solver_algorithm == "orca":
            return self.ORCA()
    
    def NeuralNetwork(self):
       

        main_uav_pos = self.uavs_list[self.ID-1].position.pose.position
        main_uav_vel = self.uavs_list[self.ID-1].velocity.twist.linear
        inputs = []
        
        for n_uav in range(self.N_uav):
            if n_uav+1 != self.ID:
                inputs.append(self.uavs_list[n_uav].position.pose.position.x-main_uav_pos.x)
                inputs.append(self.uavs_list[n_uav].position.pose.position.y-main_uav_pos.y)

        inputs.append(main_uav_vel.x)
        inputs.append(main_uav_vel.y)

        for n_uav in range(self.N_uav):
            if n_uav+1 != self.ID:
                inputs.append(self.uavs_list[n_uav].velocity.twist.linear.x)
                inputs.append(self.uavs_list[n_uav].velocity.twist.linear.y)

        for n_obs in range(self.N_obs):
            inputs.append(self.obs_pose_list_simple[n_obs][0]-main_uav_pos.x)
            inputs.append(self.obs_pose_list_simple[n_obs][1]-main_uav_pos.y)

        inputs.append(self.goal_WP_pose.position.x-main_uav_pos.x)
        inputs.append(self.goal_WP_pose.position.y-main_uav_pos.y)

        with tf.Session() as sess:
            #importa el archivo .meta donde está el grafo completo
            # new_saver = tf.train.import_meta_graph('/save_model/world_%s_%s_nn.meta' % (self.N_uav, self.N_obs))
            new_saver = tf.train.import_meta_graph('/save_model/world_1_1_nn.meta')
            #restauro los ultimos valores
            new_saver.restore(sess, tf.train.latest_checkpoint('./'))
            #solo nos interesa la funcion multilayer de la red, llamada asi en la red
            nn_to_restore = graph.get_tensor_by_name('multilayer_perceptron')
            # new_velocity_twist = sees.run(nn_to_restore, feed_dict{inputs})
        
        return new_velocity_twist
    
    def ORCA(self):

        # start = time.time()
        sim = rvo2.PyRVOSimulator(1/60.,   # 1/60.  float   timeStep           The time step of the simulation. Must be positive. 
                                  3.0,     # 1.5    float   neighborDist       The maximal distance (center point to center point) to other agents the agent takes into account in the navigation
                                  4,       # 5      size_t  maxNeighbors       The maximal number of other agents the agent takes into account in the navigation
                                  1.5,     # 1.5    float   timeHorizon        The minimal amount of time for which the agent's velocities that are computed by the simulation are safe with respect to other agents. 
                                  2.5,     # 2      float   timeHorizonObst    The minimal amount of time for which the agent's velocities that are computed by the simulation are safe with respect to obstacles.
                                  0.5,     # 0.4    float   radius             The radius of the agent. Must be non-negative
                                  2.0)     # 2      float   maxSpeed           The maximum speed of the agent. Must be non-negative. 

        agent_list = []
        for n_uas in np.arange(self.N_uav):
            agent_list.append(sim.addAgent((self.uavs_list[n_uas].position.pose.position.x, self.uavs_list[n_uas].position.pose.position.y)))
            5
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
                sim.setAgentPrefVelocity(agent_list[n_uas],(self.uavs_list[n_uas].velocity.twist.linear.x,self.uavs_list[n_uas].velocity.twist.linear.y))

        sim.doStep()
        selected_velocity = sim.getAgentVelocity(self.ID-1)
        new_velocity_twist = Twist(Vector3(0,0,prefered_velocity.linear.z),Vector3(0,0,0))
        new_velocity_twist.linear.x = selected_velocity[0]
        new_velocity_twist.linear.y = selected_velocity[1]

        # finish = time.time()
        # print finish - start

        return new_velocity_twist

    def SimpleGuidance(self):
        desired_velocity_module = 2
        desired_velocity_module_at_goal = 0
        aprox_distance = 3

        relative_distance = np.asarray([self.goal_WP_pose.position.x-self.uavs_list[self.ID-1].position.pose.position.x,\
                                self.goal_WP_pose.position.y-self.uavs_list[self.ID-1].position.pose.position.y,\
                                self.goal_WP_pose.position.z-self.uavs_list[self.ID-1].position.pose.position.z])
                                
        distance_norm = np.linalg.norm(relative_distance)
        if distance_norm < aprox_distance:
            desired_velocity_module = desired_velocity_module_at_goal - (desired_velocity_module - desired_velocity_module_at_goal)\
                                    + ((desired_velocity_module - desired_velocity_module_at_goal) *2) / (1 + math.exp(-5*distance_norm/aprox_distance))
        relative_WP_linear=Vector3(relative_distance[0]/distance_norm*desired_velocity_module,\
                                relative_distance[1]/distance_norm*desired_velocity_module,\
                                relative_distance[2]/distance_norm*desired_velocity_module)
        relative_WP_pose_degrees=Pose(relative_WP_linear,\
                                Vector3(np.arctan2(relative_WP_linear.z,relative_WP_linear.y),\
                                np.arctan2(relative_WP_linear.x,relative_WP_linear.z),\
                                np.arctan2(relative_WP_linear.y,relative_WP_linear.x)))  #### COMPROBAR ANGULOS

        orientation_list = [self.uavs_list[self.ID-1].position.pose.orientation.x, self.uavs_list[self.ID-1].position.pose.orientation.y, self.uavs_list[self.ID-1].position.pose.orientation.z, self.uavs_list[self.ID-1].position.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(orientation_list)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        new_velocity_twist = Twist(relative_WP_pose_degrees.position,\
                                   Vector3(0,\
                                   0,\
                                #    relative_WP_pose_degrees.orientation.z-yaw))
                                   0))

        # new_velocity_twist.linear.x = self.UpperLowerThresholds(new_velocity_twist.linear.x,1.5)
        # new_velocity_twist.linear.y = self.UpperLowerThresholds(new_velocity_twist.linear.y,1.5)
        # new_velocity_twist.angular.z = self.UpperLowerThresholds(new_velocity_twist.angular.z,0.5)

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
    
    def GettingWorldDefinition(self):
        self.world_definition = rospy.get_param('world_definition')
        self.project = self.world_definition['project']
        self.world_type = self.world_definition['type']
        self.n_simulation = self.world_definition['n_simulation']
        self.N_uav = self.world_definition['N_uav']
        self.N_obs = self.world_definition['N_obs']
        self.n_dataset = self.world_definition['n_dataset']
        self.solver_algorithm = self.world_definition['solver_algorithm']
        self.obs_pose_list_simple = self.world_definition['obs_pose_list_simple']
        self.home_path = self.world_definition['home_path']