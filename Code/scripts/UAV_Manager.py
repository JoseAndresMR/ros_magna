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
import os
import rospy
import std_msgs.msg
import time
import math
import numpy as np
import pandas as pd
import argparse
import tf, tf2_ros
import signal
import copy
import rospkg
# from six.moves import cPickle as pickle
from uav_abstraction_layer.srv import *
from uav_abstraction_layer.msg import State as ual_state_msg
from geometry_msgs.msg import *
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from std_msgs.msg import Int8

from UAV_Brain import *
from pydag.srv import *
from UAV_Data import UAV_Data
from UAV_Config import UAV_Config
from UAV_Manager_SM import UAV_Manager_SM
from uav_path_manager.srv import GeneratePath,GetGeneratedPath,GetGeneratedPathRequest

from FwQgc import FwQgc
class UAV_Manager(object):

    ### Initialiations ###
    def __init__(self,ID):
        self.ID = int(ID)

        self.GettingWorldDefinition()   # Global ROS parameters inizialization

        self.home_path = rospkg.RosPack().get_path('pydag')[:-5]

        self.brain = UAV_Brain(self.ID,self.role)   # Creation of an utility to treat depth camera topic data

        # Local variables initialization
        self.global_data_frame = pd.DataFrame()     # Data frame where to store this UAV dynamical simulation data for future dump into a csv
        self.global_frame_depth = []        # Data frame where to store this UAV depth image simulation data for future dump into a csv

        self.distance_to_goal = 10000
        self.goal = {}      # Structure to store goal specifications
        self.brain.goal = self.goal
        self.smooth_path_mode = 0
        self.start_time_to_target = time.time()     # Counter to check elapsed time to achieve a goal

        self.preemption_launched = False

        self.accepted_target_radio = 0.9        # In the future, received on the world definition

        self.start=time.time()      # Counter to check elapsed time in the calculation of velocity each sintant
        self.last_saved_time = 0        # Counter to check elapsed time to save based on a frequency


        if self.uav_models[self.ID-1] != "plane":
            rospy.init_node('uav_{}'.format(self.ID), anonymous=True)        # Start the node
        else:
            self.fw_qgc = FwQgc(self.ID,ual_use = True)
            
        # Creation of a list within objects of UAV class. They deal with the information of every UAV in the simu.
        # Own ID is given for the object to know if is treating any other UAV or the one dedicated for this GroundStation
        self.uavs_data_list = []
        self.uavs_config_list = []
        for n_uav in range(1,self.N_uav+1):
            self.uavs_config_list.append(UAV_Config(n_uav))
            self.uavs_data_list.append(UAV_Data(n_uav,self.ID,self.uavs_config_list[n_uav-1]))
        
        # Publishers initialisation
        self.pose_pub = rospy.Publisher(self.uavs_config_list[self.ID-1].top_pub_addr['go_to_waypoint'], PoseStamped, queue_size=1)
        self.velocity_pub= rospy.Publisher(self.uavs_config_list[self.ID-1].top_pub_addr['set_velocity'], TwistStamped, queue_size=1)
        self.path_pub = rospy.Publisher('/pydag/uav_{}/goal_path'.format(self.ID), Path, queue_size = 1)
        self.path_pub_smooth = rospy.Publisher('/pydag/uav_{}/goal_path_smooth'.format(self.ID), Path, queue_size = 1)

        # # Creation of a list with every UAVs' home frame
        # self.uav_frame_list = []
        # for n_uav in np.arange(self.N_uav):
        #     self.uav_frame_list.append(rospy.get_param( 'uav_{}_home'.format(n_uav+1)))

        # Assiignation of an ICAO number for every UAV. Used if ADSB communications are active
        self.critical_event = 'nothing'       # Initially there has not been collisions
        self.die_command = False        # Flag later activate from Ground Station to finish simulation

        self.brain.uavs_data_list = self.uavs_data_list

        # Wait time to let Gazebo's Real Time recover from model spawns
        if self.uav_models[self.ID-1] != "plane":
            while not rospy.is_shutdown() and self.uavs_data_list[self.ID-1].ual_state == ual_state_msg.UNINITIALIZED:
                time.sleep(0.1)
                # print(self.uavs_data_list[self.ID-1].ual_state)
        # time.sleep(10 * self.N_uav)
        # time.sleep((self.ID-1) * 8)
        if self.uav_models[self.ID-1] != "plane":
            self.GroundStationListener()        # Start listening

        print "UAV",ID,"ready and listening"

        uav_sm = UAV_Manager_SM(self)     # Create Ground Station State Machine
        outcome = uav_sm.uav_sm.execute()     # Execute State Machine

        # rospy.spin()

        return

    #### Listener functions ####

    # Function to create and the listeners ard serers
    def GroundStationListener(self):

        # Service to listen to the Ground Station if demands termination
        rospy.Service('/pydag/GS_UAV_{}/die_command'.format(self.ID), DieCommand, self.handle_die)

    # Function to accomplish end of this node
    def handle_die(self,req):
        # self.die_command = True
        rospy.signal_shutdown("end of experiment")      # End this node

        return True

    #### Publisher functions ###
    # Funtion to publish TFs of the actual goal
    def GoalStaticBroadcaster(self):
        broadcaster = tf2_ros.StaticTransformBroadcaster()      # Creation of the message to broadcast
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        # Fulfill the message
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "uav_{}_goal".format(self.ID)

        # Copy goal position to translation field
        static_transformStamped.transform.translation = self.goal_WP_pose.position

        # Rotation of the goal is always 0,0,0 and needs to be transformed to quaternions.
        # In the future more complex goals well be dealed so rotation will be copied here
        quat = tf.transformations.quaternion_from_euler(0,0,0)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)      # Broadcast the created message

    ## Functions to deal with any UAL server.
    # In the future, one utility should be used to replicate code for wait and send message

    ### Commander functions ###

    # Function to deal with UAL server Go To Way Point
    def GoToWPCommand(self,blocking,goal_WP_pose):

        h = std_msgs.msg.Header()
        # self.pose_pub.publish(PoseStamped(h,goal_WP_pose),blocking)
        rospy.wait_for_service('/uav_{}/ual/go_to_waypoint'.format(self.ID))
        try:
            ual_set_velocity = rospy.ServiceProxy('/uav_{}/ual/go_to_waypoint'.format(self.ID), GoToWaypoint)
            ual_set_velocity(PoseStamped(h,goal_WP_pose),True)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in go_to_waypoint"

        while not rospy.is_shutdown() and self.DistanceToGoal() > 0.2:
            print(self.DistanceToGoal())
            time.sleep(0.1)
        time.sleep(0.5)
        return

    # Function to deal with UAL server Set Velocity
    def SetVelocityCommand(self,hover):
        # Check if hover deactivated
        if hover== False:

            # Ask the Brain to decide the velocity
            self.new_velocity_twist = self.brain.Guidance(self.uavs_config_list[self.ID-1].max_speed)

            time_condition = time.time() - self.last_saved_time     # Control the elapsed time from last save

            main_role = self.role.split("_")[0]     # Parse the main role of the drone

            # Control if data must be stored depending on role, time elapsed from last save and actual state
            if ((main_role == "path" and self.state.split(" ")[0] == "to")
                or main_role == "uav_ad" or main_role == "uav_ap"
                ) and time_condition >= 1 and self.save_flag:
                # self.SaveData()     # Function to save the data of the actual instant to the frame of the global simulation
                self.last_saved_time == time.time()     # Restart time since last save

        # Check if hover activated
        elif hover == True:

            # Ask the Brain to give a zero velocity
            self.new_velocity_twist = self.brain.Hover()
            pass

        h = std_msgs.msg.Header()       # Create an empty header

        # self.finish = time.time()       # Save time elapsed since last calculation of the velocity. To control time of computation
        # print self.finish - self.start
        # self.start = time.time()      # Restart time elapsed since last calculation of the velocity. In the future should enter into the evaluatiors

        self.velocity_pub.publish(TwistStamped(h,self.new_velocity_twist))

        # time.sleep(1)

        return

    # Function to deal with UAL server Take Off
    def TakeOffCommand(self,heigth, blocking):

        if self.uav_models[self.ID-1] == "plane":
            self.TakeOffCommand_FW(heigth, blocking)
            return

        time.sleep(3)
        rospy.wait_for_service(self.uavs_config_list[self.ID-1].ser_cli_addr['take_off'])
        try:
            ual_set_velocity = rospy.ServiceProxy(self.uavs_config_list[self.ID-1].ser_cli_addr['take_off'], TakeOff)
            ual_set_velocity(heigth,blocking)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in take_off"


        while not rospy.is_shutdown() and self.uavs_data_list[self.ID-1].ual_state != ual_state_msg.FLYING_AUTO:
            time.sleep(0.1)

    def TakeOffCommand_FW(self,heigth, blocking):

        takeoff_pose = copy.deepcopy(self.uavs_data_list[self.ID-1].position.pose)
        takeoff_pose.position = Point(takeoff_pose.position.x,takeoff_pose.position.y,takeoff_pose.position.z + heigth)
        self.fw_qgc.MoveCommand("takeoff",[takeoff_pose],0,{"type":"by_angle","height": 10.0,"distance":200})

    # Function to deal with UAL server Land
    def LandCommand(self,blocking):

        if self.uav_models[self.ID-1] == "plane":
            self.LandCommand_FW()
            return

        rospy.wait_for_service(self.uavs_config_list[self.ID-1].ser_cli_addr['land'])
        try:
            ual_land = rospy.ServiceProxy(self.uavs_config_list[self.ID-1].ser_cli_addr['land'], Land)
            ual_land(blocking)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in land service"

        while not rospy.is_shutdown() and self.uavs_data_list[self.ID-1].ual_state != (ual_state_msg.LANDED_DISARMED or ual_state_msg.LANDED_ARMED):
            time.sleep(0.1)

    def LandCommand_FW(self, land_point = []):

        # actual_position = self.uavs_data_list[self.ID-1].position.pose.position
        takeoff_pose = copy.deepcopy(self.uavs_data_list[self.ID-1].position.pose)
        land_pose = Pose(Point(0,0,0),Quaternion(0,0,0,1))
        self.fw_qgc.MoveCommand("land",[land_pose],0,{"loiter_to_alt":{"type":"by_angle","height": 10.0,"distance":200}})

    # Function to deal with UAL server Set Home
    def SetHomeCommand(self):
        rospy.wait_for_service(self.uavs_config_list[self.ID-1].ser_cli_addr['set_home'])
        try:
            ual_set_velocity = rospy.ServiceProxy(self.uavs_config_list[self.ID-1].ser_cli_addr['set_home'], TakeOff)
            ual_set_velocity()
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in set_home"

        # Function to send termination instruction to each UAV
    def SmoothPath(self,raw_path):
        rospy.wait_for_service('/uav_path_manager/generator/generate_path')
        try:
            instruction_command = rospy.ServiceProxy('/uav_path_manager/generator/generate_path', GeneratePath)
            response = instruction_command(raw_path,Int8(self.smooth_path_mode))
            return response.generated_path

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in simulation_termination"

    def sendSmoothPath(self,smooth_path,cruising_speed,look_ahead):

        msg = GetGeneratedPathRequest()
        msg.generated_path = smooth_path
        msg.cruising_speed.data = cruising_speed
        msg.look_ahead.data = look_ahead

        rospy.wait_for_service('/uav_path_manager/generator/generate_path')
        try:
            instruction_command = rospy.ServiceProxy('/uav_path_manager/follower/uav_{0}/generated_path'.format(self.ID), GetGeneratedPath)
            response = instruction_command(msg)

            # if response != True:
            #     print("Unable to contact to uav_path_manager.")

            return response

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in simulation_termination"


    ### Role functions ###

    # Function to model the target of role follow static waypoint path
    def PathFollower(self,dynamic):

        if self.uav_models[self.ID-1] == "plane":
            self.PathFollower_FW()
            return

        self.goal_path = Path()
        self.goal_path.header.stamp = rospy.Time.now()
        self.goal_path.header.frame_id = "map"
        self.changed_state = False
        posestamped = PoseStamped()
        posestamped.header.stamp = rospy.Time.now()
        posestamped.header.frame_id = "map"
        posestamped.pose = copy.deepcopy(self.uavs_data_list[self.ID-1].position.pose)
        self.goal_path.poses = [posestamped]
        for pose in copy.deepcopy(self.goal_path_poses_list):
            posestamped = PoseStamped()
            posestamped.header.stamp = rospy.Time.now()
            posestamped.header.frame_id = "map"
            posestamped.pose = pose
            self.goal_path.poses.append(posestamped)

        self.path_pub.publish(self.goal_path)

        self.brain.smooth_path_mode = self.smooth_path_mode
        self.uavs_data_list[self.ID-1].smooth_path_mode = self.smooth_path_mode
        if self.smooth_path_mode != 0:
            self.goal_path_smooth = self.SmoothPath(self.goal_path)
            self.path_pub_smooth.publish(self.goal_path_smooth)
            smooth_server_response = self.sendSmoothPath(self.goal_path_smooth,1.0,1.0)

            # if smooth_server_response != True:
            #     return


        self.uavs_data_list[self.ID-1].own_path.poses = []

        # Control in every single component of the list
        for i in np.arange(len(self.goal_path_poses_list)):

            self.goal_WP_pose = self.goal_path_poses_list[i]        # Set actual goal from the path list
            self.goal["pose"] = self.goal_WP_pose       # Actualize goal variable for later storage

            self.GoalStaticBroadcaster()        # Broadcast TF of goal

            # Actualize state and send it to Ground Station
            self.state = "to WP {}".format(i+1)
            self.GSStateActualization()
            
            # Control distance to goal
            while not rospy.is_shutdown() and self.DistanceToGoal() > self.accepted_target_radio:

                if self.preemption_launched == False and self.critical_event != 'nothing':
                    self.preemption_launched = True
                    return self.critical_event

                if dynamic == "position":
                    
                    self.GoToWPCommand(False,self.goal["pose"])

                elif dynamic == "velocity":

                    self.SetVelocityCommand(False)      # If far, ask brain to give the correct velocity

                self.Evaluator()          # Evaluate the situation
                    
                time.sleep(0.2)

            self.SetVelocityCommand(True)       # If far, ask brain to give the correct velocity
            #self.goal_WP_pose = self.goal_path_poses_list[i]        # Actualize actual goal from the path list

            # Actualize state and send it to Ground Station
            self.state = "in WP {}".format(i+1)
            self.GSStateActualization()

            time.sleep(0.2)

            self.Evaluator()          # Evaluate the situation

        return 'succeeded'

    def PathFollower_FW(self):
        # goal_list = []
        # for i in np.arange(len(self.goal_path_poses_list)):
            # goal_WP_pose = self.goal_path_poses_list[i].position

            # goal_list.append([goal_WP_pose.x,goal_WP_pose.y,goal_WP_pose.z])

            # goal_list = [[200.0,0.0,10.0],[0.0,0.0,5.0],[-200.0,0.0,10.0],[0.0,0.0,5.0],[200.0,0.0,10.0]]

        self.fw_qgc.MoveCommand("pass",self.goal_path_poses_list,0)

    # Function to model the target of role UAV Follower AD
    def UAVFollowerAtDistance(self,target_ID,distance,action_time):

        if self.uav_models[self.ID-1] == "plane":
            self.PathFollower_FW()
            return

        action_start_time = rospy.Time.now()

        self.smooth_path_mode = 0
        self.brain.smooth_path_mode = self.smooth_path_mode

        self.queue_of_followers_at_distance = distance      # Save the distance requiered

        # Actualize the state and send it to Ground Station
        self.state = "following UAV {} at distance".format(target_ID)
        self.GSStateActualization()

        # Execute while Ground Station doesn't send another instruction
        while not rospy.is_shutdown() and (rospy.Time.now()-action_start_time) < rospy.Duration(action_time):

            if self.preemption_launched == False and self.critical_event != 'nothing':
                self.preemption_launched = True
                return self.critical_event

            # Create an independent copy of the position of the UAV to follow
            self.goal_WP_pose = copy.deepcopy(self.uavs_data_list[target_ID-1].position.pose)

            # If role is orca3, goal wp will be trickered moving it in veolcitywise
            if self.solver_algorithm == "orca3":

                tar_vel_lin = self.uavs_data_list[target_ID-1].velocity.twist.linear

                tar_pos = self.uavs_data_list[target_ID-1].position.pose.position
                tar_ori = self.uavs_data_list[target_ID-1].position.pose.orientation

                near_pos = np.asarray([tar_pos.x,tar_pos.y,tar_pos.z]) + \
                        np.asarray([tar_vel_lin.x,tar_vel_lin.y,tar_vel_lin.z])*1.5

                self.goal_WP_pose = Pose(Point(near_pos[0],near_pos[1],near_pos[2]),
                                        tar_ori)

            self.GoalStaticBroadcaster()        # Broadcast TF of goal. In future should not be static

            # Fulfil goal variable fields to be later stored
            self.goal["pose"] = self.goal_WP_pose
            self.goal["vel"] = self.uavs_data_list[target_ID-1].velocity.twist
            self.goal["dist"] = distance

            # Control distance to goal
            if self.DistanceToGoal() > distance:

                self.SetVelocityCommand(False)      # If far, ask brain to give the correct velocity
                
            else:
                self.SetVelocityCommand(True)       # If on goal, hover. In future should still be take care off collisions

            self.Evaluator()        # Evaluate the situation
            time.sleep(0.2)

        return 'succeeded'

    # Function to model the target of role UAV Follower AP
    def UAVFollowerAtPosition(self,target_ID,pos,action_time):

        self.queue_of_followers_ap_pos = pos        # Save the position requiered

        action_start_time = rospy.Time.now()

        self.smooth_path_mode = 0
        self.brain.smooth_path_mode = self.smooth_path_mode

        # Actualize the state and send it to Ground Station
        self.state = "following UAV {} at position".format(target_ID)
        self.GSStateActualization()

        # Execute while Ground Station doesn't send another instruction
        while not rospy.is_shutdown() and (rospy.Time.now()-action_start_time) < rospy.Duration(action_time):

            if self.preemption_launched == False and self.critical_event != 'nothing':
                self.preemption_launched = True
                return self.critical_event

            # Create an independent copy of the position of the UAV to follow
            self.goal_WP_pose = copy.deepcopy(self.uavs_data_list[target_ID-1].position.pose)

            # Add to that pose, the bias required
            self.goal_WP_pose.position.x += pos[0]
            self.goal_WP_pose.position.y += pos[1]
            self.goal_WP_pose.position.z += pos[2]

            # If role is orca3, goal wp will be trickered moving it in veolcitywise
            if self.solver_algorithm == "orca3":
                tar_vel_lin = self.uavs_data_list[target_ID-1].velocity.twist.linear

                tar_pos = self.goal_WP_pose.position
                tar_ori = self.uavs_data_list[target_ID-1].position.pose.orientation

                near_pos = np.asarray([tar_pos.x,tar_pos.y,tar_pos.z]) + \
                        np.asarray([tar_vel_lin.x,tar_vel_lin.y,tar_vel_lin.z])*1.5

                self.goal_WP_pose = Pose(Point(near_pos[0],near_pos[1],near_pos[2]),
                                        tar_ori)

            self.GoalStaticBroadcaster()        # Broadcast TF of goal. In future should not be static

            # Fulfil goal variable fields to be later stored. Those are no used, fix in the future.
            self.goal["pose"] = self.goal_WP_pose
            self.goal["vel"] = self.uavs_data_list[target_ID-1].velocity.twist

            # Control distance to goal
            if self.DistanceToGoal() > self.accepted_target_radio:
                self.SetVelocityCommand(False)      # If far, ask brain to give the correct velocity

            else:
                self.SetVelocityCommand(True)       # If on goal, hover

            self.Evaluator()         # Evaluate the situation

            time.sleep(0.2)

        return 'succeeded'

    # Function to model the target of role of single basic move
    def basic_move(self,move_type,dynamic,direction,value):

        outcome = 'succeded'
        # Select function used depending on move type
        if move_type == "take off":
            self.TakeOffCommand(value,True)
        elif move_type == "land":
            self.LandCommand(True)
        elif move_type == ("translation" or "rotation"):        # Future, rotation or turn? Choose onw

            # If translation, check direction
            change_matrix=[[0,0,0],[0,0,0]]
            if move_type == "translation":
                if direction == "up" or direction == "down":
                    change_matrix[0][2] = value*(1-2*(direction=="down"))
                elif direction == "left" or direction == "right":
                    change_matrix[0][1] = value*(1-2*(direction=="left"))
                elif direction == "forward" or direction == "backward":
                    change_matrix[0][0] = value*(1-2*(direction=="backward"))

            # If turn, check direction
            if move_type == "turn":
                change_matrix[1][3] = value*(1-2*(direction=="left"))

            # If position change, add direction, actualize goal and call PathFollower
            if dynamic == "position":
                goal_WP_pose = copy.deepcopy(self.uavs_data_list[self.ID-1].position.pose)

                goal_WP_pose.position.x += change_matrix[0][0]
                goal_WP_pose.position.y += change_matrix[0][1]
                goal_WP_pose.position.z += change_matrix[0][2]
                # self.goal_WP_pose = goal_WP_pose
                self.goal_path_poses_list = [goal_WP_pose]
                outcome = self.PathFollower("velocity")
        # for i in np.arange(len(self.goal_path_poses_list)):
                # self.GoToWPCommand(True,goal_WP_pose)

            # If velocity, set new velocity raw
            elif dynamic == "velocity":
                #### AHORA ES POR TOPICS
                rospy.wait_for_service('/uav_{}/ual/set_velocity'.format(self.ID))
                ual_set_velocity = rospy.ServiceProxy('/uav_{}/ual/set_velocity'.format(self.ID), SetVelocity)
                h = std_msgs.msg.Header()
                goal_WP_vel = TwistStamped()
                goal_WP_vel.linear.x += change_matrix[0][0]
                goal_WP_vel.linear.y += change_matrix[0][1]
                goal_WP_vel.linear.z += change_matrix[0][2]
                ual_set_velocity(TwistStamped(h,new_velocity_twist))

        return outcome

    ### Utility functions ###

    # Function to evaluate performance about different terms
    def Evaluator(self):

        # Thresholds of separation acceptance
        min_distance_uav = self.uavs_config_list[self.ID-1].security_radius*3
        min_distance_obs = self.uavs_config_list[self.ID-1].security_radius*3

        # Initialize collision as no occurred
        collision_uav = False
        collision_obs = False

        self.brain.NeighborSelector()
        # Turn to True the local collision flag if distance threshold has been raised for UAVs or obstacles
        if self.N_uav > 1:
            collision_uav = [x for x in self.brain.uav_near_neighbors_sorted_distances if x <= min_distance_uav]

        if self.N_obs > 0:
            collision_obs = [x for x in self.brain.obs_near_neighbors_sorted_distances if x <= min_distance_obs]

        checks = self.CheckFloatsArrayIntoBoundary([self.uavs_data_list[self.ID-1].position.pose.position.x,
                                                    self.uavs_data_list[self.ID-1].position.pose.position.y,
                                                    self.uavs_data_list[self.ID-1].position.pose.position.z],
                                                    self.world_boundaries)

        collision_world_boundaries = not(all(checks))

        # If confliction detected, raise the global collision flag for later storage
        if self.preemption_launched == False and (collision_uav or collision_obs or collision_world_boundaries):
            self.critical_event = "collision"

            if collision_uav:
                print self.ID,"COLLISIONED WITH ANOTHER UAV!!!!!!!!!!!!!!!!!!!!!!!!!!"
            elif collision_obs:
                print self.ID,"COLLISIONED WITH AN OBSTACLE!!!!!!!!!!!!!!!!!!!!!!!!!!"
            elif collision_world_boundaries:
                print self.ID,"OUT OF WORLD BOUNDARIES!!!!!!!!!!!!!!!!!!!!!!!!!!"

            self.GSStateActualization()

        elif self.preemption_launched == False and  self.uavs_data_list[self.ID-1].battery_percentage < 0.2:
            self.critical_event = "low_battery"

            print(self.ID,"LOW BATTERY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self.GSStateActualization()

        elif self.preemption_launched == False and  self.uavs_data_list[self.ID-1].GS_notification == 'GS_critical_event':
            print(self.ID, "GROUND STATION CRITICAL EVENT!!!!!!!!!!!!!!!!!!")
            self.critical_event = self.uavs_data_list[self.ID-1].GS_notification

            self.GSStateActualization()


        # Check if time to target must be reset and save it anyways
        if self.DistanceToGoal() < self.accepted_target_radio:
            self.start_time_to_target = time.time()
        elapsed_time_to_target = time.time() - self.start_time_to_target

        # Create an evaluation dictionary with al evaluated terms. In the future should have neighbour distances implemented
        self.evaluation = {'critical_event': self.critical_event,
                           'time_to_target': elapsed_time_to_target,
                           'distance_to_target': self.distance_to_goal}

    # Function to store every instant information in a unique dictionary
    def SaveData(self):

        # Create a single frame for every instant information storage.
        # It will be initialized with information of all UASs seen from this one and the applied velocity
        single_frame = {"main_uav" : [{"Pose" : self.parse_4CSV(self.uavs_data_list[self.ID-1].position.pose,"pose"),
                                      "Twist" : self.parse_4CSV(self.uavs_data_list[self.ID-1].velocity.twist,"twist")}],
                        "selected_velocity" : [self.parse_4CSV(self.new_velocity_twist,"twist")]}

        uav_near_neighbors_sorted, obs_near_neighbors_sorted = self.brain.uav_near_neighbors_sorted, self.brain.obs_near_neighbors_sorted

        if self.N_uav > 1:
            uavs_data_list = []
            for uav_neighbor in uav_near_neighbors_sorted:
                uavs_data_list.append({"Pose" : self.parse_4CSV(self.uavs_data_list[uav_neighbor].position_rel2main,"pose"),
                                  "Twist" : self.parse_4CSV(self.uavs_data_list[uav_neighbor].velocity.twist,"twist")})
            single_frame["uavs_neigh"] = [uavs_data_list]

        if self.N_obs > 0:
            obs_list = []
            for obs_neighbor in obs_near_neighbors_sorted:
                obs_list.append(self.parse_4CSV(self.uavs_data_list[self.ID-1].obs_poses_rel2main[obs_neighbor],"pose"))
            single_frame["obs_neigh"] = [obs_list]

        main_role = self.role.split("_")[0]     # Parse of main role

        # Addition of information into single frame depending on role.
        # In the future, this will be defined in a separate document and a for loop will add each member
        if main_role == "path":
            single_frame["goal"] = [[{"Pose" : self.parse_4CSV(self.SubstractPoses(self.goal_WP_pose,self.uavs_data_list[self.ID-1].position.pose),"pose")}]]
            single_frame["goal"][0][0]["Twist"] = [[0,0,0],[0,0,0]]

        elif main_role == "uav_ad":
            single_frame["goal"] = [[self.parse_4CSV([self.uavs_data_list[self.ID - 2]],"uavs_data_list")[0]]]
            single_frame["goal"][0][0]["distance"] = self.queue_of_followers_at_distance

        elif main_role == "uav_ap":
            single_frame["goal"] = [[self.parse_4CSV([self.uavs_data_list[self.ID - 2]],"uavs_data_list")[0]]]
            single_frame["goal"][0][0]["Pose"][0] = list(np.array(single_frame["goal"][0][0]["Pose"][0]) + np.array(self.queue_of_followers_ap_pos))

        single_frame["role"] = [self.role]

        # single_frame["evaluation"] = self.evaluation      # Addition of evaluation dictionary

        # Add the just contructed frame to a global one with every instant
        if self.global_data_frame.empty:
            self.global_data_frame = pd.DataFrame(single_frame)
        else:
            self.global_data_frame = self.global_data_frame.append(pd.DataFrame(single_frame),ignore_index=False)

        # If depth camera is in use, add its info to its single frame
        if self.depth_camera_use == True:
            single_frame_depth = self.uavs_data_list[self.ID-1].image_depth

            # If depth camera is in use, add its single frame to its global frame
            if self.global_frame_depth == []:
                self.global_frame_depth = [single_frame_depth]
            else:
                self.global_frame_depth.append(single_frame_depth)

    # Function to parse ROS objects into list and dictionaries that can be more easily stored in a csv
    def parse_4CSV(self,data,data_type):

        # For pose object, parse by position and orientation and each by member
        if data_type == "pose":
            position = data.position
            orientation = data.orientation

            parsed = [[position.x,position.y,position.z],[orientation.x,orientation.y,orientation.z,orientation.w]]

        # For twist object, parse by linear and anuglar and each by member
        elif data_type == "twist":
            linear = data.linear
            angular = data.angular
            parsed = [[linear.x,linear.y,linear.z],[angular.x,angular.y,angular.z]]

        elif data_type == "obs_pose":
            main_position = self.uavs_data_list[self.ID-1].position
            main_orientation = self.uavs_data_list[self.ID-1].orientation
            main_parsed = [[position.x,position.y,position.z],[orientation.x,orientation.y,orientation.z,orientation.w]]

            parsed = [list(np.array(data)-np.array(main_parsed[0])),main_parsed[1]] ### AÑADIR ORIENTACION DE OBSTACULOS

        # For a list of UAVs, for every UAV, separate into pose and twist and recursively parse both
        elif data_type == "uavs_data_list":
            parsed = []
            for i in range(len(data)):
                position = data[i].position.pose
                velocity = data[i].velocity.twist
                dicc = {"Pose":self.parse_4CSV(self.SubstractPoses(position,self.uavs_data_list[self.ID-1].position.pose),"pose"),
                        "Twist":self.parse_4CSV(velocity,"twist")}
                parsed.append(dicc)

        return parsed

    # Function to create and store UAV information of the whole simulation
    def StoreData(self):
        print "saving uav",self.ID,"mission data"

        first_folder_path = "{0}/Data_Storage/Simulations/{1}/{2}/{3}/{4}/Nuav{5}_Nobs{6}"\
                                 .format(self.home_path,self.world_name,self.subworld_name,
                                 self.mission_name,self.submission_name,self.N_uav,self.N_obs)

        third_folder_path = first_folder_path + "/dataset_{0}/simulation_{1}".format(self.n_dataset,self.n_simulation)

        csv_file_path = third_folder_path + '/uav_{0}.csv'.format(self.ID)
        self.global_data_frame.to_csv(csv_file_path, sep=',', mode='a') #,low_memory=False,     # Dump the global frame into

        # Dump the global depth camera frame into
        if self.depth_camera_use == True:
            with open(folder_path + "/depth_camera_{}.pickle".format(self.ID), 'wb') as f:
                pickle.dump(self.global_frame_depth, f, pickle.HIGHEST_PROTOCOL)

    # Function to inform Ground Station about actual UAV's state
    def GSStateActualization(self):
        self.uavs_data_list[self.ID-1].changed_state = True       # Actualization of the state to the main UAV object

        rospy.wait_for_service('/pydag/GS/state_actualization')
        try:
            # print "path for uav {} command".format(ID)
            GS_state_actualization = rospy.ServiceProxy('/pydag/GS/state_actualization', StateActualization)
            GS_state_actualization(self.ID, self.state, self.critical_event)
            return
            
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            print "error in state_actualization"

    # Function to calculate the distance from actual position to goal position
    def DistanceToGoal(self):
        self.distance_to_goal = math.sqrt((self.uavs_data_list[self.ID-1].position.pose.position.x-self.goal_WP_pose.position.x)**2+(self.uavs_data_list[self.ID-1].position.pose.position.y-self.goal_WP_pose.position.y)**2+(self.uavs_data_list[self.ID-1].position.pose.position.z-self.goal_WP_pose.position.z)**2)
        return self.distance_to_goal

    # Function to calculate the distance between two poses
    def DistanceBetweenPoses(self,pose_1,pose_2):
        Distance = math.sqrt((pose_1.position.x-pose_2.position.x)**2+(pose_1.position.y-pose_2.position.y)**2+(pose_1.position.z-pose_2.position.z)**2)
        return Distance

    def SubstractPoses(self,pose_1,pose_2):
        position_1, position_2, orientation_1, orientation_2 = pose_1.position, pose_2.position, pose_1.orientation, pose_2.orientation
        difference = Pose(Point(position_1.x-position_2.x,position_1.y-position_2.y,position_1.z-position_2.z),
                          Quaternion(orientation_1.x-orientation_2.x,orientation_1.y-orientation_2.y,orientation_1.z-orientation_2.z,orientation_1.w-orientation_2.w))
        return difference

    # Function to calculate the distance from actual position to an obstacle position
    def DistanceToObs(self,pose_1,vector):
        Distance = math.sqrt((pose_1.position.x-vector[0])**2+(pose_1.position.y-vector[1])**2+(pose_1.position.z-vector[2])**2)
        return Distance

    # Function to calculate the Velocity Module of a Twist
    def VelocityModule(self,twist):
        Module = np.sqrt(twist.linear.x**2+twist.linear.y**2+twist.linear.z**2)
        return Module

    def CheckFloatsArrayIntoBoundary(self,number_array,boundaries):
        if len(number_array) == 1:
            number_array = [number_array]
            boundaries = [boundaries]

        checks = [False for i in range(len(number_array))]
        for i,num in enumerate(number_array):
            if num > boundaries[i][0] and num < boundaries[i][1]:
                checks[i] = True 

        return checks

    # Function to get Global ROS parameters
    def GettingWorldDefinition(self):
        self.world_definition = rospy.get_param('world_definition')
        self.mission_name = self.world_definition['mission']
        self.submission_name = self.world_definition['submission']
        self.world_name = self.world_definition['world']
        self.subworld_name = self.world_definition['subworld']
        self.n_simulation = self.world_definition['n_simulation']
        self.N_uav = self.world_definition['N_uav']
        self.N_obs = self.world_definition['N_obs']
        self.uav_models = self.world_definition['uav_models']
        self.n_dataset = self.world_definition['n_dataset']
        self.solver_algorithm = self.world_definition['solver_algorithm']
        self.obs_pose_list = self.world_definition['obs_pose_list']
        self.depth_camera_use = self.world_definition['depth_camera_use']
        self.smach_view = self.world_definition['smach_view']
        self.save_flag = self.world_definition['save_flag']
        self.role = self.world_definition['roles_list'][self.ID-1]
        self.world_boundaries = self.world_definition['world_boundaries']


def main():                #### No estoy seguro de toda esta estructura
    parser = argparse.ArgumentParser(description='Spawn robot in Gazebo for SITL')
    parser.add_argument('-ID', type=str, default="1", help='')
    args, unknown = parser.parse_known_args()
    UAV_Manager(args.ID)

    return

if __name__ == '__main__':
    main()

