#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys
import os
import rospy
import std_msgs.msg
import time
import math
import numpy as np
import Brain
import pandas as pd
import argparse
import tf, tf2_ros
import signal
import copy
from six.moves import cPickle as pickle
from uav_abstraction_layer.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import Image

from Brain import *
from pydag.srv import *
from UAV import UAV
from Ground_Station_SM import Ground_Station_SM

from FwQgc import FwQgc
class Ground_Station(object):

    ### Initialiations ###
    def __init__(self,ID):
        self.ID = int(ID)

        self.GettingWorldDefinition()   # Global ROS parameters inizialization

        self.brain = Brain(self.ID,self.role)   # Creation of an utility to treat depth camera topic data

        # Local variables initialization
        self.global_data_frame = pd.DataFrame()     # Data frame where to store this UAV dynamical simulation data for future dump into a csv
        self.global_frame_depth = []        # Data frame where to store this UAV depth image simulation data for future dump into a csv

        self.distance_to_goal = 10000
        self.goal = {}      # Structure to store goal specifications
        self.start_time_to_target = time.time()     # Counter to check elapsed time to achieve a goal

        self.accepted_target_radio = 0.5        # In the future, received on the world definition

        self.start=time.time()      # Counter to check elapsed time in the calculation of velocity each sintant
        self.last_saved_time = 0        # Counter to check elapsed time to save based on a frequency
        # Publishers initialisation
        self.pose_pub = rospy.Publisher('/uav_{}/ual/go_to_waypoint'.format(self.ID), PoseStamped, queue_size=1)
        self.velocity_pub= rospy.Publisher('/uav_{}/ual/set_velocity'.format(self.ID), TwistStamped, queue_size=1)

        # # Creation of a list with every UAVs' home frame
        # self.uav_frame_list = []
        # for n_uav in np.arange(self.N_uav):
        #     self.uav_frame_list.append(rospy.get_param( 'uav_{}_home'.format(n_uav+1)))

        # Assiignation of an ICAO number for every UAV. Used if ADSB communications are active
        ICAO_IDs = {1: "40621D", 2:"40621D", 3: "40621D"}
        self.collision = False       # Initially there has not been collisions
        self.die_command = False        # Flag later activate from ANSP to finish simulation

        if self.uav_models[self.ID-1] != "plane":
            rospy.init_node('ground_station_{}'.format(self.ID), anonymous=True)        # Start the node
        else:
            self.fw_qgc = FwQgc(self.ID,ual_use = True)

        # Creation of a list within objects of UAV class. They deal with the information of every UAV in the simu.
        # Own ID is given for the object to know if is treating any other UAV or the one dedicated for this GroundStation
        self.uavs_list = []
        for n_uav in range(1,self.N_uav+1):
            self.uavs_list.append(UAV(n_uav,self.ID,ICAO_IDs[n_uav]))

        # Wait time to let Gazebo's Real Time recover from model spawns
        if self.uav_models[self.ID-1] != "plane":
            while self.uavs_list[self.ID-1].ual_state == 0:
                time.sleep(0.5)
                # print(self.uavs_list[self.ID-1].ual_state)
        # time.sleep(10 * self.N_uav)
        # time.sleep((self.ID-1) * 8)
        if self.uav_models[self.ID-1] != "plane":
            self.GroundStationListener()        # Start listening
        print "ground station",ID,"ready and listening"

        gs_sm = Ground_Station_SM(self)     # Create Ground Station State Machine
        outcome = gs_sm.gs_sm.execute()     # Execute State Machine

        # rospy.spin()

        return

    #### Listener functions ####

    # Function to create and the listeners ard serers
    def GroundStationListener(self):

        # Service to listen to the ANSP if demands termination
        rospy.Service('/pydag/ANSP_UAV_{}/die_command'.format(self.ID), DieCommand, self.handle_die)

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
        self.pose_pub.publish(PoseStamped(h,goal_WP_pose),blocking)
        while self.DistanceToGoal() > 0.2:
            time.sleep(0.1)
        time.sleep(1)
        return

    # Function to deal with UAL server Set Velocity
    def SetVelocityCommand(self,hover):

        # Check if hover deactivated
        if hover== False:

            # Ask the Brain to decide the velocity
            self.new_velocity_twist = self.brain.Guidance(self.uavs_list,self.goal)

            time_condition = time.time() - self.last_saved_time     # Control the elapsed time from last save

            main_role = self.role.split("_")[0]     # Parse the main role of the drone

            # Control if data must be stored depending on role, time elapsed from last save and actual state
            if ((main_role == "path" and self.state.split(" ")[0] == "to")
                or main_role == "uav_ad" or main_role == "uav_ap"
                ) and time_condition >= 1 and self.save_flag:
                self.SaveData()     # Function to save the data of the actual instant to the frame of the global simulation
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

        time.sleep(10)
        rospy.wait_for_service('/uav_{}/ual/take_off'.format(self.ID))
        try:
            ual_set_velocity = rospy.ServiceProxy('/uav_{}/ual/take_off'.format(self.ID), TakeOff)
            ual_set_velocity(heigth,blocking)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in take_off"

    def TakeOffCommand_FW(self,heigth, blocking):

        takeoff_pose = copy.deepcopy(self.uavs_list[self.ID-1].position.pose)
        takeoff_pose.position = Point(takeoff_pose.position.x,takeoff_pose.position.y,takeoff_pose.position.z + heigth)
        self.fw_qgc.MoveCommand("takeoff",[takeoff_pose],0,{"type":"by_angle","height": 10.0,"distance":200})

    # Function to deal with UAL server Land
    def LandCommand(self,blocking):

        if self.uav_models[self.ID-1] == "plane":
            self.LandCommand_FW()
            return

        rospy.wait_for_service('/uav_{}/ual/go_to_waypoint'.format(self.ID))
        try:
            ual_land = rospy.ServiceProxy('/uav_{}/ual/land'.format(self.ID), Land)
            ual_land(blocking)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in go_to_waypoint"

    def LandCommand_FW(self, land_point = []):

        # actual_position = self.uavs_list[self.ID-1].position.pose.position
        takeoff_pose = copy.deepcopy(self.uavs_list[self.ID-1].position.pose)
        land_pose = Pose(Point(0,0,0),Quaternion(0,0,0,0))
        self.fw_qgc.MoveCommand("land",[land_pose],0,{"loiter_to_alt":{"type":"by_angle","height": 10.0,"distance":200}})

    # Function to deal with UAL server Set Home
    def SetHomeCommand(self):
        rospy.wait_for_service('/uav_{}/ual/set_home'.format(self.ID))
        try:
            ual_set_velocity = rospy.ServiceProxy('/uav_{}/ual/set_home'.format(self.ID), TakeOff)
            ual_set_velocity()
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in set_home"

    ### Role functions ###

    # Function to model the target of role follow static waypoint path
    def PathFollower(self):

        if self.uav_models[self.ID-1] == "plane":
            self.PathFollower_FW()
            return

        # Control in every single component of the list
        for i in np.arange(len(self.goal_path_poses_list)):

            self.goal_WP_pose = self.goal_path_poses_list[i]        # Set actual goal from the path list
            self.goal["pose"] = self.goal_WP_pose       # Actualize goal variable for later storage

            self.GoalStaticBroadcaster()        # Broadcast TF of goal

            # Actualize state and send it to ANSP
            self.state = "to WP {}".format(i+1)
            self.ANSPStateActualization()

            # Control distance to goal
            while self.DistanceToGoal() > self.accepted_target_radio:

                self.Evaluator()          # Evaluate the situation
                self.SetVelocityCommand(False)      # If far, ask brain to give the correct velocity
                time.sleep(0.2)

            self.SetVelocityCommand(True)       # If far, ask brain to give the correct velocity
            #self.goal_WP_pose = self.goal_path_poses_list[i]        # Actualize actual goal from the path list

            # Actualize state and send it to ANSP
            self.state = "in WP {}".format(i+1)
            self.ANSPStateActualization()

            time.sleep(0.2)

            self.Evaluator()          # Evaluate the situation

    def PathFollower_FW(self):
        # goal_list = []
        # for i in np.arange(len(self.goal_path_poses_list)):
            # goal_WP_pose = self.goal_path_poses_list[i].position

            # goal_list.append([goal_WP_pose.x,goal_WP_pose.y,goal_WP_pose.z])

            # goal_list = [[200.0,0.0,10.0],[0.0,0.0,5.0],[-200.0,0.0,10.0],[0.0,0.0,5.0],[200.0,0.0,10.0]]

        self.fw_qgc.MoveCommand("pass",self.goal_path_poses_list,0)

    # Function to model the target of role UAV Follower AD
    def UAVFollowerAtDistance(self,target_ID,distance):

        if self.uav_models[self.ID-1] == "plane":
            self.PathFollower_FW()
            return

        self.queue_of_followers_at_distance = distance      # Save the distance requiered

        # Actualize the state and send it to ANSP
        self.state = "following UAV {} at distance".format(target_ID)
        self.ANSPStateActualization()

        # Execute while ANSP doesn't send another instruction
        while self.uavs_list[1].preempt_flag == False:

            # Create an independent copy of the position of the UAV to follow
            self.goal_WP_pose = copy.deepcopy(self.uavs_list[target_ID-1].position.pose)

            # If role is orca3, goal wp will be trickered moving it in veolcitywise
            if self.solver_algorithm == "orca3":
                tar_vel_lin = self.uavs_list[target_ID-1].velocity.twist.linear

                tar_pos = self.uavs_list[target_ID-1].position.pose.position
                tar_ori = self.uavs_list[target_ID-1].position.pose.orientation

                near_pos = np.asarray([tar_pos.x,tar_pos.y,tar_pos.z]) + \
                        np.asarray([tar_vel_lin.x,tar_vel_lin.y,tar_vel_lin.z])*1.5

                self.goal_WP_pose = Pose(Point(near_pos[0],near_pos[1],near_pos[2]),
                                        tar_ori)

            self.GoalStaticBroadcaster()        # Broadcast TF of goal. In future should not be static

            # Fulfil goal variable fields to be later stored
            self.goal["pose"] = self.goal_WP_pose
            self.goal["vel"] = self.uavs_list[target_ID-1].velocity.twist
            self.goal["dist"] = distance

            # Control distance to goal
            if self.DistanceToGoal() > distance:
                self.SetVelocityCommand(False)      # If far, ask brain to give the correct velocity
                self.Evaluator()        # Evaluate the situation

            else:
                self.SetVelocityCommand(True)       # If on goal, hover. In future should still be take care off collisions

            time.sleep(0.2)

    # Function to model the target of role UAV Follower AP
    def UAVFollowerAtPosition(self,target_ID,pos):

        self.queue_of_followers_ap_pos = pos        # Save the position requiered

        # Actualize the state and send it to ANSP
        self.state = "following UAV {} at position".format(target_ID)
        self.ANSPStateActualization()

        # Execute while ANSP doesn't send another instruction
        while self.uavs_list[1].preempt_flag == False:

            # Create an independent copy of the position of the UAV to follow
            self.goal_WP_pose = copy.deepcopy(self.uavs_list[target_ID-1].position.pose)

            # Add to that pose, the bias required
            self.goal_WP_pose.position.x += pos[0]
            self.goal_WP_pose.position.y += pos[1]
            self.goal_WP_pose.position.z += pos[2]

            # If role is orca3, goal wp will be trickered moving it in veolcitywise
            if self.solver_algorithm == "orca3":
                tar_vel_lin = self.uavs_list[target_ID-1].velocity.twist.linear

                tar_pos = self.goal_WP_pose.position
                tar_ori = self.uavs_list[target_ID-1].position.pose.orientation

                near_pos = np.asarray([tar_pos.x,tar_pos.y,tar_pos.z]) + \
                        np.asarray([tar_vel_lin.x,tar_vel_lin.y,tar_vel_lin.z])*1.5

                self.goal_WP_pose = Pose(Point(near_pos[0],near_pos[1],near_pos[2]),
                                        tar_ori)

            self.GoalStaticBroadcaster()        # Broadcast TF of goal. In future should not be static

            # Fulfil goal variable fields to be later stored. Those are no used, fix in the future.
            self.goal["pose"] = self.goal_WP_pose
            self.goal["vel"] = self.uavs_list[target_ID-1].velocity.twist

            # Control distance to goal
            if self.DistanceToGoal() > self.accepted_target_radio:

                self.SetVelocityCommand(False)      # If far, ask brain to give the correct velocity
                self.Evaluator()        # Evaluate the situation

            else:
                self.SetVelocityCommand(True)       # If on goal, hover

            time.sleep(0.2)

    # Function to model the target of role of single basic move
    def basic_move(self,move_type,dynamic,direction,value):

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
                goal_WP_pose = copy.deepcopy(self.uavs_list[self.ID-1].position.pose)

                goal_WP_pose.position.x += change_matrix[0][0]
                goal_WP_pose.position.y += change_matrix[0][1]
                goal_WP_pose.position.z += change_matrix[0][2]
                # self.goal_WP_pose = goal_WP_pose
                self.goal_path_poses_list = [goal_WP_pose]
                self.PathFollower()
        # for i in np.arange(len(self.goal_path_poses_list)):
                # self.GoToWPCommand(True,goal_WP_pose)

            # If velocity, set new velocity raw
            elif dynamic == "velocity":
                rospy.wait_for_service('/uav_{}/ual/set_velocity'.format(self.ID))
                ual_set_velocity = rospy.ServiceProxy('/uav_{}/ual/set_velocity'.format(self.ID), SetVelocity)
                h = std_msgs.msg.Header()
                goal_WP_vel = TwistStamped()
                goal_WP_vel.linear.x += change_matrix[0][0]
                goal_WP_vel.linear.y += change_matrix[0][1]
                goal_WP_vel.linear.z += change_matrix[0][2]
                ual_set_velocity(TwistStamped(h,new_velocity_twist))

        return "completed"

    ### Utility functions ###

    # Function to evaluate performance about different terms
    def Evaluator(self):

        # Thresholds of separation acceptance
        min_distance_uav = 1
        min_distance_obs = 1

        # Initialize collision as no occurred
        collision_uav = False
        collision_obs = False

        # Fulfill a list within distances between itself and the rest of agents
        uav_distances_list = []
        for n_uav in np.arange(self.N_uav):
            if n_uav+1 != self.ID:
                uav_distances_list.append(self.DistanceBetweenPoses(self.uavs_list[self.ID-1].position.pose,self.uavs_list[n_uav].position.pose))

        # Fulfill a list within distances between itself and static obstacles
        obs_distances_list = []
        for n_obs in np.arange(self.N_obs):
            obs_distances_list.append(self.DistanceToObs(self.uavs_list[self.ID-1].position.pose,self.obs_pose_list[n_obs]))

        # Turn to True the local collision flag if distance threshold has been raised for UAVs or obstacles
        collision_uav = [x for x in uav_distances_list if x <= min_distance_uav]
        if self.N_obs>0:
            collision_obs = [x for x in obs_distances_list if x <= min_distance_obs]

        # If confliction detected, raise the global collision flag for later storage
        if collision_uav or collision_obs:
            self.collision = True
            print self.ID,"COLLISION!!!!!!!!!!!!!!!!!!!!!!!!!!"

        # Check if time to target must be reset and save it anyways
        if self.DistanceToGoal() < self.accepted_target_radio:
            self.start_time_to_target = time.time()
        elapsed_time_to_target = time.time() - self.start_time_to_target

        # Create an evaluation dictionary with al evaluated terms. In the future should have neighbour distances implemented
        self.evaluation = {'collision_uav': collision_uav,
                           'collision_obs': collision_obs,
                           'time_to_target': elapsed_time_to_target,
                           'distance_to_target': self.distance_to_goal}

    # Function to store every instant information in a unique dictionary
    def SaveData(self):

        # Create a single frame for every instant information storage.
        # It will be initialized with information of all UASs seen from this one and the applied velocity
        single_frame = {"selected_velocity" : [self.parse_4CSV(self.new_velocity_twist,"Twist")],
                        "uavs_list": [self.parse_4CSV(self.uavs_list,"uavs_list")]}

        main_role = self.role.split("_")[0]     # Parse of main role

        # Addition of information into single frame depending on role.
        # In the future, this will be defined in a separate document and a for loop will add each member
        if main_role == "path":
            single_frame["goal"] = [[{"Pose" : self.parse_4CSV(self.goal_WP_pose,"Pose")}]]
            single_frame["goal"][0][0]["Twist"] = [[0,0,0],[0,0,0]]

        elif main_role == "uav_ad":
            single_frame["goal"] = [[self.parse_4CSV([self.uavs_list[self.ID - 2]],"uavs_list")[0]]]
            single_frame["goal"][0][0]["distance"] = self.queue_of_followers_at_distance

        elif main_role == "uav_ap":
            single_frame["goal"] = [[self.parse_4CSV([self.uavs_list[self.ID - 2]],"uavs_list")[0]]]
            single_frame["goal"][0][0]["Pose"][0] = list(np.array(single_frame["goal"][0][0]["Pose"][0]) + np.array(self.queue_of_followers_ap_pos))

        # single_frame["evaluation"] = self.evaluation      # Addition of evaluation dictionary

        # Add the just contructed frame to a global one with every instant
        if self.global_data_frame.empty:
            self.global_data_frame = pd.DataFrame(single_frame)
        else:
            self.global_data_frame = self.global_data_frame.append(pd.DataFrame(single_frame),ignore_index=False)

        # If depth camera is in use, add its info to its single frame
        if self.depth_camera_use == True:
            single_frame_depth = self.uavs_list[self.ID-1].image_depth

            # If depth camera is in use, add its single frame to its global frame
            if self.global_frame_depth == []:
                self.global_frame_depth = [single_frame_depth]
            else:
                self.global_frame_depth.append(single_frame_depth)

    # Function to parse ROS objects into list and dictionaries that can be more easily stored in a csv
    def parse_4CSV(self,data,data_type):

        # For pose object, parse by position and orientation and each by member
        if data_type == "Pose":
            position = data.position
            orientation = data.orientation
            parsed = [[position.x,position.y,position.z],[orientation.x,orientation.y,orientation.z,orientation.w]]

        # For twist object, parse by linear and anuglar and each by member
        elif data_type == "Twist":
            linear = data.linear
            angular = data.angular
            parsed = [[linear.x,linear.y,linear.z],[angular.x,angular.y,angular.z]]

        # For a list of UAVs, for every UAV, separate into pose and twist and recursively parse both
        elif data_type == "uavs_list":
            parsed = []
            for i in range(len(data)):
                position = data[i].position.pose
                velocity = data[i].velocity.twist
                dicc = {"Pose":self.parse_4CSV(position,"Pose"),
                        "Twist":self.parse_4CSV(velocity,"Twist")}
                parsed.append(dicc)

        return parsed

    # Function to create and store UAV information of the whole simulation
    def StoreData(self):
        print "saving uav",self.ID,"mission data"

        # Redefinition of number of obstacles. Obstacles in world types 1,2/3,4 are defined differently.
        # This code fixes that divergence for later storing
        if self.world_type == 2:
            N_obs_mixed = int('{0}{1}{2}'.format(self.obs_tube[0],self.obs_tube[1],self.obs_tube[2]))
        else:
            N_obs_mixed = self.N_obs

        # Create folderpath to store. In the future, the first part of this path should be got by ROS param
        folder_path = "/home/{6}/catkin_ws/src/pydag/Data_Storage/Simulations/{0}/{7}/type{1}_Nuav{2}_Nobs{3}/dataset_{4}/simulation_{5}".format(self.mission,self.world_type,self.N_uav,N_obs_mixed,self.n_dataset,self.n_simulation,self.home_path,self.solver_algorithm)

        csv_file_path = folder_path + '/uav_{0}.csv'.format(self.ID)
        self.global_data_frame.to_csv(csv_file_path, sep=',', mode='a') #,low_memory=False,     # Dump the global frame into

        # Dump the global depth camera frame into
        if self.depth_camera_use == True:
            with open(folder_path + "/depth_camera_{}.pickle".format(self.ID), 'wb') as f:
                pickle.dump(self.global_frame_depth, f, pickle.HIGHEST_PROTOCOL)

    # Function to inform ANSP about actual UAV's state
    def ANSPStateActualization(self):
        self.uavs_list[self.ID-1].changed_state = True       # Actualization of the state to the main UAV object

        rospy.wait_for_service('/pydag/ANSP/state_actualization')
        try:
            # print "path for uav {} command".format(ID)
            ANSP_state_actualization = rospy.ServiceProxy(
                '/pydag/ANSP/state_actualization', StateActualization)
            ANSP_state_actualization(self.ID, self.state, self.collision)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            print "error in state_actualization"

    # Function to calculate the distance from actual position to goal position
    def DistanceToGoal(self):
        self.distance_to_goal = math.sqrt((self.uavs_list[self.ID-1].position.pose.position.x-self.goal_WP_pose.position.x)**2+(self.uavs_list[self.ID-1].position.pose.position.y-self.goal_WP_pose.position.y)**2+(self.uavs_list[self.ID-1].position.pose.position.z-self.goal_WP_pose.position.z)**2)
        return self.distance_to_goal

    # Function to calculate the distance between two poses
    def DistanceBetweenPoses(self,pose_1,pose_2):
        Distance = math.sqrt((pose_1.position.x-pose_2.position.x)**2+(pose_1.position.y-pose_2.position.y)**2+(pose_1.position.z-pose_2.position.z)**2)
        return Distance

    # Function to calculate the distance from actual position to an obstacle position
    def DistanceToObs(self,pose_1,vector):
        Distance = math.sqrt((pose_1.position.x-vector[0])**2+(pose_1.position.y-vector[1])**2+(pose_1.position.z-vector[2])**2)
        return Distance

    # Function to calculate the Velocity Module of a Twist
    def VelocityModule(self,twist):
        Module = np.sqrt(twist.linear.x**2+twist.linear.y**2+twist.linear.z**2)
        return Module


    # Function to get Global ROS parameters
    def GettingWorldDefinition(self):
        self.world_definition = rospy.get_param('world_definition')
        self.mission = self.world_definition['mission']
        self.world_type = self.world_definition['type']
        self.n_simulation = self.world_definition['n_simulation']
        self.N_uav = self.world_definition['N_uav']
        self.N_obs = self.world_definition['N_obs']
        self.obs_tube = self.world_definition['obs_tube']
        self.uav_models = self.world_definition['uav_models']
        self.n_dataset = self.world_definition['n_dataset']
        self.solver_algorithm = self.world_definition['solver_algorithm']
        self.obs_pose_list = self.world_definition['obs_pose_list']
        self.home_path = self.world_definition['home_path']
        self.depth_camera_use = self.world_definition['depth_camera_use']
        self.smach_view = self.world_definition['smach_view']
        self.save_flag = self.world_definition['save_flag']
        self.role = self.world_definition['roles_list'][self.ID-1]


def main():                #### No estoy seguro de toda esta estructura
    parser = argparse.ArgumentParser(description='Spawn robot in Gazebo for SITL')
    parser.add_argument('-ID', type=str, default="1", help='')
    args, unknown = parser.parse_known_args()
    Ground_Station(args.ID)

    return

if __name__ == '__main__':
    main()

