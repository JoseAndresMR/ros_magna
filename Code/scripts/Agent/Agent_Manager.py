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
from six.moves import cPickle as pickle
from uav_abstraction_layer.srv import *
from uav_abstraction_layer.msg import State as ual_state_msg
from uav_abstraction_layer.msg import WaypointSet, Param_float
from geometry_msgs.msg import *
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from std_msgs.msg import Int8

from Agent_NAI import *
from magna.srv import *
from Agent_Data import Agent_Data
from Agent_Config import Agent_Config
from Agent_Manager_SM import Agent_Manager_SM
from Various import serverClient
# from uav_path_manager.srv import GeneratePath,GetGeneratedPath,GetGeneratedPathRequest

# from FwQgc import FwQgc

class Agent_Manager(object):

    ### Initialiations ###
    def __init__(self,ID):

        self.ID = int(ID)

        self.GettingWorldDefinition()   # Global ROS parameters inizialization

        self.home_path = rospkg.RosPack().get_path('magna')[:-5]

        self.nai = Agent_NAI(self.ID)   # Creation of an utility to treat depth camera topic data
        self.standby_flag = False

        # Local variables initialization
        self.global_data_frame = pd.DataFrame()     # Data frame where to store this Agent dynamical simulation data for future dump into a csv
        self.global_frame_depth = []        # Data frame where to store this Agent depth image simulation data for future dump into a csv

        self.GS_notification = 'nothing'

        self.distance_to_goal = 10000
        self.goal = {}      # Structure to store goal specifications
        self.nai.goal = self.goal
        
        self.smooth_path_mode = 0
        self.start_time_to_target = time.time()     # Counter to check elapsed time to achieve a goal

        self.preemption_launched = False

        self.start=time.time()      # Counter to check elapsed time in the calculation of velocity each sintant
        self.last_saved_time = 0        # Counter to check elapsed time to save based on a frequency

        rospy.init_node('agent_{}'.format(self.ID), anonymous=True)        # Start the node
            
        # Creation of a list within objects of Agent class. They deal with the information of every Agent in the simu.
        # Own ID is given for the object to know if is treating any other Agent or the one dedicated for this GroundStation
        self.agents_data_list = []
        self.agents_config_list = []
        for n_agent in range(1,self.N_agents+1):
            self.agents_config_list.append(Agent_Config(n_agent))
            self.agents_data_list.append(Agent_Data(n_agent,self.ID,self.agents_config_list[n_agent-1]))

        self.accepted_target_radius = self.agents_config_list[self.ID-1].accepted_target_radius

        # Publishers initialisation
        self.pose_pub = rospy.Publisher(self.agents_config_list[self.ID-1].top_pub_addr['go_to_waypoint'], PoseStamped, queue_size=1)
        self.velocity_pub= rospy.Publisher(self.agents_config_list[self.ID-1].top_pub_addr['set_velocity'], TwistStamped, queue_size=1)
        self.path_pub = rospy.Publisher('/magna/agent_{}/goal_path'.format(self.ID), Path, queue_size = 1)
        self.path_pub_smooth = rospy.Publisher('/magna/agent_{}/goal_path_smooth'.format(self.ID), Path, queue_size = 1)

        # # Creation of a list with every Agents' home frame
        # self.agent_frame_list = []
        # for n_agent in np.arange(self.N_agents):
        #     self.agent_frame_list.append(rospy.get_param( 'agent_{}_home'.format(n_agent+1)))

        # Assiignation of an ICAO number for every Agent. Used if ADSB communications are active
        self.critical_event = 'nothing'       # Initially there has not been collisions
        self.die_command = False        # Flag later activate from Ground Station to finish simulation

        self.nai.agents_data_list = self.agents_data_list

        # Wait time to let Gazebo's Real Time recover from model spawns

        print("Waiting for Agent", ID, "life inform")
        while not rospy.is_shutdown() and self.agents_data_list[self.ID-1].ual_state == ual_state_msg.UNINITIALIZED:
            time.sleep(0.1)

        self.GroundStationListener()        # Start listening

        print "Agent",ID,"ready and listening"

        agent_sm = Agent_Manager_SM(self)     # Create Ground Station State Machine
        outcome = agent_sm.agent_sm.execute()     # Execute State Machine

        # rospy.spin()

        return

    #### Listener functions ####

    # Function to create and the listeners ard serers
    def GroundStationListener(self):

        # Service to listen to the Ground Station if demands termination
        rospy.Service('/magna/GS_Agent_{}/notification'.format(self.ID), InstructionCommand, self.handle_GS_notification_command)
        rospy.Service('/magna/GS_Agent_{}/algorithm_control'.format(self.ID), AlgorithmControl, self.handle_algorithm_control)

    # Function to accomplish end of this node
    def handle_die(self):
        
        rospy.signal_shutdown("end of experiment")      # End this node

        return True

    def handle_save_csv(self):

        self.StoreData()        # Tell the GS to store the data

        return True

    def handle_algorithm_control(self,req):
        params_dicc = {}

        for i,param in enumerate(req.params):
            params_dicc[param] = req.values[i]

        # Tell the GS to execute basic move role with parsed information
        output = self.nai.algorithm_control(req.name,req.action,params_dicc)
        return True

        # Function to deal with a preemption command from Ground Station
    def handle_GS_notification_command(self,data):

        if data.instruction == "critical_event":
            self.GS_notification = "critical_event"       # Set the variable

        elif data.instruction == "utm_new_flightplan":
            self.GS_notification = "utm_new_flightplan"

        elif data.instruction == "die":
            self.handle_die()

        elif data.instruction == "save_csv":
            self.handle_save_csv()

        elif data.instruction == "standby":
            self.standby_flag = True
            self.SetVelocityCommand(True)

        elif data.instruction == "resume":
            self.standby_flag = False

        return True

    #### Publisher functions ###
    # Funtion to publish TFs of the actual goal
    def GoalStaticBroadcaster(self):
        broadcaster = tf2_ros.StaticTransformBroadcaster()      # Creation of the message to broadcast
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        # Fulfill the message
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "agent_{}_goal".format(self.ID)

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

        request = GoToWaypointRequest()
        request.waypoint = PoseStamped(std_msgs.msg.Header(),goal_WP_pose)
        request.blocking = blocking

        response = serverClient(request, '/uav_{}/ual/go_to_waypoint'.format(self.ID), GoToWaypoint)

        while not rospy.is_shutdown() and self.DistanceToGoal() > self.accepted_target_radius:
            # print(self.DistanceToGoal())
            time.sleep(0.1)
        time.sleep(0.5)
        return

    # Function to deal with UAL server Set Velocity
    def SetVelocityCommand(self,hover):
        # Check if hover deactivated
        if hover== False:

            # Ask the NAI to decide the velocity
            self.new_velocity_twist = self.nai.Guidance(self.agents_config_list[self.ID-1].max_speed)

            time_condition = time.time() - self.last_saved_time     # Control the elapsed time from last save

            # Control if data must be stored depending on role, time elapsed from last save and actual state
            if ((self.role == "path" and self.state.split(" ")[0] == "to")
                or self.role == "agent_ad" or self.role == "agent_ap"
                ) and time_condition >= 1 and self.save_flag:
                self.SaveData()     # Function to save the data of the actual instant to the frame of the global simulation
                self.last_saved_time == time.time()     # Restart time since last save

        # Check if hover activated
        elif hover == True:

            # Ask the NAI to give a zero velocity
            self.new_velocity_twist = self.nai.Hover()

        h = std_msgs.msg.Header()       # Create an empty header

        # self.finish = time.time()       # Save time elapsed since last calculation of the velocity. To control time of computation
        # print self.finish - self.start
        # self.start = time.time()      # Restart time elapsed since last calculation of the velocity. In the future should enter into the evaluatiors

        latency_pub = rospy.Publisher('/magna/latency/{}'.format(self.ID), PoseStamped, queue_size=1)
        latency_msg = PoseStamped()
        latency_msg.pose.position.x = 3
        latency_pub.publish(latency_msg)

        self.velocity_pub.publish(TwistStamped(h,self.new_velocity_twist))

        # time.sleep(1)

        return

    # Function to deal with UAL server Take Off
    def TakeOffCommand(self,height, blocking):

        time.sleep(3)

        request = TakeOffRequest()

        request.height = height
        request.blocking = blocking

        response = serverClient(request, self.agents_config_list[self.ID-1].ser_cli_addr['take_off'], TakeOff)

        while not rospy.is_shutdown() and self.agents_data_list[self.ID-1].ual_state != ual_state_msg.FLYING_AUTO:
            time.sleep(0.1)

        # Function to inform Ground Station about actual Agent's state
        self.state = "inizializating"
        self.GSStateActualization()

        return "completed"


    # Function to deal with UAL server Land
    def LandCommand(self,blocking):

        response = serverClient(blocking, self.agents_config_list[self.ID-1].ser_cli_addr['land'], Land)

        while not rospy.is_shutdown() and not (self.agents_data_list[self.ID-1].ual_state == ual_state_msg.LANDED_DISARMED or self.agents_data_list[self.ID-1].ual_state == ual_state_msg.LANDED_ARMED):
            time.sleep(0.1)

        # Function to inform Ground Station about actual Agent's state
        self.state = "landed"
        self.GSStateActualization()

        return "completed"


    # Function to deal with UAL server Set Home
    def SetHomeCommand(self):

        response = serverClient(True, self.agents_config_list[self.ID-1].ser_cli_addr['set_home'], TakeOff)

    def SetMissionCommand(self,setmission_msg,blocking):

        time.sleep(10*(self.ID-1))

        rospy.wait_for_service(self.agents_config_list[self.ID-1].ser_cli_addr['set_mission'])
        try:
            ual_setmission = rospy.ServiceProxy(self.agents_config_list[self.ID-1].ser_cli_addr['set_mission'], SetMission)
            ual_setmission(setmission_msg,blocking)
            time.sleep(0.1)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in land service"
        

        # while not rospy.is_shutdown() and self.agents_data_list[self.ID-1].ual_state == (ual_state_msg.LANDED_DISARMED or ual_state_msg.LANDED_ARMED):
        #     time.sleep(0.1)

        return "completed"

        # Function to send termination instruction to each Agent
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

    def CreateMission(self,poses_list,include_takeoff,include_land):

        WaypointSetType = {
            "TAKEOFF_POSE" : 0,
            "TAKEOFF_AUX" : 1,
            "PASS" : 2,
            "LOITER_UNLIMITED" : 3,
            "LOITER_TURNS" : 4,
            "LOITER_TIME" : 5,
            "LOITER_HEIGHT" : 6,
            "LAND_POSE" : 7,
            "LAND_AUX" : 8    }

        mission_msg = []

        if include_takeoff:
            takeoff_set = WaypointSet()
            takeoff_set.type = WaypointSetType["TAKEOFF_AUX"]
            params = [Param_float("aux_distance",0.0),Param_float("aux_angle",0.0),Param_float("aux_height",5.0),
                        Param_float("minimum_pitch",0.0),Param_float("yaw_angle",0.0)]
            takeoff_set.params = params
            mission_msg.append(takeoff_set)

        pass_set = WaypointSet()
        pass_set.type = WaypointSetType["PASS"]
        params = [Param_float("acceptance_radius",0.0),Param_float("orbit_distance",0.0),Param_float("yaw_angle",0.0)]
        pass_set.params = params
        pass_set.posestamped_list = []
        for pose in poses_list:
            posestamped = PoseStamped()
            posestamped.pose = pose
            pass_set.posestamped_list.append(posestamped)

        mission_msg.append(pass_set)

        if include_land:
            pass

        self.SetMissionCommand(mission_msg,False)

        return "succeeded"

    # Function to model the target of role follow static waypoint path
    def PathFollower(self,dynamic,action_time = 0.0,on_mission = True,):

        latency_pub = rospy.Publisher('/magna/latency/{}'.format(self.ID), PoseStamped, queue_size=1)
        latency_msg = PoseStamped()
        latency_msg.pose.position.x = 2
        latency_pub.publish(latency_msg)

        self.preemption_launched = False
        if self.critical_event == "utm_new_flightplan":
            self.critical_event = "nothing"

        self.role = "path"
        self.nai.role = self.role

        self.goal_path = self.PoseslistToGlobalPath([copy.deepcopy(self.agents_data_list[self.ID-1].position.pose)]+copy.deepcopy(self.goal_path_poses_list))
        self.path_pub.publish(self.goal_path)
        self.changed_state = False

        self.nai.smooth_path_mode = self.smooth_path_mode
        self.agents_data_list[self.ID-1].smooth_path_mode = self.smooth_path_mode
        if self.smooth_path_mode != 0:
            self.goal_path_smooth = self.SmoothPath(self.goal_path)
            self.path_pub_smooth.publish(self.goal_path_smooth)
            smooth_server_response = self.sendSmoothPath(self.goal_path_smooth,1.0,1.0)

            # if smooth_server_response != True:
            #     return

        self.agents_data_list[self.ID-1].own_path.poses = []
        # Control in every single component of the list
        for i in np.arange(len(self.goal_path_poses_list)):

            self.goal_WP_pose = self.goal_path_poses_list[i]        # Set actual goal from the path list
            self.goal["pose"] = self.goal_WP_pose       # Actualize goal variable for later storage

            start_time = time.time()

            self.GoalStaticBroadcaster()        # Broadcast TF of goal

            # Actualize state and send it to Ground Station
            self.state = "to WP {}".format(i+1)
            self.GSStateActualization()

            gtwp_cmd_sent = False

            # Control distance to goal
            while not rospy.is_shutdown() and (self.DistanceToGoal() > self.accepted_target_radius) or (time.time()-start_time < action_time):

                if on_mission and self.standby_flag == True:
                    time.sleep(2)
                
                elif self.preemption_launched == False and self.critical_event != 'nothing':
                    self.preemption_launched = True
                    return self.critical_event

                else:
                    if dynamic == "position" and gtwp_cmd_sent == False:
                        self.GoToWPCommand(True,self.goal["pose"])
                        gtwp_cmd_sent = True

                    elif dynamic == "velocity":
                        
                        self.SetVelocityCommand(False)      # If far, ask nai to give the correct velocity

                    self.Evaluator()          # Evaluate the situation
                        
                    time.sleep(0.2)

            if self.agent_models[self.ID-1] != "plane":
                self.SetVelocityCommand(True)       # If far, ask nai to give the correct velocity

            #self.goal_WP_pose = self.goal_path_poses_list[i]        # Actualize actual goal from the path list

            # Actualize state and send it to Ground Station
            self.state = "in WP {}".format(i+1)
            self.GSStateActualization()

            time.sleep(0.2)

            self.Evaluator()          # Evaluate the situation

        return 'succeeded'

    # Function to model the target of role Agent Follower AD
    def AgentFollowerAtDistance(self,target_ID,distance,action_time, on_mission = True):

        self.role = "agent_ad"
        self.nai.role = self.role

        self.preemption_launched = False

        # Tell the GS the identity of its new target
        self.state = "following Agent {0}".format(target_ID)
        self.GSStateActualization()       # Function to inform Ground Station about actual Agent's state

        action_start_time = rospy.Time.now()

        self.smooth_path_mode = 0
        self.nai.smooth_path_mode = self.smooth_path_mode

        self.queue_of_followers_at_distance = distance      # Save the distance requiered

        # Actualize the state and send it to Ground Station
        self.state = "following Agent {} at distance".format(target_ID)
        self.GSStateActualization()

        # Execute while Ground Station doesn't send another instruction
        while not rospy.is_shutdown() and (rospy.Time.now()-action_start_time) < rospy.Duration(action_time):

            if on_mission and self.standby_flag == True:
                time.sleep(2)

            elif self.preemption_launched == False and self.critical_event != 'nothing':
                self.preemption_launched = True
                return self.critical_event

            else:
                # Create an independent copy of the position of the Agent to follow
                self.goal_WP_pose = copy.deepcopy(self.agents_data_list[target_ID-1].position.pose)

                # If role is orca3, goal wp will be trickered moving it in veolcitywise
                if self.nai.algorithms_dicc.keys()[0] == "orca3":

                    tar_vel_lin = self.agents_data_list[target_ID-1].velocity.twist.linear

                    tar_pos = self.agents_data_list[target_ID-1].position.pose.position
                    tar_ori = self.agents_data_list[target_ID-1].position.pose.orientation

                    near_pos = np.asarray([tar_pos.x,tar_pos.y,tar_pos.z]) + \
                            np.asarray([tar_vel_lin.x,tar_vel_lin.y,tar_vel_lin.z])*1.5

                    self.goal_WP_pose = Pose(Point(near_pos[0],near_pos[1],near_pos[2]),
                                            tar_ori)

                self.GoalStaticBroadcaster()        # Broadcast TF of goal. In future should not be static

                # Fulfil goal variable fields to be later stored
                self.goal["pose"] = self.goal_WP_pose
                self.goal["vel"] = self.agents_data_list[target_ID-1].velocity.twist
                self.goal["dist"] = distance

                # Control distance to goal
                if self.DistanceToGoal() > distance:

                    self.SetVelocityCommand(False)      # If far, ask nai to give the correct velocity
                    
                else:
                    self.SetVelocityCommand(True)       # If on goal, hover. In future should still be take care off collisions

                self.Evaluator()        # Evaluate the situation
                time.sleep(0.2)

        return 'succeeded'

    # Function to model the target of role Agent Follower AP
    def AgentFollowerAtPosition(self,target_ID,pos,action_time, on_mission = True):

        self.role = "agent_ap"
        self.nai.role = self.role

        self.preemption_launched = False

        # Tell the GS the identity of its new target
        self.state = "following Agent {0}".format(target_ID)
        self.GSStateActualization()       # Function to inform Ground Station about actual Agent's state


        self.queue_of_followers_ap_pos = pos        # Save the position requiered

        action_start_time = rospy.Time.now()

        self.smooth_path_mode = 0
        self.nai.smooth_path_mode = self.smooth_path_mode

        # Actualize the state and send it to Ground Station
        self.state = "following Agent {} at position".format(target_ID)
        self.GSStateActualization()

        # Execute while Ground Station doesn't send another instruction
        while not rospy.is_shutdown() and (rospy.Time.now()-action_start_time) < rospy.Duration(action_time):

            if on_mission and self.standby_flag == True:
                time.sleep(2)

            elif self.preemption_launched == False and self.critical_event != 'nothing':
                self.preemption_launched = True
                return self.critical_event
            else:
                # Create an independent copy of the position of the Agent to follow
                self.goal_WP_pose = copy.deepcopy(self.agents_data_list[target_ID-1].position.pose)

                # Add to that pose, the bias required
                self.goal_WP_pose.position.x += pos[0]
                self.goal_WP_pose.position.y += pos[1]
                self.goal_WP_pose.position.z += pos[2]

                # If role is orca3, goal wp will be trickered moving it in veolcitywise
                if self.nai.algorithms_dicc.keys()[0] == "orca3":
                    tar_vel_lin = self.agents_data_list[target_ID-1].velocity.twist.linear

                    tar_pos = self.goal_WP_pose.position
                    tar_ori = self.agents_data_list[target_ID-1].position.pose.orientation

                    near_pos = np.asarray([tar_pos.x,tar_pos.y,tar_pos.z]) + \
                            np.asarray([tar_vel_lin.x,tar_vel_lin.y,tar_vel_lin.z])*1.5

                    self.goal_WP_pose = Pose(Point(near_pos[0],near_pos[1],near_pos[2]),
                                            tar_ori)

                self.GoalStaticBroadcaster()        # Broadcast TF of goal. In future should not be static

                # Fulfil goal variable fields to be later stored. Those are no used, fix in the future.
                self.goal["pose"] = self.goal_WP_pose
                self.goal["vel"] = self.agents_data_list[target_ID-1].velocity.twist

                # Control distance to goal
                if self.DistanceToGoal() > self.accepted_target_radius:
                    self.SetVelocityCommand(False)      # If far, ask nai to give the correct velocity

                else:
                    self.SetVelocityCommand(True)       # If on goal, hover

                self.Evaluator()         # Evaluate the situation

                time.sleep(0.2)

        return 'succeeded'

    # Function to model the target of role of single basic move
    def basic_move(self,move_type,dynamic,direction,value,duration = 0):

        start_time = time.time()
        outcome = 'succeded'

        while not rospy.is_shutdown() and time.time() - start_time < duration:

            # Select function used depending on move type
            if move_type == "take off":
                self.TakeOffCommand(value,True)
            elif move_type == "land":
                self.LandCommand(True)
            elif move_type == "hover":
                self.SetVelocityCommand(True)
                
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
                    goal_WP_pose = copy.deepcopy(self.agents_data_list[self.ID-1].position.pose)

                    goal_WP_pose.position.x += change_matrix[0][0]
                    goal_WP_pose.position.y += change_matrix[0][1]
                    goal_WP_pose.position.z += change_matrix[0][2]
                    # self.goal_WP_pose = goal_WP_pose
                    self.goal_path_poses_list = [goal_WP_pose]
                    outcome = self.PathFollower("velocity",False)
            # for i in np.arange(len(self.goal_path_poses_list)):
                    # self.GoToWPCommand(True,goal_WP_pose)

                # If velocity, set new velocity raw
                elif dynamic == "velocity":
                    #### AHORA ES POR TOPICS

                    goal_vel = TwistStamped()
                    goal_vel.linear.x += change_matrix[0][0]
                    goal_vel.linear.y += change_matrix[0][1]
                    goal_Wgoal_velP_vel.linear.z += change_matrix[0][2]
                    self.velocity_pub.publish(goal_vel)

        return outcome

    ### Utility functions ###

    # Function to evaluate performance about different terms
    def Evaluator(self):

        # Thresholds of separation acceptance
        safety_distance = self.agents_config_list[self.ID-1].safety_radius*2

        # Initialize collision as no occurred
        collision_detected = False

        if True:
            self.nai.NeighborSelector(int(self.nai.algorithms_dicc["orca3"]["N_neighbors_aware"]))
            # Turn to True the local collision flag if distance threshold has been raised for Agents or obstacles

            if len(self.nai.near_neighbors_sorted["distances"]) > 0:
                if self.nai.near_neighbors_sorted["distances"][0] <= safety_distance:
                    collision_detected = True

            # if self.N_agents > 1:
            #     collision_agent = [x for x in self.nai.agent_near_neighbors_sorted_distances if x <= min_distance_agent]

            # if self.N_obs > 0:
            #     collision_obs = [x for x in self.nai.obs_near_neighbors_sorted_distances if x <= min_distance_obs]

            checks = self.CheckFloatsArrayIntoBoundary([self.agents_data_list[self.ID-1].position.pose.position.x,
                                                        self.agents_data_list[self.ID-1].position.pose.position.y,
                                                        self.agents_data_list[self.ID-1].position.pose.position.z],
                                                        self.world_boundaries)

            collision_world_boundaries = not(all(checks))

            # If confliction detected, raise the global collision flag for later storage
            if self.preemption_launched == False and (collision_detected or collision_world_boundaries):
                self.critical_event = "collision"

                if collision_detected:
                    print self.ID,"COLLISIONED!!!!!!!!!!!!!!!!!!!!!!!!!!"
                # elif collision_obs:
                #     print self.ID,"COLLISIONED WITH AN OBSTACLE!!!!!!!!!!!!!!!!!!!!!!!!!!"
                elif collision_world_boundaries:
                    print self.ID,"OUT OF WORLD BOUNDARIES!!!!!!!!!!!!!!!!!!!!!!!!!!"

        if self.preemption_launched == False and  self.agents_data_list[self.ID-1].battery_percentage < 0.2:
            self.critical_event = "low_battery"
            print(self.ID,"LOW BATTERY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self.GSStateActualization()

        elif self.preemption_launched == False and  ( self.GS_notification == 'GS_critical_event'):
            print(self.ID, "GROUND STATION CRITICAL EVENT!!!!!!!!!!!!!!!!!!")
            self.critical_event = 'GS_critical_event'
            self.GSStateActualization()

        elif self.preemption_launched == False and  ( self.GS_notification == 'utm_new_flightplan' ):
            print(self.ID, "UTM NEW FLIGHT PLAN!!!!!!!!!!!!!!!!!!")
            self.critical_event = 'utm_new_flightplan'
            self.GS_notification = "nothing"
            self.GSStateActualization()


        # Check if time to target must be reset and save it anyways
        if self.DistanceToGoal() < self.accepted_target_radius:
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
        single_frame = {"main_agent" : [{"Pose" : self.parse_4CSV(self.agents_data_list[self.ID-1].position.pose,"pose"),
                                      "Twist" : self.parse_4CSV(self.agents_data_list[self.ID-1].velocity.twist,"twist")}],
                        "selected_velocity" : [self.parse_4CSV(self.new_velocity_twist,"twist")]}
        # print(self.agents_data_list[self.ID-1].velocity.twist)
        # print(single_frame["main_agent"][0]["Twist"])

        single_frame["role"] = [self.role]
        single_frame["N_neigh_aware"] = [self.nai.algorithms_dicc["orca3"]["N_neighbors_aware"]]
        single_frame["algorithms_list"] = [self.nai.algorithms_dicc.keys()]

        if self.nai.algorithms_dicc["orca3"]["N_neighbors_aware"] > 0:
            neigh_data_list = []
            for n_neighbor in range(len(self.nai.near_neighbors_sorted["types"])):

                if self.nai.near_neighbors_sorted["types"][n_neighbor] == "agent":
                    n_agent = self.nai.near_neighbors_sorted["ids"][n_neighbor]

                    neigh_data_list.append({"Pose" : self.parse_4CSV(self.agents_data_list[n_agent].position_rel2main,"pose"),
                                    "Twist" : self.parse_4CSV(self.agents_data_list[n_agent].velocity.twist,"twist")})
                
                elif self.nai.near_neighbors_sorted["types"][n_neighbor] == "obs":
                    n_obs = self.nai.near_neighbors_sorted["ids"][n_neighbor]
                    try:
                        neigh_data_list.append({"Pose": self.parse_4CSV(self.agents_data_list[self.ID-1].obs_poses_rel2main[n_obs],"pose"),
                                    "Twist" : [[0,0,0],[0,0,0]]})
                    except:
                        print("FLAG error list index out of range. lengt of self.agents_data_list[self.ID-1].obs_poses_rel2main =", len(self.agents_data_list[self.ID-1].obs_poses_rel2main),self.ID-1, n_obs)
                        print("FLAG !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        return
                        # time.sleep(1000000)

            single_frame["neigh"] = [neigh_data_list]

        # Addition of information into single frame depending on role.
        # In the future, this will be defined in a separate document and a for loop will add each member
        if self.role == "path":
            # single_frame["goal"] = [[{"Pose" : self.SubstractPoses(self.goal_WP_pose,self.agents_data_list[self.ID-1].position.pose)}]]
            # single_frame["goal"][0][0]["Twist"] = self.nai.TwistFromArray([[0,0,0],[0,0,0]])
            single_frame["goal"] = [[{"Pose" : self.parse_4CSV(self.SubstractPoses(self.goal_WP_pose,self.agents_data_list[self.ID-1].position.pose),"pose")}]]
            single_frame["goal"][0][0]["Twist"] = [[0,0,0],[0,0,0]]

        elif self.role == "agent_ad":
            single_frame["goal"] = [[self.parse_4CSV([self.agents_data_list[self.ID - 2]],"agents_data_list")[0]]]
            single_frame["goal"][0][0]["distance"] = self.queue_of_followers_at_distance

        elif self.role == "agent_ap":
            single_frame["goal"] = [[self.parse_4CSV([self.agents_data_list[self.ID - 2]],"agents_data_list")[0]]]
            single_frame["goal"][0][0]["Pose"][0] = list(np.array(single_frame["goal"][0][0]["Pose"][0]) + np.array(self.queue_of_followers_ap_pos))

        # single_frame["evaluation"] = self.evaluation      # Addition of evaluation dictionary

        # Add the just contructed frame to a global one with every instant
        if self.global_data_frame.empty:
            self.global_data_frame = pd.DataFrame(single_frame)
        else:
            self.global_data_frame = self.global_data_frame.append(pd.DataFrame(single_frame),ignore_index=False)

        # If depth camera is in use, add its info to its single frameç
        if self.depth_camera_use == True:
            single_frame_depth = self.agents_data_list[self.ID-1].image_depth

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
            main_position = self.agents_data_list[self.ID-1].position
            main_orientation = self.agents_data_list[self.ID-1].orientation
            main_parsed = [[position.x,position.y,position.z],[orientation.x,orientation.y,orientation.z,orientation.w]]

            parsed = [list(np.array(data)-np.array(main_parsed[0])),main_parsed[1]] ### AÑADIR ORIENTACION DE OBSTACULOS

        # For a list of Agents, for every Agent, separate into pose and twist and recursively parse both
        elif data_type == "agents_data_list":
            parsed = []
            for i in range(len(data)):
                position = data[i].position.pose
                velocity = data[i].velocity.twist
                dicc = {"Pose":self.parse_4CSV(self.SubstractPoses(position,self.agents_data_list[self.ID-1].position.pose),"pose"),
                        "Twist":self.parse_4CSV(velocity,"twist")}
                parsed.append(dicc)

        return parsed

    # Function to create and store Agent information of the whole simulation
    def StoreData(self):
        print "saving agent",self.ID,"mission data"

        first_folder_path = "{0}/Data_Storage/Simulations/{1}/{2}/{3}/{4}"\
                                 .format(self.home_path,self.world_name,self.subworld_name,
                                 self.mission_name,self.submission_name)

        third_folder_path = first_folder_path + "/dataset_{0}/simulation_{1}".format(self.n_dataset,self.n_simulation)

        csv_file_path = third_folder_path + '/agent_{0}.csv'.format(self.ID)
        self.global_data_frame.to_csv(csv_file_path, sep=',', mode='a') #,low_memory=False,     # Dump the global frame into

        # Dump the global depth camera frame into
        if self.depth_camera_use == True:
            with open(third_folder_path + "/depth_camera_{}.pickle".format(self.ID), 'wb') as f:
                pickle.dump(self.global_frame_depth, f, pickle.HIGHEST_PROTOCOL)

    # Function to inform Ground Station about actual Agent's state
    def GSStateActualization(self):
        self.agents_data_list[self.ID-1].changed_state = True       # Actualization of the state to the main Agent object

        rospy.wait_for_service('/magna/GS/state_actualization')
        try:
            # print "path for agent {} command".format(ID)
            GS_state_actualization = rospy.ServiceProxy('/magna/GS/state_actualization', StateActualization)
            GS_state_actualization(self.ID, self.state, self.critical_event)
            return
            
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            print "error in state_actualization"

    # Function to calculate the distance from actual position to goal position
    def DistanceToGoal(self):
        self.distance_to_goal = math.sqrt((self.agents_data_list[self.ID-1].position.pose.position.x-self.goal_WP_pose.position.x)**2+(self.agents_data_list[self.ID-1].position.pose.position.y-self.goal_WP_pose.position.y)**2+(self.agents_data_list[self.ID-1].position.pose.position.z-self.goal_WP_pose.position.z)**2)
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
    
    def PoseslistToGlobalPath(self,waypoints_list):

        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        self.changed_state = False
        mean_vel = self.agents_config_list[self.ID-1].max_speed
        prior_pose = copy.deepcopy( waypoints_list[0] )
        prior_stamp = rospy.Time.now()

        path.poses = []
        for pose in waypoints_list:
            posestamped = PoseStamped()

            segment_distance = self.DistanceBetweenPoses(prior_pose,pose)
            posestamped.header.stamp = prior_stamp
            posestamped.header.stamp.secs += segment_distance / mean_vel
            prior_stamp =  copy.deepcopy(posestamped.header.stamp)
            prior_pose = copy.deepcopy(pose)

            posestamped.header.frame_id = "map"
            posestamped.pose = pose
            path.poses.append(posestamped)

        return path


    # Function to get Global ROS parameters
    def GettingWorldDefinition(self):
        self.hyperparameters = rospy.get_param('magna_hyperparameters')
        self.mission_name = self.hyperparameters['mission']
        self.submission_name = self.hyperparameters['submission']
        self.world_name = self.hyperparameters['world']
        self.subworld_name = self.hyperparameters['subworld']
        self.n_simulation = self.hyperparameters['n_simulation']
        self.N_agents = self.hyperparameters['N_agents']
        self.N_obs = self.hyperparameters['N_obs']
        self.agent_models = self.hyperparameters['agent_models']
        self.n_dataset = self.hyperparameters['n_dataset']
        self.obs_pose_list = self.hyperparameters['obs_pose_list']
        self.depth_camera_use = self.hyperparameters['depth_camera_use']
        self.smach_view = self.hyperparameters['smach_view']
        self.save_flag = self.hyperparameters['save_flag']
        self.world_boundaries = self.hyperparameters['world_boundaries']


def main():                #### No estoy seguro de toda esta estructura
    parser = argparse.ArgumentParser(description='Spawn robot in Gazebo for SITL')
    parser.add_argument('-ID', type=str, default="1", help='')
    args, unknown = parser.parse_known_args()
    Agent_Manager(args.ID)

    return

if __name__ == '__main__':
    main()

