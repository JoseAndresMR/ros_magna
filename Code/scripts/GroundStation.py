#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 21 2018

@author: josmilrom
"""

import sys, os
import rospy, tf, tf2_ros
import std_msgs.msg
import time
import math
import numpy as np
import roslaunch
import csv
import rosbag
import subprocess
import signal
import xml.etree.ElementTree
from std_msgs.msg import Int32, String
from uav_abstraction_layer.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from gazebo_msgs.srv import DeleteModel
from tf2_msgs.msg import *

import utils
from Worlds import *
from pydag.srv import *
from GroundStation_SM import GroundStation_SM

class GroundStation(object):
    def __init__(self):
        # Global parameters inizialization.
        self.GettingWorldDefinition()

        ### Initializations

        self.setRoles()     # Decompose mision into rules
        self.simulation_succeed = True      # Initialize succeed flag of this single simulation

        # Start by landed the state of every UAV
        self.states_list = []
        for i in np.arange(self.N_uav):
            self.states_list.append("landed")

        # Start by not collisioned the the list within all UAVs
        self.critical_events_list = []
        for i in np.arange(self.N_uav):
            self.critical_events_list.append('nothing')

        rospy.init_node('ground_station', anonymous=True)     # Start node

        self.GSListener()     # Start subscribers

        self.gs_sm = GroundStation_SM(self)        # Create Ground Station State Machine
        outcome = self.gs_sm.gs_sm.execute()        # Execute State Machine

        self.Die()      # Once out of the State Machine, execute al commands of closig
        self.bag.close()        # Close the rosbag that stores the simulation


    #### Starter functions ####

    # Function to decompose each mission on a role for each UAV and set its param
    def setRoles(self):
        # Definition of decomposing dictionary
        mission_to_role_dicc = {"follow_paths_sbys":["path","path","path"],
                                "queue_of_followers_ad":["path","uav_ad","uav_ad"],
                                "queue_of_followers_ap":["path","uav_ap","uav_ap"],
                                "long_wait":["wait","wait","wait"],
                                "inspector":["wait","wait","wait"],
                                "smooth_path":["path","path","wait"],
                                "safety":["path","path","wait"]}

        # Actualize own world definition with role list
        self.world_definition["roles_list"] = mission_to_role_dicc[self.mission][:self.N_uav]

        # Add tag to role if depth camera is used
        if self.depth_camera_use == True:
            for n_uav in range(self.N_uav):
                self.world_definition["roles_list"][n_uav] = self.world_definition["roles_list"][n_uav] \
                                                            + "_depth"
        rospy.set_param('world_definition',self.world_definition)       # Set the ROS param

    # Function to set UAV's ROSparameters. Launched by State Machine
    def SetUavSpawnFeatures(self,ID,model,position,yaw=0):
        # uav_frame = rospy.get_param( 'uav_{}_home'.format(ID))      # Read from ROS param the home position
        uav_frame = {}
        uav_frame['translation'] = position
        if model == "typhoon_h480":     # If typhoon, change yaw ????? CHANGE FOR ALL. Updated on UAL
            yaw = yaw + np.pi
        uav_frame['gz_initial_yaw'] =  yaw # radians
        uav_frame['model'] = model      # Actualize on received param info
        rospy.set_param('uav_{}_home'.format(ID), uav_frame)        # Set the modified ROS param

        rospy.set_param('uav_{}/ual/home_pose'.format(ID),position)
        rospy.set_param('uav_{}/ual/pose_frame_id'.format(ID),"map")
        time.sleep(1)

    # Function to spawn each new UAV (GAZEBO model, UAL server and dedicated Ground Station)
    def UAVSpawner(self,ID):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_path = "/home/{0}/catkin_ws/src/pydag/Code/launch/{1}_spawner_JA.launch".format(self.home_path,self.world_definition['px4_use'])

        et = xml.etree.ElementTree.parse(launch_path)
        root = et.getroot()

        root[7].attrib["ns"] = '$(arg ns_prefix){}'.format(ID+1)
        root[7][0].attrib["value"] = str(ID+1)
        root[8].attrib["ns"] = '$(arg ns_prefix){}'.format(ID+1)
        root[8][0].attrib["value"] = str(ID+1)
        root[9].attrib["name"] = 'uav_{}'.format(ID+1)
        root[9].attrib["args"] = '-ID={}'.format(ID+1)
        root[10].attrib["name"] = 'path_follower_node_{}'.format(ID+1)
        root[10][0].attrib["value"] = str(ID+1)

        if ID+1 == 1:
            root[11].attrib["if"] = "true"
        else:
            root[11].attrib["if"] = "false"

        et.write(launch_path)

        self.uav_spawner_launch = roslaunch.parent.ROSLaunchParent(uuid,[launch_path])
        self.uav_spawner_launch.start()

    # Function to create folders and file to store simulations data
    def CreatingSimulationDataStorage(self):
        # Redefinition of number of obstacles. Obstacles in world types 1,2/3,4 are defined differently.
        # This code fixes that divergence for later storing
        if self.world_type == 2:
            N_obs_mixed = int('{0}{1}{2}'.format(self.obs_tube[0],self.obs_tube[1],self.obs_tube[2]))
        else:
            N_obs_mixed = self.N_obs

        # Definition of root path
        first_folder_path = "/home/{4}/catkin_ws/src/pydag/Data_Storage/Simulations/{0}/{5}/type{1}_Nuav{2}_Nobs{3}"\
                            .format(self.mission,self.world_type,self.N_uav,N_obs_mixed,self.home_path,self.solver_algorithm)

        # Check existance of root path. Create in case
        if not os.path.exists(first_folder_path):
            os.makedirs(first_folder_path)

        # Check existance of dataset path. Create in case
        second_folder_path = first_folder_path + "/dataset_{}".format(self.n_dataset)
        if not os.path.exists(second_folder_path):
            os.makedirs(second_folder_path)

        # Check existance of simulation path. Create in case
        self.third_folder_path = second_folder_path + "/simulation_{}".format(self.n_simulation)
        if not os.path.exists(self.third_folder_path):
            os.makedirs(self.third_folder_path)

        # Create path for rosbag and create object to manage it
        bag_folder_path = "/home/{6}/catkin_ws/src/pydag/Data_Storage/Simulations/{0}/{7}/type{1}_Nuav{2}_Nobs{3}/dataset_{4}/simulation_{5}/tf_bag.bag"\
                                .format(self.mission,self.world_type,self.N_uav,N_obs_mixed,self.n_dataset,self.n_simulation,self.home_path,self.solver_algorithm)
        self.bag = rosbag.Bag(bag_folder_path, 'w')


    def MakePath(self,path_def):

        return [self.world.getFSPoseGlobal(path_def)]


    #### Finisher functions ####

    # Function to close active child processess
    def Die(self):
        self.SavingWorldDefinition()        # Update ROS params
        self.UAVKiller()        # Terminate UAVs nodes
        self.GazeboModelsKiller()       # Delete robot models from Gazebo
        # self.world.eraseAllObstacles()      # Delete obstacles from Gazebo
        self.SimulationTerminationCommand()     # Send to Master message of simulation ended
        rospy.signal_shutdown("end of experiment")      # Finish Ground Station process


    # Function to update ROS parameters about simulation performance and store them
    def SavingWorldDefinition(self):
        self.world_definition = rospy.get_param('world_definition')
        self.world_definition['criticals_event_list'] = self.critical_events_list
        self.world_definition['simulation_succeed'] = self.simulation_succeed
        rospy.set_param('world_definition',self.world_definition)
        file_path = self.third_folder_path + '/world_definition.csv'
        with open(file_path,'wb') as f:
            w = csv.writer(f)
            w.writerows(self.world_definition.items())

    # Function to close UAV Ground Station process.In the furure would be done by GS
    def UAVKiller(self):
        for n_uav in np.arange(self.N_uav):
            rospy.wait_for_service('/pydag/GS_UAV_{}/die_command'.format(n_uav+1))
            try:
                die_command = rospy.ServiceProxy('/pydag/GS_UAV_{}/die_command'.format(n_uav+1), DieCommand)
                die_command(True)
                time.sleep(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                print "error in die_command"
        return

    # Function to close GAZEBO process
    def GazeboModelsKiller(self):
        for n_uav in np.arange(self.N_uav):
            rospy.wait_for_service('gazebo/delete_model')
            try:
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                model_name = "{0}_{1}".format(self.uav_models[n_uav],n_uav+1)
                del_model_prox(model_name)
                time.sleep(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                print "error in delete_model uav"
        return

    # Function to send termination instruction to each UAV
    def SimulationTerminationCommand(self):
        rospy.wait_for_service('/pydag/GS/simulation_termination')
        try:
            instruction_command = rospy.ServiceProxy('/pydag/GS/simulation_termination', DieCommand)
            instruction_command(True)
            return

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in simulation_termination"

    #### listener functions ####

    # Function to start subscribing and offering
    def GSListener(self):

        # Start service for UAVs to actualize its state
        rospy.Service('/pydag/GS/state_actualization', StateActualization, self.handle_uav_status)

        # Start listening to TFs topics
        rospy.Subscriber('/tf_static', TFMessage, self.tf_static_callback)
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)

    # Function to update and local parameters of UAVs status
    def handle_uav_status(self,data):

        # Store UAV status into UAVs state list
        self.states_list[data.id-1] = data.state

        # Check collisions to update it UAVs collision list
        if data.critical_event != 'nothing':
            if self.critical_events_list[data.id-1] != data.critical_event:
                self.critical_events_list[data.id-1] == data.critical_event
            # Change succeed information if a collision has been reported
            self.simulation_succeed = False
            rospy.loginfo_throttle(0,"Simulation not succeded, UAV {} collisioned".format(data.id))

        print self.states_list
        return True

    # Functions to save every TF information inside ROS bag
    def tf_static_callback(self,data):
        try:
            self.bag.write('/tf_static', data)
        except:
            print "The bag file does not exist"

    def tf_callback(self,data):
        try:
            self.bag.write('/tf', data)
        except:
            print "The bag file does not exist"

    # Function to get Global ROS parameters
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

def main():
    GroundStation()

if __name__ == '__main__':
    main()