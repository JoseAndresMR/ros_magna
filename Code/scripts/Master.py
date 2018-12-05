#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 21 2018

@author: josmilrom
"""

import sys, os
import rospy
import std_msgs.msg
import time
import math
import numpy as np
import roslaunch
import shutil
import csv
import subprocess
import signal
from uav_abstraction_layer.srv import *
from geometry_msgs.msg import *
from std_srvs.srv import *
from sensor_msgs.msg import *
import pandas as pd

from pydag.srv import *
import utils

class Master(object):
    def __init__(self):
        # self.processess_killer(2)
        # World paramenters initialization     follow_paths_sbys, queue_of_followers_ap, queue_of_followers_ad long_wait
        self.world_definition = {
        'mission'            :              "follow_paths_sbys",              # Global mission that characterizes every UAV's role
        'type'               :                       1,                       # Type of the world or sceneario created
        'n_dataset'          :                       1,                       # Number of the dataset to create
        'n_simulation'       :                       1,                       # Number of simulation where to start instide the dataset
        'N_uav'              :                       1,                       # Number of aerial vehicles that take part in the simulations
        'uav_models'         :   ["plane", "plane", "plane"],   # Type of airborne of each vehicle
        'N_obs'              :                       0,                       # Number of obstacles placed onto some kind of scenarios
        'obs_tube'           :                    [6,3,3],                    # Shape of the obstacle tube for some kind of scenarios
        'path_length'        :                       1,                       # Length of the path for roles that follow one
        'solver_algorithm'   :                    "orca3",                    # Algorithm for path-avoiding
        'N_iter'             :                      200,                      # Bunch of simulations developed in the defined dataset
        'px4_use'            :                    "complete",                 # Flag to decide if PX4 is used
        'communications'     :                    "direct",                   # Kind of communications between UAVs
        'heading_use'        :                     False,                     # Flag to decide if heading is controlled
        'depth_camera_use'   :                     False,                     # Flag to decide if the info from depth camera is used
        'smach_view'         :                     False,                     # Flag to decide if smach introspector is actived
        }

        rospy.set_param('gazebo_gui',True)   # Gazebo visulization

        # computer' path definition for each user
        user = "JA"
        if user == "JA":
            home_path = "joseandresmr"
        self.world_definition["home_path"] = home_path

        # Flag to save simulation data if active. The user will be asked to deactive
        self.world_definition["save_flag"] = True

        # Function to check if current dataset is already created and ask the user what to do in each case
        botton_selected = self.DatasetExistanceChecker()

        ### Creation of the dictionary that defines the State Machine that composes the global mission
        # First part of mission is common
        ansp_sm_def=[]
        step_1 = {"type":"new_world"}
        ansp_sm_def.append(step_1)
        step_2 = {"type":"spawn_uavs"}
        ansp_sm_def.append(step_2)
        step_3 = {"type":"all_take_off_ccr"}
        ansp_sm_def.append(step_3)
        # step_4 = {"type":"wait"}
        # ansp_sm_def.append(step_4)

        # This mission part is customized for every type of mission
        step_5 = {"type":"{}_sm".format(self.world_definition['mission'])}
        ansp_sm_def.append(step_5)

        # This part of the mission is only introduced if save flag is active
        if self.world_definition["save_flag"]:
            step_6 = {"type":"all_save_csv_ccr"}
            ansp_sm_def.append(step_6)

        # Last part is common for every mission
        step_7 = {"type":"all_land_ccr"}
        ansp_sm_def.append(step_7)
        step_4 = {"type":"wait"}
        ansp_sm_def.append(step_4)
        self.world_definition['ansp_sm_def'] = ansp_sm_def

        ### Initializations
        self.GazeboLauncher()    # Start Gazebo standalone
        rospy.set_param('world_definition', self.world_definition)    # Upload ROS params above defined
        rospy.init_node('master', anonymous=True)     # Start node
        self.MasterListener()       # Start subscribers
        self.n_simulation_bias = 0      # Initi

        ### Bunch of simulations
        if botton_selected != "q":    # If user decission was not to abort, start bunch of simulations
            # Run every simulation
            for n_simulation in range(self.n_simulation_bias+1, self.n_simulation_bias + self.world_definition["N_iter"] + 1):
                try:
                    ### Initialization for each simulation
                    # Set ROS param of current simulation id
                    rospy.set_param('world_definition/n_simulation', n_simulation)
                    print('n_simulation',n_simulation)

                    self.ANSPSpawner()   # Start ANSP node. It will be in charge of this particuar mission

                    # Initialize flag to finish simulation. Will be activated later
                    self.simulation_finished = False

                    # Initialize time counter to check mission time
                    timer_start = time.time()

                    ### Process to finish simulation when time exceeded
                    # Wait until simulation finishes, it is when flag is raised or master breaks
                    while (self.simulation_finished == False) and not rospy.is_shutdown():
                        time.sleep(2)
                        # Control of exceeded simulation duration
                        if (time.time() - timer_start) > self.world_definition["path_length"]*600:
                            self.ANSP_launch.shutdown()     # Terminate ANSP
                            self.simulation_finished = True    # Activate end flag
                except:
                    self.processess_killer(2)       # Kill unwanted processess

                self.processess_killer(1)       # Kill unwanted processess
                self.GazeboRestart()        # Restart Gazebo for next simulation


        self.SavingDatasetDefinition(1)     # Save all dataset information
        self.processess_killer(2)       # Kill unwanted processess

    #### commander functions ####

    # Launching GAZEBO client and server
    def GazeboLauncher(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.Gazebo_launch = roslaunch.parent.ROSLaunchParent(uuid,[\
            "/home/{0}/catkin_ws/src/pydag/Code/launch/gazebo_spawner_JA.launch".format(self.world_definition["home_path"])])

        self.Gazebo_launch.start()
        time.sleep(0.5)

    # Launching ANSP node
    def ANSPSpawner(self):
        uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid1)
        self.ANSP_launch = roslaunch.parent.ROSLaunchParent(uuid1,[\
            "/home/{0}/catkin_ws/src/pydag/Code/launch/ANSP_spawner_JA.launch".format(self.world_definition["home_path"])])

        self.ANSP_launch.start()

    # Control if defined new dataset already exists
    def DatasetExistanceChecker(self):
        # Redefinition of number of obstacles. Obstacles in world types 1,2/3,4 are defined differently.
        # This code fixes that divergence for later storing.
        # In the future, this piece of code should be a common utility
        if self.world_definition["type"] == 3 or self.world_definition["type"] == 4:
            N_obs_mixed = int('{0}{1}{2}'.format(self.world_definition["obs_tube"][0],self.world_definition["obs_tube"][1],self.world_definition["obs_tube"][2]))
        else:
            N_obs_mixed = self.world_definition['N_obs']

        # Build path from definition
        self.first_folder_path = "/home/{4}/catkin_ws/src/pydag/Data_Storage/Simulations/{0}/{5}/type{1}_Nuav{2}_Nobs{3}"\
                                 .format(self.world_definition["mission"],self.world_definition["type"],self.world_definition["N_uav"],
                                 N_obs_mixed,self.world_definition["home_path"],self.world_definition["solver_algorithm"])
        self.second_folder_path = self.first_folder_path + "/dataset_{}".format(self.world_definition["n_dataset"])

        # Ask user if the new simulation already exists
        if os.path.exists(self.second_folder_path):
            selected = raw_input("Selected dataset already exists. \
                \nTo finish simulation, press \"q\". \nTo skip saving, press \"n\". \
                \nTo add, press \"a\". \nTo renew the dataset, press any other.")
            # Quit option
            if selected == "q":
                return "q"

            # No saving option, deactivate its flag
            elif selected == "n":
                self.world_definition['save_flag'] = False
                return "n"

            # Add option, from existing number of dataset
            elif selected == "a":
                # Read how mane simulations already exist
                n_prior_simulations = len(os.walk(self.second_folder_path).next()[1])

                # Create a bias to add it to the new simulation pointer
                if self.world_definition["N_iter"] != 0:
                    shutil.rmtree(self.second_folder_path + "/simulation_{}".format(n_prior_simulations))
                    self.n_simulation_bias = n_prior_simulations -1
                else:
                    self.n_simulation_bias = n_prior_simulations

                return "a"

            # Reset dataset option, remove existing simulations and start by 0
            elif (selected != "q") and (selected != "a"):
                shutil.rmtree(self.second_folder_path)
                return "r"
        else:
            return "r"

        return "g"

    # Save parameters that define the whole dataset.
    # This function is called to sum up the dataset and contains information of its performance
    def SavingDatasetDefinition(self):
        global_dicc = {'simulation_n' : [], 'succeed' : [], 'message': []}

        # Check every file existance, read its success and message info and add it to a
        # global dictionary with info of all dataset
        for f in os.listdir(self.second_folder_path):
            if os.path.isdir(os.path.join(self.second_folder_path, f)):
                try:
                    df = pd.read_csv(self.second_folder_path + '/' + f + '/' + 'world_definition.csv')
                    global_dicc['simulation_n'].append(int(f.split('_')[1]))
                    global_dicc['succeed'].append(df['simulation_succeed'][0])
                    global_dicc['message'].append("still_nothing")
                except:
                    pass

        # Convert the global dictionary into csv and write it to file
        self.performance_info = pd.DataFrame(global_dicc).sort_values(by=['simulation_n'])
        self.performance_info.to_csv(self.second_folder_path + '/performance_info.csv', sep=',')
        with open(self.second_folder_path + '/dataset_definition.csv','wb') as f:
            w = csv.writer(f)
            w.writerows(self.world_definition.items())

    # Close and start Gazebo for every simulation
    def GazeboRestart(self):
        self.Gazebo_launch.shutdown()
        time.sleep(1)
        self.GazeboLauncher()
        rospy.set_param('world_definition', self.world_definition)
        time.sleep(5)

    # After a simulation, some processes stay alive nut must be closed
    def processess_killer(self,level):
        [os.system("pkill -9 {}".format(proc)) for proc in ["px4","mavros_node"]]
        if level >= 2:
            [os.system("pkill -9 {}".format(proc)) for proc in ["server","python","python2"]]

    #### listener functions ####
    # Master only listens to ANSP for the end of simulation message
    def MasterListener(self):
        rospy.Service('/pydag/ANSP/simulation_termination', DieCommand, self.handle_simulation_termination)

    def handle_simulation_termination(self,data):
        self.simulation_finished = True
        return True

def main():

    Master()

if __name__ == '__main__':
    main()
