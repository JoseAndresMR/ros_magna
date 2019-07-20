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
import xml.etree.ElementTree
import pandas as pd
import rospkg
import json
from uav_abstraction_layer.srv import *
from geometry_msgs.msg import *
from std_srvs.srv import *
from sensor_msgs.msg import *
import pandas as pd
import rospkg

from magna.srv import *
import utils

class Master(object):
    def __init__(self):
        # self.processess_killer(2)
        # World paramenters initialization     follow_paths_sbys, queue_of_followers_ap, queue_of_followers_ad long_wait
        self.hyperparameters = {
        'world'              :                   "Empty",                    # Type of the world or sceneario created
        'subworld'           :                   "Empty",
        'mission'            :                   "Safedrone",                    # Global mission that characterizes every Agent's role
        'submission'         :                   "1UAV_UTM_test",
        'n_dataset'          :                       1,                       # Number of the dataset to create
        'n_simulation'       :                       1,                       # Number of simulation where to start instide the dataset
        'N_iter'             :                      200,                      # Bunch of simulations developed in the defined dataset
        'algorithms_list'    :                    ["orca3"],
        'communications'     :                    "direct",                   # Kind of communications between Agents
        'heading_use'        :                     False,                     # Flag to decide if heading is controlled
        'depth_camera_use'   :                     False,                     # Flag to decide if the info from depth camera is used
        'smach_view'         :                     True,                      # Flag to decide if smach introspector is actived
        'rviz_gui'           :                     True,
        }
        # 00.00022
        rospy.set_param('gazebo_gui',False)   # Gazebo visulization

        self.hyperparameters["home_path"] = rospkg.RosPack().get_path('magna')[:-5]

        # Flag to save simulation data if active. The user will be asked to deactive
        self.hyperparameters["save_flag"] = True
        self.hyperparameters["rosbag_flag"] = False

        # Function to check if current dataset is already created and ask the user what to do in each case
        botton_selected = self.DatasetExistanceChecker()

        ### Initializations
        self.GazeboLauncher()    # Start Gazebo standalone
        rospy.set_param('magna_hyperparameters', self.hyperparameters)    # Upload ROS params above defined
        rospy.init_node('master', anonymous=True)     # Start node
        self.MasterListener()       # Start subscribers
        self.n_simulation_bias = 0      # Initi

        ### Bunch of simulations
        if botton_selected != "q":    # If user decission was not to abort, start bunch of simulations
            # Run every simulation
            for n_simulation in range(self.n_simulation_bias+1, self.n_simulation_bias + self.hyperparameters["N_iter"] + 1):
                try:
                    ### Initialization for each simulation
                    # Set ROS param of current simulation id
                    rospy.set_param('magna_hyperparameters/n_simulation', n_simulation)
                    print('n_simulation',n_simulation)

                    self.GSSpawner()   # Start Ground Station node. It will be in charge of this particuar mission

                    # Initialize flag to finish simulation. Will be activated later
                    self.simulation_finished = False

                    # Initialize time counter to check mission time
                    timer_start = time.time()

                    ### Process to finish simulation when time exceeded
                    # Wait until simulation finishes, it is when flag is raised or master breaks
                    while not rospy.is_shutdown() and (self.simulation_finished == False):
                        time.sleep(2)
                        # Control of exceeded simulation duration
                        if (time.time() - timer_start) > 990000:
                            self.GS_launch.shutdown()     # Terminate Ground Station
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

        world_config_path = "{0}/Code/JSONs/Worlds/{1}/{2}.json"\
                            .format(self.hyperparameters["home_path"],
                            self.hyperparameters["world"],self.hyperparameters["subworld"])
        
        with open(world_config_path) as f:
            world_config = json.load(f)

        if "origin_geo" in world_config["scenario"].keys():
            origin_geo = world_config["scenario"]["origin_geo"]

        else:
            origin_geo = [37.558542, -5.931074, 7.89]

        gazebo_spawn_launch_path = "{0}/Code/launch/gazebo_spawner_JA.launch".format(self.hyperparameters["home_path"])

        et = xml.etree.ElementTree.parse(gazebo_spawn_launch_path)
        root = et.getroot()

        root[1][1][0].text = str(origin_geo)

        et.write(gazebo_spawn_launch_path)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.Gazebo_launch = roslaunch.parent.ROSLaunchParent(uuid,[\
            "{0}/Code/launch/gazebo_spawner_JA.launch".format(self.hyperparameters["home_path"])])

        self.Gazebo_launch.start()
        time.sleep(0.5)

    # Launching Ground Station node
    def GSSpawner(self):
        uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid1)

        launch_path = "{0}/Code/launch/GS_spawner_JA.launch".format(self.hyperparameters['home_path'])

        # et = xml.etree.ElementTree.parse(launch_path)
        # root = et.getroot()
        # if self.rviz_gui == True:
        #     root[1].attrib["if"] = "true"
        #     root[1][0].attrib["args"] = "-d $(find magna)/rviz/{0}.rviz".format(self.hyperparameters["world"])
        # else:
        #     root[1].attrib["if"] = "false"
        # et.write(launch_path)

        self.GS_launch = roslaunch.parent.ROSLaunchParent(uuid1,[launch_path])
        self.GS_launch.start()

    # Control if defined new dataset already exists
    def DatasetExistanceChecker(self):

        # Build path from definition
        self.first_folder_path = "{0}/Data_Storage/Simulations/{1}/{2}/{3}/{4}"\
                                 .format(self.hyperparameters["home_path"],self.hyperparameters["world"],self.hyperparameters["subworld"],
                                 self.hyperparameters["mission"],self.hyperparameters["submission"])

        self.second_folder_path = self.first_folder_path + "/dataset_{}".format(self.hyperparameters["n_dataset"])

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
                self.hyperparameters['save_flag'] = False
                return "n"

            # Add option, from existing number of dataset
            elif selected == "a":
                # Read how mane simulations already exist
                n_prior_simulations = len(os.walk(self.second_folder_path).next()[1])

                # Create a bias to add it to the new simulation pointer
                if self.hyperparameters["N_iter"] != 0:
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
            w.writerows(self.hyperparameters.items())

    # Close and start Gazebo for every simulation
    def GazeboRestart(self):
        self.Gazebo_launch.shutdown()
        time.sleep(1)
        self.GazeboLauncher()
        rospy.set_param('magna_hyperparameters', self.hyperparameters)
        time.sleep(5)

    # After a simulation, some processes stay alive nut must be closed
    def processess_killer(self,level):
        [os.system("pkill -9 {}".format(proc)) for proc in ["px4","mavros_node"]]
        if level >= 2:
            [os.system("pkill -9 {}".format(proc)) for proc in ["server","python","python2"]]

    #### listener functions ####
    # Master only listens to Ground Station for the end of simulation message
    def MasterListener(self):
        rospy.Service('/magna/GS/simulation_termination', DieCommand, self.handle_simulation_termination)

    def handle_simulation_termination(self,data):
        self.simulation_finished = True
        return True

def main():

    Master()

if __name__ == '__main__':
    main()
