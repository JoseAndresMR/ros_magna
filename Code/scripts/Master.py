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
from cv_bridge import CvBridge, CvBridgeError
from uav_abstraction_layer.srv import *
from geometry_msgs.msg import *
from std_srvs.srv import *
from sensor_msgs.msg import *
import pandas as pd

from pydag.srv import *
import utils

class Master(object):
    def __init__(self):
        # World paramenters inizialization
        self.world_definition = {'project'       :             "gauss"                  ,\
                            'type'               :                1                     ,\
                            'n_dataset'          :                1                     ,\
                            'n_simulation'       :                1                     ,\
                            'N_uav'              :                1                     ,\
                            'uav_models'         : ["typhoon_h480","typhoon_h480","typhoon_h480"]     ,\
                            'N_obs'              :                2                     ,\
                            'obs_tube'           :             [5,3,2]                ,\
                            'path_length'        :                4                     ,\
                            'solver_algorithm'   :             "orca3"                   ,\
                            'N_iter'             :               200                      ,\
                            'px4_use'            :             "complete"               ,\
                            'communications'     :             "direct"                 ,\
                            'depth_camera_use'   :              True                 ,\
                        }
        
        # Gazebo visulization parameter definition
        rospy.set_param('gazebo_gui',True)

        # Project path definition for each user
        user = "JA"
        if user == "JA":
            home_path = "josmilrom"
        elif user == "Rebeca":
            home_path = "rebeca/Documentos/ROS"
        self.world_definition["home_path"] = home_path

        # Inizializations
        self.GazeboLauncher()
        rospy.set_param('world_definition', self.world_definition)
        rospy.init_node('master', anonymous=True)
        self.MasterListener()
        self.n_simulation_bias = 0

        # Ask if current dataset already exists
        if self.DatasetExistanceChecker() != "q":
            # Run every simulation
            for n_simulation in range(self.n_simulation_bias+1, self.n_simulation_bias + self.world_definition["N_iter"] + 1):
                # Inizialization for each simulation
                rospy.set_param('world_definition/n_simulation', n_simulation)
                print 'n_simulation',n_simulation
                self.ANSPSpawner()
                self.simulation_finished = False
                timer_start = time.time()
                # Wait until simulation finishes
                while (self.simulation_finished == False) and not rospy.is_shutdown():
                    time.sleep(2)
                    # Control of exceeded simulation duration
                    if (time.time() - timer_start) > self.world_definition["path_length"]*50:
                        self.ANSP_launch.shutdown()
                        self.error_msg = "Simulation time exceeded"
                        self.simulation_finished = True
                self.GazeboRestart()
                
        # Save all dataset information
        self.SavingDatasetDefinition()

    #### commander functions ####
    # Launching GAZEBO client and server
    def GazeboLauncher(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.Gazebo_launch = roslaunch.parent.ROSLaunchParent(uuid,[\
                    "/home/{0}/catkin_ws/src/pydag/Code/launch/server_empty_JA.launch".format(self.world_definition["home_path"])])

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
        # Redefinition of number of obstacles
        if self.world_definition["project"] == 'dcdaa':
            self.N_obs_mixed = int('{0}{1}{2}'.format(self.world_definition["obs_tube"][0],self.world_definition["obs_tube"][1],self.world_definition["obs_tube"][2]))
        elif self.world_definition["project"] == 'gauss':
            self.N_obs_mixed = self.world_definition['N_obs']
        # Build path from definition
        self.first_folder_path = "/home/{4}/catkin_ws/src/pydag/Data_Storage/Simulations/{0}/{5}/type{1}_Nuav{2}_Nobs{3}".format(self.world_definition["project"],self.world_definition["type"],self.world_definition["N_uav"],self.world_definition["N_obs"],self.world_definition["home_path"],self.world_definition["solver_algorithm"])
        self.second_folder_path = self.first_folder_path + "/dataset_{}".format(self.world_definition["n_dataset"])
        
        # Ask user if conflict
        if os.path.exists(self.second_folder_path):
            selected = raw_input("Selected dataset already exists. To finish simulation, press \"q\". To add, press \"a\". To renew the dataset, press any other.")
            # Quit option
            if selected == "q":
                return "q"

            # Add option, from existing number of dataset
            elif selected == "a":
                n_prior_simulations = len(os.walk(self.second_folder_path).next()[1])
                if self.world_definition["N_iter"] != 0:
                    shutil.rmtree(self.second_folder_path + "/simulation_{}".format(n_prior_simulations))
                    self.n_simulation_bias = n_prior_simulations -1
                else:
                    self.n_simulation_bias = n_prior_simulations

                return "a"

            # Reset dataset option
            elif (selected != "q") and (selected != "a"):
                shutil.rmtree(self.second_folder_path)
                return "r"
        else:
            return "r"

    # Save parameters that define the whole dataset
    def SavingDatasetDefinition(self):
        local_dicc = {'simulation_n' : [], 'succeed' : [], 'message': []}

        # Check file existance and read it
        for f in os.listdir(self.second_folder_path):
            if os.path.isdir(os.path.join(self.second_folder_path, f)):
                try:
                    df = pd.read_csv(self.second_folder_path + '/' + f + '/' + 'world_definition.csv')
                    local_dicc['simulation_n'].append(int(f.split('_')[1]))
                    local_dicc['succeed'].append(df['simulation_succeed'][0])
                    local_dicc['message'].append("still_nothing")
                except:
                    pass
        
        # Write new info
        self.performance_info = pd.DataFrame(local_dicc).sort_values(by=['simulation_n'])
        self.performance_info.to_csv(self.second_folder_path + '/performance_info.csv', sep=',') #,low_memory=False,
        with open(self.second_folder_path + '/dataset_definition.csv', 'wb') as f:
            w = csv.DictWriter(f, self.world_definition.keys())
            w.writeheader()
            w.writerow(self.world_definition)

    # Close and start Gazebo for every simulation
    def GazeboRestart(self):
        self.Gazebo_launch.shutdown()
        time.sleep(1)
        self.GazeboLauncher()
        rospy.set_param('world_definition', self.world_definition)
        time.sleep(5)


    #### listener functions ####
    def MasterListener(self):
        rospy.Service('/pydag/ANSP/simulation_termination', DieCommand, self.handle_simulation_termination)

    def handle_simulation_termination(self,data):
        self.simulation_finished = True
        return True

def main():

    Master()

if __name__ == '__main__':
    main()