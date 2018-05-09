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
from cv_bridge import CvBridge, CvBridgeError
from uav_abstraction_layer.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

from gauss.srv import *


class Master(object):
    def __init__(self):
        self.world_definition = {'project'       :             "gauss"                  ,\
                            'type'               :                1                     ,\
                            'n_dataset'          :                1                     ,\
                            'n_simulation'       :                1                     ,\
                            'N_uav'              :                2                     ,\
                            'uav_models'         : ["typhoon_h480","typhoon_h480","typhoon_h480"]     ,\
                            'N_obs'              :                1                     ,\
                            'path_length'        :                1                     ,\
                            'solver_algorithm'   :             "orca"                   ,\
                            'N_iter'             :                2                     ,\
                        }

        self.GazeboLauncher()
        rospy.set_param('world_definition', self.world_definition)

        rospy.init_node('Master', anonymous=True)

        self.simulation_finished = True

        self.MasterListener()

        # if self.DatasetExistanceChecker() != "q":
        #     for n_simulation in range(self.world_definition["N_iter"]):
        #         rospy.set_param('world_definition/n_simulation', n_simulation+1)
        #         self.ANSPSpawner()
        #         self.simulation_finished = False
        #         while self.simulation_finished == False:
        #             time.sleep(5)

        for n_simulation in range(self.world_definition["N_iter"]):
            rospy.set_param('world_definition/n_simulation', n_simulation+1)
            self.ANSPSpawner()
            self.simulation_finished = False
            while self.simulation_finished == False:
                time.sleep(5)

        time.sleep(1000)



    
    #### commander functions ####
    def GazeboLauncher(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,[\
                    "/home/joseandres/catkin_ws/src/jamrepo/gauss/launch/test_server_empty_JA.launch"])
        launch.start()
        time.sleep(1)

    def ANSPSpawner(self):
        uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid1)
        launch = roslaunch.parent.ROSLaunchParent(uuid1,[\
            "/home/joseandres/catkin_ws/src/jamrepo/gauss/launch/ANSP_spawner_JA.launch"])
        launch.start()

    def DatasetExistanceChecker(self):
        first_folder_path = "/home/joseandres/catkin_ws/src/jamrepo/Simulation_data/{0}/type{1}_Nuav{2}_Nobs{3}".format(self.world_definition["project"],self.world_definition["type"],self.world_definition["N_uav"],self.world_definition["N_obs"])
        second_folder_path = first_folder_path + "/dataset_{}".format(self.world_definition["n_dataset"])
        if os.path.exists(second_folder_path):
            selected = input("Selected dataset already exists. To finish simulation, press \"q\". Other case, press any other. ")
            if selected == "q":
                return "q"
            elif selected != "q":
                shutil.rmtree(second_folder_path)
                return "r" 
        else:
            return "r"

    #### listener functions ####
    def MasterListener(self):
        rospy.Service('/gauss/ANSP/simulationtermination', DieCommand, self.handle_simulation_termination)

    def handle_simulation_termination(self,data):
        self.simulation_finished = True
        print "simulation finished msg"
        return True

def main():
    Master()

if __name__ == '__main__':
    main()