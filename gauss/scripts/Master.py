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
from sensor_msgs.msg import *

from gauss.srv import *
import utils

class Master(object):
    def __init__(self):
        self.world_definition = {'project'       :             "dcdaa"                  ,\
                            'type'               :                1                     ,\
                            'n_dataset'          :                1                     ,\
                            'n_simulation'       :                1                     ,\
                            'N_uav'              :                1                     ,\
                            'uav_models'         : ["typhoon_h480","typhoon_h480","typhoon_h480"]     ,\
                            'N_obs'              :                2                     ,\
                            'obs_tube'           :             [5,3,2]                  ,\
                            'path_length'        :                5                     ,\
                            'solver_algorithm'   :             "simple"                   ,\
                            'N_iter'             :                1                     ,\
                            'px4_use'            :             "complete"                  ,\
                        }
        self.gazebo_gui = True

        self.GazeboLauncher()
        rospy.set_param('world_definition', self.world_definition)

        rospy.init_node('master', anonymous=True)

        self.simulation_finished = True

        self.MasterListener()

        self.iteration_succeed_list = []
        self.iteration_not_succeed_list = []

        if self.DatasetExistanceChecker() != "q":
            for n_simulation in range(self.world_definition["N_iter"]):
                rospy.set_param('world_definition/n_simulation', n_simulation+1)
                self.ANSPSpawner()
                self.simulation_finished = False
                while self.simulation_finished == False:
                    time.sleep(5)

        self.SavingWorldDefinition()

        time.sleep(1000)

    #### commander functions ####
    def GazeboLauncher(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.ANSP_launch = roslaunch.parent.ROSLaunchParent(uuid,[\
                    "/home/joseandres/catkin_ws/src/jamrepo/gauss/launch/server_empty_JA.launch"])
        self.ANSP_launch.start()
        time.sleep(1)

        # gazebo_gui = 'false'                                           -----> Necesita ROS LUNAR
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # cli_args = ['gauss', 'server_empty_JA.launch', 'gui:=gazebo_gui']
        # roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
        # roslaunch_args = cli_args[2:]
        # parent = roslaunch.parent.ROSLaunchParent(uuid, [(roslaunch_file, roslaunch_args)])
        # parent.start()

        # time.sleep(0.2)
        # temp_dir = utils.temp_dir(None)
        # subprocess.call("mkdir -p " + temp_dir, shell=True)
        # if self.gazebo_gui == True:
        #     args = "roslaunch gauss test_server_empty_JA.launch gui:='true'"
        # elif self.gazebo_gui == False:
        #     args = "roslaunch gauss test_server_empty_JA.launch gui:='false'"
        # gzlaunch_out = open(temp_dir + '/gzlaunch.out', 'w')
        # gzlaunch_err = open(temp_dir + '/gzlaunch.err', 'w')
        # gazebo_subprocess = subprocess.Popen(args, stdout=gzlaunch_out, stderr=gzlaunch_err, cwd=temp_dir, shell=True, preexec_fn=os.setsid)

        


    def ANSPSpawner(self):
        uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid1)
        launch = roslaunch.parent.ROSLaunchParent(uuid1,[\
            "/home/joseandres/catkin_ws/src/jamrepo/gauss/launch/ANSP_spawner_JA.launch"])
        launch.start()

        # time.sleep(0.2)
        # temp_dir = utils.temp_dir(None)
        # subprocess.call("mkdir -p " + temp_dir, shell=True)
        # ANSPspawner_out = open(temp_dir + '/uavspawner.out', 'w')
        # ANSPspawner_err = open(temp_dir + '/uavspawner.err', 'w')

        # args = "roslaunch gauss ANSP_spawner_JA.launch"
        # self.ANSPspawner_subprocess = subprocess.Popen(args, stdout=ANSPspawner_out, stderr=ANSPspawner_err, cwd=temp_dir, shell=True, preexec_fn=os.setsid)


    def DatasetExistanceChecker(self):
        if self.world_definition["project"] == 'dcdaa':
            N_obs_mixed = int('{0}{1}{2}'.format(self.world_definition["obs_tube"][0],self.world_definition["obs_tube"][1],self.world_definition["obs_tube"][2]))
        elif self.world_definition["project"] == 'gauss':
            N_obs_mixed = self.world_definition['N_obs']
        first_folder_path = "/home/joseandres/catkin_ws/src/jamrepo/Simulation_data/{0}/type{1}_Nuav{2}_Nobs{3}".format(self.world_definition["project"],self.world_definition["type"],self.world_definition["N_uav"],N_obs_mixed)

        self.second_folder_path = first_folder_path + "/dataset_{}".format(self.world_definition["n_dataset"])
        if os.path.exists(self.second_folder_path):
            selected = raw_input("Selected dataset already exists. To finish simulation, press \"q\". Other case, press any other. ")
            if selected == "q":
                return "q"
            elif selected != "q":
                shutil.rmtree(self.second_folder_path)
                return "r" 
        else:
            return "r"

    def IterationClassifier(self,n_simulation):
        iteration_succeed = rospy.get_param('world_definition/simulation_succeed')
        if iteration_succeed == True:
            self.iteration_succeed_list.append(n_simulation)
        elif iteration_succeed == False:
            self.iteration_not_succeed_list.append(n_simulation)

    def SavingWorldDefinition(self):
        self.world_definition['iteration_succeed_list'] = self.iteration_succeed_list
        self.world_definition['iteration_not_succeed_list'] = self.iteration_not_succeed_list
        file_path = self.second_folder_path + '/world_definition.csv'
        with open(file_path, 'wb') as f:
            w = csv.DictWriter(f, self.world_definition.keys())
            w.writeheader()
            w.writerow(self.world_definition)

    def HandleException(self):
        self.ANSP_spawner_launch.shutdown()
        time.sleep(1)
        if self.ANSPspawner_subprocess.poll() is None:
            os.killpg(os.getpgid(self.ANSPspawner_subprocess.pid), signal.SIGTERM)
        # client_out.close()
        # client_err.close()
        # server_out.close()
        # server_err.close()

    #### listener functions ####
    def MasterListener(self):
        rospy.Service('/gauss/ANSP/simulation_termination', DieCommand, self.handle_simulation_termination)

    def handle_simulation_termination(self,data):
        self.simulation_finished = True
        return True

def main():
    try:
        Master()
    except KeyboardInterrupt:
        time.sleep(1)
        rospy.signal_shutdown("end of experiment")
        # Master.HandleException()
    except rospy.is_shutdown:
        time.sleep(1)
        rospy.signal_shutdown("end of experiment")
        # Master.HandleException()

if __name__ == '__main__':
    main()