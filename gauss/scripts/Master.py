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

from gauss.srv import *
import utils

# rospy.on_shutdown(self.shutdown)

class Master(object):
    def __init__(self):
        self.world_definition = {'project'       :             "gauss"                  ,\
                            'type'               :                1                     ,\
                            'n_dataset'          :                1                     ,\
                            'n_simulation'       :                1                     ,\
                            'N_uav'              :                1                     ,\
                            'uav_models'         : ["typhoon_h480","typhoon_h480","typhoon_h480"]     ,\
                            'N_obs'              :                2                     ,\
                            'obs_tube'           :             [5,3,2]                ,\
                            'path_length'        :                4                     ,\
                            'solver_algorithm'   :             "neural_network"                   ,\
                            'N_iter'             :               0                      ,\
                            'px4_use'            :             "complete"               ,\
                            'communications'     :             "direct"                 ,\
                        }
        
        rospy.set_param('gazebo_gui',False)

        self.GazeboLauncher()
        rospy.set_param('world_definition', self.world_definition)

        rospy.init_node('master', anonymous=True)

        self.simulation_finished = True

        self.MasterListener()

        self.n_simulation_bias = 0

        if self.DatasetExistanceChecker() != "q":
            for n_simulation in range(self.n_simulation_bias, self.n_simulation_bias + self.world_definition["N_iter"]):
                # try:
                #     print "flag 2"
                #     rospy.wait_for_service('gazebo/spawn_sdf_model')
                #     print "flag 3"
                # except rospy.ServiceException, e:
                #     print "Service call failed: %s"%e
                #     print "Restarting Gazebo & PX4"
                #           
                rospy.set_param('world_definition/n_simulation', n_simulation+1)
                print 'n_simulation',n_simulation
                self.ANSPSpawner()
                self.simulation_finished = False
                timer_start = time.time()
                while (self.simulation_finished == False) and not rospy.is_shutdown():
                    time.sleep(2)
                    if (time.time() - timer_start) > self.world_definition["path_length"]*50:
                        self.ANSP_launch.shutdown()
                        self.error_msg = "Simulation time exceeded"
                        self.simulation_finished = True
                # self.ANSPspawner_subprocess.terminate()
                # os.killpg(self.ANSP_launch, signal.SIGKILL)
                self.GazeboRestart()
                
        self.SavingDatasetDefinition()

        # time.sleep(1000)

    #### commander functions ####
    def GazeboLauncher(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.Gazebo_launch = roslaunch.parent.ROSLaunchParent(uuid,[\
                    "/home/josmilrom/catkin_ws/src/jamrepo/gauss/launch/server_empty_JA.launch"])
        self.Gazebo_launch.start()
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
        self.ANSP_launch = roslaunch.parent.ROSLaunchParent(uuid1,[\
            "/home/josmilrom/catkin_ws/src/jamrepo/gauss/launch/ANSP_spawner_JA.launch"])
        self.ANSP_launch.start()

        # time.sleep(0.2)
        # temp_dir = "/home/josmilrom/ANSP"
        # subprocess.call("mkdir -p " + temp_dir, shell=True)
        # ANSPspawner_out = open(temp_dir + '/uavspawner.out', 'w')
        # ANSPspawner_err = open(temp_dir + '/uavspawner.err', 'w')

        # args = "roslaunch gauss ANSP_spawner_JA.launch"
        # # self.ANSPspawner_subprocess = subprocess.Popen(args, stdout=ANSPspawner_out, stderr=ANSPspawner_err, cwd=temp_dir, shell=True, preexec_fn=os.setsid)
        # self.ANSPspawner_subprocess = subprocess.Popen(args, stdout=subprocess.PIPE, shell=True)

    def DatasetExistanceChecker(self):
        if self.world_definition["project"] == 'dcdaa':
            self.N_obs_mixed = int('{0}{1}{2}'.format(self.world_definition["obs_tube"][0],self.world_definition["obs_tube"][1],self.world_definition["obs_tube"][2]))
        elif self.world_definition["project"] == 'gauss':
            self.N_obs_mixed = self.world_definition['N_obs']
        self.first_folder_path = "/home/josmilrom/catkin_ws/src/jamrepo/Simulation_data/{0}/type{1}_Nuav{2}_Nobs{3}".format(self.world_definition["project"],self.world_definition["type"],self.world_definition["N_uav"],self.N_obs_mixed)

        self.second_folder_path = self.first_folder_path + "/dataset_{}".format(self.world_definition["n_dataset"])
        if os.path.exists(self.second_folder_path):
            selected = raw_input("Selected dataset already exists. To finish simulation, press \"q\". To add, press \"a\". To renew the dataset, press any other.")
            if selected == "q":
                return "q"

            elif selected == "a":

                # n_prior_simulations = sum(os.path.isdir(i) for i in os.listdir(self.second_folder_path))
                # for i in os.listdir(self.second_folder_path):
                #     print i
                n_prior_simulations = len(os.walk(self.second_folder_path).next()[1])
                if self.world_definition["N_iter"] != 0:
                    shutil.rmtree(self.second_folder_path + "/simulation_{}".format(n_prior_simulations))
                self.n_simulation_bias = n_prior_simulations -1
                return "a"

            elif (selected != "q") and (selected != "a"):
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

    def SavingDatasetDefinition(self):
        local_dicc = {'simulation_n' : [], 'succeed' : [], 'message': []}
        for f in os.listdir(self.second_folder_path):
            if os.path.isdir(os.path.join(self.second_folder_path, f)):
                try:
                    df = pd.read_csv(self.second_folder_path + '/' + f + '/' + 'world_definition.csv')
                    local_dicc['simulation_n'].append(int(f.split('_')[1]))
                    local_dicc['succeed'].append(df['simulation_succeed'][0])
                    local_dicc['message'].append("still_nothing")
                except:
                    pass
        
        self.world_definition['succeed_info'] = pd.DataFrame(local_dicc).sort_values(by=['simulation_n'])
        with open(self.second_folder_path + '/dataset_definition.csv', 'wb') as f:
            w = csv.DictWriter(f, self.world_definition.keys())
            w.writeheader()
            w.writerow(self.world_definition)

    # def HandleException(self):
    #     self.ANSP_spawner_launch.shutdown()
    #     time.sleep(1)
    #     if self.ANSPspawner_subprocess.poll() is None:
    #         os.killpg(os.getpgid(self.ANSPspawner_subprocess.pid), signal.SIGTERM)
        # client_out.close()
        # client_err.close()
        # server_out.close()
        # server_err.close()

    def GazeboRestart(self):
        self.Gazebo_launch.shutdown()
        time.sleep(1)
        self.GazeboLauncher()
        rospy.set_param('world_definition', self.world_definition)
        time.sleep(5)


    #### listener functions ####
    def MasterListener(self):
        rospy.Service('/gauss/ANSP/simulation_termination', DieCommand, self.handle_simulation_termination)

    def handle_simulation_termination(self,data):
        self.simulation_finished = True
        return True

def main():
    # try:
    Master()
    # except KeyboardInterrupt:
    #     time.sleep(1)
    #     rospy.signal_shutdown("end of experiment")
    #     # Master.HandleException()
    # except rospy.is_shutdown:
    #     time.sleep(1)
    #     rospy.signal_shutdown("end of experiment")
    #     # Master.HandleException()

if __name__ == '__main__':
    main()