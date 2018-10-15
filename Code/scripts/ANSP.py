#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 21 2018

@author: josmilrom
"""

import sys
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
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge, CvBridgeError
from uav_abstraction_layer.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from gazebo_msgs.srv import DeleteModel
from tf2_msgs.msg import *

import utils
from Ground_Station import *
from Worlds import *
from pydag.srv import *
from ANSP_SM import ANSP_SM

class ANSP(object):
    def __init__(self):
        # Global parameters inizialization
        self.GettingWorldDefinition()
        self.simulation_succeed = True

        self.states_list = []
        for i in np.arange(self.N_uav):
            self.states_list.append("landed")

        self.collisioned_list = []
        for i in np.arange(self.N_uav):
            self.collisioned_list.append(False)

        # ROS items inizialization
        rospy.init_node('ansp', anonymous=True)
        self.ANSPListener()

        # Start mission depending project
        if self.project == "dcdaa":
            self.Dcdaa()

        if self.project == "gauss":
            # self.Gauss()
            self.ansp_sm_def=[]
            step_1 = {"type":"new_world"}
            self.ansp_sm_def.append(step_1)
            step_2 = {"type":"spawn_uavs"}
            self.ansp_sm_def.append(step_2)
            step_3 = {"type":"all_take_off_ccr"}
            self.ansp_sm_def.append(step_3)
            step_4 = {"type":"wait"}
            self.ansp_sm_def.append(step_4)
            # step_5 = {"type":"queue_of_followers_ap_sm"}
            step_5 = {"type":"queue_of_followers_ad_sm"}
            # step_5 = {"type":"follow_paths_sbys_sm"}
            self.ansp_sm_def.append(step_5)
            step_6 = {"type":"all_save_csv_ccr"}
            self.ansp_sm_def.append(step_6)
            step_7 = {"type":"all_land_ccr"}
            self.ansp_sm_def.append(step_7)

            self.world_definition = rospy.get_param('world_definition')
            self.world_definition['global_mission'] = self.ansp_sm_def[4]["type"]
            rospy.set_param('world_definition',self.world_definition)

            self.ansp_sm = ANSP_SM(self)
            outcome = self.ansp_sm.ansp_sm.execute()

        self.Die()

        self.bag.close()

    #### Commander functions ####
    def Dcdaa(self):
        # Local parameters inizialization
        self.CreatingSimulationDataStorage()
        self.world = Worlds(1)
        self.GettingWorldDefinition()
        uavs_goal_paths_list = []
        for n_uav in np.arange(self.N_uav):
            # UAV's mission parameters inizialization
            uav_goal_path = self.world.PathGenerator()
            uavs_goal_paths_list.append(uav_goal_path)
            first_pose = uav_goal_path[0]
            self.SetUavSpawnFeatures(n_uav+1,self.uav_models[n_uav],[first_pose.position.x,first_pose.position.y,0])       
 
        # UAV spawn and path command
            self.UAVSpawner()
        for n_uav in np.arange(self.N_uav):
            self.PathCommand(n_uav+1,uavs_goal_paths_list[n_uav])

    def Gauss(self):
        # Local parameters inizialization
        self.CreatingSimulationDataStorage()
        self.world = Worlds(1)
        self.GettingWorldDefinition()
        uavs_goal_paths_list = []
        for n_uav in np.arange(self.N_uav):
            # UAV's mission parameters inizialization
            uav_goal_path = self.world.PathGenerator()
            uavs_goal_paths_list.append(uav_goal_path)
            first_pose = uav_goal_path[0]
            self.SetUavSpawnFeatures(n_uav+1,self.uav_models[n_uav],[first_pose.position.x,first_pose.position.y,0])       

        # UAV spawn and path command
            self.UAVSpawner()
            
        for n_uav in np.arange(self.N_uav):
            self.PathCommand(n_uav+1,uavs_goal_paths_list[n_uav])

        # Syncronization control of every UAV
        for i in np.arange(self.path_length):
            all_in_place = False
            while all_in_place == False:
                counter = 0
                for j in np.arange(self.N_uav):
                    if self.states_list[j] == "in WP {}".format(i+1):
                        counter = counter+1
                if counter == self.N_uav:
                    time.sleep(2)
                    for j in np.arange(self.N_uav):
                        self.InstructionCommand(j+1,"GoOn")
                    all_in_place = True
                    counter = 0
                time.sleep(0.3)

    # Function to send paths to each UAV
    def PathCommand(self,ID,goal_path_poses_list):
        # time.sleep()
        rospy.wait_for_service('/pydag/ANSP_UAV_{}/wp_list_command'.format(ID))
        try:
            ual_path_command = rospy.ServiceProxy('/pydag/ANSP_UAV_{}/wp_list_command'.format(ID), WpPathCommand)
            ual_path_command(np.array(goal_path_poses_list),False)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in wp_list_command"

    # Function to send instructions to each UAV
    def InstructionCommand(self,ID,Instruction):
        rospy.wait_for_service('/pydag/ANSP_UAV_{}/instruction_command'.format(ID))
        try:
            instruction_command = rospy.ServiceProxy('/pydag/ANSP_UAV_{}/instruction_command'.format(ID), InstructionCommand)
            instruction_command(Instruction)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in instruction_command"

    # Function to send termination instruction to each UAV
    def SimulationTerminationCommand(self):
        rospy.wait_for_service('/pydag/ANSP/simulation_termination')
        try:
            instruction_command = rospy.ServiceProxy('/pydag/ANSP/simulation_termination', DieCommand)
            instruction_command(True)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in simulation_termination"

    # Function to set UAV's ROSparameters
    def SetUavSpawnFeatures(self,ID,model,position,yaw=0):
        uav_frame = rospy.get_param( 'uav_{}_home'.format(ID))
        uav_frame['translation'] = position
        if model == "typhoon_h480":
            yaw = yaw + np.pi
        uav_frame['gz_initial_yaw'] =  yaw # radians
        uav_frame['model'] = model
        rospy.set_param('uav_{}_home'.format(ID), uav_frame)

    # Function to spawn each new UAV (GAZEBO model, UAL server and dedicated Ground Station)
    def UAVSpawner(self):  
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.uav_spawner_launch = roslaunch.parent.ROSLaunchParent(uuid,[\
            "/home/{1}/catkin_ws/src/pydag/Code/launch/{0}_spawner_JA.launch".format(self.world_definition['px4_use'],self.home_path)])
        self.uav_spawner_launch.start()

    def UAVSpawner1(self,ID):  
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.uav_spawner_launch = roslaunch.parent.ROSLaunchParent(uuid,[\
            "/home/{1}/catkin_ws/src/pydag/Code/launch/{0}_spawner_JA_{2}.launch".format(self.world_definition['px4_use'],self.home_path,ID+1)])
        self.uav_spawner_launch.start()

    # Function to get Global ROS parameters
    def GettingWorldDefinition(self):
        self.world_definition = rospy.get_param('world_definition')
        self.project = self.world_definition['project']
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

    # Function to create folders and file to store simulations data
    def CreatingSimulationDataStorage(self):
        if self.project == 'dcdaa':
            N_obs_mixed = int('{0}{1}{2}'.format(self.obs_tube[0],self.obs_tube[1],self.obs_tube[2]))
        elif self.project == 'gauss':
            N_obs_mixed = self.N_obs
        first_folder_path = "/home/{4}/catkin_ws/src/pydag/Data_Storage/Simulations/{0}/{5}/type{1}_Nuav{2}_Nobs{3}".format(self.project,self.world_type,self.N_uav,N_obs_mixed,self.home_path,self.solver_algorithm)

        if not os.path.exists(first_folder_path):
            os.makedirs(first_folder_path)

        second_folder_path = first_folder_path + "/dataset_{}".format(self.n_dataset)
        if not os.path.exists(second_folder_path):
            os.makedirs(second_folder_path)   

        self.third_folder_path = second_folder_path + "/simulation_{}".format(self.n_simulation)
        if not os.path.exists(self.third_folder_path):
            os.makedirs(self.third_folder_path)

        bag_folder_path = "/home/{6}/catkin_ws/src/pydag/Data_Storage/Simulations/{0}/{7}/type{1}_Nuav{2}_Nobs{3}/dataset_{4}/simulation_{5}/tf_bag.bag".format(self.project,self.world_type,self.N_uav,N_obs_mixed,self.n_dataset,self.n_simulation,self.home_path,self.solver_algorithm)
        self.bag = rosbag.Bag(bag_folder_path, 'w')

    # Function to update ROS parameters about simulation performance
    def SavingWorldDefinition(self):
        self.world_definition = rospy.get_param('world_definition')
        self.world_definition['collisioned_list'] = self.collisioned_list
        self.world_definition['simulation_succeed'] = self.simulation_succeed
        rospy.set_param('world_definition',self.world_definition)
        file_path = self.third_folder_path + '/world_definition.csv'
        with open(file_path,'wb') as f:
            w = csv.writer(f)
            w.writerows(self.world_definition.items())

    # Function to close active child processess
    def Die(self):
        self.SavingWorldDefinition()
        self.UAVKiller()
        self.world.eraseAllObstacles()
        self.GazeboModelsKiller()
        self.SimulationTerminationCommand()
        rospy.signal_shutdown("end of experiment")

    # Function to close UAV Ground Station  process
    def UAVKiller(self):
        for n_uav in np.arange(self.N_uav):
            rospy.wait_for_service('/pydag/ANSP_UAV_{}/die_command'.format(n_uav+1))
            try:
                die_command = rospy.ServiceProxy('/pydag/ANSP_UAV_{}/die_command'.format(n_uav+1), DieCommand)
                die_command(True)
                time.sleep(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                print "error in die_command"
        return

    # def PreemptCommand(self):
    #     rospy.wait_for_service('/pydag/ANSP/preemption_command_to_2')
    #     try:
    #         # print "path for uav {} command".format(ID)
    #         PreemptCommander = rospy.ServiceProxy('/pydag/ANSP/preemption_command_to_2', StateActualization)
            # PreemptCommander(self.ID,"preempt",self.collision)
    #         return
    #     except rospy.ServiceException, e:
    #         print "Service call failed: %s"%e
    #         print "error in state_actualization"

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

    #### listener functions ####
    def ANSPListener(self):
        rospy.Service('/pydag/ANSP/state_actualization', StateActualization, self.handle_uav_status)
        rospy.Subscriber('/tf_static', TFMessage, self.tf_static_callback)
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)

    # Function to update and local parameters of UAVs status
    def handle_uav_status(self,data):
        self.states_list[data.id-1] = data.state
        if data.collision == True:
            if self.collisioned_list[data.id-1] == False:
                self.collisioned_list[data.id-1] == True
            self.simulation_succeed = False
            rospy.loginfo_throttle(0,"Simulation not succeded, UAV {} collisioned".format(data.id))
        print self.states_list
        return True

    # Functions to sava TF information inside ROS bag
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
        
def main():
    ANSP()

if __name__ == '__main__':
    main()