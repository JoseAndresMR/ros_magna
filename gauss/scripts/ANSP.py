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
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge, CvBridgeError
from uav_abstraction_layer.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from gazebo_msgs.srv import DeleteModel
from tf2_msgs.msg import *

from Ground_Station import *
from Worlds import *
from gauss.srv import *

class ANSP(object):
    def __init__(self):
        self.world_definition = rospy.get_param('world_definition')
        self.project = self.world_definition['project']
        self.world_type = self.world_definition['type']
        self.n_simulation = self.world_definition['n_simulation']
        self.N_uav = self.world_definition['N_uav']
        self.N_obs = self.world_definition['N_obs']
        self.n_dataset = self.world_definition['n_dataset']
        self.uav_models = self.world_definition['uav_models']
        self.path_length = self.world_definition['path_length']

        self.states_list = []
        for i in np.arange(self.N_uav):
            self.states_list.append("landed")

        rospy.init_node('ANSP', anonymous=True)
        
        self.ANSPListener()

        if self.project == "dcdaa":
            self.Dcdaa()

        if self.project == "gauss":
            self.Gauss()

        killing = False
        while killing == False:
            counter = 0
            for n_uav in np.arange(self.N_uav):
                if self.states_list[n_uav] == "landed":
                    counter = counter+1
            if counter == self.N_uav:
                killing = True
                self.Die()
            time.sleep(0.2)

        self.bag.close()


    #### Commander functions ####
    def Dcdaa(self):
        self.CreatingSimulationDataStorage()
        self.world = Worlds(1)
        uavs_goal_paths_list = []
        for n_uav in np.arange(self.N_uav):
            uav_goal_path = self.world.PathGenerator()
            uavs_goal_paths_list.append(uav_goal_path)
            first_pose = uav_goal_path[0]
            self.SetUavSpawnFeatures(n_uav+1,self.uav_models[n_uav],[first_pose.position.x,first_pose.position.y,0])       
 
        self.UAVSpawner()

        for n_uav in np.arange(self.N_uav):
            self.PathCommand(n_uav+1,uavs_goal_paths_list[n_uav])
        return


    def Gauss(self):
        self.CreatingSimulationDataStorage()
        self.world = Worlds(1)
        uavs_goal_paths_list = []
        for n_uav in np.arange(self.N_uav):
            uav_goal_path = self.world.PathGenerator()
            uavs_goal_paths_list.append(uav_goal_path)
            first_pose = uav_goal_path[0]
            print n_uav+1, first_pose
            self.SetUavSpawnFeatures(n_uav+1,self.uav_models[n_uav],[first_pose.position.x,first_pose.position.y,0])       

        self.UAVSpawner()

        for n_uav in np.arange(self.N_uav):
            self.PathCommand(n_uav+1,uavs_goal_paths_list[n_uav])


        for i in np.arange(self.path_length):
            all_in_place = False
            while all_in_place == False:
                counter = 0
                for j in np.arange(self.N_uav):
                    if self.states_list[j] == "in WP {}".format(i+1):
                        counter = counter+1
                if counter == self.N_uav:
                    for j in np.arange(self.N_uav):
                        self.InstructionCommand(j+1,"GoOn")
                    all_in_place = True
                    counter = 0
                time.sleep(0.3)

    def PathCommand(self,ID,goal_path_poses_list):
        rospy.wait_for_service('/gauss/ANSP_UAV_{}/WP_list_command'.format(ID))
        try:
            ual_path_command = rospy.ServiceProxy('/gauss/ANSP_UAV_{}/WP_list_command'.format(ID), WpPathCommand)
            ual_path_command(np.array(goal_path_poses_list),False)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def InstructionCommand(self,ID,Instruction):
        rospy.wait_for_service('/gauss/ANSP_UAV_{}/instruction_command'.format(ID))
        try:
            instruction_command = rospy.ServiceProxy('/gauss/ANSP_UAV_{}/instruction_command'.format(ID), InstructionCommand)
            instruction_command(Instruction)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def SimulationTerminationCommand(self):
        rospy.wait_for_service('/gauss/ANSP/simulationtermination')
        try:
            instruction_command = rospy.ServiceProxy('/gauss/ANSP/simulationtermination', DieCommand)
            instruction_command(True)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def SetUavSpawnFeatures(self,ID,model,position,yaw=0):
        uav_frame = rospy.get_param( 'uav_{}_home'.format(ID))
        uav_frame['translation'] = position
        if model == "typhoon_h480":
            yaw = yaw + np.pi
        uav_frame['gz_initial_yaw'] =  yaw # radianes
        uav_frame['model'] = model
        rospy.set_param('uav_{}_home'.format(ID), uav_frame)

    def UAVSpawner(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,[\
            "/home/joseandres/catkin_ws/src/jamrepo/gauss/launch/test_server_spawner_JA.launch"])
        launch.start()


        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # cli_args = ['gauss', 'test_server_spawner_JA.launch', "multi_2:=true"]
        # roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
        # roslaunch_args = cli_args[2:]
        # parent = roslaunch.parent.ROSLaunchParent(uuid, [(roslaunch_file, roslaunch_args)])
        # parent.start()

    def CreatingSimulationDataStorage(self):
        first_folder_path = "/home/joseandres/catkin_ws/src/jamrepo/Simulation_data/{0}/type{1}_Nuav{2}_Nobs{3}".format(self.project,self.world_type,self.N_uav,self.N_obs)
        if not os.path.exists(first_folder_path):
            os.makedirs(first_folder_path)

        second_folder_path = first_folder_path + "/dataset_{}".format(self.n_dataset)
        if not os.path.exists(second_folder_path):
            os.makedirs(second_folder_path)   

        third_folder_path = second_folder_path + "/simulation_{}".format(self.n_simulation)
        if not os.path.exists(third_folder_path):
            os.makedirs(third_folder_path)
        
        self.world_definition = rospy.get_param('world_definition')
        file_path = third_folder_path + '/world_definition.csv'
        with open(file_path, 'wb') as f:
            w = csv.DictWriter(f, self.world_definition.keys())
            w.writeheader()
            w.writerow(self.world_definition)

        bag_folder_path = "/home/joseandres/catkin_ws/src/jamrepo/Simulation_data/{0}/type{1}_Nuav{2}_Nobs{3}/dataset_{4}/simulation_{5}/tf_bag.bag".format(self.project,self.world_type,self.N_uav,self.N_obs,self.n_dataset,self.n_simulation)
        self.bag = rosbag.Bag(bag_folder_path, 'w')

    def Die(self):
        self.UAVKiller()
        self.world.EraseAllObstacles()
        self.GazeboModelsKiller()
        self.SimulationTerminationCommand()
        # rospy.signal_shutdown("end of experiment")

    def UAVKiller(self):
        for n_uav in np.arange(self.N_uav):
            rospy.wait_for_service('/gauss/ANSP_UAV_{}/die_command'.format(n_uav+1))
            try:
                die_command = rospy.ServiceProxy('/gauss/ANSP_UAV_{}/die_command'.format(n_uav+1), DieCommand)
                die_command(True)
                time.sleep(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        return

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
        return

    #### listener functions ####
    def ANSPListener(self):
        rospy.Service('/gauss/ANSP/State_Actualization', StateActualization, self.handle_uav_status)
        rospy.Subscriber('/tf_static', TFMessage, self.tf_static_callback)
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)

    def handle_uav_status(self,data):
        self.states_list[data.ID-1] = data.state
        if data.collision == True:
            message = "uav",data.ID,"collisioned"
            rospy.loginfo_throttle(0, message)
            # self.Die()
        print self.states_list
        rospy.loginfo_throttle(0,self.states_list)
        return True

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