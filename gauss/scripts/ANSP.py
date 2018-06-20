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
from gauss.srv import *

class ANSP(object):
    def __init__(self):
        self.GettingWorldDefinition()

        self.simulation_succeed = True

        self.states_list = []
        for i in np.arange(self.N_uav):
            self.states_list.append("landed")

        self.collisioned_list = []
        for i in np.arange(self.N_uav):
            self.collisioned_list.append(False)

        rospy.init_node('ansp', anonymous=True)
        
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
            time.sleep(0.5)

        self.bag.close()


    #### Commander functions ####
    def Dcdaa(self):
        self.CreatingSimulationDataStorage()
        self.world = Worlds(1)
        self.GettingWorldDefinition()
        uavs_goal_paths_list = []
        for n_uav in np.arange(self.N_uav):
            uav_goal_path = self.world.PathGenerator()
            uavs_goal_paths_list.append(uav_goal_path)
            first_pose = uav_goal_path[0]
            self.SetUavSpawnFeatures(n_uav+1,self.uav_models[n_uav],[first_pose.position.x,first_pose.position.y,0])       
 
        self.UAVSpawner()

        for n_uav in np.arange(self.N_uav):
            self.PathCommand(n_uav+1,uavs_goal_paths_list[n_uav])
        time.sleep(30)


    def Gauss(self):
        self.CreatingSimulationDataStorage()
        self.world = Worlds(1)
        self.GettingWorldDefinition()
        uavs_goal_paths_list = []
        for n_uav in np.arange(self.N_uav):
            uav_goal_path = self.world.PathGenerator()
            uavs_goal_paths_list.append(uav_goal_path)
            first_pose = uav_goal_path[0]
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
                    time.sleep(2)
                    for j in np.arange(self.N_uav):
                        self.InstructionCommand(j+1,"GoOn")
                    all_in_place = True
                    counter = 0
                time.sleep(0.3)

    def PathCommand(self,ID,goal_path_poses_list):
        rospy.wait_for_service('/gauss/ANSP_UAV_{}/wp_list_command'.format(ID))
        try:
            ual_path_command = rospy.ServiceProxy('/gauss/ANSP_UAV_{}/wp_list_command'.format(ID), WpPathCommand)
            ual_path_command(np.array(goal_path_poses_list),False)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in wp_list_command"

    def InstructionCommand(self,ID,Instruction):
        rospy.wait_for_service('/gauss/ANSP_UAV_{}/instruction_command'.format(ID))
        try:
            instruction_command = rospy.ServiceProxy('/gauss/ANSP_UAV_{}/instruction_command'.format(ID), InstructionCommand)
            instruction_command(Instruction)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in instruction_command"

    def SimulationTerminationCommand(self):
        rospy.wait_for_service('/gauss/ANSP/simulation_termination')
        try:
            instruction_command = rospy.ServiceProxy('/gauss/ANSP/simulation_termination', DieCommand)
            instruction_command(True)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in simulation_termination"

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
        self.uav_spawner_launch = roslaunch.parent.ROSLaunchParent(uuid,[\
            "/home/{1}/catkin_ws/src/jamrepo/gauss/launch/{0}_spawner_JA.launch".format(self.world_definition['px4_use'],self.home_path)])
        self.uav_spawner_launch.start()

        # time.sleep(0.2)
        # temp_dir = utils.temp_dir(None)
        # subprocess.call("mkdir -p " + temp_dir, shell=True)
        # self.uavspawner_out = open(temp_dir + '/uavspawner.out', 'w')
        # self.uavspawner_err = open(temp_dir + '/uavspawner.err', 'w')

        # args = "roslaunch gauss {0}_spawner_JA.launch multi_{1}:='true'".format(self.world_definition['px4_use'],self.world_definition['N_uav'])
        # self.uavspawner_subprocess = subprocess.Popen(args, stdout=screen, stderr=screen, cwd=temp_dir, shell=True, preexec_fn=os.setsid)

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

    def CreatingSimulationDataStorage(self):
        if self.project == 'dcdaa':
            N_obs_mixed = int('{0}{1}{2}'.format(self.obs_tube[0],self.obs_tube[1],self.obs_tube[2]))
        elif self.project == 'gauss':
            N_obs_mixed = self.N_obs
        first_folder_path = "/home/{4}/catkin_ws/src/jamrepo/Data_Storage/Simulations/{0}/{5}/type{1}_Nuav{2}_Nobs{3}".format(self.project,self.world_type,self.N_uav,N_obs_mixed,self.home_path,self.solver_algorithm)

        if not os.path.exists(first_folder_path):
            os.makedirs(first_folder_path)

        second_folder_path = first_folder_path + "/dataset_{}".format(self.n_dataset)
        if not os.path.exists(second_folder_path):
            os.makedirs(second_folder_path)   

        self.third_folder_path = second_folder_path + "/simulation_{}".format(self.n_simulation)
        if not os.path.exists(self.third_folder_path):
            os.makedirs(self.third_folder_path)

        bag_folder_path = "/home/{6}/catkin_ws/src/jamrepo/Data_Storage/Simulations/{0}/{7}/type{1}_Nuav{2}_Nobs{3}/dataset_{4}/simulation_{5}/tf_bag.bag".format(self.project,self.world_type,self.N_uav,N_obs_mixed,self.n_dataset,self.n_simulation,self.home_path,self.solver_algorithm)
        self.bag = rosbag.Bag(bag_folder_path, 'w')

    def SavingWorldDefinition(self):
        self.world_definition = rospy.get_param('world_definition')
        self.world_definition['collisioned_list'] = self.collisioned_list
        self.world_definition['simulation_succeed'] = self.simulation_succeed
        rospy.set_param('world_definition',self.world_definition)
        file_path = self.third_folder_path + '/world_definition.csv'
        with open(file_path, 'wb') as f:
            w = csv.DictWriter(f, self.world_definition.keys())
            w.writeheader()
            w.writerow(self.world_definition)

    def Die(self):
        self.SavingWorldDefinition()
        self.UAVKiller()
        self.world.eraseAllObstacles()
        self.GazeboModelsKiller()
        self.SimulationTerminationCommand()
        rospy.signal_shutdown("end of experiment")

    def HandleException(self):
        # self.uav_spawner_launch.shutdown()
        time.sleep(1)
        if self.uavspawner_subprocess.poll() is None:
            os.killpg(os.getpgid(self.uavspawner_subprocess.pid), signal.SIGTERM)
        self.uavspawner_out.close()
        self.uavspawner_err.close()

    def UAVKiller(self):
        for n_uav in np.arange(self.N_uav):
            rospy.wait_for_service('/gauss/ANSP_UAV_{}/die_command'.format(n_uav+1))
            try:
                die_command = rospy.ServiceProxy('/gauss/ANSP_UAV_{}/die_command'.format(n_uav+1), DieCommand)
                die_command(True)
                time.sleep(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                print "error in die_command"
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
                print "error in delete_model uav"
        return

    #### listener functions ####
    def ANSPListener(self):
        rospy.Service('/gauss/ANSP/state_actualization', StateActualization, self.handle_uav_status)
        rospy.Subscriber('/tf_static', TFMessage, self.tf_static_callback)
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)

    def handle_uav_status(self,data):
        self.states_list[data.id-1] = data.state
        if data.collision == True:
            if self.collisioned_list[data.id-1] == False:
                self.collisioned_list[data.id-1] == True
            # self.Die()
            self.simulation_succeed = False
            rospy.loginfo_throttle(0,"Simulation not succeded, UAV {} collisioned".format(data.id))
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
    # try:
    ANSP()
    # except KeyboardInterrupt:
    #     rospy.signal_shutdown("end of experiment")
    #     # ANSP.HandleException()
    # except rospy.is_shutdown:
    #     rospy.signal_shutdown("end of experiment")
    #     # ANSP.HandleException()

if __name__ == '__main__':
    main()