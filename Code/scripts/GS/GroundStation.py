#!/usr/bin/env python2
# -*- coding: utf-8 -*-

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
import json
import copy
import rospkg
from std_msgs.msg import Int32, String
from uav_abstraction_layer.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from gazebo_msgs.srv import DeleteModel
from tf2_msgs.msg import *
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from actionlib import SimpleActionClient

from magna.srv import *
from magna.msg import *

from GroundStation_SM import GroundStation_SM
from Various import serverClient
# from Worlds import *

class GroundStation(object):
    def __init__(self):
        # Global parameters inizialization.
        self.GetHyperparameters()

        self.home_path = rospkg.RosPack().get_path('magna')[:-5]

        ### Initializations

        self.simulation_succeed = True      # Initialize succeed flag of this single simulation
        self.agent_spawner_launchs_list = []

        mission_def_path = "{0}/Code/JSONs/Missions/{1}/{2}.json"\
                            .format(self.home_path,self.mission_name,self.submission_name)
        with open(mission_def_path) as f:
            self.mission_def = json.load(f)

        self.N_agents = len(self.mission_def["Agents_Config"])
        self.hyperparameters["N_agents"] = self.N_agents
        rospy.set_param('magna_hyperparameters', self.hyperparameters)

        # Start by landed the state of every Agent
        self.states_list = []
        for i in np.arange(self.N_agents):
            self.states_list.append("landed")

        # Start by not collisioned the the list within all Agents
        self.critical_events_list = []
        for i in np.arange(self.N_agents):
            self.critical_events_list.append('nothing')

        self.CreatingSimulationDataStorage()

        rospy.init_node('ground_station', anonymous=True)     # Start node

        self.agent_models = []
        self.utm_flightplan_list = []
        for N_agents in self.mission_def["Agents_Config"]:
            self.agent_models.append(N_agents["model"])
            self.utm_flightplan_list.append([])

        self.hyperparameters["agent_models"] = self.agent_models
        
        rospy.set_param('magna_hyperparameters', self.hyperparameters)

        self.Listener()     # Start subscribers

        self.gs_sm = GroundStation_SM(self)        # Create Ground Station State Machine
        outcome = self.gs_sm.gs_sm.execute()        # Execute State Machine

        self.Die()      # Once out of the State Machine, execute al commands of closig
        # self.bag.close()        # Close the rosbag that stores the simulation


    #### Starter functions ####

    def SpawnAgents(self,initial_poses):

        for i in range(self.N_agents):     # Do it for every Agent
            # first_pose = agent_goal_path[0]       # Select first waypoint. It will determinate where it spawns
            # first_pose = self.world.getFSPoseGlobal(initial_poses[i])
            first_pose = self.worldGetFSPset(initial_poses[i])[0]

            if len(initial_poses[i]) > 4:
                yaw = initial_poses[i][4]

            else:
                # Change spawn features so it spawns under its first waypoint
                yaw = tf.transformations.euler_from_quaternion((first_pose.orientation.x,
                                                            first_pose.orientation.y,
                                                            first_pose.orientation.z,
                                                            first_pose.orientation.w))[2]
            self.SetAgentSpawnFeatures(i+1,self.agent_models[i],[first_pose.position.x,first_pose.position.y,0],yaw)
            time.sleep(2)
            self.AgentSpawner(i,self.agent_models[i])      # Call service to spawn the Agent

            # Wait until the Agent is in ready state
            while not rospy.is_shutdown() and self.states_list[i] != "waiting for action command":
                time.sleep(0.5)


            print("Ground Station: agent {} is ready!".format(i))
            # time.sleep(5)
        # time.sleep(20 * heritage.N_agents)

        return "completed"

    # Function to set Agent's ROSparameters. Launched by State Machine
    def SetAgentSpawnFeatures(self,ID,model,position,yaw=0):
        # agent_frame = rospy.get_param( 'agent_{}_home'.format(ID))      # Read from ROS param the home position
        agent_frame = {}
        agent_frame['translation'] = position
        # if model == "typhoon_h480":     # If typhoon, change yaw ????? CHANGE FOR ALL. Updated on UAL
        #     yaw = 0.5
        agent_frame['gz_initial_yaw'] =  yaw # radians
        agent_frame['model'] = model      # Actualize on received param info
        rospy.set_param('uav_{}_home'.format(ID), agent_frame)        # Set the modified ROS param

        rospy.set_param('uav_{}/ual/home_pose'.format(ID),position)

        rospy.set_param('uav_{}/ual/pose_frame_id'.format(ID),"map")

        # if model == "crazyflie":

        #     cf_yaml_data = {"crazyflies":[{"id":ID,"channel":80,"initialPosition":position, "type":"default"}]}
        #     crazyswarm_path = rospkg.RosPack().get_path('crazyswarm')[:-5]
        #     cf_yaml_path = "{0}/launch/crazyflies_magna.launch".format(crazyswarm_path)

        #     # Write YAML file
        #     with io.open(cf_yaml_path, 'w', encoding='utf8') as outfile:
                # yaml.dump(cf_yaml_data, outfile, default_flow_style=False, allow_unicode=True)

        time.sleep(1)

    # Function to spawn each new Agent (GAZEBO model, UAL server and dedicated Ground Station)
    def AgentSpawner(self,ID, agent_model):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        world_config_path = "{0}/Code/JSONs/Worlds/{1}/{2}.json"\
                            .format(self.home_path,self.world_name,self.subworld_name)
        
        with open(world_config_path) as f:
            world_config = json.load(f)

        if "origin_geo" in world_config["scenario"].keys():
            origin_geo = world_config["scenario"]["origin_geo"]

        else:
            origin_geo = [37.558542, -5.931074, 7.89]

        launch_path = "{0}/Code/launch/complete_spawner_JA.launch".format(self.home_path)

        et = xml.etree.ElementTree.parse(launch_path)
        root = et.getroot()

        config_def_path = "{0}/Code/JSONs/Agent_Configurations/{1}.json"\
                            .format(self.home_path,self.mission_def["Agents_Config"][ID]["model"])
        with open(config_def_path) as f:
            config_def = json.load(f)

        config_def.update(copy.deepcopy(self.mission_def["Agents_Config"][ID]))

        # sitl
        root[2].attrib["default"] = config_def["mode"]

        # ual use
        if config_def["ual_use"] == True:
            root[7].attrib["default"] = 'true'
        else:
            root[7].attrib["default"] = 'false'

        # autopilot
        root[8].attrib["default"] = config_def["autopilot"]

        # agent manager embedded
        if config_def["agent_manager_on_gs"] == True:
            root[9].attrib["default"] = 'true'
        else:
            root[9].attrib["default"] = 'false'

        # smooth path
        if ID+1 == 1:
            root[10].attrib["default"] = 'true'
        else:
            root[10].attrib["default"] = 'false'

        # if sitl
        # root[11][0][0][0].attrib["ns"] = '$(arg ns_prefix){}'.format(ID+1)
        root[11].attrib["ns"] = '$(arg ns_prefix){}'.format(ID+1)
        root[11][0][0][0][0].attrib["value"] = str(ID+1)
        # root[11][0][1][0].attrib["ns"] = '$(arg ns_prefix){}'.format(ID+1)

        # if ual use
        # root[11][1][0][0].attrib["ns"] = '$(arg ns_prefix){}'.format(ID+1)
        root[11][1][0][0][0].attrib["value"] = str(ID+1)


        if config_def["autopilot"] == "px4":
            if agent_model == "plane":
                root[11][1][0][0][6].attrib["value"] = "mavros_fw"
            else:
                root[11][1][0][0][6].attrib["value"] = "mavros"

            root[11][1][0][0][7].text = str(origin_geo)

        elif config_def["autopilot"] == "dji":

            root[11][1][1][0][3].text = str(origin_geo)

            if config_def["laser_altimeter"] == True:
                root[11][1][1][0][1].attrib["value"] = 'true'
            else:
                root[11][1][1][0][1].attrib["value"] = 'false'

            if config_def["self_arming"] == True:
                root[11][1][1][0][2].attrib["value"] = 'true'
            else:
                root[11][1][1][0][2].attrib["value"] = 'false'

            # root[12][0][7].attrib["value"] = config_def["laser_altimeter"]
            # root[12][0][8].attrib["value"] = config_def["self_arming"]

        # agent manager
        root[12][0].attrib["name"] = 'agent_{}'.format(ID+1)
        root[12][0].attrib["args"] = '-ID={}'.format(ID+1)

        et.write(launch_path)

        agent_spawner_launch = roslaunch.parent.ROSLaunchParent(uuid,[launch_path])
        agent_spawner_launch.start()

        self.agent_spawner_launchs_list.append(agent_spawner_launch)


    def worldConfig(self, parameters):

        if parameters["action"] == "spawn":

            self.worldSpawn(parameters["id"])

        elif parameters["action"] == "add":

            self.worldAdd(parameters["world_part_def"])

        elif parameters["action"] == "delete":

            self.worldKiller(parameters["id"])

        return "completed"

    def worldSpawn(self, id):

        # Local parameters inizialization
        # self.world = Worlds(1)      # Create a World of id 1
        self.GetHyperparameters()       # Get ROS param of definitions

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_path = "{0}/Code/launch/World_spawner_JA.launch".format(self.hyperparameters['home_path'])

        et = xml.etree.ElementTree.parse(launch_path)
        root = et.getroot()
        if self.rviz_gui == True:
            root[1].attrib["if"] = "true"
            root[1][0].attrib["args"] = "-d $(find magna)/rviz/{0}.rviz".format(self.hyperparameters["world"])
        else:
            root[1].attrib["if"] = "false"
        et.write(launch_path)

        self.world_spawner_launch = roslaunch.parent.ROSLaunchParent(uuid,[launch_path])
        self.world_spawner_launch.start()

        time.sleep(2)


    def worldAdd(self, world_part_definition):

        if world_part_definition == "from_hyperparameters":
            world_part_definition = [{ "name" : "JSON", "world" : self.world_name, "subworld" : self.subworld_name}]

        req = InstructionCommandRequest()
        req.instruction = json.dumps(world_part_definition)

        response = serverClient(req, "/magna/Worlds/add", InstructionCommand)

        return response


    def worldGetFSPset(self,fspset_path):

        req = WorldGetFSPsetRequest()
        req.fspset_path = json.dumps(fspset_path)

        response = serverClient(req, "/magna/Worlds/get_fspset", WorldGetFSPset)

        return response.fspset

    def worldKiller(self,id):

        req = InstructionCommandRequest()
        req.instruction = "die"

        response = serverClient(req, "/magna/Worlds/die", InstructionCommand)

        self.world_spawner_launch.shutdown()


    # Function to create folders and file to store simulations data
    def CreatingSimulationDataStorage(self):

        # Definition of root path
        first_folder_path = "{0}/Data_Storage/Simulations/{1}/{2}/{3}/{4}"\
                                 .format(self.home_path,self.world_name,self.subworld_name,
                                 self.mission_name,self.submission_name)

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
        bag_folder_path = self.third_folder_path + "/everything.bag"
        
        if self.rosbag_flag:
            self.bag = rosbag.Bag(bag_folder_path, 'w')


    def MakePath(self,path_def,agent_id):

        latency_pub = rospy.Publisher('/magna/latency/{}'.format(agent_id), PoseStamped, queue_size=1)
        latency_msg = PoseStamped()
        latency_msg.pose.position.x = 1
        response = latency_pub.publish(latency_msg)
        time.sleep(1)

        # return [self.world.getFSPoseGlobal(path_def)]
        return [self.worldGetFSPset(path_def)]

    def wait(self,exit_type,duration=1):
        if exit_type == "time":
            time.sleep(duration)
        elif exit_type == "button":
            button = raw_input("Wait State asks for a button press")

        return "completed" 

    #### Finisher functions ####

    # Function to close active child processess
    def Die(self):
        self.SavingWorldDefinition()        # Update ROS params
        self.AgentKiller()        # Terminate Agents nodes
        self.GazeboModelsKiller()       # Delete robot models from Gazebo
        # self.world.eraseAllObstacles()      # Delete obstacles from Gazebo
        self.SimulationTerminationCommand()     # Send to Master message of simulation ended
        rospy.signal_shutdown("end of experiment")      # Finish Ground Station process


    # Function to update ROS parameters about simulation performance and store them
    def SavingWorldDefinition(self):
        self.hyperparameters = rospy.get_param('magna_hyperparameters')
        self.hyperparameters['criticals_event_list'] = self.critical_events_list
        self.hyperparameters['simulation_succeed'] = self.simulation_succeed
        rospy.set_param('magna_hyperparameters',self.hyperparameters)
        file_path = self.third_folder_path + '/world_definition.csv'
        with open(file_path,'wb') as f:
            w = csv.writer(f)
            w.writerows(self.hyperparameters.items())

    # Function to close Agent Ground Station process.In the furure would be done by GS
    def AgentKiller(self):
        
        self.sendNotificationsToAgents(range(1,self.N_agents+1),"die")

        for agent_launch in self.agent_spawner_launchs_list:
            agent_launch.shutdown()

        return

    # Function to close GAZEBO process
    def GazeboModelsKiller(self):
        for n_agent in np.arange(self.N_agents):
            model_name = "{0}_{1}".format(self.agent_models[n_agent],n_agent+1)

            response = serverClient(model_name, 'gazebo/delete_model', DeleteModel)
            time.sleep(0.1)

        return


    # Function to send termination instruction to each Agent
    def SimulationTerminationCommand(self):

        response = serverClient(True, '/magna/GS/simulation_termination', DieCommand)

        return


    def AlgorithmControlCommand(self,agents_list,name,action,params,values):

        if type(agents_list) == int:
            agents_list = [agents_list]

        for agent in agents_list:

            request = AlgorithmControlRequest()
            request.name = name
            request.params = params
            request.action = action
            request.values = values

            response = serverClient(request, '/magna/GS_Agent_{}/algorithm_control'.format(agent), AlgorithmControl)

        return "completed"

        

    #### Commander functions ####

    def sendNotificationsToAgents(self,agents_list,message):
        if type(agents_list) == int:
            agents_list = [agents_list]

        msg = InstructionCommandRequest()
        
        for agent in agents_list:

            msg.instruction = message
            print(agent,msg)

            response = serverClient(msg, '/magna/GS_Agent_{}/notification'.format(agent), InstructionCommand)

        return "completed"

    #### listener functions ####

    # Function to start subscribing and offering
    def Listener(self):

        # Start service for Agents to actualize its state
        rospy.Service('/magna/GS/state_actualization', StateActualization, self.handle_agent_status)
        rospy.Service('/magna/gyal_GS/command', GyalCommand, self.handle_gyal_command)
        rospy.Service('/magna/utm/notification', UTMnotification, self.utm_notification_command)

        # Start listening to topics stored on rosbag
        if self.rosbag_flag:
            rospy.Subscriber('/tf_static', TFMessage, self.tf_static_callback)
            rospy.Subscriber('/tf', TFMessage, self.tf_callback)

        # for n_agent in range(self.N_agents):
        #     rospy.Subscriber('/magna/agent_{0}/path'.format(n_agent+1), Path, self.path_callback,'/magna/agent_{0}/path'.format(n_agent+1))
        #     rospy.Subscriber('/magna/agent_{0}/goal_path'.format(n_agent+1), Path, self.path_callback,'/magna/agent_{0}/goal_path'.format(n_agent+1))
        #     rospy.Subscriber('/magna/agent_{0}/goal_path_smooth'.format(n_agent+1), Path, self.path_callback,'/magna/agent_{0}/goal_path_smooth'.format(n_agent+1))

        # rospy.Subscriber('/visualization_marker', Marker, self.visualization_marker_callback)

    # Function to update and local parameters of Agents status
    def handle_agent_status(self,data):

        # Store Agent status into Agents state list
        self.states_list[data.id-1] = data.state

        # Check collisions to update it Agents collision list
        if data.critical_event != 'nothing':
            
            if self.critical_events_list[data.id-1] != data.critical_event:
                self.critical_events_list[data.id-1] = data.critical_event
                rospy.loginfo_throttle(0,"Agent {0} reported {1}".format(data.id,data.critical_event))
            
                if data.critical_event == "collision":
                    agents_to_inform = range(1,self.N_agents+1)
                    agents_to_inform = agents_to_inform[:data.id-1] + agents_to_inform[data.id:]

                    # Change succeed information if a collision has been reported
                    self.simulation_succeed = False

                    self.sendNotificationsToAgents(agents_to_inform,"GS_critical_event")

        print self.states_list
        return True

    def handle_gyal_command(self,req):

        params_dicc = {}

        response = GyalCommandResponse()

        for i, param in enumerate(req.params):

            if req.values_str[i] != "from_integer":
                params_dicc[param] = req.values_str[i]
            else:
                params_dicc[param] = req.values_int[i]

        if req.intent == "parameters":
            if req.action == "get":
                if params_dicc["parameter"] == "state":

                    agent_id = int(req.agent_id) -1 

                    response.result = [self.states_list[agent_id],self.critical_events_list[agent_id]]

                    return response
                
                    
        if req.intent == "mission":
            if req.action == "set":
                self.sendNotificationsToAgents(req.agent_id,params_dicc["action"])

                return ["ok"]

        if req.intent == "basic_action":
            if req.action == "set":

                self.generic_action_client(req.agent_id,params_dicc["action_type"],params_dicc)

                return ["ok"]

        if req.intent == "algorithm":
            if req.action == "set":
                pass


    def utm_notification_command(self,req):

        instruction = json.loads(req.instruction)

        if instruction["action"] == "new flightplan":

            self.utm_flightplan_list[instruction["ids"][0]-1] = req.goal_path_poses_list
            self.sendNotificationsToAgents(instruction["ids"],"utm_new_flightplan")

        elif instruction["action"] == "new world part":

            self.worldConfig(instruction["world_part_def"])

        response = UTMnotificationResponse()
        response.success = True
        return response


    def generic_action_client(self,agent_id,name,params):

        if len(name.split(" ")) > 1: name = "_".join(name.split(" "))

        client = SimpleActionClient('magna/GS_Agent_{0}/{1}_command'.format(agent_id,name), self.gs_sm.SASMsgTypeDic[name])
        client.wait_for_server()

        goal = self.gs_sm.SASGoalCBDic[name]([], self.gs_sm.SASGoalDic[name](), params)        # get goal
        print(goal)
        client.send_goal(goal)

        # client.wait_for_result()

        # return client.get_result()

        return


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
    def GetHyperparameters(self):
        self.hyperparameters = rospy.get_param('magna_hyperparameters')
        self.mission_name = self.hyperparameters['mission']
        self.submission_name = self.hyperparameters['submission']
        self.world_name = self.hyperparameters['world']
        self.subworld_name = self.hyperparameters['subworld']
        self.n_simulation = self.hyperparameters['n_simulation']
        self.n_dataset = self.hyperparameters['n_dataset']
        self.smach_view = self.hyperparameters['smach_view']
        self.depth_camera_use = self.hyperparameters['depth_camera_use']
        self.rosbag_flag = self.hyperparameters['rosbag_flag']
        self.rviz_gui = self.hyperparameters['rviz_gui']

def main():
    GroundStation()

if __name__ == '__main__':
    main()