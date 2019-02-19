#!/usr/bin/env python

# import pyModeS as pms
import numpy as np
import rospy, time, tf, tf2_ros
import json
import rospkg
from copy import deepcopy

class UAV_Config(object):
    def __init__(self,ID):

        self.ID = ID

        self.GettingWorldDefinition()

        self.home_path = rospkg.RosPack().get_path('pydag')[:-5]

        mission_def_path = "{0}/Code/JSONs/Missions/{1}/{2}.json"\
                            .format(self.home_path,self.mission_name,self.submission_name)
        with open(mission_def_path) as f:
            mission_def = json.load(f)

        self.model = mission_def["UAVs_Config"][self.ID-1]["model"]
        

        config_def_path = "/home/{0}/catkin_ws/src/pydag/Code/JSONs/UAV_Configurations/{1}.json"\
                            .format(self.home_path,self.model)
        with open(config_def_path) as f:
            self.config_def = json.load(f)

        self.config_def.update(mission_def["UAVs_Config"][self.ID-1])

        self.autopilot = self.config_def["autopilot"]
        self.ual_use = bool(self.config_def["ual_use"])
        self.security_radius = self.config_def["security_radius"]
        self.mode = self.config_def["mode"]
        self.marker_color = self.config_def["marker_color"]

        self.top_sub_addr = {}
        self.top_pub_addr = {}
        self.ser_ser_addr = {}
        self.ser_cli_addr = {}

        if self.autopilot == "px4":
            self.autupilot_px4()

        elif self.autopilot == "dji":
            self.autupilot_dji()

        elif self.autopilot == "crazy":
            self.autupilot_crazy()

        if self.ual_use == True:
            self.ual()

        if self.model == "iris":
            self.model_iris()
            
        elif self.model == "crazyflie":
            self.model_crazyflie()

    
    def autupilot_px4(self):

        self.top_sub_addr["battery_level"] = '/uav_{}/mavros/battery'.format(self.ID)

    def autupilot_dji(self):
        self.top_sub_addr["battery_level"] = '/uav_{}/mavros/battery'.format(self.ID)

    def autupilot_crazy(self):
        pass

    def ual(self):

        self.top_sub_addr['pose'] = '/uav_{}/ual/pose'.format(self.ID)
        self.top_sub_addr['velocity'] = '/uav_{}/ual/velocity'.format(self.ID)
        self.top_sub_addr['state'] = '/uav_{}/ual/state'.format(self.ID)

        self.top_pub_addr['go_to_waypoint'] = '/uav_{}/ual/go_to_waypoint'.format(self.ID)
        self.top_pub_addr['set_velocity'] = '/uav_{}/ual/set_velocity'.format(self.ID)

        self.ser_cli_addr['take_off'] = '/uav_{}/ual/take_off'.format(self.ID)
        self.ser_cli_addr['land'] = '/uav_{}/ual/land'.format(self.ID)
        self.ser_cli_addr['set_home'] = '/uav_{}/ual/set_home'.format(self.ID)


    def model_iris(self):
        # self.security_radius = 0.3
        pass

    def model_crazyflie(self):
        # self.security_radius = 0.1
        pass


    # Function to get Global ROS parameters
    def GettingWorldDefinition(self):
        self.world_definition = rospy.get_param('world_definition')
        self.mission_name = self.world_definition['mission']
        self.submission_name = self.world_definition['submission']
        self.world_name = self.world_definition['world']
        self.subworld_name = self.world_definition['subworld']

