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

# import pyModeS as pms
import numpy as np
import rospy, time, tf, tf2_ros
import json
import rospkg
from copy import deepcopy

class Agent_Config(object):
    def __init__(self,ID):

        self.ID = ID

        self.GettingWorldDefinition()

        self.home_path = rospkg.RosPack().get_path('magna')[:-5]

        mission_def_path = "{0}/Code/JSONs/Missions/{1}/{2}.json"\
                            .format(self.home_path,self.mission_name,self.submission_name)
        with open(mission_def_path) as f:
            mission_def = json.load(f)

        self.model = mission_def["Agents_Config"][self.ID-1]["model"]
        

        config_def_path = "{0}/Code/JSONs/Agent_Configurations/{1}.json"\
                            .format(self.home_path,self.model)
        with open(config_def_path) as f:
            self.config_def = json.load(f)

        self.config_def.update(mission_def["Agents_Config"][self.ID-1])

        self.autopilot = self.config_def["autopilot"]
        self.ual_use = bool(self.config_def["ual_use"])
        self.safety_radius = self.config_def["safety_radius"]
        self.accepted_target_radius = self.config_def["accepted_target_radius"]
        self.max_speed = self.config_def["max_speed"]
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

        # if self.ual_use == True:
        self.ual()

        if self.model == "iris":
            self.model_iris()
            
        elif self.model == "crazyflie":
            self.model_crazyflie()

    
    def autupilot_px4(self):

        self.top_sub_addr["battery_level"] = '/uav_{}/mavros/battery2'.format(self.ID)

    def autupilot_dji(self):
        self.top_sub_addr["battery_level"] = '/uav_{}/dji/battery'.format(self.ID)

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
        self.ser_cli_addr['set_mission'] = '/uav_{}/ual/set_mission'.format(self.ID)


    def model_iris(self):
        # self.safety_radius = 0.3
        pass

    def model_crazyflie(self):
        # self.safety_radius = 0.1
        pass


    # Function to get Global ROS parameters
    def GettingWorldDefinition(self):
        self.hyperparameters = rospy.get_param('magna_hyperparameters')
        self.mission_name = self.hyperparameters['mission']
        self.submission_name = self.hyperparameters['submission']
        self.world_name = self.hyperparameters['world']
        self.subworld_name = self.hyperparameters['subworld']

