#!/usr/bin/env python

# import pyModeS as pms
import numpy as np
import rospy, time, tf, tf2_ros
from copy import deepcopy

class UAV_Config(object):
    def __init__(self,ID,model,autopilot,ual_use):

        self.ID = ID
        self.model = model
        self.autopilot = autopilot
        self.ual_use = ual_use

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

    
    def autupilot_px4(self):

        self.top_sub_addr["battery_level"] = '/uav_{}/mavros/battery'.format(self.ID)

    def autupilot_dji(self):
        pass

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