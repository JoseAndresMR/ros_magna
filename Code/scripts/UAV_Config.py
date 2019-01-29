#!/usr/bin/env python

# import pyModeS as pms
import numpy as np
import rospy, time, tf, tf2_ros
from copy import deepcopy

class UAV_Config(object):
    def __init__(self,ID,main_uav,ICAO = "40621D",with_ref = True,pos_ref = [0,0]):
        