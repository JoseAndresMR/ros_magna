#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import pandas as pd
from geometry_msgs.msg import *
import sys
import os
from ast import literal_eval

df = pd.read_csv("/home/joseandresmr/catkin_ws/src/magna/Data_Storage/Simulations/Delivery/Delivery/Delivery/2UAVs_follow_ap_safe/dataset_1/simulation_1/agent_1.csv")

print(literal_eval(df["goal"]))