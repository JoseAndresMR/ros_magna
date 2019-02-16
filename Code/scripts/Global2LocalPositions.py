#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 21 2018

@author: josmilrom
"""

import sys
import os
import rospy
import std_msgs.msg
import time
import math
import numpy as np
import pandas as pd
import signal
import copy
import geodesy.utm

from GeoLocalPose import GeoLocalPose
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import *

polygons_geo = {}
polygons_local = {}

polygons_geo["Grupo_Izqueirda"] = [
    [39.333192, -5.357926,0.0],
    [39.333500, -5.357968,0.0],
    [39.333496, -5.358334,0.0],
    [39.335431, -5.358385,0.0],
    [39.335450, -5.358007,0.0],
    [39.335707, -5.358025,0.0],
    [39.335718, -5.356831,0.0],
    [39.334847, -5.356806,0.0],
    [39.334841, -5.357191,0.0],
    [39.334432, -5.357184,0.0],
    [39.334446, -5.356814,0.0],
    [39.333222, -5.356785,0.0]
]

polygons_geo["Grupo_Centro"] = [
    [39.333271, -5.356671,0.0],
    [39.334532, -5.356678,0.0],
    [39.334546, -5.356291,0.0],
    [39.334859, -5.356301,0.0],
    [39.334853, -5.356691,0.0],
    [39.335526, -5.356711,0.0],
    [39.335534, -5.355931,0.0],
    [39.335443, -5.355923,0.0],
    [39.335449, -5.355538,0.0],
    [39.335298, -5.355528,0.0],
    [39.335309, -5.355140,0.0],
    [39.334293, -5.355108,0.0],
    [39.334263, -5.355883,0.0],
    [39.333966, -5.355881,0.0],
    [39.333974, -5.355125,0.0],
    [39.333650, -5.355108,0.0],
    [39.333642, -5.355508,0.0],
    [39.333334, -5.355488,0.0]
]

# polygons_geo["Grupo_Izqueirda"] = [
#     [,0.0],
#     [,0.0],
#     [,0.0],
#     [,0.0],
#     [,0.0],
#     [,0.0],
#     [,0.0],
#     [,0.0],
#     [,0.0],
#     [,0.0],
#     [,0.0],
#     [,0.0]
# ]


origin_geo = [39.332938, -5.357793,0.0]
new_coordinate_geo = GeoPoint(*origin_geo)

for key in polygons_geo.keys():
    vertex_local_list = []
    for new_geo in polygons_geo[key]:
        new_geo = GeoPoint(*new_geo)
        geo_local_pose = GeoLocalPose(new_geo)
        local_pose = geo_local_pose.geoToUtmToCartesian(new_coordinate_geo)
        vertex_local_list.append([local_pose.x,local_pose.y,local_pose.z])

    polygons_local[key] = vertex_local_list

print(polygons_local)


