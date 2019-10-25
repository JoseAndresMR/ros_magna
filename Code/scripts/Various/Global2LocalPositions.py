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
polygons_geo["Origin"] = [
    [37.199941, -5.881098,0.0]
]

polygons_geo["Flying_Zone"] = [
    [37.200925, -5.881467,0.0],
    [37.200812, -5.881150,0.0],
    [37.200692, -5.881115,0.0],
    [37.200634, -5.881145,0.0],
    [37.200591, -5.881094,0.0],
    [37.200373, -5.880995,0.0],
    [37.200311, -5.880928,0.0],
    [37.200290, -5.880822,0.0],
    [37.200251, -5.880501,0.0],
    [37.200251, -5.880501,0.0],
    [37.199892, -5.880068,0.0],
    [37.199834, -5.880246,0.0],
    [37.199885, -5.880941,0.0],
    [37.197510, -5.880538,0.0],
    [37.197488, -5.880775,0.0]
]

polygons_geo["Furgo"] = [
    [37.199833, -5.880922,0.0]
]

polygons_geo["Take_offs"] = [
    [37.199603, -5.881056, 0.0],
    [37.199941, -5.881098,0.0],
    [37.200247, -5.881208,0.0]
]

# polygons_geo["Trial"] = [[37.410372, -6.000350, 0.0],[37.415372, -6.002350, 0.0]]

# polygons_geo["Largo"] = [
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


polygons_local = {}

origin_geo = polygons_geo["Origin"][0]
new_coordinate_geo = GeoPoint(*origin_geo)
limits = [[99999,-99999],[99999,-99999]]

for key in polygons_geo.keys():
    vertex_local_list = []
    for new_geo in polygons_geo[key]:
        new_geo = GeoPoint(*new_geo)
        geo_local_pose = GeoLocalPose(new_geo)
        local_pose_prior = geo_local_pose.geoToUtmToCartesian(new_coordinate_geo)
        local_pose = Point32(-local_pose_prior.x,-local_pose_prior.y,local_pose_prior.z)
        # local_pose = [-local_pose_prior[0],-local_pose_prior[1],local_pose_prior[2]]
        vertex_local_list.append([local_pose.x,local_pose.y,local_pose.z])

        if local_pose.x < limits[0][0]:
            limits[0][0] = local_pose.x
        if local_pose.x > limits[0][1]:
            limits[0][1] = local_pose.x

        if local_pose.y < limits[1][0]:
            limits[1][0] = local_pose.y
        if local_pose.y > limits[1][1]:
            limits[1][1] = local_pose.y

    polygons_local[key] = vertex_local_list

scale_center = [(limits[0][1]+limits[0][0])/2,(limits[1][1]+limits[1][0])/2]
scale_factor = 1.0

print polygons_local
polygons_scaled_local = {}

vertex_local_scaled_list = []
for new_local in polygons_local["Origin"]:
    # Relative to scale center
    new_local[0]-=scale_center[0]
    new_local[1]-=scale_center[1]

    # Scaling
    new_local[0]*=scale_factor
    new_local[1]*=scale_factor

    vertex_local_scaled_list.append(new_local)

polygons_local_aux = copy.deepcopy(polygons_local)

polygons_scaled_local["Origin"] = vertex_local_scaled_list

for key in polygons_local_aux.keys():
    if key != "Origin":
        vertex_local_scaled_list = []
        for new_local in polygons_local_aux[key]:

            # Relative to scale center
            new_local[0]-=scale_center[0]
            new_local[1]-=scale_center[1]

            # Scaling
            new_local[0]*=scale_factor
            new_local[1]*=scale_factor

            # Relative to Origin
            new_local[0]-=polygons_scaled_local["Origin"][0][0]
            new_local[1]-=polygons_scaled_local["Origin"][0][1]

            vertex_local_scaled_list.append(new_local)

        polygons_scaled_local[key] = vertex_local_scaled_list




# print(polygons_local)
# print("limits",limits)
# print("scale_center",scale_center)
# print("unscaled",polygons_local)
