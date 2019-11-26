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
    [38.139037, -3.173351,0.0]
]

polygons_geo["Flying_Zone"] = [
    [38.138530, -3.176771,0.0],
    [38.138378, -3.176707,0.0],
    [38.139185, -3.173550,0.0],
    [38.139137, -3.173444,0.0],
    [38.139005, -3.173387,0.0],
    [38.138909, -3.173461,0.0],
    [38.138985, -3.173160,0.0],
    [38.139027, -3.173293,0.0],
    [38.139154, -3.173346,0.0],
    [38.139249, -3.173292,0.0],
    [38.140054, -3.170147,0.0],
    [38.140207, -3.170206,0.0]
]

polygons_geo["Edificios"] = [
    [38.138540, -3.173743,0.0],
    [38.138678, -3.173226,0.0],
    [38.138759, -3.172860,0.0]
]

polygons_geo["Take_offs"] = [
    [38.138996, -3.172971, 0.0],
    [38.139233, -3.173657,0.0],
    [38.139349, -3.173215,0.0]
]

polygons_geo["Safe_perimeter"] = [
    [38.138755, -3.174071, 0.0],
    [38.138343, -3.173909,0.0],
    [38.138633, -3.172745,0.0],
    [38.139040, -3.172934,0.0]
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
