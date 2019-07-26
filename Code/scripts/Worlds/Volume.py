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
import rospy
import std_msgs.msg
import time
import math
import numpy as np
import tf, tf2_ros
import json
import copy
import random
import rospkg
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from xml.dom import minidom
from gazebo_msgs.srv import DeleteModel,SpawnModel
from visualization_msgs.msg import Marker
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray, TorusArray, PolygonArray
from jsk_recognition_msgs.msg import Torus as jsk_Torus
from sympy import Point3D, Line3D, Segment3D
from sympy import Point as Point2D
from sympy import Polygon as Polygon2D
import xml.etree.ElementTree

from magna.srv import *
from Geometries import *

class Volume(object):
    def __init__(self,volume_def,transforms_list):

        self.geometry_classes = {"cube" : Cube, "sphere" : Sphere, "cylinder" : Cylinder,
                                 "prism": Prism, "torus": Torus}
        self.geometries = {}

        self.update(volume_def,transforms_list)


    def update(self,volume_def,transforms_list):

        if "name" in volume_def.keys():
            self.name = volume_def["name"]

        if "prefix" in volume_def.keys():    
            self.prefix = volume_def["prefix"]

        if "permits" in volume_def.keys():
            self.permits = volume_def["permits"]

        if "origin" in volume_def.keys():
            self.origin = volume_def["origin"]

        self.transforms_list = transforms_list
        print("on volume", self.name)
        # print("on volume",len(self.transforms_list))
        self.obs_pose_list = []
        self.fsp_list = []

        tfbroadcaster = StaticTfBroadcaster(self.name,"map",self.origin,self.transforms_list)
        self.transforms_list = tfbroadcaster.getTransforms()
        # print("on volume",len(self.transforms_list))

        for geometry_def in volume_def["geometries"]:

            self.geometries[geometry_def["name"]] = self.geometry_classes[geometry_def["shape"]](geometry_def,self.name,self.prefix,self.transforms_list)
            # self.transforms_list = self.geometries[geometry_def["name"]].getTransforms()
            # print("on volume",len(self.transforms_list))

            if self.geometries[geometry_def["name"]].obs_pose_list != []:
                [self.obs_pose_list.append(obs) for obs in self.geometries[geometry_def["name"]].obs_pose_list]

    def getFSPGlobalPosefromCoordinates(self,coordinates):

        self.fsp_list.append(FreeSpacePose(str(len(self.fsp_list)+1),self.PoseFromArray(coordinates),self.name,self.prefix,self.transforms_list).global_pose)

        return [self.fsp_list[-1]]

                
    def getTransforms(self):

        return self.transforms_list

    def getGeometry(self,geometry_name):

        if geometry_name != "volume":
            return self.geometries[geometry_name]
        else:
            return self

    def PoseFromArray(self,Array):
        quat = tf.transformations.quaternion_from_euler(Array[1][0],Array[1][1],Array[1][2])

        return Pose(Point(Array[0][0],Array[0][1],Array[0][2]),Quaternion(quat[0],quat[1],quat[2],quat[3]))

    def ArrayFromPose(self,pose):

        euler = tf.transformations.euler_from_quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orienation.w)

        return [[pose.position.x,pose.position.y,pose.position.z],[euler[0],euler[1],euler[2]]]
        
