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
from RvizElements import *
   


# Class to deal with one single obstacle
class Obstacle(object):
    def __init__(self,ID,shape,dimensions,pose,parent_name,parent_prefix,transforms_list):

        # Save params from args
        self.ID = ID
        self.name = parent_prefix + '_' + 'obs_{0}'.format(self.ID)
        self.parent_name = parent_name
        self.pose = pose
        self.home_path = rospkg.RosPack().get_path('magna')[:-5]

        basic_path = "{0}/Code/gz_models/".format(self.home_path)
        product_xml_path_dict = {"cube":basic_path + "{0}.sdf".format("cube"),\
                                "cylinder":basic_path + "{0}.sdf".format("cylinder"),\
                                "sphere":basic_path + "{0}.sdf".format("sphere"),\
                                "brick":basic_path + "{0}.sdf".format("brick"),\
                                }

        et = xml.etree.ElementTree.parse(product_xml_path_dict[shape])
        root = et.getroot()

        if shape == "cube":
            root[0][1][0][0][0][0].text = '{0} {1} {2}'.format(dimensions[0],dimensions[1],dimensions[2])
            root[0][1][1][0][0][0].text = '{0} {1} {2}'.format(dimensions[0],dimensions[1],dimensions[2])
        elif shape == "sphere":
            root[0][1][0][0][0][0].text = str(dimensions[0]/2)
            root[0][1][1][0][0][0].text = str(dimensions[0]/2)
        elif shape == "cylinder":
            root[0][1][0][0][0][0].text = str(dimensions[0]/2)
            root[0][1][0][0][0][1].text = str(dimensions[2])
            root[0][1][1][0][0][0].text = str(dimensions[0]/2)
            root[0][1][1][0][0][1].text = str(dimensions[2])

        et.write(product_xml_path_dict[shape])

        # Definition of the dictionary with the paths where every type of obstacle is stores. In the future should be packed
        self.product_xml = open(product_xml_path_dict[shape],"r").read()

        self.rviz_marker_def = {"shape" : shape}
        self.rviz_marker_def["origin"] = [[0,0,0],[0,0,0]]
        self.rviz_marker_def["parent_name"] = str(self.name)
        self.rviz_marker_def["name"] = self.name
        self.rviz_marker_def["id"] = 1
        self.rviz_marker_def["scale"] = dimensions

        color = [0,0,0]
        color.append(0)
        self.rviz_marker_def["color"] = [0,0,0,1]
           
        # Get ROS params
        self.hyperparameters = rospy.get_param('magna_hyperparameters')

        # Start service proxis to spawn and delete obstacles
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

        self.transforms_list = transforms_list

        tfbroadcaster = StaticTfBroadcaster(self.name,self.parent_name,self.pose,self.transforms_list)

        self.transforms_list = tfbroadcaster.getTransforms()
        
        self.global_pose = tfbroadcaster.getGlobalPose()

        self.Spawner()      # Spawn obstacle with defined data

        self.MakeRvizMarker()

        # self.transforms_list = self.transforms_list[:-1]
        # tfbroadcaster = StaticTfBroadcaster(self.name,self.parent_name,self.pose,self.transforms_list)


    # Function to query the service to spawn an obstacle
    def Spawner(self):
        try:
            rospy.wait_for_service('gazebo/spawn_sdf_model')
            response = self.spawn_model(self.name, self.product_xml, "", self.global_pose, "world")
            time.sleep(0.01)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in spawn_sdf_model"

    # Function to query the service to erase an obstacle
    def Erase(self):
        rospy.wait_for_service('gazebo/delete_model')
        try:
            delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
            item_name = self.name
            response = self.delete_model(item_name)
            time.sleep(0.01)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in delete_model obstacle"

    def getTransforms(self):
        return self.transforms_list

    def getGlobalPose(self):
        return self.global_pose

    def MakeRvizMarker(self):

        self.marker = RvizMarker(self.rviz_marker_def)







class FreeSpacePose(object):
    def __init__(self,ID,pose,parent_name,parent_prefix,transforms_list):

        # Save params from args
        self.ID = ID
        self.name = parent_prefix + '_' + 'pos_{0}'.format(self.ID)
        self.parent_name = parent_name
        self.pose = pose

        self.transforms_list = transforms_list

        tfbroadcaster = StaticTfBroadcaster(self.name,self.parent_name,self.pose,self.transforms_list)

        self.transforms_list = tfbroadcaster.getTransforms()
        
        self.global_pose = tfbroadcaster.getGlobalPose()

    def getTransforms(self):
        return self.transforms_list

    def getGlobalPose(self):
        return self.global_pose
