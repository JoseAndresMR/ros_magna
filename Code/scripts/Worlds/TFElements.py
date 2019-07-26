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
# from gazebo_msgs.srv import DeleteModel,SpawnModel
from visualization_msgs.msg import Marker
# from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray, TorusArray, PolygonArray
# from jsk_recognition_msgs.msg import Torus as jsk_Torus
# from sympy import Point3D, Line3D, Segment3D
# from sympy import Point as Point2D
# from sympy import Polygon as Polygon2D
# import xml.etree.ElementTree

from magna.srv import *


class StaticTfBroadcaster(object):
    def __init__(self,name,parent_name = "map",poses = [],transforms_list = []):
        self.pose = poses
        self.name = name
        self.transforms_list = copy.deepcopy(transforms_list)

        self.lookup_transform_listener = tf.TransformListener()

        transformStamped = geometry_msgs.msg.TransformStamped()
        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = parent_name

        if type(poses) == list:
            poses = np.array(poses)
            if len(poses.shape) == 2: 
                transformStamped.child_frame_id = name
                transformStamped.transform.translation = Point(poses[0][0],poses[0][1],poses[0][2])
                quat = tf.transformations.quaternion_from_euler(poses[1][0],poses[1][1],poses[1][2])
                transformStamped.transform.rotation = Quaternion(quat[0],quat[1],quat[2],quat[3])   

                self.transforms_list.append(copy.deepcopy(transformStamped))

            elif len(poses.shape) == 3: 
                identity = 0
                for i in range(poses.shape[0]):
                    for j in range(poses.shape[1]):
                        for k in range(poses.shape[2]):
                            transformStamped.child_frame_id = name + '_{0}_{1}_{2}'.format(i,j,k)
                            transformStamped.transform.translation = poses[i][j][k].position
                            transformStamped.transform.rotation = poses[i][j][k].orientation  

                            identity + 1

                            self.transforms_list.append(copy.deepcopy(transformStamped))

        elif type(poses) == type(Pose()):
            transformStamped.child_frame_id = name
            transformStamped.transform.translation = poses.position
            transformStamped.transform.rotation = poses.orientation  

            self.transforms_list.append(copy.deepcopy(transformStamped))

        self.Broadcast()


    def Broadcast(self):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        broadcaster.sendTransform(self.transforms_list)
        time.sleep(0.05)

    def getTransforms(self):
        return self.transforms_list

    def LookUpTransformFromFrame(self,frame):
        
        exit_flag = False
        while not(rospy.is_shutdown()) and exit_flag == False:
            try:
                trans,rot = self.lookup_transform_listener.lookupTransform(frame,self.name, rospy.Time.now())
                exit_flag = True
            except:
                time.sleep(0.05)

        pose = Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))

        return pose,trans,rot

    def getGlobalPose(self):

        self.global_pose,_,_ = self.LookUpTransformFromFrame("map")

        return self.global_pose



class DynamicTfBroadcaster(object):
    def __init__(self,name,parent_name = "map",poses = [],transforms_list = []):

        self.pose = poses
        self.name = name
        self.transforms_list = copy.deepcopy(transforms_list)

        self.lookup_transform_listener = tf.TransformListener()

        transformStamped = geometry_msgs.msg.TransformStamped()
        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = parent_name

        if type(poses) == list:
            poses = np.array(poses)

            if len(poses.shape) == 2: 
                transformStamped.child_frame_id = name
                transformStamped.transform.translation = Point(poses[0][0],poses[0][1],poses[0][2])
                quat = tf.transformations.quaternion_from_euler(poses[1][0],poses[1][1],poses[1][2])
                transformStamped.transform.rotation = Quaternion(quat[0],quat[1],quat[2],quat[3])   

                self.transforms_list.append(copy.deepcopy(transformStamped))

            elif len(poses.shape) == 3: 
                identity = 0
                for i in range(poses.shape[0]):
                    for j in range(poses.shape[1]):
                        for k in range(poses.shape[2]):
                            transformStamped.child_frame_id = name + '_{0}_{1}_{2}'.format(i,j,k)
                            transformStamped.transform.translation = poses[i][j][k].position
                            transformStamped.transform.rotation = poses[i][j][k].orientation  

                            identity + 1

                            self.transforms_list.append(copy.deepcopy(transformStamped))

        elif type(poses) == type(Pose()):
            transformStamped.child_frame_id = name
            transformStamped.transform.translation = poses.position
            transformStamped.transform.rotation = poses.orientation  

            self.transforms_list.append(copy.deepcopy(transformStamped))

        self.Broadcast()


    def Broadcast(self):
        broadcaster = tf2_ros.TransformBroadcaster()
        broadcaster.sendTransform(self.transforms_list)
        time.sleep(0.05)

    def getTransforms(self):
        return self.transforms_list

    def LookUpTransformFromFrame(self,frame):
        
        exit_flag = False
        while not(rospy.is_shutdown()) and exit_flag == False:
            try:
                trans,rot = self.lookup_transform_listener.lookupTransform(frame,self.name, rospy.Time.now())
                exit_flag = True
            except:
                time.sleep(0.05)

        pose = Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))

        return pose,trans,rot

    def getGlobalPose(self):

        self.global_pose,_,_ = self.LookUpTransformFromFrame("map")

        return self.global_pose
