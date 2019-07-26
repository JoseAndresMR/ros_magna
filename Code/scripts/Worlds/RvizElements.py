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
# from xml.dom import minidom
# from gazebo_msgs.srv import DeleteModel,SpawnModel
from visualization_msgs.msg import Marker
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray, TorusArray, PolygonArray
from jsk_recognition_msgs.msg import Torus as jsk_Torus
# from sympy import Point3D, Line3D, Segment3D
# from sympy import Point as Point2D
# from sympy import Polygon as Polygon2D
# import xml.etree.ElementTree

from magna.srv import *
from TFElements import *

class RvizMarker(object):
    def __init__(self,rviz_marker_def):
        self.rviz_marker_def = rviz_marker_def

        self.hyperparameters = rospy.get_param('magna_hyperparameters')

        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size = 1)

        self.Spawner()

    def Spawner(self):

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.rviz_marker_def["parent_name"]
        marker.ns = str(self.rviz_marker_def["name"])
        marker.id = self.rviz_marker_def["id"]

        if self.rviz_marker_def["shape"] == "arrow":
            marker.type = 0
        elif self.rviz_marker_def["shape"] == "cube":
            marker.type = 1
        elif self.rviz_marker_def["shape"] == "sphere":
            marker.type = 2
        elif self.rviz_marker_def["shape"] == "cylinder":
            marker.type = 3

        marker.action = 0
        euler = self.rviz_marker_def["origin"][1]
        quaternion = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
        marker.pose = Pose(Point(self.rviz_marker_def["origin"][0][0],self.rviz_marker_def["origin"][0][1],self.rviz_marker_def["origin"][0][2]),
                        Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
        marker.scale = Point(self.rviz_marker_def["scale"][0],self.rviz_marker_def["scale"][1],self.rviz_marker_def["scale"][2])
        marker.color = ColorRGBA(self.rviz_marker_def["color"][0],self.rviz_marker_def["color"][1],self.rviz_marker_def["color"][2],self.rviz_marker_def["color"][3])
        # marker.lifetime = 0# rospy.Duration(0)
        
        self.marker = marker
        t = 0
        while not rospy.is_shutdown() and t < 10:
            self.marker_pub.publish(self.marker)
            t = t+1
            time.sleep(0.05)

    def Erase(self):
        self.marker.action = 2

        self.marker_pub.publish(self.marker)

    def Actualize(self,posestamped):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose = posestamped.pose
        self.marker_pub.publish(self.marker)





class RvizPolygonArray(object):
    def __init__(self,rviz_polygon_array_def):
        self.rviz_polygon_array_def = rviz_polygon_array_def

        self.polygon_array_pub = rospy.Publisher('/visualization_polygon_array/{}'.format(rviz_polygon_array_def["name"]), PolygonArray, queue_size = 1)

        self.Spawner()

    def Spawner(self):

        polygon_array = PolygonArray()
        polygon_array.header.stamp = rospy.Time.now()
        polygon_array.header.frame_id = "map"

        for i,polygon_poses in enumerate(self.rviz_polygon_array_def["polygon_array_poses"]):
            polygon = PolygonStamped()
            polygon.header.stamp = rospy.Time.now()
            polygon.header.frame_id = "map"

            for pose in polygon_poses:
                polygon.polygon.points.append(Point32(pose.position.x,pose.position.y,pose.position.z))
                polygon_array.labels.append(1)
                polygon_array.likelihood.append(1.0)

            polygon_array.polygons.append(polygon)

            # polygon_array.labels = self.rviz_polygon_array_def["labels"]#[i]
            # polygon_array.likelihood = self.rviz_polygon_array_def["likelihood"]#[i]
        
        # polygon_array.labels = [1,1,1,1,1,1]
        # polygon_array.likelihood = [1.0,1.0,1.0,1.0,1.0,1.0]

        self.polygon_array = polygon_array

        t = 0
        while not rospy.is_shutdown() and t < 10:
            self.polygon_array_pub.publish(self.polygon_array)
            t = t+1
            time.sleep(0.05)

    def Erase(self):
        pass

    def Actualize(self,posestamped):
        pass

class RvizTorusArray(object):
    def __init__(self,rviz_torus_array_def):
        self.rviz_torus_array_def = rviz_torus_array_def
        self.torus_array_pub = rospy.Publisher('/visualization_torus_array/{}'.format(rviz_torus_array_def["name"]), TorusArray, queue_size = 1)

        self.Spawner()

    def Spawner(self):

        torus_array = TorusArray()
        torus_array.header.stamp = rospy.Time.now()
        torus_array.header.frame_id = "map"

        for i,torus_poses in enumerate(self.rviz_torus_array_def["torus_array_poses"]):
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"

            torus_poses = [header] + torus_poses

            torus = jsk_Torus(*torus_poses)

            torus_array.toruses.append(torus)

        self.torus_array = torus_array

        t = 0
        while not rospy.is_shutdown() and t < 10:
            self.torus_array_pub.publish(self.torus_array)
            t = t+1
            time.sleep(0.05)

    def Erase(self):
        pass

    def Actualize(self,posestamped):
        pass

