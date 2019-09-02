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
from Volume import Volume

class Worlds(object):
    def __init__(self,ID):
        self.ID = int(ID)

        self.GettingWorldDefinition()       # Global parameters inizialization.
        self.home_path = rospkg.RosPack().get_path('magna')[:-5]

        print "creating world",ID

        rospy.init_node('world', anonymous=True)     # Start node

        self.world_def = self.ReadJSON("{0}/Code/JSONs/Worlds/{1}/{2}.json"\
                            .format(self.home_path,self.world_name,self.subworld_name))

        self.hyperparameters["world_boundaries"] = self.world_def["scenario"]["world_boundaries"]
        rospy.set_param('magna_hyperparameters', self.hyperparameters)

        

        # Initialization of lists
        self.n_obs = 0
        self.obs_list = []      # List with the objects that deal with an obstacle
        self.obs_shape_list = []         # List with the shapes of the objects
        self.obs_pose_list = []         # List with the poses of the objects
        self.transforms_list = []       # List of obstacles TF
        self.markers_list = []

        self.volumes = {}

        self.scenario_def = self.world_def["scenario"]

        self.Listener()

        rospy.spin()

    # Function to start subscribing and offering
    def Listener(self):

        # Start service for Agents to actualize its state
        rospy.Service('/magna/Worlds/add', InstructionCommand, self.handle_add)
        rospy.Service('/magna/Worlds/get_fspset', WorldGetFSPset, self.handle_get_fspset)
        rospy.Service('/magna/Worlds/die', InstructionCommand, self.handle_die)

    def handle_add(self,data):

        world_part_definition = json.loads(data.instruction)

        self.addVolumes(world_part_definition)

        self.hyperparameters["N_obs"] = len(self.obs_pose_list)
        self.hyperparameters["obs_shape"] = []
        self.hyperparameters["obs_pose_list"] = self.obs_pose_list
        rospy.set_param('magna_hyperparameters', self.hyperparameters)      # Actualize the ROS param

        response = InstructionCommandResponse()
        response.success = True

        return response


    def handle_get_fspset(self, data):

        fspset_path = json.loads(data.fspset_path)

        response = WorldGetFSPsetResponse()
        response.fspset = self.getFSPoseGlobal(fspset_path)
        
        return response

    def handle_die(self,data):

        self.die()


    def addVolumes(self,volumes_def_list):

        for volume_def in volumes_def_list:

            if volume_def["name"] == "JSON":

                extra_world_def = self.ReadJSON("{0}/Code/JSONs/Worlds/{1}/{2}.json"\
                            .format(self.home_path,volume_def["world"],volume_def["subworld"]))
                self.addVolumes(extra_world_def["scenario"]["volumes"])

            else:
                if volume_def["name"] not in self.volumes.keys():
                    self.volumes[volume_def["name"]] = Volume(volume_def,self.transforms_list)

                else:
                    self.volumes[volume_def["name"]].update(volume_def,self.transforms_list)

                self.transforms_list = self.volumes[volume_def["name"]].getTransforms()
                if self.volumes[volume_def["name"]].obs_pose_list != []:
                    [self.obs_pose_list.append(obs) for obs in self.volumes[volume_def["name"]].obs_pose_list]

    def ReadJSON(self,path):

        with open(path) as f:
            content = json.load(f)

        return content


    def getFSPoseGlobal(self,params):

        if params[2] == "matrix":
            return self.volumes[params[0]].getGeometry(params[1]).getFSPGlobalPosefromMatrix(params[3])

        elif params[2] == "random":

            return self.volumes[params[0]].getGeometry(params[1]).genFSPRandomGlobalPoseList(params[3])

        elif params[2] == "path":
            return self.volumes[params[0]].getGeometry(params[1]).getFSPGlobalPosefromPath(params[3])

        elif params[2] == "coordinates":
            return self.volumes[params[0]].getGeometry(params[1]).getFSPGlobalPosefromCoordinates(params[3])

    def eraseAllObstacles(self):

        pass

    # Function to close active child processess
    def die(self):

        self.eraseAllObstacles()      # Delete obstacles from Gazebo
        rospy.signal_shutdown("World node closed")      # Finish Ground Station process


    
    def GettingWorldDefinition(self):
        self.hyperparameters = rospy.get_param('magna_hyperparameters')
        self.mission_name = self.hyperparameters['mission']
        self.submission_name = self.hyperparameters['submission']
        self.world_name = self.hyperparameters['world']
        self.subworld_name = self.hyperparameters['subworld']
        self.n_simulation = self.hyperparameters['n_simulation']
        self.n_dataset = self.hyperparameters['n_dataset']
        self.smach_view = self.hyperparameters['smach_view']
        self.depth_camera_use = self.hyperparameters['depth_camera_use']

def main():
    Worlds(1)

if __name__ == '__main__':
    main()