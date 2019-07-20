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

# import pyModeS as pms
import numpy as np
import rospy, time, tf, tf2_ros, math
from copy import deepcopy
from geometry_msgs.msg import *
from sensor_msgs.msg import Image, BatteryState
from std_msgs.msg import String
from nav_msgs.msg import Path
from uav_abstraction_layer.msg import State
from magna.srv import *
from magna.msg import *

class UTM(object):

    ### Initialiations ###
    def __init__(self):

        self.GettingWorldDefinition()   # Global ROS parameters inizialization

        self.flightplans_list = range(self.N_agents)
        self.flightplans_checks = range(self.N_agents)

        rospy.init_node('utm', anonymous=True)     # Start node

        self.dist_th = 3
        self.time_th = 3

        self.init_comms()

        self.listener()

        self.newFlightplanTest()

        # self.utmThread()

        # a0 = np.array([-1, 1, 1])
        # a1 = np.array([-1, -1, 1])
        # b0 = np.array([1, -1, 0])
        # b1 = np.array([1, -2, 0])

        # pA, pB, min_dist = self.closestDistanceBetweenSegments(a0, a1, b0, b1, clampAll=True)

        # print(pA)
        # print(pB)
        # print(min_dist)

    def init_comms(self):

        self.conflict_path_pub = rospy.Publisher('/magna/utm/conflict_path', Path, queue_size = 1)

    def listener(self):

        for agent_id in range(1, self.N_agents + 1) :

            rospy.Subscriber('/magna/agent_{}/goal_path'.format(agent_id), Path, self.flightplan_callback, agent_id, queue_size=1)

    def flightplan_callback(self, data, agent_id):

        self.flightplans_list[agent_id -1] = data.poses
        time.sleep(0.5)
        self.flightplans_checks[agent_id -1] = True
        time.sleep(0.5)

    def checkAllFlighPlans(self):

        conflict_list = []

        for agent_A in range(1, self.N_agents + 1):

            if self.flightplans_checks[agent_A -1] == True:

                for agent_B in range(agent_A + 1, self.N_agents + 1):

                    if self.flightplans_checks[agent_B -1] == True:

                        conflict_list = conflict_list + self.check2FlightPlans(agent_A, agent_B)

        for i, conflict_dict in enumerate(conflict_list):

            print(conflict_dict["shortest_points"])
            print(i)

            self.publish_conflict(conflict_dict["shortest_points"],i)

    
    def check2FlightPlans(self, agent_A, agent_B):

        conflicts_list = []

        aA_fp = self.flightplans_list[agent_A -1]
        aB_fp = self.flightplans_list[agent_B -1]
        aA_points, aA_time, aB_points, aB_time = [], [], [], []
        for posestamped in aA_fp:
            aA_points.append( np.array([posestamped.pose.position.x, posestamped.pose.position.y, posestamped.pose.position.z]) )
            aA_time.append(posestamped.header.stamp.secs)
        for posestamped in aB_fp:
            aB_points.append( np.array([posestamped.pose.position.x, posestamped.pose.position.y, posestamped.pose.position.z]) )
            aB_time.append(posestamped.header.stamp.secs)

        glob_short_points = []

        for i in range(len(aA_points) -1):

            for j in range(len(aB_points) -1):

                pA, pB, min_dist = self.closestDistanceBetweenSegments(aA_points[i],aA_points[i+1],aB_points[j],aB_points[j+1], clampAll=True)

                if type(pA) == type(np.array([])):
                    min_time = self.closestTimeBetweenSegments(aA_points[i],aA_points[i+1], pA, aB_points[j],aB_points[j+1], pB, aA_time[i],aA_time[i+1], aB_time[j],aB_time[j+1])

                    if not(self.checkDistanceThreshold(min_dist,self.dist_th)) and not(self.checkTimeThreshold(min_time,self.time_th)):

                        conflict_dict = {"agents" : [agent_A,agent_B], "segments" : [i,j], "shortest_points" : [pA, pB], "min_distance" : min_dist, "min_time" : min_time}

                        conflicts_list.append(conflict_dict)
                else: 
                    print("PARALLEL SEGMENTS NOT SUPPORTED")

        return conflicts_list



    def closestDistanceBetweenSegments(self,a0,a1,b0,b1,clampAll=False,clampA0=False,clampA1=False,clampB0=False,clampB1=False):

        ''' Given two lines defined by numpy.array pairs (a0,a1,b0,b1)
            Return the closest points on each segment and their distance
        '''

        # If clampAll=True, set all clamps to True
        if clampAll:
            clampA0=True
            clampA1=True
            clampB0=True
            clampB1=True


        # Calculate denomitator
        A = a1 - a0
        B = b1 - b0
        magA = np.linalg.norm(A)
        magB = np.linalg.norm(B)

        _A = A / magA
        _B = B / magB

        cross = np.cross(_A, _B);
        denom = np.linalg.norm(cross)**2


        # If lines are parallel (denom=0) test if lines overlap.
        # If they don't overlap then there is a closest point solution.
        # If they do overlap, there are infinite closest positions, but there is a closest distance
        if not denom:
            d0 = np.dot(_A,(b0-a0))

            # Overlap only possible with clamping
            if clampA0 or clampA1 or clampB0 or clampB1:
                d1 = np.dot(_A,(b1-a0))

                # Is segment B before A?
                if d0 <= 0 >= d1:
                    if clampA0 and clampB1:
                        if np.absolute(d0) < np.absolute(d1):
                            return a0,b0,np.linalg.norm(a0-b0)
                        return a0,b1,np.linalg.norm(a0-b1)


                # Is segment B after A?
                elif d0 >= magA <= d1:
                    if clampA1 and clampB0:
                        if np.absolute(d0) < np.absolute(d1):
                            return a1,b0,np.linalg.norm(a1-b0)
                        return a1,b1,np.linalg.norm(a1-b1)


            # Segments overlap, return distance between parallel segments
            return None,None,np.linalg.norm(((d0*_A)+a0)-b0)



        # Lines criss-cross: Calculate the projected closest points
        t = (b0 - a0);
        detA = np.linalg.det([t, _B, cross])
        detB = np.linalg.det([t, _A, cross])

        t0 = detA/denom;
        t1 = detB/denom;

        pA = a0 + (_A * t0) # Projected closest point on segment A
        pB = b0 + (_B * t1) # Projected closest point on segment B


        # Clamp projections
        if clampA0 or clampA1 or clampB0 or clampB1:
            if clampA0 and t0 < 0:
                pA = a0
            elif clampA1 and t0 > magA:
                pA = a1

            if clampB0 and t1 < 0:
                pB = b0
            elif clampB1 and t1 > magB:
                pB = b1

            # Clamp projection A
            if (clampA0 and t0 < 0) or (clampA1 and t0 > magA):
                dot = np.dot(_B,(pA-b0))
                if clampB0 and dot < 0:
                    dot = 0
                elif clampB1 and dot > magB:
                    dot = magB
                pB = b0 + (_B * dot)

            # Clamp projection B
            if (clampB0 and t1 < 0) or (clampB1 and t1 > magB):
                dot = np.dot(_A,(pB-a0))
                if clampA0 and dot < 0:
                    dot = 0
                elif clampA1 and dot > magA:
                    dot = magA
                pA = a0 + (_A * dot)


        return pA,pB,np.linalg.norm(pA-pB)

    def closestTimeBetweenSegments(self,a0,a1,ai,b0,b1,bi,ta0,ta1,tb0,tb1):

        tai = ta0 + (self.DistanceBetweenPoints(a0,ai) / self.DistanceBetweenPoints(a0,a1))*(ta1 - ta0)
        tbi = tb0 + (self.DistanceBetweenPoints(b0,bi) / self.DistanceBetweenPoints(b0,b1))*(tb1 - tb0)

        return abs(tai - tbi)

    def checkDistanceThreshold(self,real_dist, threshold):

        return real_dist > threshold

    def checkTimeThreshold(self,real_time, threshold):

        return real_time > threshold


    def publish_conflict(self, edges_list, conflict_n):

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for point in edges_list:

            posestamped = PoseStamped()
            posestamped.pose.position.x = point[0]
            posestamped.pose.position.y = point[1]
            posestamped.pose.position.z = point[2]
            path_msg.poses.append(posestamped)

        conflict_path_pub = rospy.Publisher('/magna/utm/conflict_path/{}'.format(conflict_n +1 ), Path, queue_size = 1)
        time.sleep(3)
        conflict_path_pub.publish(path_msg)

        
    def utmThread(self):

        button = raw_input("Waiting for a button press")
        self.checkAllFlighPlans()                

        while not rospy.is_shutdown():
            time.sleep(1)

    def newFlightplanTest(self):

        selected = raw_input("Waiting for order")

        ids = [1]
        poses_list = [Pose(Point(10,10,4),Quaternion(0,0,0,1)),Pose(Point(10,20,4),Quaternion(0,0,0,1)),Pose(Point(20,20,4),Quaternion(0,0,0,1))]

        self.sendNewFlightplanToGS(ids, poses_list)

    
    def sendNewFlightplanToGS(self, ids, poses_list):

        req = UTMnotificationRequest()
        req.instruction = "new flightplan"
        req.ids = ids
        req.goal_path_poses_list = poses_list

        rospy.wait_for_service("/magna/utm/notification")
        try:
            notification_client = rospy.ServiceProxy("/magna/utm/notification", UTMnotification)
            response = notification_client(req)

            return response
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in take_off"


    def DistanceBetweenPoints(self,pose_1,pose_2):
        Distance = math.sqrt((pose_1[0]-pose_2[0])**2+(pose_1[1]-pose_2[1])**2+(pose_1[2]-pose_2[2])**2)
        return Distance


    def GettingWorldDefinition(self):
        self.hyperparameters = rospy.get_param('magna_hyperparameters')
        self.mission_name = self.hyperparameters['mission']
        self.submission_name = self.hyperparameters['submission']
        self.world_name = self.hyperparameters['world']
        self.subworld_name = self.hyperparameters['subworld']
        self.n_simulation = self.hyperparameters['n_simulation']
        self.N_agents = self.hyperparameters['N_agents']
        self.N_obs = self.hyperparameters['N_obs']
        self.agent_models = self.hyperparameters['agent_models']
        self.n_dataset = self.hyperparameters['n_dataset']
        self.communications = self.hyperparameters['communications']
        self.depth_camera_use = self.hyperparameters['depth_camera_use']
        self.obs_pose_list = self.hyperparameters['obs_pose_list']

utm = UTM()
