#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 21 2018

@author: josmilrom
"""
import tf, tf2_ros
import rospy
import time
from geometry_msgs.msg import *

rospy.init_node('ground_station', anonymous=True)     # Start node


transformStamped = geometry_msgs.msg.TransformStamped()
transformStamped.header.stamp = rospy.Time.now()
transformStamped.header.frame_id = "map"
transformStamped.child_frame_id = "1"
transformStamped.transform.translation = Point(1,1,1)
quat = tf.transformations.quaternion_from_euler(0,0,0)
transformStamped.transform.rotation = Quaternion(quat[0],quat[1],quat[2],quat[3])   

broadcaster = tf2_ros.StaticTransformBroadcaster()
response = broadcaster.sendTransform([transformStamped])
time.sleep(0.05)
print response


transformStamped2 = geometry_msgs.msg.TransformStamped()
transformStamped2.header.stamp = rospy.Time.now()
transformStamped2.header.frame_id = "local"
transformStamped2.child_frame_id = "2"
transformStamped2.transform.translation = Point(2,2,2)
quat = tf.transformations.quaternion_from_euler(0,0,0)
transformStamped2.transform.rotation = Quaternion(quat[0],quat[1],quat[2],quat[3])   

broadcaster2 = tf2_ros.StaticTransformBroadcaster()
broadcaster2.sendTransform([transformStamped2])
time.sleep(0.05)


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
                time.sleep(0.2)

        pose = Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))

        return pose,trans,rot

    def getGlobalPose(self):

        self.global_pose,_,_ = self.LookUpTransformFromFrame("map")

        return self.global_pose
