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
import rospy, time, tf, tf2_ros
from copy import deepcopy
from geometry_msgs.msg import *
from sensor_msgs.msg import Image, BatteryState
from std_msgs.msg import String
from nav_msgs.msg import Path
from uav_abstraction_layer.msg import State
from pydag.srv import *
from pydag.msg import *
# from ADSB import ADSB
# from cv_bridge import CvBridge, CvBridgeError
from Worlds import *

class UAV_Data(object):
    def __init__(self,ID,main_uav,uav_config,ICAO = "40621D",with_ref = True,pos_ref = [0,0]):
        # Local parameters inizialization
        self.ID = ID
        self.main_uav = main_uav
        self.uav_config = uav_config

        self.ICAO = ICAO
        self.with_ref = with_ref
        self.pos_ref = pos_ref
        # self.ADSB = ADSB(self.ICAO,self.with_ref,self.pos_ref)

        self.GS_notification = 'nothing'
        self.Rviz_flag = True
        self.ual_state = 0
        self.battery_percentage = 1

        self.distance_rel2main = 9999

        self.main_uav_position = []

        self.GettingWorldDefinition()

        if self.depth_camera_use:      # Creation of the CV bridge to deal with the depth camera
            self.cv_bridge = CvBridge()

        self.br = tf.TransformBroadcaster()
        self.smooth_path_mode = 0
        self.smooth_velocity = Twist(Vector3(0,0,0),Vector3(0,0,0))

        if self.Rviz_flag == True and self.ID == self.main_uav:
            self.own_path = Path()
            self.own_path.header.stamp = rospy.Time.now()
            self.own_path.header.frame_id = "map"
            self.path_pub = rospy.Publisher('/pydag/uav_{}/path'.format(self.ID), Path, queue_size = 1)
            self.velocity_on_uav_pub = rospy.Publisher('/pydag/uav_{}/velocity_on_uav'.format(self.ID), TwistStamped, queue_size = 1)

            marker_def = {"shape" : "cylinder"}
            marker_def["origin"] = [[0,0,0],[0,0,0]]
            marker_def["parent_name"] = "map"
            marker_def["name"] = "uav_{0}".format(self.ID)
            marker_def["id"] = 1
            marker_def["scale"] = [self.uav_config.security_radius,self.uav_config.security_radius,self.uav_config.security_radius*0.7]
            marker_def["color"] = self.uav_config.marker_color

            self.marker = RvizMarker(marker_def)

        self.listener()     # Start listening to subcribed topics

    #### Listener functions ####

    # Function to decide where to subscribe
    def listener(self):

        # Subscribe to position and velocity of every UAV if comms are direct or is own UAV
        if self.main_uav == self.ID or (self.main_uav != self.ID and self.communications == "direct"):
            rospy.Subscriber(self.uav_config.top_sub_addr['pose'], PoseStamped, self.uav_pose_callback, queue_size=1)
            rospy.Subscriber(self.uav_config.top_sub_addr['velocity'], TwistStamped, self.uav_vel_callback, queue_size=1)
            rospy.Subscriber(self.uav_config.top_sub_addr['state'], State, self.ual_state_callback, queue_size=1)
            rospy.Subscriber(self.uav_config.top_sub_addr['battery_level'], BatteryState, self.battery_callback, queue_size=1)
            rospy.Subscriber('/uav_path_manager/follower/uav_{}/output_vel'.format(self.ID), TwistStamped, self.smooth_path_vel_callback, queue_size=1)
            
            # Subscribe to depth camera if its use flag is activated
            if self.depth_camera_use == True:
                rospy.Subscriber('/typhoon_h480_{}/r200/r200/depth/image_raw'.format(self.ID), Image, self.image_raw_callback, queue_size=1)


        if self.main_uav != self.ID:
            rospy.Subscriber('/uav_{}/ual/pose'.format(self.main_uav), PoseStamped, self.main_uav_pose_callback, queue_size=1)

            # Subscribe to ADSB's raw info if comms are ADSB. This part of the project is not still implemented
            if self.communications == "ADSB":
                rospy.Subscriber('/Environment/ADSB/raw', String, self.incoming_ADSB_msg_callback, queue_size=1)
            
        # Subcribe to preemption command if this is GS for UAV 1 and the UAV 1 object
        # In the future this will be implemented to wrap different roles and different IDs
        if self.ID == self.main_uav:
            rospy.Service('/pydag/GS/notification_command_to_{}'.format(self.main_uav), StateActualization, self.handle_GS_notification_command)

    ## Callbacks

    # Function to deal with pose data
    def uav_pose_callback(self,data):
        self.position = data        # Store the received position into the position of the UAV
        if self.Rviz_flag == True and self.ID == self.main_uav:

            self.marker.Actualize(self.position)        # NO FUNCTIONA AQUI

            self.own_path.poses.append(data)
            self.path_pub.publish(self.own_path)

        # If pose has been received via ADSB, pubish TF of it
        if self.main_uav != self.ID and self.communications == "ADSB":
            self.PoseBroadcast()

        if self.main_uav != self.ID and self.main_uav_position !=[]:
            self.position_rel2main = self.SubstractPoses(self.position.pose,self.main_uav_position.pose)
            self.distance_rel2main = self.PoseModule(self.position_rel2main)

        if self.main_uav == self.ID:
            self.obs_poses_rel2main = []
            self.obs_distances_rel2main = []
            for obs_pose in self.obs_pose_list:
                self.obs_poses_rel2main.append(self.SubstractPoses(Pose(Point(obs_pose[0][0],obs_pose[0][1],obs_pose[0][2]),
                                                                        Quaternion(obs_pose[1][0],obs_pose[1][1],obs_pose[1][2],obs_pose[1][3])),
                                                                   self.position.pose))
                self.obs_distances_rel2main.append(self.PoseModule(self.obs_poses_rel2main[-1]))
                
        time.sleep(0.1)

    def main_uav_pose_callback(self,data):
        self.main_uav_position = data

    # Function to deal with velocity data
    def uav_vel_callback(self,data):

        self.velocity = data        # Store the received velocity into the position of the UAV

        if self.Rviz_flag == True and self.ID == self.main_uav:
            data.header.frame_id = "uav_{}".format(self.ID)
            self.velocity_on_uav_pub.publish(data)

        time.sleep(0.1)

    # Function to deal with velocity data
    def ual_state_callback(self,data):

        self.ual_state = data.state        # Store the received velocity into the position of the UAV

        time.sleep(0.1)

    def battery_callback(self,data):

        self.battery_percentage = data.percentage
        time.sleep(0.1)

    # Function to deal with depth image data
    def image_raw_callback(self,data):
        try:
            # Transform the received information eith the bridge and store it into its variable
            self.image_depth = np.array(self.cv_bridge.imgmsg_to_cv2(data, desired_encoding=data.encoding).data)
            # print(self.image_depth.size)
        except CvBridgeError as e:
            print(e)

        time.sleep(0.5)
        return

    def smooth_path_vel_callback(self,data):
        self.smooth_velocity = data.twist

    # Function to deal with a preemption command from Ground Station
    def handle_GS_notification_command(self,data):
        self.GS_state = data.state 
        self.GS_notification = data.critical_event        # Set the variable
        return True

    #### Commander functions ####

    # Function for distributing ADS-B msg depending on its TC
    def incoming_ADSB_msg_callback(self,msg):
        TC, extracted_info = self.ADSB.incoming_msg(msg.msg)
        if TC == "callsign":
            self.callsign = extracted_info[0]
        elif TC == "surface_position":
            self.nic = extracted_info[0]
        elif TC == "airborne_position_baro_altitude":
            self.position = extracted_info[0]
            self.airborne_position = extracted_info[1]
            self.surface_position = extracted_info[2]
            self.nic = extracted_info[3]
        elif TC == "airborne_velocity":
            self.velocity = extracted_info[0]
            self.nac_v = extracted_info[1]
        elif TC == "airborne_position_GNSS_Height":
            self.nic = extracted_info[0]
        elif TC == "status":
            self.sil = extracted_info[0]
            self.nac_p = extracted_info[1]

    # Function to broadcast transforms to visualize it on Rviz
    def PoseBroadcast(self):
        self.br.sendTransform((self.position.pose.position.x,self.position.pose.position.y,self.position.pose.position.z),
                        (self.position.pose.orientation.x,self.position.pose.orientation.y,self.position.pose.orientation.z,self.position.pose.orientation.w),
                        rospy.Time.now(),
                        "uav_{}_by_{}".format(self.ID,self.main_uav),
                        "map")
        time.sleep(0.1)

    def SubstractPoses(self,pose_1,pose_2):         # TO UTILS
        position_1, position_2, orientation_1, orientation_2 = pose_1.position, pose_2.position, pose_1.orientation, pose_2.orientation
        difference = Pose(Point(position_1.x-position_2.x,position_1.y-position_2.y,position_1.z-position_2.z),
                          Quaternion(orientation_1.x-orientation_2.x,orientation_1.y-orientation_2.y,orientation_1.z-orientation_2.z,orientation_1.w-orientation_2.w))
        return difference

    def PoseModule(self,pose):   # To UTILS
        Distance = math.sqrt((pose.position.x)**2+(pose.position.y)**2+(pose.position.z)**2)
        return Distance



    # Function to get Global ROS parameters
    def GettingWorldDefinition(self):
        self.world_definition = rospy.get_param('world_definition')
        self.mission_name = self.world_definition['mission']
        self.submission_name = self.world_definition['submission']
        self.world_name = self.world_definition['world']
        self.subworld_name = self.world_definition['subworld']
        self.n_simulation = self.world_definition['n_simulation']
        self.N_uav = self.world_definition['N_uav']
        self.N_obs = self.world_definition['N_obs']
        self.uav_models = self.world_definition['uav_models']
        self.n_dataset = self.world_definition['n_dataset']
        self.solver_algorithm = self.world_definition['solver_algorithm']
        self.communications = self.world_definition['communications']
        self.depth_camera_use = self.world_definition['depth_camera_use']
        self.obs_pose_list = self.world_definition['obs_pose_list']

# if __name__ == '__main__':
#     uav = UAV("485020",True,[0,0])
#     uav.incoming_ADSB_msg_callback("8D485020994409940838175B284F")
