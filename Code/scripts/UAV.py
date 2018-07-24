#!/usr/bin/env python

# import pyModeS as pms
import numpy as np
import rospy, time, tf, tf2_ros
from geometry_msgs.msg import *
from sensor_msgs.msg import Image
from std_msgs.msg import String
from pydag.srv import *
from pydag.msg import *
from ADSB import ADSB


class UAV(object):
    def __init__(self,ID,main_uav,ICAO = "40621D",with_ref = True,pos_ref = [0,0]):
        # Local parameters inizialization
        self.ID = ID
        self.main_uav = main_uav

        self.ICAO = ICAO
        self.with_ref = with_ref
        self.pos_ref = pos_ref
        self.ADSB = ADSB(self.ICAO,self.with_ref,self.pos_ref)

        self.GettingWorldDefinition()
        self.br = tf.TransformBroadcaster()

        # Start listening
        self.listener()

    def listener(self):
        if self.main_uav == self.ID or (self.main_uav != self.ID and self.communications == "direct"):
            rospy.Subscriber('/uav_{}/ual/pose'.format(self.ID), PoseStamped, self.uav_pose_callback)
            rospy.Subscriber('/uav_{}/ual/velocity'.format(self.ID), TwistStamped, self.uav_vel_callback)
            
        elif self.main_uav != self.ID and self.communications == "ADSB":
            rospy.Subscriber('/Environment/ADSB/raw', String, self.incoming_ADSB_msg_callback)
        if self.project == 'dcdaa' and self.main_uav == self.ID:
            rospy.Subscriber('/typhoon_h480_{}/r200/r200/depth/image_raw'.format(self.ID), Image, self.image_raw_callback)

    # Callbacks
    def uav_pose_callback(self,data):
        self.position = data
        if self.main_uav != self.ID and self.communications == "ADSB":
            self.PoseBroadcast()
        time.sleep(0.1)

    def uav_vel_callback(self,data):
        self.velocity = data
        time.sleep(0.1)

    def image_raw_callback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose.position)
        #print "Image info recieved", data.height, data.width, data.encoding, data.is_bigendian, data.step
        # bridge = CvBridge()
        # self.image_depth = bridge.imgmsg_to_cv2(data, desired_encoding=data.encoding)[:,:,1]
        self.image_depth = data
        time.sleep(0.1)
        return

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

    # Function to get Global ROS parameters
    def GettingWorldDefinition(self):
        self.world_definition = rospy.get_param('world_definition')
        self.project = self.world_definition['project']
        self.world_type = self.world_definition['type']
        self.n_simulation = self.world_definition['n_simulation']
        self.N_uav = self.world_definition['N_uav']
        self.N_obs = self.world_definition['N_obs']
        self.obs_tube = self.world_definition['obs_tube']
        self.uav_models = self.world_definition['uav_models']
        self.n_dataset = self.world_definition['n_dataset']
        self.solver_algorithm = self.world_definition['solver_algorithm']
        self.communications = self.world_definition['communications']
        self.home_path = self.world_definition['home_path']

if __name__ == '__main__':
    uav = UAV("485020",True,[0,0])
    uav.incoming_ADSB_msg_callback("8D485020994409940838175B284F")
    print uav.velocity
    print uav.nic