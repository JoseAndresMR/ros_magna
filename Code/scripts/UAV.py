#!/usr/bin/env python

# import pyModeS as pms
import numpy as np
import rospy, time, tf, tf2_ros
from geometry_msgs.msg import *
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Path
from pydag.srv import *
from pydag.msg import *
from ADSB import ADSB
from cv_bridge import CvBridge, CvBridgeError

class UAV(object):
    def __init__(self,ID,main_uav,ICAO = "40621D",with_ref = True,pos_ref = [0,0]):
        # Local parameters inizialization
        self.ID = ID
        self.main_uav = main_uav

        self.ICAO = ICAO
        self.with_ref = with_ref
        self.pos_ref = pos_ref
        self.ADSB = ADSB(self.ICAO,self.with_ref,self.pos_ref)

        self.preempt_flag = False

        self.Rviz_flag = True

        self.GettingWorldDefinition()

        if self.depth_camera_use:      # Creation of a brain associated to this drone
            self.cv_bridge = CvBridge()

        self.br = tf.TransformBroadcaster()

        if self.Rviz_flag == True and self.ID == self.main_uav:
            self.own_path = Path()
            self.own_path.header.stamp = rospy.Time.now()
            self.own_path.header.frame_id = "map"
            self.changed_state = False
            self.path_pub = rospy.Publisher('/pydag/uav_{}/path'.format(self.ID), Path, queue_size = 1)
            self.velocity_on_uav_pub = rospy.Publisher('/pydag/uav_{}/velocity_on_uav'.format(self.ID), TwistStamped, queue_size = 1)

        self.listener()     # Start listening to subcribed topics

    #### Listener functions ####

    # Function to decide where to subscribe
    def listener(self):

        # Subscribe to position and velocity of every UAV if comms are direct or is own UAV
        if self.main_uav == self.ID or (self.main_uav != self.ID and self.communications == "direct"):
            rospy.Subscriber('/uav_{}/ual/pose'.format(self.ID), PoseStamped, self.uav_pose_callback)
            rospy.Subscriber('/uav_{}/ual/velocity'.format(self.ID), TwistStamped, self.uav_vel_callback)

        # Subscribe to ADSB's raw info if comms are ADSB. This part of the project is not still implemented
        elif self.main_uav != self.ID and self.communications == "ADSB":
            rospy.Subscriber('/Environment/ADSB/raw', String, self.incoming_ADSB_msg_callback)

        # Subscribe to depth camera if its use flag is activated
        if self.depth_camera_use == True and self.main_uav == self.ID:
            rospy.Subscriber('/typhoon_h480_{}/r200/r200/depth/image_raw'.format(self.ID), Image, self.image_raw_callback)

        # Subcribe to preemption command if this is GS for UAV 1 and the UAV 1 object
        # In the future this will be implemented to wrap different roles and different IDs
        if self.ID == self.main_uav and  self.main_uav != 1: # and mission ==
            rospy.Service('/pydag/ANSP/preemption_command_to_{}'.format(self.main_uav), StateActualization, self.handle_preemption_command)

    # Callbacks

    # Function to deal with pose data
    def uav_pose_callback(self,data):
        self.position = data        # Store the received position into the position of the UAV
        if self.Rviz_flag == True and self.ID == self.main_uav:

            if self.changed_state == True:
                self.own_path.poses = []
                self.changed_state = False

            self.own_path.poses.append(data)
            self.path_pub.publish(self.own_path)

        # If pose has been received via ADSB, pubish TF of it
        if self.main_uav != self.ID and self.communications == "ADSB":
            self.PoseBroadcast()

        time.sleep(0.1)

    # Function to deal with velocity data
    def uav_vel_callback(self,data):

        self.velocity = data        # Store the received velocity into the position of the UAV

        if self.Rviz_flag == True and self.ID == self.main_uav:
            data.header.frame_id = "uav_{}".format(self.ID)
            self.velocity_on_uav_pub.publish(data)

        time.sleep(0.1)

    # Function to deal with depth image data
    def image_raw_callback(self,data):
        try:
            # Transform the received information eith the bridge and store it into its variable
            self.image_depth = np.array(self.cv_bridge.imgmsg_to_cv2(data, desired_encoding=data.encoding).data)
            print("IAMGE RECIEVEEEEEEEEED",len(data.data))
        except CvBridgeError as e:
            print(e)

        time.sleep(0.5)
        return

    # Function to deal with a preemption command from ANSP
    def handle_preemption_command(self,data):
        # print("preempted flag",self.preempt_flag)
        self.preempt_flag = True        # Set the variable

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

    # Function to get Global ROS parameters
    def GettingWorldDefinition(self):
        self.world_definition = rospy.get_param('world_definition')
        self.mission = self.world_definition['mission']
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
        self.depth_camera_use = self.world_definition['depth_camera_use']

if __name__ == '__main__':
    uav = UAV("485020",True,[0,0])
    uav.incoming_ADSB_msg_callback("8D485020994409940838175B284F")
