#!/usr/bin/env python

import pyModeS as pms
import numpy as np
from geometry_msgs.msg import *

class UAV(object):
    def __init__(self,main_uav = True,ICAO = "40621D",with_ref = True,pos_ref = [0,0]):
        self.main_uav = main_uav
        self.ICAO = ICAO
        self.with_ref = with_ref
        self.pos_ref = pos_ref
        self.callsign = []
        self.surface_position = []
        self.airborne_position  = []
        self.surface_position_with_ref = []
        self.altitude = []
        self.velocity = []
        self.speed_heading = []
        self.airborne_velocity = []
        self.nic = []

        self.GettingWorldDefinition()

    def listener(self):
        if self.main_uav == True or (self.main_uav == False and self.communications == "direct"):
            rospy.Subscriber('/uav_{}/ual/pose'.format(n_uav+1), PoseStamped, self.uav_pose_callback,n_uav+1)
            rospy.Subscriber('/uav_{}/ual/velocity'.format(n_uav+1), TwistStamped, self.uav_vel_callback,n_uav+1)
            
                

    def incoming_ADSB_msg(self,msg,msg_aux = None,even_pos = 0):
        if pms.adsb.icao(msg) == self.ICAO and pms.crc(msg, encode=False) == "000000000000000000000000":
            h = std_msgs.msg.Header()
            typecode = pms.adsb.typecode(msg)
            print typecode

            ### Aircraft Identificaction, TC = 1 - 4
            if typecode in range(1,4):
                self.callsign = pms.adsb.callsign(msg)

            ### Surface position, TC = 5 - 8
            if typecode in range(5,8):
                pass

            ### Airborne position (w/ Baro Altitude), TC = 9 - 18
            if typecode in range(9,18):
                self.nic = pms.adsb.nic(msg)
                if self.with_ref == True:
                    position = pms.adsb.position_with_ref(msg, self.pos_ref[0],self.pos_ref[1])
                    airborne_position = pms.adsb.airborne_position_with_ref(msg, self.pos_ref[0],self.pos_ref[1])
                    surface_position = pms.adsb.surface_position_with_ref(msg, self.pos_ref[0],self.pos_ref[1])
                    altitude = pms.adsb.altitude(msg)

                if self.with_ref == False:
                    if even_pos == 0:
                        msg_even = msg
                        msg_odd = msg_aux
                    elif even_pos == 1:
                        msg_even = msg_aux
                        msg_odd = msg
                    position = pms.adsb.position(msg_even, msg_odd, t_even, t_odd, lat_ref=None, lon_ref=None)
                    
                    airborne_position = pms.adsb.airborne_position(msg_even, msg_odd, t_even, t_odd)
                    surface_position = pms.adsb.surface_position(msg_even, msg_odd, t_even, t_odd, self.pos_ref[0],self.pos_ref[1])
                    altitude = pms.adsb.altitude(msg_even)

                self.position = PoseStamped(h,Pose(Point(position[0],position[1],altitude),Quaternion(0,0,0,0)))
                self.airborne_position = PoseStamped(h,Pose(Point(airborne_position[0],airborne_position[1],altitude),Quaternion(0,0,0,0)))
                self.surface_position = PoseStamped(h,Pose(Point(surface_position[0],surface_position[1],altitude),Quaternion(0,0,0,0)))

            ### Airborne velocities, TC = 19
            if typecode == 19:
                velocity = pms.adsb.velocity(msg) # handles both surface & airborne messages. return: v(modulo en kt), h (heading en deg), Vr (verticalrate en ft/min)
                self.velocity = TwistStamped(h,Twist(Vector3(velocity[0]*np.cos(velocity[1]),velocity[0]*np.sin(velocity[1]),velocity[2]),\
                                                     Vector3(0,0,0)))

            ### Airborne position (w/ GNSS Height), TC = 20 - 22
            if typecode in range(20,22):
                self.nic = pms.adsb.nic(msg)

            ### Aircraft status, TC = 28
            if typecode == 28:
                pass

            ### Target state and status information, TC = 29
            if typecode == 29:
                pass

            ### Aircraft operation status, TC = 31
            if typecode == 31:
                pass

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
        self.obs_pose_list_simple = self.world_definition['obs_pose_list_simple']
        self.communications = self.world_definition['communications']

if __name__ == '__main__':
    uav = UAV("485020",True,[0,0])
    uav.incoming_ADSB_msg("8D485020994409940838175B284F")
    print uav.velocity
    print uav.nic