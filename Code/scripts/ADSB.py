import pyModeS as pms
import numpy as np
from std_msgs.msg import String,Header
from geometry_msgs.msg import TwistStamped,Twist,Vector3,PoseStamped,Pose,Point,Quaternion

class ADSB(object):
    def __init__(self,ICAO = "40621D",with_ref = True,pos_ref = [0,0]):
        self.ICAO = ICAO
        self.with_ref = with_ref
        self.pos_ref = pos_ref

        self.version = None    
        self.nic_s = None
        self.nic_a = None
        self.nic_b = None
        self.nic_c = None
        self.version = 2    
        self.nic_s = 0
        self.nic_a = 0
        self.nic_b = 0
        self.nic_c = 0

    def incoming_msg(self,msg,msg_aux = None,even_pos = 0):
        if pms.adsb.icao(msg) == self.ICAO and pms.crc(msg, encode=False) == "000000000000000000000000":
            h = Header()
            typecode = pms.adsb.typecode(msg)
            print typecode

            ### Aircraft Identificaction, TC = 1 - 4
            if typecode in range(1,4):
                self.callsign = pms.adsb.callsign(msg)
                TC = "callsign"
                extracted_info = [self.callsign]


            ### Surface position, TC = 5 - 8
            if typecode in range(5,8):
                if self.version == 1:
                    if self.nic_s != None:
                        self.nic = pms.adsb.nic_v1(msg,self.nic_s)
                elif self.version == 2:
                    if self.nic_a != None and self.nic_b != None:
                        self.nic = pms.adsb.nic_v2(msg,self.nic_a,self.nic_b,self.nic_c)
                TC = "surface_position"
                extracted_info = [self.nic]

            ### Airborne position (w/ Baro Altitude), TC = 9 - 18
            if typecode in range(9,18):
                if self.version == 1:
                    if self.nic_s != None:
                        self.nic = pms.adsb.nic_v1(msg,self.nic_s)
                elif self.version == 2:
                    self.nic_b = pms.adsb.nic_b(msg)
                    if self.nic_a != None and self.nic_b != None:
                        self.nic = pms.adsb.nic_v2(msg,self.nic_a,self.nic_b,self.nic_c)

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

                TC = "airborne_position_baro_altitude"
                extracted_info = [self.position,self.airborne_position,self.surface_position,self.nic]
                

            ### Airborne velocities, TC = 19
            if typecode == 19:
                velocity = pms.adsb.velocity(msg) # handles both surface & airborne messages. return: v(modulo en kt), h (heading en deg), Vr (verticalrate en ft/min)
                self.velocity = TwistStamped(h,Twist(Vector3(velocity[0]*np.cos(velocity[1]),velocity[0]*np.sin(velocity[1]),velocity[2]),\
                                                     Vector3(0,0,0)))
                self.nac_v = pms.adsb.nac_v(msg)

                TC = "airborne_velocity"
                extracted_info = [self.velocity,self.nac_v]

            ### Airborne position (w/ GNSS Height), TC = 20 - 22
            if typecode in range(20,22):
                if self.version == 1:
                    if self.nic_s != None:
                        self.nic = pms.adsb.nic_v1(msg,self.nic_s)
                elif self.version == 2:
                    if self.nic_a != None and self.nic_b != None:
                        self.nic = pms.adsb.nic_v2(msg,self.nic_a,self.nic_b,self.nic_c)
                TC = "airborne_position_GNSS_Height"
                extracted_info = [self.nic]

            ### Aircraft status, TC = 28
            if typecode == 28:
                pass

            ### Target state and status information, TC = 29
            if typecode == 29:
                self.sil = pms.adsb.sil(msg,self.version)
                self.nac_p = pms.adsb.nac_p(msg)

                TC = "status"
                extracted_info = [self.sil, self.nac_p]


            ### Aircraft operation status, TC = 31
            if typecode == 31:
                self.version = pms.adsb.version(msg)
                self.sil = pms.adsb.version(msg)
                self.nac_p = pms.adsb.nac_p(msg)
                if self.version == 1:
                    self.nic_s = pms.adsb.nic_s(msg)
                elif self.version == 2:
                    self.nic_a, self.nic_c = pms.adsb.nic_a_and_c(msg)

                TC = "status"
                extracted_info = [self.sil, self.nac_p]

            return TC, extracted_info
