#!/usr/bin/env python
# from multidrone_msgs.msg import TargetStateArray, SystemStatus, DroneActionStatus
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from uav_abstraction_layer.msg import State
import rospkg
import rospy
import time
from os import system

def ualState(state):
    if state == State.UNINITIALIZED:
        state_as_string = "UNINITIALIZED"
    elif state == State.LANDED_DISARMED:
        state_as_string = "LANDED_DISARMED"
    elif state == State.LANDED_ARMED:
        state_as_string = "LANDED_ARMED"
    elif state == State.TAKING_OFF:
        state_as_string = "TAKING_OFF"
    elif state == State.FLYING_AUTO:
        state_as_string = "FLYING_AUTO"
    elif state == State.FLYING_MANUAL:
        state_as_string = "FLYING_MANUAL"
    elif state == State.LANDING:
        state_as_string = "LANDING"
    return state_as_string


class DroneStatus:
    def __init__(self,id):
        self.id = id
        self.gps_fix = 0
        self.n_satellites = 0
        self.gps_cov = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0
        self.px4_mode = ""
        self.ual_state = ""
        self.mission_id = ""
        self.sas_id = ""
        self.action_id = ""
        self.action_status = ""
        self.magna_state = ""

        rospy.Subscriber("/uav_" + str(id) + "/ual/pose", PoseStamped, self.dronePoseCallback)
        rospy.Subscriber("/uav_" + str(id) + "/ual/state", State, self.droneStateCallback)
        rospy.Subscriber("/uav_" + str(id) + "/mavros/global_position/global", NavSatFix, self.globalCallback)
        rospy.Subscriber("/magna/status/agent_" + str(id), String, self.magnaStateCallback)

    def dronePoseCallback(self,data):
        self.pose_x = data.pose.position.x
        self.pose_y = data.pose.position.y
        self.pose_z = data.pose.position.z

    def droneStateCallback(self,data):
        self.ual_state = ualState(data.state)

    def globalCallback(self,data):
        self.gps_cov = data.position_covariance[0]

    def magnaStateCallback(self,data):
        self.magna_state = data.data

class TargetStatus:
    def __init__(self,id):
        self.id = id
        self.gps_fix = 0
        self.n_satellites = 0
        self.gps_cov = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0

        rospy.Subscriber("/target_" + str(id) + "/mavros/global_position/global", NavSatFix, self.globalCallback)
        rospy.Subscriber("/mission_controller/system_status", SystemStatus, self.systemStatusCallback)

    def globalCallback(self,data):
        self.gps_cov = data.position_covariance[0]

class MultidroneStatus:
    def __init__(self):
        self.targets = {}
        self.drones = {}

        rospy.Subscriber("/diagnostics", DiagnosticArray, self.diagnosticsCallback)

    def targetPoseCallback(self,data):
        for target in data.targets:
            if target.target_id > 0:
                if not( target.target_id in self.targets ):
                    self.targets[target.target_id] = TargetStatus(target.target_id)
                self.targets[target.target_id].pose_x = target.pose.pose.position.x
                self.targets[target.target_id].pose_y = target.pose.pose.position.y
                self.targets[target.target_id].pose_z = target.pose.pose.position.z

    def diagnosticsCallback(self,data):

        fix_type = 0
        found_fix = False
        n_satellites = 0
        px4_mode = ""
        for element in data.status:
            if element.name.find("GPS") >= 0:
                for value in element.values:
                    if "Fix type" in value.key:
                        fix_type = int(value.value)
                        found_fix = True
                    if "Satellites visible" in value.key:
                        n_satellites = int(value.value)
            if element.name.find("Heartbeat") >= 0:
                for value in element.values:
                    if "Mode" in value.key:
                        px4_mode = value.value
        
        if found_fix:
            drone_id_pos = data.status[0].name.find("uav_")
            if drone_id_pos >= 0:
                drone_id = int( data.status[0].name[drone_id_pos+4] )
                if not( drone_id in self.drones ):
                    self.drones[drone_id] = DroneStatus(drone_id)
                self.drones[drone_id].gps_fix = fix_type
                self.drones[drone_id].n_satellites = n_satellites
                self.drones[drone_id].px4_mode = px4_mode
            else:
                target_id_pos = data.status[0].name.find("target_")
                if target_id_pos >= 0:
                    target_id = int( data.status[0].name[target_id_pos+7] )
                    if not( target_id in self.targets ):
                        self.targets[target_id] = TargetStatus(target_id)
                    self.targets[target_id].gps_fix = fix_type
                    self.targets[target_id].n_satellites = n_satellites


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def print_info(system_status):
    system("clear")
    print bcolors.OKBLUE + "==================================" + bcolors.ENDC
    print bcolors.OKBLUE + "==== " + bcolors.ENDC + bcolors.BOLD + "MULTIDRONE system status" + bcolors.ENDC + bcolors.OKBLUE + " ====\n" + bcolors.ENDC
    print "          --- UAVs  ---       "
    for id, drone in system_status.drones.items():
        print bcolors.BOLD + "\nAGENT " + str(drone.id) + bcolors.ENDC
        print "  PX4 mode: " + drone.px4_mode
        print "  UAL state: " + drone.ual_state
        print "  MAGNA state: " + drone.magna_state
        print "  GPS Fix/nSat/Cov: " + bcolors.BOLD + (bcolors.FAIL if drone.gps_fix<=3 else (bcolors.OKGREEN if drone.gps_fix==6 else bcolors.WARNING) ) + str(drone.gps_fix) + bcolors.ENDC + " / " + str(drone.n_satellites) + " / {:.2f}".format(drone.gps_cov)
        print "  Pose: {:.2f}  {:.2f}  {:.2f}".format(drone.pose_x,drone.pose_y,drone.pose_z)
    
    print bcolors.OKBLUE + "==================================" + bcolors.ENDC

if __name__ == "__main__":
    rospy.init_node('pygcs', anonymous=True)

    system_status = MultidroneStatus()

    while not rospy.is_shutdown():   
        print_info(system_status)
        time.sleep(0.1)
