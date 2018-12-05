import rospy
import time
import numpy as np
import tf2_ros
import tf
import math
import collections
import copy
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_srvs.srv import *
from geographic_msgs.msg import GeoPoint

from GeoLocalPose import GeoLocalPose

class FwQgc(object):
    def __init__(self,robot_id = 1,ual_use = False, pose_frame_id = "", ns_prefix = "uav_",
                 position_th_param = 0.33, orientation_th_param = 0.65):

        self.ual_use = ual_use
        if self.ual_use == True:
            self.ual_prefix = "/{0}{1}".format(ns_prefix,robot_id)
        else:
            self.ual_prefix = ""

        rospy.init_node('fw_qgc', anonymous=True)
        self.waypoint_reached = -1

        self.listener()
        time.sleep(1)
        self.InitializePublishers()
        self.missionStarter()

        if self.ual_use == False:
            # self.TrialMission()

            raw_input("next")
            self.MoveCommand("takeoff",[200.0,0.0,10.0],0,{"type":"by_angle","angle": 0,"height": 10.0,"distance":200})
            raw_input("next")
            # self.MoveCommand("loiter",[[200.0,200.0,10.0]],0)
            # raw_input("next")
            # self.MoveCommand("pass",[[200.0,0.0,10.0],
            #                                     [500.0,300.0,10.0],
            #                                     [0.0,300.0,10.0]],0)
            # raw_input("next")
            self.MoveCommand("land",[[0.0,0.0,0.0]],0,{"loiter_to_alt":{"type":"by_angle","angle": -math.pi/2,"height": 10.0,"distance":200}})

            self.DisarmAtLand()

    '''
    Publishers
    '''
    def InitializePublishers(self):
        self.mavros_ref_pose_pub = rospy.Publisher('{0}/mavros/setpoint_position/local'.format(self.ual_prefix), PoseStamped, queue_size=10)
        self.mavros_ref_pose_global_pub = rospy.Publisher('{0}/mavros/setpoint_raw/global'.format(self.ual_prefix),GlobalPositionTarget, queue_size=10)
        self.mavros_ref_vel_pub = rospy.Publisher('{0}/mavros/setpoint_velocity/cmd_vel'.format(self.ual_prefix),TwistStamped, queue_size=10)
        self.mavros_ref_att_pose_pub = rospy.Publisher('{0}/mavros/setpoint_attitude/pose'.format(self.ual_prefix),PoseStamped, queue_size=10)

    '''
    Listeners
    '''

    def listener(self):
        rospy.Subscriber('{0}/mavros/local_position/pose'.format(self.ual_prefix), PoseStamped, self.local_pose_callback)
        rospy.Subscriber('{0}/mavros/local_position/velocity'.format(self.ual_prefix), TwistStamped, self.local_velocity_callback)
        rospy.Subscriber('{0}/mavros/global_position/global'.format(self.ual_prefix), NavSatFix, self.global_pose_callback)
        # rospy.Subscriber('{0}/mavros/local_position/velocity'.format(self.ual_prefix), TwistStamped, self.local_velocity_callback)
        rospy.Subscriber('{0}/mavros/state'.format(self.ual_prefix), State, self.state_callback)
        rospy.Subscriber('{0}/mavros/extended_state'.format(self.ual_prefix), ExtendedState, self.extended_state_callback)
        rospy.Subscriber('{0}/mavros/mission/reached'.format(self.ual_prefix), WaypointReached, self.waypoint_reached_callback)
        rospy.Subscriber('{0}/mavros/mission/waypoints'.format(self.ual_prefix), WaypointList, self.current_mission_list_callback)


    def local_pose_callback(self,data):
        self.cur_local_pose = data
        self.mavros_has_pose = True

    def local_velocity_callback(self,data):
        self.cur_local_vel = data

    def global_pose_callback(self,data):
        self.cur_global_pose = data

    def state_callback(self,data):
        self.mavros_state = data
        # print("mode",data.mode)
        # print("armed",data.armed)

    def extended_state_callback(self,data):
        self.mavros_extended_state = data

    def waypoint_reached_callback(self,data):
        if self.waypoint_reached != data.wp_seq:
            self.waypoint_reached = data.wp_seq
            # print("WP {} reached".format(self.waypoint_reached))

    def current_mission_list_callback(self,data):
        self.current_mission = data
        print("WP {} waypoints".format(data.current_seq))


    '''
    Server Callbacks
    '''

    def setArmedSrvCall(self, value):
        rospy.wait_for_service('{0}/mavros/cmd/arming'.format(self.ual_prefix))
        try:
            request = CommandBoolRequest()
            request.value = value
            ual_arm = rospy.ServiceProxy(
                '{0}/mavros/cmd/arming'.format(self.ual_prefix), CommandBool)
            response = ual_arm(request)

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            print "error in setArmedSrvCall"

    def setFlightModeSrvCall(self, flight_mode):
        rospy.wait_for_service('{0}/mavros/set_mode'.format(self.ual_prefix))
        try:
            request = SetModeRequest()
            request.base_mode = 0
            request.custom_mode = flight_mode
            ual_set_mode = rospy.ServiceProxy(
                '{0}/mavros/set_mode'.format(self.ual_prefix), SetMode)
            response = ual_set_mode(request)
            # print("mode change response",response)

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            print "error in setFlightMode"

    def setHomePosReqUpdtSrvCall(self):
        rospy.wait_for_service(
            '{0}/mavros/home_position/req_update'.format(self.ual_prefix))
        try:
            home_pose_req_updt_mode = rospy.ServiceProxy(
                '{0}/mavros/set_mode'.format(self.ual_prefix), Trigger)
            response = home_pose_req_updt_mode()
            print(response)

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            print "error in HomePosReqUpd"

    def WpListSrvCall(self, msg):
        rospy.wait_for_service(
            '{0}/mavros/mission/push'.format(self.ual_prefix))
        try:
            proxy = rospy.ServiceProxy(
                '{0}/mavros/mission/push'.format(self.ual_prefix), WaypointPush)
            response = proxy(msg)
            if response.success == True:
                print("{} waypoint transfer secceeded".format(
                    response.wp_transfered))
            else:
                print("Waypoint transfer REJECTED")

            return

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            print "error in WpListSrvCall"

    def ClearMissionSrvCall(self):
        print("Trying to clear mission")
        print(self.ual_prefix)
        rospy.wait_for_service(
            '{0}/mavros/mission/clear'.format(self.ual_prefix))
        try:
            proxy = rospy.ServiceProxy(
                '{0}/mavros/mission/clear'.format(self.ual_prefix), WaypointClear)
            msg = WaypointClearRequest()
            response = proxy()
            if response.success == True:
                print("Mission clear")
            else:
                print("Mission clear REJECTED")
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            print "error in ClearMissionSrvCall"

    '''
    Mission Actions
    '''

    def addTakeOffWp(self, wpList, wp_mini = [], params={}):
        wp = Waypoint()
        wp.frame = 3
        wp.command = 22
        wp.is_current = True
        wp.autocontinue = True

        if "minimum_pitch" not in params.keys():
            params["minimum_pitch"] = 15.0
        if "yaw" not in params.keys():
            params["yaw"] = float('nan')
        if "type" not in params.keys():
            params["type"] = "waypoint"
   
        if params["type"] == "by_waypoint" and wp_mini != []:
            print(wp_mini)
            wp.x_lat,wp.y_long,wp.z_alt = wp_mini[0][0],wp_mini[0][1],wp_mini[0][2]

        elif params["type"] == "by_angle":
            if "distance" not in params.keys():
                params["distance"] = 200.0
            if "angle" not in params.keys():
                quaterions = self.cur_local_pose.pose.orientation
                euler = tf.transformations.euler_from_quaternion([quaterions.x,quaterions.y,quaterions.z,quaterions.w])
                params["angle"] = euler[2]
                
            if "height" not in params.keys():
                params["height"] = 20.0

            wp.x_lat = self.cur_local_pose.pose.position.x + params["distance"] * math.cos(params["angle"])
            wp.y_long = self.cur_local_pose.pose.position.y + params["distance"] * math.sin(params["angle"])
            print("flaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaag",wp.x_lat,wp.y_long)
            wp.z_alt = params["height"]
    
        takeoff_mini_wp = copy.deepcopy([wp.x_lat,wp.y_long,wp.z_alt])

        wpList.waypoints.append(self.toGlobal(wp))
        # wpList.waypoints.append(wp)
        return wpList, takeoff_mini_wp

    def addPassWps(self, wpList, wp_mini_list, params={}):
        for i in range(len(wp_mini_list)):
            wp = Waypoint()
            wp.frame = 3
            wp.command = 16
            wp.is_current = False
            wp.autocontinue = True

            if "acceptance_radius" not in params.keys():
                params["acceptance_radius"] = 1.0
            if "orbit_distance" not in params.keys():
                params["orbit_distance"] = 50.0
            if "yaw_angle" not in params.keys():
                params["yaw_angle"] = float('nan')

            wp.param1 = 0.0                                 # Ignored by fixed wing
            wp.param2 = params["acceptance_radius"]         # In meters
            wp.param3 = params["orbit_distance"]
            wp.param4 = params["yaw_angle"]                 # In meters, positive is clockwise

            wp.x_lat,wp.y_long,wp.z_alt = wp_mini_list[i][0],wp_mini_list[i][1],wp_mini_list[i][2]

            wpList.waypoints.append(self.toGlobal(wp))
            # wpList.waypoints.append(wp)

        return wpList

    def addLotierWps(self, wpList, wp_mini_list, params={}):
        for i in range(len(wp_mini_list)):
            wp = Waypoint()
            wp.frame = 3
            wp.is_current = False
            wp.autocontinue = True

            if "acceptance_radius" not in params.keys():
                params["type"] = "unlimited"

            try:
                if params['type'] == "unlimited":
                    if "acceptance_radius" not in params.keys():
                        params["radius"] = 40.0

                    wp.command = 17
                    wp.param3 = params['radius']                # Meters. Positive is clockwise

                elif params['type'] == "turns":
                    if "turns" not in params.keys():
                        params["turns"] = 5
                    elif "radius" not in params.keys():
                        params["radius"] = 40.0
                    elif "forward_moving" not in params.keys():
                        params["forward_moving"] = 0

                    wp.command = 18
                    wp.param1 = params['turns']                 # Number of turns
                    wp.param3 = params['radius']                # Meters. Positive is clockwise
                    wp.param4 = params['forward_moving']        # 0:center, 1: exit: location, else: yaw

                elif params['type'] == "time":
                    if "seconds" not in params.keys():
                        params["seconds"] = 5.0
                    elif "radius" not in params.keys():
                        params["radius"] = 40.0
                    elif "forward_moving" not in params.keys():
                        params["forward_moving"] = 0

                    wp.command = 19
                    wp.param1 = params['seconds']               # In decimal
                    wp.param3 = params['radius']                # Meters. Positive is clockwise
                    wp.param4 = params['forward_moving']        # 0:center, 1: exit location, else: yaw

                elif params['type'] == "alt":
                    if "heading" not in params.keys():
                        params["heading"] = 0
                    elif "radius" not in params.keys():
                        params["radius"] = 40.0
                    elif "forward_moving" not in params.keys():
                        params["forward_moving"] = 0

                    wp.command = 31
                    wp.param1 = params['heading']               # heading required. 0 = False
                    wp.param2 = params['radius']                # Meters. Positive is clockwise
                    wp.param4 = params['forward_moving']        # 0:center, 1: exit location, else: yaw

            except:
                print("Parameters introduced do not follow convention")

            wp.x_lat,wp.y_long,wp.z_alt = wp_mini_list[i][0],wp_mini_list[i][1],wp_mini_list[i][2]

            wpList.waypoints.append(self.toGlobal(wp))
            # wpList.waypoints.append(wp)

        return wpList

    def addLandWps(self, wpList, wp_mini_list, params={}):

        ### Mission command to perform a landing

        wp = Waypoint()
        wp.frame = 2
        wp.command = 189
        wp.is_current = False
        wp.autocontinue = True

        wp.x_lat,wp.y_long,wp.z_alt = 0.0,0.0,0.0          # Required?

        wpList.waypoints.append(self.toGlobal(wp))
        # wpList.waypoints.append(wp)

        ### Begin loiter at the specified Latitude and Longitude to get to an Altitude

        if "loiter_to_alt" not in params.keys():
            params["loiter_to_alt"] = {}

        if "heading" not in params["loiter_to_alt"].keys():
            params["loiter_to_alt"]["heading"] = 0
        if "radius" not in params["loiter_to_alt"].keys():
            params["loiter_to_alt"]["radius"] = 40.0                # Meters. Positive is clockwise
        if "forward_moving" not in params["loiter_to_alt"].keys():
            params["loiter_to_alt"]["forward_moving"] = 0        # 0:center, 1: exit location
        if "type" not in params["loiter_to_alt"].keys():
            params["loiter_to_alt"]["type"] = "waypoint"


        wp = Waypoint()
        wp.frame = 3
        wp.command = 31
        wp.is_current = False
        wp.autocontinue = True

        wp.param1 = params["loiter_to_alt"]["heading"]
        wp.param2 = params["loiter_to_alt"]["radius"]
        wp.param4 = params["loiter_to_alt"]["forward_moving"]

        if params["loiter_to_alt"]["type"] == "waypoint" and len(wp_mini_list) == 2:
            wp.x_lat,wp.y_long,wp.z_alt = wp_mini_list[0][0],wp_mini_list[0][1],wp_mini_list[0][2]
        elif params["loiter_to_alt"]["type"] == "by_angle" and len(wp_mini_list) == 1:
            if "distance" not in params["loiter_to_alt"].keys():
                params["loiter_to_alt"]["distance"] = 500.0
            if "angle" not in params.keys():
                current_pos = copy.deepcopy(self.cur_local_pose.pose.position)

                aux_vect = [current_pos.x-wp_mini_list[0][0],
                            current_pos.y-wp_mini_list[0][1],
                            current_pos.z-wp_mini_list[0][2]]

                params["loiter_to_alt"]["angle"] = np.arctan2(aux_vect[1],aux_vect[0])
            if "height" not in params.keys():
                params["loiter_to_alt"]["height"] = 10.0

            wp.x_lat = wp_mini_list[0][0] + params["loiter_to_alt"]["distance"] * math.cos(params["loiter_to_alt"]["angle"])
            wp.y_long = wp_mini_list[0][1] + params["loiter_to_alt"]["distance"] * math.sin(params["loiter_to_alt"]["angle"])
            wp.z_alt = params["loiter_to_alt"]["height"]

        wpList.waypoints.append(self.toGlobal(wp))
        # wpList.waypoints.append(wp)

        ### Land at location

        if "land" not in params.keys():
            params["land"] = {}

        if "abort_alt" not in params["land"].keys():
            params["land"]["abort_alt"] = float('nan')
        if "precision_mode" not in params["land"].keys():
            params["land"]["precision_mode"] = 0            # 0: normal, 1; oportunistic, 2: required
        if "yaw" not in params["land"].keys():
            params["land"]["yaw"] = float('nan')

        wp = Waypoint()
        wp.frame = 3
        wp.command = 21
        wp.is_current = False
        wp.autocontinue = True

        wp.param1 = params["land"]["abort_alt"]
        wp.param2 = params["land"]["precision_mode"]
        wp.param4 = params["land"]["yaw"]

        wp.x_lat,wp.y_long,wp.z_alt = wp_mini_list[-1][0],wp_mini_list[-1][1],wp_mini_list[-1][2]

        wpList.waypoints.append(self.toGlobal(wp))
        # wpList.waypoints.append(wp)

        return wpList


    '''
    Commanders
    '''

    def MoveCommand(self,move_type,poses_list,add_pos,params = {}):

        newWpList = []
        for wp in poses_list:
            print type(wp),wp

            newWpList.append([wp.position.x,wp.position.y,wp.position.z])
        # newWpList = poses_list

        wpList = self.WpListMixer(add_pos)

        if move_type == "takeoff":
            if self.ual_use:
                self.missionStarter()
            wpList, takeoff_mini_wp = self.addTakeOffWp(wpList,newWpList,params)
            wpList = self.addLotierWps(wpList,[takeoff_mini_wp])

        elif move_type == "pass":
            wpList = self.addPassWps(wpList,newWpList,params)
            wpList = self.addLotierWps(wpList,[newWpList[-1]])

        elif move_type == "loiter":
            wpList = self.addLotierWps(wpList,newWpList,params)

        elif move_type == "land":
            wpList = self.addLandWps(wpList,newWpList,params)
            wpList = self.addPassWps(wpList,[newWpList[-1]],params)

        if add_pos == 0:
            wpList.waypoints[0].is_current = True
        elif add_pos != 0:
            wpList.waypoints[wpList.start_index].is_current = True

        # print(wpList)
        # self.clearMission()
        # time.sleep(1)
        print(wpList)
        self.WpListSrvCall(wpList)
        time.sleep(0.5)
        self.setArmed(True)
        self.setFlightMode("AUTO.MISSION")

        while self.current_mission.current_seq + 1 != len(wpList.waypoints):
            time.sleep(1)

        # if move_type == "land":
        #     self.DisarmAtLand()

    def WpListMixer(self, add_pos = 2):

        wpList = WaypointPushRequest()

        if add_pos == 0:
            # self.ClearMissionSrvCall()
            wpList.start_index = 0

        elif add_pos == 1:

            wpList.waypoints = self.current_mission.waypoints[:self.current_mission.current_seq+1]
            wpList.waypoints[self.current_mission.current_seq].is_current = False
            wpList.start_index = self.current_mission.current_seq

        elif add_pos == 2:

            wpList.waypoints = self.current_mission.waypoints
            wpList.waypoints[self.current_mission.current_seq].is_current = False
            wpList.start_index = len(self.current_mission.waypoints)

        return wpList

    def missionStarter(self):

        self.ClearMissionSrvCall()
        self.setFlightMode("AUTO.LAND")
        if self.ual_use:
            self.setArmed(False)
        self.setArmed(True)
    
        self.setFCUParam('NAV_DLL_ACT',0)
        # time.sleep(1)
        self.setHome()

    def setArmed(self,value = True):
        last_request = time.time()

        if value == True:
            print("Trying to arm...")
        elif value == False:
            print("Trying to disarm...")

        self.mavros_state = []
        while self.mavros_state == []:
            time.sleep(0.5)

        while (not self.mavros_state.armed == value) and (time.time() - last_request < 5):
            self.setArmedSrvCall(value)
            last_request = time.time()
            time.sleep(0.2)
        if value == True:
            print("Armed!")
        elif value == False:
            print("Disarmed!")

    def clearMission(self):
        last_request = time.time()

        print("Trying to clear mission...")

        while (not self.current_mission.waypoints == []) and (time.time() - last_request < 5):
            self.ClearMissionSrvCall()
            last_request = time.time()
            time.sleep(0.2)

        print("Mission clear!")

    def setFlightMode(self,mode,topic_call = None):
        last_request = time.time()
        print("Trying to set {} mode".format(mode))
        while (self.mavros_state.mode != mode):
            self.setFlightModeSrvCall(mode)
            if topic_call:
                topic_call()
            last_request = time.time()
            time.sleep(0.2)
        print("{} mode set!".format(mode))

    def setFCUParam(self,param,value):
        rospy.wait_for_service('{0}/mavros/param/set'.format(self.ual_prefix))
        try:
            request = ParamSetRequest()
            request.param_id = param
            request.value.integer = value
            proxy = rospy.ServiceProxy('{0}/mavros/param/set'.format(self.ual_prefix), ParamSet)
            response = proxy(request)
            if response.success == True:
                print("{0} param set to {1}".format(param,value))
            else:
                print("{} param could not be set".format(param))
                # print("response received:", response)
            return

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in ParamSet"

    def DisarmAtLand(self):
        while ((not self.mavros_extended_state.landed_state == 4) or
               (not self.cur_local_pose.pose.position.z <= self.local_start_pos.pose.position.z + 0.2)):
            print("Waiting to land. Actual altitude:",self.cur_global_pose.altitude - self.global_start_pos.altitude - 0.2)
            time.sleep(0.2)
        self.setArmed(False)

    def setHome(self):

        # self.setHomePosReqUpdtSrvCall()

        self.cur_global_pose = []

        while self.cur_global_pose == []:
            time.sleep(0.5)

        origin_geo = GeoPoint(self.cur_global_pose.latitude,
                              self.cur_global_pose.longitude,
                              self.cur_global_pose.altitude)
        self.geoLocalPose = GeoLocalPose(origin_geo)

        self.local_start_pos = self.cur_local_pose
        self.global_start_pos = self.cur_global_pose

    def toGlobal(self,wp):

        actual_coordinate_geo = self.geoLocalPose.setGeo(wp)

        wp.x_lat = actual_coordinate_geo.latitude
        wp.y_long = actual_coordinate_geo.longitude
        wp.z_alt = actual_coordinate_geo.altitude

        return wp

# FwQgc()

# make posix gazebo_plane
# roslaunch mavros px4.launch fcu_url:=udp://:14550@localhost:14556