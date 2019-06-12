import rospy
import time
import numpy as np
import tf2_ros
import tf
import math
import collections
import copy
import json
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_srvs.srv import *
from geographic_msgs.msg import GeoPoint
from sympy import Point, Polygon, Line, Segment

from GeoLocalPose import GeoLocalPose

class SafedroneInverseNode(object):
    def __init__(self,ids = [1,2], ns_prefix = "uav_",origin_geo = [37.558542, -5.931074,0.0]):

        self.ids = ids
        self.ns_prefix = ns_prefix

        self.use_multi_polython = True

        self.polygons_geo = {}
        self.polygons_geo["Origin"] = [origin_geo]

        rospy.init_node('SafedroneInverseNode', anonymous=True)

        self.listener()

        self.MavrosPullMission()

        time.sleep(1)

        if self.use_multi_polython == False:

            polygons_local = self.Global2Local(self.polygons_geo)

        elif self.use_multi_polython == True:

            auxiliar_polygons = self.Global2Local(self.polygons_geo)
            main_polygon_array = auxiliar_polygons["1"]

            # main_polygon_array = [[-50,-50],[-50,50],[50,50],[50,-50]]
            polygons_local = self.areaAllocation(main_polygon_array)

        self.writeJSON(self.createJSONdata(polygons_local))

    '''
    Publishers
    '''
    def InitializePublishers(self):

        pass

    '''
    Listeners
    '''

    def listener(self):

        for id in self.ids:

            if (self.use_multi_polython == True and id == 1) or self.use_multi_polython == False:

                ual_prefix = "/{0}{1}".format(self.ns_prefix,id)
                rospy.Subscriber('{}/mavros/mission/waypoints'.format(ual_prefix), WaypointList, self.current_mission_list_callback,id)


    def current_mission_list_callback(self,data,id):

        poses_list = []
        for wp in data.waypoints:
            poses_list.append([wp.x_lat, wp.y_long, wp.z_alt])

        self.polygons_geo["{}".format(id)] = poses_list

        print("Retrieved {} waypoints from UAV {}".format(len(data.waypoints),id))


    '''
    Commanders
    '''

    def areaAllocation(self,main_polygon_array):

        perim_segments = []
        vertex_points_list = []
        main_polygon_vertex_num = len(main_polygon_array)
        limits = [[-100000000,100000000][-100000000,100000000]]

        for i in range(main_polygon_vertex_num):

            point1 = main_polygon_array[i]
            point2 = main_polygon_array[(i+1)%main_polygon_vertex_num]

            perim_segments.append(Segment(Point(round(point1[0],2),round(point1[1],2)), Point(round(point2[0],2),round(point2[1],2))))

            vertex_points_list.append(Point(point1[0],point1[1]))

            if point1[0] < limits[0][0]:
                limits[0][0] = point1[0]
            if point1[0] > limits[0][1]:
                limits[0][1] = point1[0]
            if point1[1] < limits[1][0]:
                limits[1][0] = point1[1]
            if point1[1] > limits[1][1]:
                limits[1][1] = point1[1]

        main_polygon = Polygon(*vertex_points_list)

        main_polygon_area = abs(round(main_polygon.area,2))

        cross_segments = []

        pad = 

        x_init = deep.copy(limits[0][0])
        y_init = deep.copy(limits[1][0])

        for i in len(self.ids)-1:

            area_threshold = 10
            area_error = 10000000

            x1 = x_init + (limits[0][1] - x_init) / 2
            x2 = x1
            y1 = y_init
            y2 = deep.copy(limits[1][1])

            while area_threshold < area_error:

                x1 = x_init + x_increment

                try_cross_segment = Segment(Point(x1,y1),Point(x2,y2))




            cross_segments.append(deep.copy(try_cross_segment))

        



    def Global2Local(self, polygons_geo):

        polygons_local = {}

        origin_geo = polygons_geo["Origin"][0]
        new_coordinate_geo = GeoPoint(*origin_geo)

        for key in self.polygons_geo.keys():
            vertex_local_list = []
            for new_geo in polygons_geo[key]:
                new_geo = GeoPoint(*new_geo)
                geo_local_pose = GeoLocalPose(new_geo)
                local_pose = geo_local_pose.geoToUtmToCartesian(new_coordinate_geo)
                vertex_local_list.append([-local_pose.x,-local_pose.y,local_pose.z])

            polygons_local[key] = vertex_local_list

        return polygons_local

    def createJSONdata(self,polygons_local):

        volume_data = {}
        volume_data["name"] = "PolygonsFromUnifly"
        volume_data["prefix"] = "PFU"
        volume_data["origin"] = [[0.0,0.0,0.0],[0.0,0.0,0.0]]
        volume_data["permits"] = "geofence"
        volume_data["geometries"] = []

        for uav_id in polygons_local.keys():

            geometry_data = {}         
            geometry_data["name"] = "Polygon_{}".format(uav_id)
            geometry_data["prefix"] = "P{}".format(uav_id)
            geometry_data["shape"] = "prism"
            geometry_data["id"] = 0
            geometry_data["origin"] = [[0.0,0.0,10.0],[0.0,0.0,0.0]]
            geometry_data["dimensions"] = [3.0,polygons_local[uav_id]]
            geometry_data["color"] = [85,255,255]
            geometry_data["alpha"] = 0.3
            geometry_data["poses_sets"] = []

            poseset_data = {}
            poseset_data["type"] = "zigzag"
            poseset_data["use"] = "poses"
            poseset_data["orientation"] = [0,0,0]
            poseset_data["height"] = 2.5
            poseset_data["spacing"] = [10,10]
            poseset_data["sweep_angle"] =-1.5708
            poseset_data["initial_sense"] = "left"
            poseset_data["margins"] = [1.0,1.0]

            geometry_data["poses_sets"] = [poseset_data]

            volume_data["geometries"].append(geometry_data)

        data = { "scenario" : { "volumes" : [volume_data]} }

        print(data)

        return data


    def writeJSON(self, data):
        
        file_path = "/home/joseandresmr/catkin_ws/src/magna/Code/JSONs/Worlds/Safedrone/volumes_from_unifly.json"

        with open(file_path, 'w') as outfile:  
            json.dump(data, outfile)

    def MavrosPullMission(self):

        for uav_id in self.ids:
            rospy.wait_for_service('/uav_{}/mavros/mission/pull'.format(uav_id))
            try:
                # print "path for agent {} command".format(ID)
                mavros_pull_mission = rospy.ServiceProxy('/uav_{}/mavros/mission/pull'.format(uav_id), WaypointPull)
                mavros_pull_mission()
                
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
                print "error in state_actualization"

        return

SafedroneInverseNode()

# make posix gazebo_plane
# roslaunch mavros px4.launch fcu_url:=udp://:14550@localhost:14556