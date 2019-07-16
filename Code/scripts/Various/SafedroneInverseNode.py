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
    def __init__(self,ids = [1,2,3], ns_prefix = "uav_",origin_geo = [37.558542, -5.931074,0.0]):

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

        if self.use_multi_polython == True:

            auxiliar_polygons = self.Global2Local(self.polygons_geo)
            main_polygon_array = auxiliar_polygons["1"]
            print("main_polygon_array",main_polygon_array)

            # main_polygon_array = [[-150.01255746543757, 149.85376389184967, -12.5], [-125.65221288704197, -159.8376554097049, -50.0], [105.37052664515795, -236.5466791386716, -50.0], [1026.092006954481, -168.59934886405244, -50.0], [1155.3705004769145, 90.11888963589445, -50.0], [995.6826912371034, 315.442073025275, -50.0], [582.8812200819957, 104.96125665307045, -50.0]]
            # main_polygon_array = [[-150,150,10],[3,70],[150,150],[150,-150],[3,-190],[-150,-150]]
            # main_polygon_array = [[-150,150,10],[150,150],[150,-150],[-150,-150]]
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
        limits = [[100000000,-100000000],[100000000,-100000000]]
        polygons_dict = {}

        for i in range(main_polygon_vertex_num):

            point1 = main_polygon_array[i]
            point2 = main_polygon_array[(i+1) % main_polygon_vertex_num]

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

            self.main_polygon_width = limits[1][1] - limits[1][0]

        print("limits", limits)

        main_polygon = Polygon(*vertex_points_list)

        main_polygon_area = abs(round(main_polygon.area,2))

        print("main_polygon_area", main_polygon_area)

        x_init = copy.deepcopy(limits[0][0])
        y_init = copy.deepcopy(limits[1][0])

        prior_cross_segment = Segment(Point(-5 +limits[0][0],limits[1][0]),Point(-5 +limits[0][0],limits[1][1]))
        allocated_segments = []

        for i in range(len(self.ids)):

            print("NEW ID!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("id", i)

            area_error_threshold = main_polygon_area*0.015
            area_error = 10000000

            y1 = y_init
            y2 = copy.deepcopy(limits[1][1])

            x_increment = (limits[0][1] - x_init) / 2
            cross_section_distance = copy.deepcopy(x_increment)

            while area_error_threshold < abs(area_error) and not rospy.is_shutdown():

                print("cross_section_distance", cross_section_distance)

                polygon_vertex_list = []
                
                # cross_section_distance = (limits[0][1] - x_init) / 2

                x1 = x_init + cross_section_distance
                x2 = x1

                cross_segment = Segment(Point(x1,y1),Point(x2,y2))

                if i == len(self.ids)-1:

                    cross_segment = Segment(Point(limits[0][1]+5,y1),Point(limits[0][1]+5,y2))

                # print("cross_sections", prior_cross_segment,cross_segment)

                # area calculus

                for j in range(len(perim_segments)):

                    print("segment", j)

                    p0_side = "center"
                    
                    if perim_segments[j].points[0].x < prior_cross_segment.points[0].x:

                        p0_side = "left"

                    if perim_segments[j].points[0].x > cross_segment.points[0].x:

                        p0_side = "right"

                    # print("p0_side",p0_side)

                    detected_prior_intersections = perim_segments[j].intersection(prior_cross_segment)
                    detected_last_intersections = perim_segments[j].intersection(cross_segment)

                    detected_intersections = [detected_prior_intersections,detected_last_intersections]

                    # print("detected_intersections",detected_intersections)

                    which_intersected = [False,False]

                    if detected_prior_intersections != []:
                        which_intersected[0] = True

                    if detected_last_intersections != []:
                        which_intersected[1] = True

                    print("which_intersected", which_intersected)

                    if which_intersected.count(True) == 0:

                        print("not intersected")

                        if p0_side == "center":

                            polygon_vertex_list.append(perim_segments[j].points[0])
                            allocated_segments.append(j)


                    elif which_intersected.count(True) == 1:

                        print("intersected once")

                        if p0_side == "center":

                            polygon_vertex_list.append(perim_segments[j].points[0])
                            intersection_point = detected_intersections[which_intersected.index(True)][0]
                            polygon_vertex_list.append(intersection_point)

                        if p0_side != "center":

                            intersection_point = detected_intersections[which_intersected.index(True)][0]
                            polygon_vertex_list.append(intersection_point)
                            polygon_vertex_list.append(perim_segments[j].points[1])

                        # print(perim_segments[j])
                        # print(intersection_point)


                    elif which_intersected.count(True) == 2:                        

                        print("intersected twice")
                        print("intersections", detected_intersections)
                        intersection_point_0 = detected_intersections[0][0]
                        p0 = perim_segments[j].points[0]
                        distances = [p0.distance(intersection_point_0)]

                        intersection_point_1 = detected_intersections[1][0]
                        p0 = perim_segments[j].points[0]
                        distances.append(p0.distance(intersection_point_1))
                        print(distances.index(min(distances)),(distances.index(min(distances))+1)%2)
                        polygon_vertex_list.append(detected_intersections[distances.index(min(distances))][0])
                        polygon_vertex_list.append(detected_intersections[(distances.index(min(distances))+1)%2][0])
                        print("intersections", detected_intersections)
                        print("nearers",polygon_vertex_list[-2:])

                        print(polygon_vertex_list)

                polygon = Polygon(*polygon_vertex_list)

                area = abs(polygon.area)
                print("polygon",polygon)
                print("area", area)
                print("ITERATION COMPLETED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

                # area check

                area_error = area - (main_polygon_area / len(self.ids))

                print("area error", area_error)

                if area_error_threshold < abs(area_error):

                    x_increment = x_increment / 2
                    print("x_increment",x_increment)
                    cross_section_distance += x_increment* np.sign(-area_error)

                if i == len(self.ids)-1:
                    area_error = 0


            polygon_array = []
            for vertex in polygon.vertices:
                polygon_array.append([float(vertex.x),float(vertex.y),float(main_polygon_array[0][2])])
            polygons_dict[str(self.ids[i])] = polygon_array
            prior_cross_segment = cross_segment

        print("polygons_dict", polygons_dict)

        return polygons_dict
        

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
            spacing = self.main_polygon_width / ( len(self.ids) * 4 )
            poseset_data["spacing"] = [spacing,spacing]
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