#!/usr/bin/env python

import sys
import os
import rospy
import std_msgs.msg
import time
import math
import numpy as np
import Brain
import pandas as pd
import argparse
import tf, tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from uav_abstraction_layer.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

from Brain import *
from gauss.srv import *

class Ground_Station(object):

    def __init__(self,ID):
        self.ID = int(ID)
        self.brain = Brain(self.ID)
        self.global_data_frame = pd.DataFrame()
        self.distance_to_goal = 10000
        self.world_definition = rospy.get_param('world_definition')
        self.project = self.world_definition['project']
        self.world_type = self.world_definition['type']
        self.n_simulation = self.world_definition['n_simulation']
        self.N_uav = self.world_definition['N_uav']
        self.N_obs = self.world_definition['N_obs']
        self.uav_models = self.world_definition['uav_models']
        self.n_dataset = self.world_definition['n_dataset']
        self.solver_algorithm = self.world_definition['solver_algorithm']
        self.obs_pose_list_simple = self.world_definition['obs_pose_list_simple']


        self.start=time.time()

        self.ANSP_instruction = ""

        self.uav_frame_list = []
        for n_uav in np.arange(self.N_uav):
            self.uav_frame_list.append(rospy.get_param( 'uav_{}_home'.format(n_uav+1)))

        self.actual_uav_pose_list = list(np.arange(self.N_uav))
        self.actual_uav_vel_list = list(np.arange(self.N_uav))
        rospy.init_node('ground_station_{}'.format(self.ID), anonymous=True)
        self.state = "landed"
        self.collision = False
        self.new_path_incoming = False
        self.die_command = False

        self.GroundStationListener()
        print "ground station",ID,"ready and listening"

        self.GroundStationCommander()

        return

    #### Listener functions ####
    def GroundStationListener(self):
        for n_uav in np.arange(self.N_uav):
            rospy.Subscriber('/uav_{}/ual/pose'.format(n_uav+1), PoseStamped, self.uav_pose_callback,n_uav+1)
            rospy.Subscriber('/uav_{}/ual/velocity'.format(n_uav+1), TwistStamped, self.uav_vel_callback,n_uav+1)

        # rospy.Subscriber('/typhoon_h480_{}/r200/r200/depth/image_raw'.format(self.ID), Image, self.image_raw_callback)
        rospy.Service('/gauss/ANSP_UAV_{}/WP_list_command'.format(self.ID), WpPathCommand, self.handle_WP_list_command)
        rospy.Service('/gauss/ANSP_UAV_{}/instruction_command'.format(self.ID), InstructionCommand, self.handle_ANSP_instruction)
        rospy.Service('/gauss/ANSP_UAV_{}/die_command'.format(self.ID), DieCommand, self.handle_die)


    def uav_pose_callback(self,data,ID):
        self.actual_uav_pose_list[ID-1] = data.pose

        time.sleep(0.1)

    def uav_vel_callback(self,data,ID):
        self.actual_uav_vel_list[ID-1] = data.twist

        time.sleep(0.1)

    def image_raw_callback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose.position)
        #print "Image info recieved", data.height, data.width, data.encoding, data.is_bigendian, data.step
        # bridge = CvBridge()
        # self.image_depth = bridge.imgmsg_to_cv2(data, desired_encoding=data.encoding)[:,:,1]
        self.image_depth = data
        time.sleep(0.1)
        return

    def handle_WP_list_command(self,req):
        self.goal_path_poses_list = req.goal_path_poses_list
        self.distance_to_goal = 10000
        self.new_path_incoming = True
        return True
        
    def handle_ANSP_instruction(self,req):
        self.ANSP_instruction = req.instruction
        time.sleep(1)
        self.ANSP_instruction = ""
        return True

    def handle_die(self,req):
        self.die_command = True               
        return True

    #### Publisher functions #####
    def GoalStaticBroadcaster(self):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()
    
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "uav_{}_goal".format(self.ID)
    
        static_transformStamped.transform.translation = self.goal_WP_pose.position
    
        quat = tf.transformations.quaternion_from_euler(0,0,0)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]
    
        broadcaster.sendTransform(static_transformStamped)


    #### Commander functions ####
    def GroundStationCommander(self):
        while self.die_command != True:
            data_saved = True
            if self.new_path_incoming and self.state == "landed":
                ### Take Off
                self.TakeOffCommand(5,True)
                self.state = "inizializating"
                self.ANSPStateActualization()
                data_saved = False

            if self.new_path_incoming and (self.state == "inizializating" or self.state == "following_path"): ##### cambiar folloing path por in/to WP *
                ### Path follow
                self.new_path_incoming = False
                self.PathFollower()
                if self.new_path_incoming == False:
                    self.state = "stabilizing"
                    self.ANSPStateActualization()
                else:
                    pass

            if self.state == "stabilizing":
                self.SetVelocityCommand(True)
                time.sleep(0.1)
                self.state = "hovering"
                self.ANSPStateActualization()

            if data_saved == False:
                self.CreatingCSV()
                data_saved = True

            if self.state == "hovering":
                ### Land 
                self.LandCommand(True)
                self.state = "landed"
                self.ANSPStateActualization()
                
            if self.die_command == True:
                rospy.signal_shutdown("end of experiment")
                return

            time.sleep(0.5)

        return 

    def LandCommand(self,blocking):
        rospy.wait_for_service('/uav_{}/ual/go_to_waypoint'.format(self.ID))
        try:
            ual_land = rospy.ServiceProxy('/uav_{}/ual/land'.format(self.ID), Land)
            ual_land(blocking)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def GoToWPCommand(self,blocking):
        rospy.wait_for_service('/uav_{}/ual/go_to_waypoint'.format(self.ID))
        try:
            ual_go_to_waypoint = rospy.ServiceProxy('/uav_{}/ual/go_to_waypoint'.format(self.ID), GoToWaypoint)
            h = std_msgs.msg.Header()
            ual_go_to_waypoint(PoseStamped(h,self.goal_WP_pose),blocking)
            while self.DistanceToGoal() > 0.2:
                time.sleep(0.1)
            time.sleep(1)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def SetVelocityCommand(self,hover):
        rospy.wait_for_service('/uav_{}/ual/set_velocity'.format(self.ID))
        try:
            #print "velocity command"
            ual_set_velocity = rospy.ServiceProxy('/uav_{}/ual/set_velocity'.format(self.ID), SetVelocity)
            if hover== False:
                self.new_velocity_twist = self.brain.Guidance(self.actual_uav_pose_list,self.actual_uav_vel_list,self.goal_WP_pose)
                self.SaveData()

            elif hover == True:
                self.new_velocity_twist = self.brain.Hover()
                pass

            h = std_msgs.msg.Header()

            self.finish = time.time()
            # print self.finish - self.start
            self.start = time.time()

            ual_set_velocity(TwistStamped(h,self.new_velocity_twist))

            #print "veclocity move done"
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def TakeOffCommand(self,heigth, blocking):
        rospy.wait_for_service('/uav_{}/ual/take_off'.format(self.ID))
        try:
            ual_set_velocity = rospy.ServiceProxy('/uav_{}/ual/take_off'.format(self.ID), TakeOff)
            ual_set_velocity(heigth,blocking)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def DistanceToGoal(self):
        self.distance_to_goal = math.sqrt((self.actual_uav_pose_list[self.ID-1].position.x-self.goal_WP_pose.position.x)**2+(self.actual_uav_pose_list[self.ID-1].position.y-self.goal_WP_pose.position.y)**2+(self.actual_uav_pose_list[self.ID-1].position.z-self.goal_WP_pose.position.z)**2)
        print self.ID, self.distance_to_goal
        return self.distance_to_goal

    def DistanceBetweenPoses(self,pose_1,pose_2):
        Distance = math.sqrt((pose_1.position.x-pose_2.position.x)**2+(pose_1.position.y-pose_2.position.y)**2+(pose_1.position.z-pose_2.position.z)**2)
        return Distance

    def DistanceToObs(self,pose_1,vector):
        Distance = math.sqrt((pose_1.position.x-vector[0])**2+(pose_1.position.y-vector[1])**2+(pose_1.position.z-vector[2])**2)
        return Distance

    def PathFollower(self):
        for i in np.arange(len(self.goal_path_poses_list)):
            self.goal_WP_pose = self.goal_path_poses_list[i]
            self.GoalStaticBroadcaster()
            self.state = "to WP {}".format(i+1)
            self.ANSPStateActualization()
            while self.DistanceToGoal() > 0.5:
                self.SetVelocityCommand(False)
                time.sleep(0.1)
                if self.new_path_incoming == True:
                    break
            self.goal_WP_pose = self.goal_path_poses_list[i]
            self.state = "in WP {}".format(i+1)
            self.ANSPStateActualization()
            self.SetVelocityCommand(True)
            self.Evaluator()

            if self.die_command == True:
                rospy.signal_shutdown("end of experiment")

            if self.new_path_incoming == True:
                break

            if self.project == "gauss":
                while self.ANSP_instruction != "GoOn":
                    if self.die_command == True:
                        rospy.signal_shutdown("end of experiment")
                    time.sleep(0.1)

    def Evaluator(self):
        min_distance_uav = 2
        min_distance_obs = 2
        uav_distances_list = []
        obs_distances_list = []
        for n_uav in np.arange(self.N_uav):
            if n_uav+1 != self.ID:
                uav_distances_list.append(self.DistanceBetweenPoses(self.actual_uav_pose_list[self.ID-1],self.actual_uav_pose_list[n_uav]))

        for n_obs in np.arange(self.N_obs):
            obs_distances_list.append(self.DistanceToObs(self.actual_uav_pose_list[self.ID-1],self.obs_pose_list_simple[n_obs]))

        collision_uav = (x for x in uav_distances_list if x <= min_distance_uav)
        collision_obs = (x for x in obs_distances_list if x <= min_distance_obs)

        if collision_uav or collision_obs:
            self.collision = True
            print self.ID,"COLLISION!!!!!!!!!!!!!!!!!!!!!!!!!!"


    def SaveData(self):
        single_frame = {"goal_UAV_{}_pose".format(self.ID): [self.goal_WP_pose], "UAV_{}_new_velocity_twist".format(self.ID) : [self.new_velocity_twist]}   #  , "UAV_{}_image_depth".format(self.ID): [self.image_depth]
        for n_uav in np.arange(self.N_uav):
            single_frame["actual_UAV_{}_pose".format(n_uav+1)] = self.actual_uav_pose_list[n_uav]
            single_frame["actual_UAV_{}_vel".format(n_uav+1)] = self.actual_uav_vel_list[n_uav]

        if self.global_data_frame.empty:
            self.global_data_frame = pd.DataFrame(single_frame)
        else:
            self.global_data_frame = self.global_data_frame.append(pd.DataFrame(single_frame),ignore_index=True)

    def CreatingCSV(self):
        print "saving uav",self.ID,"mission data"
        folder_path = "/home/joseandres/catkin_ws/src/jamrepo/Simulation_data/{0}/type{1}_Nuav{2}_Nobs{3}/dataset_{4}/simulation_{5}".format(self.project,self.world_type,self.N_uav,self.N_obs,self.n_dataset,self.n_simulation)


        file_path = folder_path + '/uav_{0}.csv'.format(self.ID)
        self.global_data_frame.to_csv(file_path, sep=',') #,low_memory=False,

    def ANSPStateActualization(self):
        rospy.wait_for_service('/gauss/ANSP/State_Actualization')
        try:
            # print "path for uav {} command".format(ID)
            ANSP_state_actualization = rospy.ServiceProxy('/gauss/ANSP/State_Actualization', StateActualization)
            ANSP_state_actualization(self.ID,self.state,self.collision)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

def main():                #### No estoy seguro de toda esta estructura
    parser = argparse.ArgumentParser(description='Spawn robot in Gazebo for SITL')
    parser.add_argument('-ID', type=str, default="1", help='')
    args, unknown = parser.parse_known_args()
    Ground_Station(args.ID)
    return

if __name__ == '__main__':
    main()