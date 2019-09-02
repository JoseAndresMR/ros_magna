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

import sys
import rospy
import std_msgs.msg
import time
import math
import numpy as np
import tf
# import rvo2
import rvo23d
import time
# from cv_bridge import CvBridge, CvBridgeError
from uav_abstraction_layer.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
# import tensorflow as tflow
# from tensorflow.python.tools import inspesct_checkpoint as chkp

class Agent_NAI(object):
    def __init__(self,ID):
        # Local parameters inizialization from arguments
        self.ID = ID

        self.smooth_path_mode = 0

        self.algorithms_dicc = {}

        self.GettingWorldDefinition()    # Global ROS parameters inizialization
        # self.timer_start = time.time()

    # Function to decide which algorithm is used for new velocity depending on parameters
    def Guidance(self,desired_speed):

        self.NeighborSelector(int(self.algorithms_dicc["orca3"]["N_neighbors_aware"])+1)
        self.desired_speed = desired_speed

        # print "loop time", time.time() - self.timer_start
        # self.timer_start = time.time()

        if "simple" in self.algorithms_dicc.keys():
            return self.SimpleGuidance()

        if "neural_network" in self.algorithms_dicc.keys():
            return self.NeuralNetwork()

        # elif self.algorithms_dicc[0] == "orca":
        #     return self.ORCA()

        if "orca3" in self.algorithms_dicc.keys():
            return self.ORCA3()
            # return self.ORCA3_from_node()
            

    # Function to set new velocity using a Neural Network
    def NeuralNetwork(self):

                # If the solver is a neural network, make some additional initializations
        if not hasattr(self,'nn_loaded'):

            self.nn_loaded = True
            self.session = tflow.Session()      # Start a TensorFlow session

            self.learning_dataset_def = {"teacher_role" : self.role,
                                         "teacher_algorithm" : "orca3",
                                         "N_neighbors_aware" : self.algorithms_dicc["neural_network"]["N_neighbors_aware"]}

            gml_folder_path = "/home/{0}/Libraries/gml".format("joseandresmr")
            self.session_path = gml_folder_path + "/Sessions/{0}/{1}/{2}".format(self.learning_dataset_def["teacher_role"],self.learning_dataset_def["teacher_algorithm"],self.learning_dataset_def["N_neighbors_aware"])

            # Import the metagraph from specific path. In the future will be better path management
            new_saver = tflow.train.import_meta_graph(self.session_path + "/model.meta")

            # Restore to the last chechpoint
            new_saver.restore(self.session,tflow.train.latest_checkpoint(self.session_path))

            # Initialize inputs and outputs from graph
            self.graph_inputs = tflow.get_default_graph().get_tensor_by_name("single_input:0")
            self.graph_outputs = tflow.get_default_graph().get_tensor_by_name("vel_posttreated:0")

            # self.single_vel_logits_tensor = tflow.get_default_graph().get_tensor_by_name("single_vel_logits:0")


        # Definition of neural network's inputs and outputs for every role.
        # In the future this will be imported from a common place
        if self.role == "path":
            input_dicc = ['own_vel','goal_pose_rel','others_pos_rel','others_vel']
            output_dicc = ["sel_vel"]
        elif self.role == "agent_ad":
            input_dicc = ['own_vel','goal_pose_rel','goal_vel','distance','others_pos_rel','others_vel']
            output_dicc = ["sel_vel"]
        elif self.role == "agent_ap":
            input_dicc = ['own_vel','goal_pose_rel','goal_vel','others_pos_rel','others_vel']

        output_dicc = ["sel_vel"]

        # Initialization of pos and vel that will be taken as inputs
        inputs = []
        main_agent_pos = self.agents_data_list[self.ID-1].position.pose
        main_agent_vel = self.agents_data_list[self.ID-1].velocity.twist.linear

        # For every input in the dictionary, crate if needed and add it to inputs
        for n_input in input_dicc:

            # own vel
            if n_input == "own_vel":
                inputs += [main_agent_vel.x,main_agent_vel.y,main_agent_vel.z]

            # own goal
            elif n_input == "goal_pose_rel":
                goal_lin_rel = self.OperatePoses(self.goal["pose"],main_agent_pos,'-').position
                inputs += [goal_lin_rel.x,goal_lin_rel.y,goal_lin_rel.z]

            elif n_input == "goal_vel":
                inputs += [self.goal["vel"].linear.x,self.goal["vel"].linear.y,self.goal["vel"].linear.z]

            elif n_input == "distance":
                inputs.append(self.goal["dist"])

            elif n_input == "others_pos_rel":
                for n_neighbor in range(self.algorithms_dicc["neural_network"]["N_neighbors_aware"]):
                    if self.near_neighbors_sorted["types"][n_neighbor] == "agent":
                        n_agent = self.near_neighbors_sorted["ids"][n_neighbor]

                        other_pos_rel = self.OperatePoses(self.agents_data_list[n_agent].position.pose,main_agent_pos,'-').position
                        inputs += [other_pos_rel.x,other_pos_rel.y,other_pos_rel.z]

                    elif self.near_neighbors_sorted["types"][n_neighbor] == "obs":
                        n_obs = self.near_neighbors_sorted["ids"][n_neighbor]
                        obs_pose = self.obs_pose_list[n_obs]

                        other_pos_rel = self.OperatePoses(self.PoseFromArray(obs_pose),main_agent_pos,'-').position

                        inputs += [other_pos_rel.x,other_pos_rel.y,other_pos_rel.z]


            elif n_input == "others_vel":
                for n_neighbor in range(self.algorithms_dicc["neural_network"]["N_neighbors_aware"]):
                    if self.near_neighbors_sorted["types"][n_neighbor] == "agent":
                        n_agent = self.near_neighbors_sorted["ids"][n_neighbor]
                        other_vel_lin = self.agents_data_list[n_agent].velocity.twist.linear
                        inputs += [other_vel_lin.x,other_vel_lin.y,other_vel_lin.z]

                    elif self.near_neighbors_sorted["types"][n_neighbor] == "obs":

                        inputs += [0,0,0]

        # Reshape the inputs to a single row
        inputs_trans = np.asarray(inputs)
        inputs_trans = inputs_trans.reshape((1, inputs_trans.shape[0]))

        # Run session once to predict new selected velocity
        selected_velocity = self.session.run(self.graph_outputs, feed_dict={self.graph_inputs:inputs_trans})

        # Depict the output into the defined variables
        output_index = 0
        for n_output in output_dicc:
            if n_output == "sel_vel":
                selected_velocity = selected_velocity[0][output_index:output_index+3]

                # Construct the twist
                new_velocity_twist = Twist(Vector3(selected_velocity[0],selected_velocity[1],selected_velocity[2]),Vector3(0,0,0))
                output_index += 3

        # print("nn",new_velocity_twist)
        # self.ORCA3()
        print(new_velocity_twist)
        return new_velocity_twist

    # Function to set velocity using ORCA on 3D
    def ORCA3(self):

        self.algorithms_dicc["orca3"]["N_neighbors_aware"] = int(self.algorithms_dicc["orca3"]["N_neighbors_aware"])

        params_dicc = self.algorithms_dicc["orca3"]

        # Give value to orca algorithm parameters
        timeStep = params_dicc["timeStep"]          # 1/60.  float   The time step of the simulation. Must be positive.
        neighborDist = params_dicc["neighborDist"]     # 1.5    float   The maximal distance (center point to center point) to other agents the agent takes into account in the navigation
        maxNeighbors = params_dicc["N_neighbors_aware"]  # 5      size_t  The maximal number of other agents the agent takes into account in the navigation
        timeHorizon = params_dicc["timeHorizon"]  # 2.5    float   The minimal amount of time for which the agent's velocities that are computed by the simulation are safe with respect to other agents.
        agent_radius = params_dicc["agent_radius"]        # 2      float   The radius of the agent. Must be non-negative
        maxSpeed = params_dicc["maxSpeed"]          # 0.4    float   The maximum speed of the agent. Must be non-negative.
        velocity = (1, 1, 1)

        obs_radius = 0.5

        # Create an object of orca3 solver class and give the above defined parameters
        sim = rvo23d.PyRVOSimulator(params_dicc["timeStep"], params_dicc["neighborDist"], params_dicc["N_neighbors_aware"], params_dicc["timeHorizon"], params_dicc["agent_radius"], params_dicc["maxSpeed"], velocity)

        # Select nearest Agents and Neighbors
        orca_agent_list = []

        prefered_velocity = self.SimpleGuidance() # Select a velocity directly to goal as if there weren't exist neighbors
        # Add to orca3 and to own list every agent created by own params
        position_array = self.ArrayFromPose(self.agents_data_list[self.ID-1].position.pose)[0]
        velocity_array = self.ArrayFromTwist(self.agents_data_list[self.ID-1].velocity.twist)[0]
        prefered_velocity_array = self.ArrayFromTwist(prefered_velocity)[0]

        orca_agent_list = [sim.addAgent((position_array[0],position_array[1],position_array[2]),
            params_dicc["neighborDist"], params_dicc["N_neighbors_aware"], params_dicc["timeHorizon"], params_dicc["agent_radius"], params_dicc["maxSpeed"], (velocity_array[0],velocity_array[1],velocity_array[2]))]
        # Set the prefered velocity of own Agent as decided avobe
        sim.setAgentPrefVelocity(orca_agent_list[0],(prefered_velocity_array[0],prefered_velocity_array[1],prefered_velocity_array[2]))
        
        for n_neighbor in range(len(self.near_neighbors_sorted["ids"])):

            if self.near_neighbors_sorted["types"][n_neighbor] == "agent":
                n_agent = self.near_neighbors_sorted["ids"][n_neighbor]
                position_array = self.ArrayFromPose(self.agents_data_list[n_agent].position.pose)[0]
                velocity_array = self.ArrayFromTwist(self.agents_data_list[n_agent].velocity.twist)[0]
                orca_agent_list.append(sim.addAgent((position_array[0],position_array[1],position_array[2]),
                                        params_dicc["neighborDist"], params_dicc["N_neighbors_aware"], params_dicc["timeHorizon"], params_dicc["agent_radius"], params_dicc["maxSpeed"],
                                        (velocity_array[0],velocity_array[1],velocity_array[2])))

                sim.setAgentPrefVelocity(orca_agent_list[-1],(velocity_array[0],velocity_array[1],velocity_array[2]))

            # Add to orca3 and to own list every obstacle created by own params
            elif self.near_neighbors_sorted["types"][n_neighbor] == "obs":
                n_obs = self.near_neighbors_sorted["ids"][n_neighbor]

                obs_pose = self.obs_pose_list[n_obs]
                orca_agent_list.append(sim.addAgent((obs_pose[0][0],obs_pose[0][1],obs_pose[0][2]),
                params_dicc["neighborDist"], params_dicc["N_neighbors_aware"], params_dicc["timeHorizon"], obs_radius, 0.0, (0, 0, 0)))

                sim.setAgentPrefVelocity(orca_agent_list[-1],(0,0,0))

        sim.doStep()        # Perform a step of orca3

        selected_velocity = sim.getAgentVelocity(orca_agent_list[0])     # Extract own velocity decided by orca3

        # Become that velocity in a twist
        new_velocity_twist = Twist(Vector3(0,0,0),Vector3(0,0,0))
        new_velocity_twist.linear.x = selected_velocity[0]
        new_velocity_twist.linear.y = selected_velocity[1]
        new_velocity_twist.linear.z = selected_velocity[2]

        # If head use selected, decide it by direct by simple algorithm. In future, put lower threshold.
        if self.heading_use == True:
            new_velocity_twist.angular.z = prefered_velocity.angular.z

        return new_velocity_twist

    def ORCA3_from_node(self):

        if "agent_created" not in self.algorithms_dicc["orca3"].keys():

            # rospy.wait_for_service('orca/add_agents')
            # try:
            #     add_agent_prox = rospy.ServiceProxy('orca/add_agents', MESSAGEEEEEEEEEEEEEE )
            #     # model_name = "{0}_{1}".format(self.agent_models[n_agent],n_agent+1)
            #     add_agent_prox(model_name)
            #     time.sleep(0.1)
            # except rospy.ServiceException, e:
            #     print "Service call failed: %s"%e
            #     print "error in add orca agent"

            self.orca_optimal_velocity = Twist()

            self.algorithms_dicc["orca3"]["agent_created"] = True

            self.algorithms_dicc["orca3"]["prefered_velocity_pub"] = rospy.Publisher('/orca/agent_{}/prefered_velocity'.format(self.ID), TwistStamped, queue_size = 1)

            def handle_orca_optimal_velocity(data):

                self.orca_optimal_velocity = data.twist

            rospy.Subscriber('/orca/agent_{}/optimal_velocity'.format(self.ID), TwistStamped, handle_orca_optimal_velocity)

        prefered_velocity = self.SimpleGuidance()

        prefered_velocity_stamped = TwistStamped()
        prefered_velocity_stamped.twist = prefered_velocity

        self.algorithms_dicc["orca3"]["prefered_velocity_pub"].publish(prefered_velocity_stamped)

        time.sleep(0.1)

        return self.orca_optimal_velocity

    # Function to set velocity directly to goal
    def SimpleGuidance(self):
        
        if self.smooth_path_mode != 0:
            return self.agents_data_list[self.ID-1].smooth_velocity
        
        # Set algorithm params
        desired_speed_at_goal = 0
        aprox_distance = 3

        # Create a vector from actual position to goal position
        relative_distance = np.asarray([self.goal["pose"].position.x-self.agents_data_list[self.ID-1].position.pose.position.x,\
                                self.goal["pose"].position.y-self.agents_data_list[self.ID-1].position.pose.position.y,\
                                self.goal["pose"].position.z-self.agents_data_list[self.ID-1].position.pose.position.z])

        distance_norm = np.linalg.norm(relative_distance)       # Calculate its norm

        # If at the distance shorter than aproximation distance, reduce the velocity module
        if distance_norm < aprox_distance:
            self.desired_speed = desired_speed_at_goal - (self.desired_speed - desired_speed_at_goal)\
                                    + ((self.desired_speed - desired_speed_at_goal) *2) / (1 + math.exp(-5*distance_norm/aprox_distance))

        # Multiply each axis by the velocity module
        relative_WP_linear=Vector3(relative_distance[0]/distance_norm*self.desired_speed,\
                                relative_distance[1]/distance_norm*self.desired_speed,\
                                relative_distance[2]/distance_norm*self.desired_speed)

        # Transform it in a pose position and calculate its orientation in Euler angles
        relative_WP_pose_degrees=Pose(relative_WP_linear,\
                                Vector3(np.arctan2(relative_WP_linear.z,relative_WP_linear.y),\
                                np.arctan2(relative_WP_linear.x,relative_WP_linear.z),\
                                np.arctan2(relative_WP_linear.y,relative_WP_linear.x)))  #### COMPROBAR ANGULOS

        # Transform the orientation from Eurler angles to quaternions
        orientation_list = [self.agents_data_list[self.ID-1].position.pose.orientation.x, self.agents_data_list[self.ID-1].position.pose.orientation.y, self.agents_data_list[self.ID-1].position.pose.orientation.z, self.agents_data_list[self.ID-1].position.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(orientation_list)

        # Create the velocity twist with calculated data
        new_velocity_twist = Twist(relative_WP_pose_degrees.position,\
                                   Vector3(0,\
                                   0,\
                                   relative_WP_pose_degrees.orientation.z-euler[2]))

        # Thresholds imposition
        # new_velocity_twist.linear.x = self.UpperLowerSaturation(new_velocity_twist.linear.x,1.5)
        # new_velocity_twist.linear.y = self.UpperLowerSaturation(new_velocity_twist.linear.y,1.5)
        # new_velocity_twist.angular.z = self.UpperLowerSaturation(new_velocity_twist.angular.z,0.5)

        return new_velocity_twist


    # Function to set hovering velocity equal to zeros
    def Hover(self):
        new_velocity_twist = Twist(Vector3(0,0,0),Vector3(0,0,0))

        return new_velocity_twist

    # Function to saturate a value
    def UpperLowerSaturation(self,value,threshold):
        if value > threshold:
            value = threshold
        elif value < -threshold:
            value = -threshold
        return value

    def NeighborSelector(self,N_neighbors_aware):
        agent_distances = []
        for n_agent in range(self.N_agents):
            if n_agent != self.ID-1:
                agent_distances.append(self.agents_data_list[n_agent].distance_rel2main)
            else:
                agent_distances.append(0)

        obs_distances = self.agents_data_list[self.ID-1].obs_distances_rel2main

        all_distances = agent_distances + obs_distances
        self.near_neighbors_sorted = {"distances" : sorted(all_distances)[1:N_neighbors_aware]}
        
        ids_list = []
        types_list = []
        for neigh in list(np.argsort(all_distances))[1:N_neighbors_aware]:

            if neigh < self.N_agents:
                neigh_type = "agent"
                neith_id = neigh
            else:
                neigh_type = "obs"
                neith_id = neigh - self.N_agents

            types_list.append(neigh_type)
            ids_list.append(neith_id)


        self.near_neighbors_sorted["ids"] = ids_list
        self.near_neighbors_sorted["types"] = types_list

        return self.near_neighbors_sorted

    def PoseFromArray(self,Array):
        quat = tf.transformations.quaternion_from_euler(Array[1][0],Array[1][1],Array[1][2])

        return Pose(Point(Array[0][0],Array[0][1],Array[0][2]),Quaternion(quat[0],quat[1],quat[2],quat[3]))

    def ArrayFromPose(self,pose):
        euler = [0,0,0]
        # euler = tf.transformations.euler_from_quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orienation.w)

        return [[pose.position.x,pose.position.y,pose.position.z],[euler[0],euler[1],euler[2]]]

    def TwistFromArray(self,Array):

        return Twist(Vector3(Array[0][0],Array[0][1],Array[0][2]),Vector3(Array[1][0],Array[1][1],Array[1][2]))

    def ArrayFromTwist(self,twist):

        return [[twist.linear.x,twist.linear.y,twist.linear.z],[twist.angular.x,twist.angular.y,twist.angular.z]]

    def OperatePoses(self,pose1,pose2,op = '+'):

        if op == '+':
            aux = 1
        elif op == '-':
            aux = -1

        result_pose = Pose()
        result_pose.position.x = pose1.position.x+aux*pose2.position.x
        result_pose.position.y = pose1.position.y+aux*pose2.position.y
        result_pose.position.z = pose1.position.z+aux*pose2.position.z
        result_pose.orientation.x = pose1.orientation.x+aux*pose2.orientation.x
        result_pose.orientation.y = pose1.orientation.y+aux*pose2.orientation.y
        result_pose.orientation.z = pose1.orientation.z+aux*pose2.orientation.z
        result_pose.orientation.w = pose1.orientation.w+aux*pose2.orientation.w

        return result_pose

    def algorithm_control(self, name, action, params_dicc):

        if action == "delete":
            self.algorithms_dicc.pop(name)

        elif action == "set":
            if name in self.algorithms_dicc.keys():
                self.algorithms_dicc[name].update(params_dicc)
            else:
                self.algorithms_dicc[name] = params_dicc




    # Function to get Global ROS parameters
    def GettingWorldDefinition(self):
        self.hyperparameters = rospy.get_param('magna_hyperparameters')
        self.mission_name = self.hyperparameters['mission']
        self.submission_name = self.hyperparameters['submission']
        self.world_name = self.hyperparameters['world']
        self.subworld_name = self.hyperparameters['subworld']
        self.n_simulation = self.hyperparameters['n_simulation']
        self.N_agents = self.hyperparameters['N_agents']
        self.N_obs = self.hyperparameters['N_obs']
        self.n_dataset = self.hyperparameters['n_dataset']
        self.obs_pose_list = self.hyperparameters['obs_pose_list']
        self.heading_use = self.hyperparameters['heading_use']
