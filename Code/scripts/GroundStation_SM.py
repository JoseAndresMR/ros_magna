from smach import StateMachine, State, CBState, cb_interface,Concurrence, Sequence
import smach_ros
import time
from smach_ros import SimpleActionState
import numpy as np
import tf
import json
import copy

from Worlds import *
from pydag.msg import *

class GroundStation_SM(object):
    # At init, the State Machine receives as "heritage" the hole "self" of GS
    def __init__(self,heritage):
        # Creation of State Machine and definition of its outcomes
        self.gs_sm = StateMachine(outcomes=['completed', 'failed'])

        with self.gs_sm:
            # Initialization of id, number of waypoints and target
            self.DictionarizeCBStateCallbacks()
            self.DictionarizeSASCallbacks()
            self.DictionarizeCCRCallbacks()

            mission_def_path = "/home/{0}/catkin_ws/src/pydag/Code/JSONs/Missions/{1}/{2}.json"\
                            .format(heritage.home_path,heritage.mission_name,heritage.submission_name)
            with open(mission_def_path) as f:
                self.mission_def = json.load(f)

            # For every entity in sm defining steps list, call function to add the step to sm
            for mission_part_def in self.mission_def:
                self.add_sm_from_CSV(self.gs_sm,mission_part_def,{},heritage)

            # If flag to view the sm is arised, start the introspector
            if heritage.smach_view == True:
                sis = smach_ros.IntrospectionServer('pydag/GS_introspection', self.gs_sm, '/GS')
                sis.start()

    # Function to add to current State Machine depending on step in sm defining steps list
    def add_sm_from_CSV(self, sm, mission_part_def, parent_params, heritage):

        # Selection depending on step value. Completed ouput is next step value.
        # For single states, check callback

        if mission_part_def["type"] == "CBState":

            ### PONER TAMBIEN LA UNION DE PARAMS

            cb_kwargs = {'heritage' : heritage}
            if "parameters" in mission_part_def.keys():
                cb_kwargs.update(mission_part_def["parameters"])

            sm.add(mission_part_def["name"],
                             CBState(self.CBStateCBDic[mission_part_def["state_type"]],
                                cb_kwargs=cb_kwargs),
                             mission_part_def["outcomes"])

        elif mission_part_def["type"] == "SimpleActionState":

            if "outcomes" not in mission_part_def.keys():
                mission_part_def["outcomes"] = None

            mission_part_def = self.IdsExtractor(mission_part_def,heritage)

            for ids in mission_part_def["ids"]:

                params = self.UpdateLocalParameters(ids,mission_part_def,parent_params)

                params.update({'heritage' : heritage})

                self.params = params        ### AHORRARME SELF

                sm.add('{0}_{1}'.format(ids,mission_part_def["name"]),
                        SimpleActionState('/pydag/GS_UAV_{0}/{1}_command'.format(params["uav"],mission_part_def["state_type"]),
                                            self.SASMsgTypeDic[mission_part_def["state_type"]],
                                            goal_cb=self.SASGoalCBDic[mission_part_def["state_type"]],
                                            result_cb=self.SASResultCBDic[mission_part_def["state_type"]],
                                            input_keys=['params'],
                                            output_keys=['id','n_wp'],
                                            goal_cb_kwargs={"params" : self.params},
                                            result_cb_kwargs={"heritage":heritage},
                                            outcomes=['succeeded','collision','low_battery','GS_critical_event']),
                        mission_part_def["outcomes"])


        elif mission_part_def["type"] == "StateMachine":

            mission_part_def = self.IdsExtractor(mission_part_def,heritage)

            for ids in mission_part_def["ids"]:
                # self.id = ids

                params = self.UpdateLocalParameters(ids,mission_part_def,parent_params)

                new_sm = StateMachine(outcomes=mission_part_def["outcomes"].keys(),
                                                        input_keys=[])

                with new_sm:

                    for occurrence in mission_part_def["occurrencies"]:

                        self.add_sm_from_CSV(new_sm,occurrence,params,heritage)

                # concurrence = Concurrence(input_keys = [],
                #         output_keys = [],
                #         default_outcome = 'completed',
                #         outcome_map = {})


                sm.add('{0}_{1}'.format(ids,mission_part_def["name"]), new_sm, mission_part_def["outcomes"])




        elif mission_part_def["type"] == "Concurrence":

            mission_part_def = self.IdsExtractor(mission_part_def,heritage)

            for ids in mission_part_def["ids"]:
                # self.id = ids

                params = self.UpdateLocalParameters(ids,mission_part_def,parent_params)

                if "child_termination_cb" not in mission_part_def.keys():
                    child_termination_cb = None
                else:
                    child_termination_cb = self.CCR_CT_Dic[mission_part_def["child_termination_cb"]]

                if "outcome_cb" not in mission_part_def.keys():
                    outcome_cb = None
                else:
                    outcome_cb = self.CCR_O_Dic[mission_part_def["outcome_cb"]]

                concurrence = Concurrence(outcomes=mission_part_def["outcomes"].keys(),
                                        input_keys = [],
                                        output_keys = [],
                                        default_outcome = 'completed',
                                        outcome_map = {},
                                        outcome_cb = outcome_cb,
                                        child_termination_cb = child_termination_cb)

                with concurrence:
                    for occurrence in mission_part_def["occurrencies"]:

                        self.add_sm_from_CSV(concurrence,occurrence,params,heritage)


                for outcome in  mission_part_def["occurrencies_outcome_map"].keys():
                    for state in mission_part_def["occurrencies_outcome_map"][outcome].keys():
                        outcome_dict = mission_part_def["occurrencies_outcome_map"][outcome]

                        if type(state) == list:
                            for i in state:
                                outcome_dict[i] = outcome_dict[state]

                            del outcome_dict[state]

                        elif state == "all":
                            for occurrency in mission_part_def["occurrencies"]:
                                if type(occurrency["ids"]) == int:
                                    outcome_dict['{0}_{1}'.format(occurrency["ids"],occurrency["name"])] = outcome_dict[state]

                                elif type(occurrency["ids"]) == list:
                                    for occurency_ids in occurrency["ids"]:
                                        outcome_dict['{0}_{1}'.format(occurency_ids,occurrency["name"])] = outcome_dict[state]

                            del outcome_dict[state]

                concurrence._outcome_map = mission_part_def["occurrencies_outcome_map"]

                if "outcomes" not in mission_part_def.keys():
                    mission_part_def["outcomes"] = None

                sm.add('{0}_{1}'.format(ids,mission_part_def["name"]),
                        concurrence,
                        transitions = mission_part_def["outcomes"])


        elif mission_part_def["type"] == "Sequence":

            mission_part_def = self.IdsExtractor(mission_part_def,heritage)

            for ids in mission_part_def["ids"]:

                params = self.UpdateLocalParameters(ids,mission_part_def,parent_params)

                sequence = Sequence(outcomes = mission_part_def["outcomes"].keys(),
                                    connector_outcome = mission_part_def["connector_outcome"])

                with sequence:
                    for occurrence in mission_part_def["occurrencies"]:

                        self.add_sm_from_CSV(sequence,occurrence,params,heritage)

                sm.add('{0}_{1}'.format(ids,mission_part_def["name"]),
                            sequence,
                            transitions = mission_part_def["outcomes"])


    #### STATE CALLBACKS ####

    # State callback to create a new world
    @cb_interface(outcomes=['completed','failed'])
    def new_world_stcb(self,heritage):
        # Local parameters inizialization
        heritage.CreatingSimulationDataStorage()        # Create storage forders if do not exist
        heritage.world = Worlds(1)      # Create a World of id 1
        heritage.GettingWorldDefinition()       # Get ROS param of definitions
        heritage.uavs_goal_paths_list = []      # Initialize path list

        return "completed"

    # State callback to wait one second. In future will fuse with next callback
    @cb_interface(outcomes=['completed','failed'])
    def wait_stcb(self,heritage,exit_type,duration = 0):

        outcome = heritage.wait(exit_type,duration)

        return outcome

    # State callback to spawn UAVs
    @cb_interface(outcomes=['completed','failed'])
    def spawn_uavs_stcb(self,heritage):

        for i in range(heritage.N_uav):     # Do it for every UAV
            # first_pose = uav_goal_path[0]       # Select first waypoint. It will determinate where it spawns
            first_pose = heritage.world.getFSPoseGlobal(["Ground_Station","UAVs_take_off","matrix",[i,0,0]])
            # Change spawn features so it spawns under its first waypoint
            yaw = tf.transformations.euler_from_quaternion((first_pose.orientation.x,
                                                           first_pose.orientation.y,
                                                           first_pose.orientation.z,
                                                           first_pose.orientation.w))[2]
            heritage.SetUavSpawnFeatures(i+1,heritage.uav_models[i],[first_pose.position.x,first_pose.position.y,0],yaw)
            heritage.UAVSpawner(i)      # Call service to spawn the UAV

            # Wait until the UAV is in ready state
            while not rospy.is_shutdown() and heritage.states_list[i] != "waiting for action command":
                time.sleep(0.5)
                # print(heritage.states_list[i])
                # print("UAV {} with state".format(i+1), heritage.states_list[i])

            print("uav {} is ready!".format(i))
            # time.sleep(5)
        # time.sleep(20 * heritage.N_uav)

        return "completed"

    def DictionarizeCBStateCallbacks(self):
        self.CBStateCBDic = {}
        self.CBStateCBDic["new_world"] = self.new_world_stcb
        self.CBStateCBDic["spawn_uavs"] = self.spawn_uavs_stcb
        self.CBStateCBDic["wait"] = self.wait_stcb

    #### ACTIONS CALLBACKS ####

    # Goal callback for takeoff state service
    def take_off_goal_cb(self, ud, goal, params):
        goal.height = params["height"]     # Define takeoff height, in future will be received in ud or mission dictionary
        # time.sleep(5*ud.id)
        return goal

    # Result callback for takeoff state service
    def take_off_result_cb(self, ud, status, result):
        return "succeeded"

    # Goal callback for basic move service
    def basic_move_goal_cb(self, ud, goal, params):
        goal.move_type = params["move_type"]
        goal.dynamic = params["dynamic"]
        goal.direction = params["direction"]
        goal.value = params["value"]

        return goal

    # Result callback for basic move service
    def basic_move_result_cb(self, ud, status, result):
        return "succeeded"

    # Goal callback for follow path service
    def follow_path_goal_cb(self, ud, goal, params):

        goal.goal_path_poses_list = np.array([])
        for path_part in params["path"]:
            goal.goal_path_poses_list = np.append(goal.goal_path_poses_list,
                                                np.array(params["heritage"].MakePath(path_part["definition"])))
        goal.smooth_path_mode = 0
        if "smooth_path_mode" in params.keys():
            goal.smooth_path_mode = params["smooth_path_mode"]

        return goal

    # Result callback for follow path service. In the future should be implemented out of SM
    def follow_path_result_cb(self, ud, status, result):
        print(result.output)
        ### CONSEGUIR METER HERITAGE
        # print("ud",ud)
        # print("status",status)
        # print("result",result)
        # print("heritage",heritage)
        # for uav in range(1,heritage.N_uav):
        #     rospy.wait_for_service('/pydag/GS/preemption_command_to_{}'.format(uav))
        #     try:
        #         # print "path for uav {} command".format(ID)
        #         PreemptCommander = rospy.ServiceProxy('/pydag/GS/preemption_command_to_{}'.format(uav), StateActualization)
        #         PreemptCommander(1,"preempt",False)
        #         return
        #     except rospy.ServiceException, e:
        #         print "Service call failed: %s"%e
        #         print "error in state_actualization"
        return result.output

    # Goal callback for follow wp service
    # def follow_wp_goal_cb(self, ud, goal, params):
    #     # Select UAV wp from ID and wp number and give it as goal
    #     goal.goal_path_poses_list = [params["heritage"].uavs_goal_paths_list[params["id"]-1][params["n_wp"]]]

    #     return goal

    # # Result callback for follow wp service
    # def follow_wp_result_cb(self, ud, result, heritage):
    #     return "succeeded"

    # Goal callback for follow uav ad service
    def follow_uav_ad_goal_cb(self, ud, goal, params):
        # Build the goal from arguments
        goal.target_ID = params["target_ID"]
        goal.distance = params["distance"]
        goal.time = 10      # In the future should be received from argument or mission dictionary

        return goal

    # Result callback for follow uav ad service
    def follow_uav_ad_result_cb(self, ud, result, heritage):
        return "succeeded"

    # Goal callback for follow uav ap service
    def follow_uav_ap_goal_cb(self, ud, goal, params):
        # Build the goal from arguments
        goal.target_ID = params["target_ID"]
        goal.pos = params["pos"]
        goal.time = params["time"]      # In the future should be received from argument or mission dictionary

        return goal

    # Result callback for follow uav ap service
    def follow_uav_ap_result_cb(self, ud, result, heritage):
        return "succeeded"

    # Goal callback for land service
    def land_goal_cb(self, ud, goal, params):
        goal.something = True       # In the future should disappear

        return goal

    # Result callback for land service
    def land_result_cb(self, ud, status,result):
        return "succeeded"

    # Goal callback for save csv service
    def save_csv_goal_cb(self, ud, goal, params):
        goal.something = True       # In the future should disappear

        return goal

    # Result callback for save csv service
    def save_csv_result_cb(self, ud, result, something):
        return "succeeded"


    def DictionarizeSASCallbacks(self):

        self.SASGoalCBDic = {}
        self.SASGoalCBDic["take_off"] = self.take_off_goal_cb
        self.SASGoalCBDic["basic_move"] = self.basic_move_goal_cb
        self.SASGoalCBDic["follow_path"] = self.follow_path_goal_cb
        self.SASGoalCBDic["follow_uav_ad"] = self.follow_uav_ad_goal_cb
        self.SASGoalCBDic["follow_uav_ap"] = self.follow_uav_ap_goal_cb
        self.SASGoalCBDic["land"] = self.land_goal_cb
        self.SASGoalCBDic["save_csv"] = self.save_csv_goal_cb

        self.SASResultCBDic = {}
        self.SASResultCBDic["take_off"] = self.take_off_result_cb
        self.SASResultCBDic["basic_move"] = self.basic_move_result_cb
        self.SASResultCBDic["follow_path"] = self.follow_path_result_cb
        self.SASResultCBDic["follow_uav_ad"] = self.follow_uav_ad_result_cb
        self.SASResultCBDic["follow_uav_ap"] = self.follow_uav_ap_result_cb
        self.SASResultCBDic["land"] = self.land_result_cb
        self.SASResultCBDic["save_csv"] = self.save_csv_result_cb

        self.SASMsgTypeDic = {}
        self.SASMsgTypeDic["take_off"] = TakeOffAction
        self.SASMsgTypeDic["basic_move"] = BasicMoveAction
        self.SASMsgTypeDic["follow_path"] = FollowPathAction
        self.SASMsgTypeDic["follow_uav_ad"] = FollowUAVADAction
        self.SASMsgTypeDic["follow_uav_ap"] = FollowUAVAPAction
        self.SASMsgTypeDic["land"] = LandAction
        self.SASMsgTypeDic["save_csv"] = LandAction


    #### CONCURRENCE OUTCOMES CALLBACKS ####

    def preempt_all_at_one_collision_child_termination_cb(self,actual_outcomes_map):
        # print(actual_outcomes_map)

        for outcome in actual_outcomes_map.values():
            if outcome == "collision":
                return True

        return False

    def outcome_collision_if_any_collided_outcome_cb(self,final_outcomes_map):
        if any("collision" in s for s in final_outcomes_map.values()):
            return "collision"

        else:
            return None

    def DictionarizeCCRCallbacks(self):

        self.CCR_CT_Dic = {}
        self.CCR_CT_Dic["all_at_one_collision"] = self.preempt_all_at_one_collision_child_termination_cb

        self.CCR_O_Dic = {}
        self.CCR_O_Dic["collision_if_any_collided"] = self.outcome_collision_if_any_collided_outcome_cb


    #### Miscellaneous ####
    #     
    def IdsExtractor(self,mission_part_def,heritage):

        if type(mission_part_def["ids"]) == int:
            ids = [mission_part_def["ids"]]

        elif type(mission_part_def["ids"]) == list:
            ids = mission_part_def["ids"]

        elif mission_part_def["ids"] == "all":

            if mission_part_def["ids_var"] == "uav":
                ids = range(1, heritage.N_uav + 1)

            elif mission_part_def["ids_var"] == "wp":
                ids = range(1, heritage.path_length + 1)

        mission_part_def["ids"] = ids

        return mission_part_def


    def UpdateLocalParameters(self,ids,mission_part_def,parent_params):

        params = copy.deepcopy(parent_params)
        if "parameters" in mission_part_def.keys():
            params.update(mission_part_def["parameters"])

        if "ids_var" in mission_part_def.keys():
            params.update({mission_part_def["ids_var"] : ids})

        return params