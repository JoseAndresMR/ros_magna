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

class ANSP_SM(object):
    # At init, the State Machine receives as "heritage" the hole "self" of ANSP
    def __init__(self,heritage):
        # Creation of State Machine and definition of its outcomes
        self.ansp_sm = StateMachine(outcomes=['completed', 'failed'])

        with self.ansp_sm:
            # Initialization of id, number of waypoints and target
            self.DictionarizeCBStateCallbacks()
            self.DictionarizeSASCallbacks()

            mission_def_path = "/home/{0}/catkin_ws/src/pydag/Code/Missions/{1}.json"\
                            .format(heritage.home_path,heritage.mission)
            with open(mission_def_path) as f:
                self.mission_def = json.load(f)

            # For every entity in sm defining steps list, call function to add the step to sm
            for mission_part_def in self.mission_def:
                self.add_sm_from_CSV(self.ansp_sm,mission_part_def,{},heritage)

            # If flag to view the sm is arised, start the introspector
            if heritage.smach_view == True:
                sis = smach_ros.IntrospectionServer('pydag/ANSP_introspection', self.ansp_sm, '/ANSP')
                sis.start()

    # Function to add to current State Machine depending on step in sm defining steps list
    def add_sm_from_CSV(self, sm, mission_part_def, parent_params, heritage):

        # Selection depending on step value. Completed ouput is next step value.
        # For single states, check callback

        #########                   GENERICS                        ###########
        if mission_part_def["type"] == "CBState":

            ### PONER TAMBIEN LA UNION DE PARAMS

            cb_kwargs = {'heritage' : heritage}
            if "parameters" in mission_part_def.keys():
                cb_kwargs.update(mission_part_def["parameters"])

            sm.add(mission_part_def["name"],
                             CBState(self.CBStateCBDic[mission_part_def["name"]],
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
                        SimpleActionState('/pydag/ANSP_UAV_{0}/{1}_command'.format(params["uav"],mission_part_def["name"]),
                                            self.SASMsgTypeDic[mission_part_def["name"]],
                                            goal_cb=self.SASGoalCBDic[mission_part_def["name"]],
                                            result_cb=self.SASResultCBDic[mission_part_def["name"]],
                                            input_keys=['params'],
                                            output_keys=['id','n_wp'],
                                            goal_cb_kwargs={"params" : self.params},
                                            result_cb_kwargs={"heritage":heritage}),
                        mission_part_def["outcomes"])


        elif mission_part_def["type"] == "StateMachine":

            mission_part_def = self.IdsExtractor(mission_part_def,heritage)

            for ids in mission_part_def["ids"]:
                # self.id = ids

                params = self.UpdateLocalParameters(ids,mission_part_def,parent_params)

                new_sm = StateMachine(outcomes=['completed', 'failed'],
                                                        input_keys=[])

                with new_sm:

                    for occurrence in mission_part_def["occurrencies"]:

                        self.add_sm_from_CSV(new_sm,occurrence,heritage)

                sm.add(mission_part_def["name"],
                                new_sm,
                                transitions= mission_part_def["outcomes"])



        elif mission_part_def["type"] == "Concurrence":

            mission_part_def = self.IdsExtractor(mission_part_def,heritage)

            for ids in mission_part_def["ids"]:
                # self.id = ids

                params = self.UpdateLocalParameters(ids,mission_part_def,parent_params)
                    
      
                concurrence = Concurrence(outcomes=['completed','failed'],
                                        input_keys = [],
                                        output_keys = [],
                                        default_outcome = 'completed',
                                        outcome_map = {})

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

                sequence = Sequence(outcomes = ['completed','failed'],
                                    connector_outcome = mission_part_def["connector_outcome"])

                with sequence:
                    for occurrence in mission_part_def["occurrencies"]:

                        self.add_sm_from_CSV(sequence,occurrence,params,heritage)

                sm.add('{0}_{1}'.format(ids,mission_part_def["name"]),
                            sequence,
                            transitions = mission_part_def["outcomes"])

            


        #########                   END GENERICS                    ###########

        ### FOLLOW PATHS SBYS
        if mission_part_def["type"] == "follow_paths_sbys_sm":

            # Create global State machine and define its outputs
            follow_paths_sbys_sm = StateMachine(outcomes=['completed', 'failed'],
                                                    input_keys=[])

            with follow_paths_sbys_sm:

                for self.n_wp in range(heritage.path_length):       # Add a state for every waypoint

                    # Creation of a dictionary that defines outcomes. Now only implemented completed outcomes
                    outcome_map = {"completed" : {}}
                    for self.id in range(1,heritage.N_uav+1):
                        outcome_map["completed"]["{0}_follow_wp".format(self.id)] = "succeeded"

                    # Create Concurrence State machine and define its outputs
                    follow_wp_ccr = Concurrence(outcomes=['completed','failed'],
                                            input_keys = [],
                                            output_keys = [],
                                            default_outcome = 'completed',
                                            outcome_map = outcome_map)


                    with follow_wp_ccr:        # Define inside of Concurrence SM

                        for self.id in range(1,heritage.N_uav+1):       # Add an Action State for every AUV
                            Concurrence.add('{0}_follow_wp'.format(self.id),
                                SimpleActionState('/pydag/ANSP_UAV_{}/follow_path_command'.format(self.id),
                                                    FollowPathAction,
                                                    goal_cb=self.follow_wp_goal_cb,
                                                    result_cb=self.follow_wp_result_cb,
                                                    input_keys=['id','n_wp'],
                                                    output_keys=['id','n_wp'],
                                                    goal_cb_kwargs={'heritage':heritage,'id':self.id,'n_wp':self.n_wp},
                                                    result_cb_kwargs={"heritage":heritage}),
                                remapping={})

                    # Define outcomes for each waypoint's state and for the last virtual one
                    if self.n_wp != int(heritage.path_length)-1:
                        transitions = {'completed':'follow_wp_{0}_ccr'.format(self.n_wp+1),
                                        'failed': 'failed'}
                    else:
                        transitions = {'completed':'completed',
                                        'failed': 'failed'}

                    StateMachine.add('follow_wp_{0}_ccr'.format(self.n_wp)   # Add definded Concurrence State Machine
                                    ,follow_wp_ccr,
                                    transitions = transitions)

            StateMachine.add('follow_paths_sbys_sm',        # Add Concurrence State Machine
                            follow_paths_sbys_sm,
                            transitions= mission_part_def["outcomes"])


        ### ALL TAKE OFF
        if mission_part_def["type"] == "uavs_take_off_ccr":

            if mission_part_def["uavs"] == "all":
                mission_part_def["uavs"] = range(1,heritage.N_uav+1)

            # Creation of a dictionary that defines outcomes. Now only implemented completed outcomes
            outcome_map = {"completed" : {}}
            for self.id in mission_part_def["uavs"]:
                outcome_map["completed"]["{0}_take_off".format(self.id)] = "succeeded"

            # Create Concurrence State machine and define its outputs
            uavs_take_off_ccr = Concurrence(outcomes=['completed','failed'],
                                    input_keys = [],
                                    output_keys = [],
                                    default_outcome = 'completed',
                                    outcome_map = outcome_map)

            with uavs_take_off_ccr:     # Define inside of Concurrence S
                for self.id in mission_part_def["uavs"]:       # Add an Action takeoff State for every AUV
                     Concurrence.add('{0}_take_off'.format(self.id),
                                SimpleActionState('/pydag/ANSP_UAV_{}/take_off_command'.format(self.id),
                                                    TakeOffAction,
                                                    goal_cb=self.take_off_goal_cb,
                                                    result_cb=self.take_off_result_cb,
                                                    input_keys=['id','n_wp'],
                                                    output_keys=['id','n_wp'],
                                                    goal_cb_kwargs={},
                                                    result_cb_kwargs={}),
                                remapping={})

            StateMachine.add('uavs_take_off_ccr'     # Add definded Concurrence State Machine
                                ,uavs_take_off_ccr,
                                transitions = mission_part_def["outcomes"])

        ### ALL LAND
        if mission_part_def["type"] == "uavs_land_ccr":

            if mission_part_def["uavs"] == "all":
                mission_part_def["uavs"] = [uav for uav in range(1,heritage.N_uav+1)]

            # Creation of a dictionary that defines outcomes. Now only implemented completed outcomes
            outcome_map = {"completed" : {}}
            for self.id in mission_part_def["uavs"]:
                outcome_map["completed"]["{0}_land".format(self.id)] = "succeeded"

            # Create Concurrence State machine and define its outputs
            uavs_land_ccr = Concurrence(outcomes=['completed','failed'],
                                    input_keys = [],
                                    output_keys = [],
                                    default_outcome = 'completed',
                                    outcome_map = outcome_map)

            with uavs_land_ccr:

                # Add an Action land State for every AUV
                for self.id in mission_part_def["uavs"]:
                     Concurrence.add('{0}_land'.format(self.id),
                                SimpleActionState('/pydag/ANSP_UAV_{}/land_command'.format(self.id),
                                                    LandAction,
                                                    goal_cb=self.land_goal_cb,
                                                    result_cb=self.land_result_cb,
                                                    input_keys=['id','n_wp'],
                                                    output_keys=['id','n_wp'],
                                                    goal_cb_kwargs={},
                                                    result_cb_kwargs={}),
                                remapping={})

            StateMachine.add('uavs_land_ccr'.format(self.n_wp)
                                    ,uavs_land_ccr,
                                    transitions = mission_part_def["outcomes"])

        ### BASIC MOVE
        if mission_part_def["type"] == "basic_move_sm":

            # Create State machine and define its outputs
            basic_move_sm = StateMachine(outcomes=['completed', 'failed'],
                                                    input_keys=[])

            with basic_move_sm:           # Define inside of SM

                StateMachine.add('basic_move_ansp',     # Add simple action state and its definition
                                    SimpleActionState('/pydag/ANSP_UAV_1/basic_move_command',
                                                        BasicMoveAction,
                                                        goal_cb=self.basic_move_goal_cb,
                                                        result_cb=self.basic_move_result_cb,
                                                        input_keys=[],
                                                        output_keys=[],
                                                        goal_cb_kwargs={},
                                                        result_cb_kwargs={}),
                                    transitions = mission_part_def["outcomes"],
                                    remapping = {})

            StateMachine.add('basic_move_sm',       # Add definde State Machine
                    basic_move_sm,
                    transitions={'completed':'completed',
                                    'failed': 'failed'})



        ### QUEUE OF FOLLOWERS AD
        if mission_part_def["type"] == "queue_of_followers_ad_sm":

            # Create global State machine and define its outputs
            queue_of_followers_ad_sm = StateMachine(outcomes=['completed', 'failed'],
                                                    input_keys=[])

            with queue_of_followers_ad_sm:

                # Creation of a dictionary that defines outcomes. Now only implemented completed outcomes
                outcome_map = {"completed" : {"1_follow_path" : "succeeded"}}
                # outcome_map["completed"]["1_follow_path"] = "succeeded"

                # if heritage.N_uav > 1:
                #     outcome_map["completed"]["2_follow_uav_ad_1"] = "succeeded"

                # if heritage.N_uav > 2:
                #     outcome_map["completed"]["3_follow_uav_ad_2"] = "succeeded"

                # Create Concurrence State machine and define its outputs
                queue_of_followers_ad_ccr = Concurrence(outcomes=['completed','failed'],
                                        input_keys= [],
                                        default_outcome = 'completed',
                                        outcome_map = outcome_map)


                with queue_of_followers_ad_ccr:        # Define inside of Concurrence SM

                    # For first UAV, define a state to follow waypoint path
                    self.id = 1
                    Concurrence.add('1_follow_path',
                        SimpleActionState('/pydag/ANSP_UAV_{}/follow_path_command'.format(self.id),
                                            FollowPathAction,
                                            goal_cb=self.follow_path_goal_cb,
                                            result_cb=self.follow_path_result_cb,
                                            input_keys=['id'],
                                            output_keys=['id'],
                                            goal_cb_kwargs={'heritage':heritage,'id':self.id},
                                            result_cb_args={'heritage':heritage}),
                        remapping={'id':'id'})

                    # For the rest of UAVs, define a state to follow the previous UAV at distance
                    for self.id in range(2,heritage.N_uav+1):
                        target_ID = self.id - 1

                        # Definition of distance, in the future will be defined externally
                        distance = 3

                        Concurrence.add('{}_follow_uav_ad_{}'.format(self.id,self.id-1),
                            SimpleActionState('/pydag/ANSP_UAV_{}/follow_uav_ad_command'.format(self.id),
                                                FollowUAVADAction,
                                                goal_cb=self.follow_uav_ad_goal_cb,
                                                result_cb=self.follow_uav_ad_result_cb,
                                                input_keys=['id','target_ID'],
                                                output_keys=['id','target_ID'],
                                                goal_cb_kwargs={'heritage':heritage,'target_ID':target_ID,'distance':distance},
                                                result_cb_kwargs={"heritage":heritage}),
                            remapping={'id':'id','target_ID':'target_ID'})

                StateMachine.add('queue_of_followers_ad_ccr',queue_of_followers_ad_ccr,
                                        transitions={'completed':'completed',
                                                    'failed': 'failed'})
            StateMachine.add('queue_of_followers_ad_sm',
                            queue_of_followers_ad_sm,
                            transitions= mission_part_def["outcomes"])

        ### QUEUE OF FOLLOWERS AP
        if mission_part_def["type"] == "queue_of_followers_ap_sm":

            # Create global State machine and define its outputs
            queue_of_followers_ap_sm = StateMachine(outcomes=['completed', 'failed'],
                                                    input_keys=[])

            with queue_of_followers_ap_sm:

                # Creation of a dictionary that defines outcomes. Now only implemented completed outcomes
                outcome_map = {"completed" : {"follow_path_sm" : "completed"}}

                 # Create Concurrence State machine and define its outputs
                queue_of_followers_ap_ccr = Concurrence(outcomes=['completed','failed'],
                                        input_keys= [],
                                        default_outcome = 'completed',
                                        outcome_map = outcome_map)

                with queue_of_followers_ap_ccr:        # Define inside of Concurrence SM
                    self.id = 1

                    self.follow_path_sm = StateMachine(outcomes=['completed', 'failed'],
                                                    input_keys=[])

                    with self.follow_path_sm:   # For first UAV, define a state to follow waypoint path
                        StateMachine.add('1_follow_path',
                            SimpleActionState('/pydag/ANSP_UAV_{}/follow_path_command'.format(self.id),
                                                FollowPathAction,
                                                goal_cb=self.follow_path_goal_cb,
                                                result_cb=self.follow_path_result_cb,
                                                input_keys=[],
                                                output_keys=[],
                                                goal_cb_kwargs={'heritage':heritage,'id':self.id},
                                                result_cb_kwargs={'heritage':heritage}),
                            remapping={'id':'id','heritage':'heritage','preempt_others':'preempt_others'},
                            transitions = {'succeeded':'completed',
                                           'preempted':'completed',
                                           'aborted':'completed'})

                    Concurrence.add('follow_path_sm',
                        self.follow_path_sm)

                    # For the rest of UAVs, define a state to follow the previous UAV at point
                    for self.id in range(2,heritage.N_uav+1):
                        target_ID = self.id - 1

                        # Definition of the point, in the future will be defined externally
                        pos = [0,0,3]

                        Concurrence.add('{}_follow_uav_ap_{}'.format(self.id,self.id-1),
                            SimpleActionState('/pydag/ANSP_UAV_{}/follow_uav_ap_command'.format(self.id),
                                                FollowUAVAPAction,
                                                goal_cb=self.follow_uav_ap_goal_cb,
                                                result_cb=self.follow_uav_ap_result_cb,
                                                input_keys=['id','target_ID'],
                                                output_keys=['id','target_ID'],
                                                goal_cb_kwargs={'heritage':heritage,'target_ID':target_ID,'pos':pos},
                                                result_cb_kwargs={"heritage":heritage}),
                            remapping={'id':'id','target_ID':'target_ID'})

                StateMachine.add('queue_of_followers_ap_ccr',queue_of_followers_ap_ccr,
                                        transitions={'completed':'completed',
                                                    'failed': 'failed'})
            StateMachine.add('queue_of_followers_ap_sm',
                            queue_of_followers_ap_sm,
                            transitions= mission_part_def["outcomes"])

        ### ALL SAVE CSV
        if mission_part_def["type"] == "uavs_save_csv_ccr":

            if mission_part_def["uavs"] == "all":
                mission_part_def["uavs"] = [uav for uav in range(1,heritage.N_uav+1)]

            # Creation of a dictionary that defines outcomes. Now only implemented completed outcomes
            outcome_map = {"completed" : {}}
            for self.id in mission_part_def["uavs"]:
                outcome_map["completed"]["{0}_save_csv".format(self.id)] = "succeeded"

            # Create Concurrence State machine and define its outputs
            uavs_save_csv_ccr = Concurrence(outcomes=['completed','failed'],
                                    input_keys = [],
                                    output_keys = [],
                                    default_outcome = 'completed',
                                    outcome_map = outcome_map)

            with uavs_save_csv_ccr:

                for self.id in mission_part_def["uavs"]:

                    # Add an Action save data State for every AUV
                    Concurrence.add('{0}_save_csv'.format(self.id),
                                SimpleActionState('/pydag/ANSP_UAV_{}/save_csv_command'.format(self.id),
                                                    LandAction,
                                                    goal_cb=self.save_csv_goal_cb,
                                                    result_cb=self.save_csv_result_cb,
                                                    input_keys=['id','n_wp'],
                                                    output_keys=['id','n_wp'],
                                                    goal_cb_kwargs={},
                                                    result_cb_kwargs={}),
                                remapping={})

            StateMachine.add('uavs_save_csv_ccr',
                                uavs_save_csv_ccr,
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
    def wait_stcb(self,heritage,duration):
        time.sleep(duration)

        return "completed"

    # State callback to spawn UAVs
    @cb_interface(outcomes=['completed','failed'])
    def spawn_uavs_stcb(self,heritage):

        for i in range(heritage.N_uav):     # Do it for every UAV
            uav_goal_path = heritage.world.PathGenerator(heritage.uav_models[i])      # Ask world to generate a path

            heritage.path_length = len(uav_goal_path)       # Reset path length information
            heritage.uavs_goal_paths_list.append(uav_goal_path)     # Append the asked path to existing one QUITAR
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
            while heritage.states_list[i] != "waiting for action command":
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
        time.sleep(191991)

        return goal

    # Result callback for basic move service
    def basic_move_result_cb(self, ud, status, result):
        return "succeeded"

    # Goal callback for follow path service
    def follow_path_goal_cb(self, ud, goal, params):

        # Select UAV path from ID and give it as goal
        # if "wp" not in params.keys():
        #     goal.goal_path_poses_list = np.array(params["heritage"].uavs_goal_paths_list[params["uav"]-1])
        # else:
        #     goal.goal_path_poses_list = [params["heritage"].uavs_goal_paths_list[params["uav"]-1][params["wp"]-1]]
        goal.goal_path_poses_list = np.array([])
        for path_part in params["path"]:
            goal.goal_path_poses_list = np.append(goal.goal_path_poses_list,
                                                np.array(params["heritage"].MakePath(path_part["definition"])))

        return goal

    # Result callback for follow path service. In the future should be implemented out of SM
    def follow_path_result_cb(self, ud, status, result):
        ### CONSEGUIR METER HERITAGE
        # print("ud",ud)
        # print("status",status)
        # print("result",result)
        # print("heritage",heritage)
        # for uav in range(1,heritage.N_uav):
        #     rospy.wait_for_service('/pydag/ANSP/preemption_command_to_{}'.format(uav))
        #     try:
        #         # print "path for uav {} command".format(ID)
        #         PreemptCommander = rospy.ServiceProxy('/pydag/ANSP/preemption_command_to_{}'.format(uav), StateActualization)
        #         PreemptCommander(1,"preempt",False)
        #         return
        #     except rospy.ServiceException, e:
        #         print "Service call failed: %s"%e
        #         print "error in state_actualization"
        return "succeeded"

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