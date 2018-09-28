from smach import StateMachine, State, CBState, cb_interface,Concurrence
import smach_ros
import time
from smach_ros import SimpleActionState
import numpy as np

from Worlds import *
from pydag.msg import *

class ANSP_SM(object):
    def __init__(self,heritage):
        print(heritage.N_uav)
        self.ansp_sm = StateMachine(outcomes=['completed', 'failed'])

        with self.ansp_sm:

            ### CALLBACKS
            def take_off_goal_cb(ud, goal):
                goal.heigth = 5

                return goal

            def take_off_result_cb(ud, result, something):
                return "succeeded"

            def land_goal_cb(ud, goal):
                goal.something = True

                return goal

            def land_result_cb(ud, result, something):
                return "succeeded"

            def basic_move_goal_cb(ud, goal):
                goal.move_type = "take_off"
                goal.dynamic = "position"
                goal.direction = "up"
                goal.value = 5.0

                return goal

            def basic_move_result_cb(ud, result, something):
                return "succeeded"

            def save_csv_goal_cb(ud, goal):
                goal.something = True

                return goal

            def save_csv_result_cb(ud, result, something):
                return "succeeded"

            def follow_path_goal_cb(ud, goal, heritage, id):
                goal.goal_path_poses_list = np.array(heritage.uavs_goal_paths_list[id-1])

                return goal

            def follow_wp_goal_cb(ud, goal, heritage, id, n_wp):
                goal.goal_path_poses_list = [heritage.uavs_goal_paths_list[id-1][n_wp]]

                return goal

            def follow_path_result_cb(ud, result, heritage):
                return "succeeded"

            def follow_uav_ad_goal_cb(ud, goal, heritage, target_ID, distance):
                goal.target_ID = id_to_follow
                goal.distance = distance
                goal.time = 10

                return goal

            def follow_uav_ad_result_cb(ud, result, heritage):
                return "succeeded"

            def follow_uav_ap_goal_cb(ud, goal, heritage, target_ID, pos):
                goal.target_ID = id_to_follow
                goal.pos = pos
                goal.time = 10

                return goal

            def follow_uav_ap_result_cb(ud, result, heritage):
                return "succeeded"

            ### TOPOLOGY

            StateMachine.add('new_world', CBState(self.new_world_stcb,cb_kwargs={'heritage':heritage}),
                                    {'completed':'spawn_uavs'})

            self.id = 1
            self.n_wp = 0
            self.id_to_follow = 1

            ### SPAWN UAVS

            StateMachine.add('spawn_uavs'.format(self.id),
                    CBState(self.spawn_uavs_stcb,
                                        input_keys=[],
                                        cb_kwargs={'heritage':heritage}),
                    {'completed': 'all_take_off_ccr'},
                    remapping={})

            # ### BASIC MOVE

            # self.basic_move_sm = StateMachine(outcomes=['completed', 'failed'],
            #                                         input_keys=[])

            # with self.basic_move_sm:
            #     StateMachine.add('basic_move',
            #                         SimpleActionState('/pydag/ANSP_UAV_1/basic_move_command',
            #                                             BasicMoveAction,
            #                                             goal_cb=basic_move_goal_cb,
            #                                             result_cb=basic_move_result_cb,
            #                                             input_keys=[],
            #                                             output_keys=[],
            #                                             goal_cb_kwargs={},
            #                                             result_cb_kwargs={}),
            #                         transitions = {'succeeded':'completed',
            #                                         'preempted':'completed',
            #                                         'aborted':'failed'},
            #                         remapping = {})

            # StateMachine.add('basic_move_sm',
            #         self.basic_move_sm,
            #         transitions={'completed':'completed',
            #                         'failed': 'failed'})

            # ### WAIT

            # StateMachine.add('wait', CBState(self.wait_stcb,cb_kwargs={'heritage':heritage}),
            #                         {'completed':'completed'})

            ## ALL TAKE OFF

            outcome_map = {"completed" : {}}

            for self.id in range(1,heritage.N_uav+1):
                outcome_map["completed"]["{0}_take_off".format(self.id)] = "succeeded"
            
            self.all_take_off_ccr = Concurrence(outcomes=['completed','failed'],
                                    input_keys = [],
                                    output_keys = [],
                                    default_outcome = 'completed',
                                    outcome_map = outcome_map)
            with self.all_take_off_ccr:
                for self.id in range(1,heritage.N_uav+1):
                     Concurrence.add('{0}_take_off'.format(self.id),
                                SimpleActionState('/pydag/ANSP_UAV_{}/take_off_command'.format(self.id),
                                                    TakeOffAction,
                                                    goal_cb=take_off_goal_cb,
                                                    result_cb=take_off_result_cb,
                                                    input_keys=['id','n_wp'],
                                                    output_keys=['id','n_wp'],
                                                    goal_cb_kwargs={},
                                                    result_cb_kwargs={}),
                                remapping={})

            StateMachine.add('all_take_off_ccr'
                                    ,self.all_take_off_ccr,
                                    transitions = {'completed':'wait',
                                                    'failed': 'failed'})

            ### WAIT

            StateMachine.add('wait', CBState(self.wait_stcb,cb_kwargs={'heritage':heritage}),
                                    {'completed':'follow_paths_sbys_sm'})

            

            ### SM - PATHFOLLOWERS, SINGLE WPS

            self.follow_paths_sbys_sm = StateMachine(outcomes=['completed', 'failed'],
                                                    input_keys=[])

            with self.follow_paths_sbys_sm:

                for self.n_wp in range(heritage.path_length):

                    outcome_map = {"completed" : {}}

                    for self.id in range(1,heritage.N_uav+1):
                        outcome_map["completed"]["{0}_follow_wp_{1}".format(self.id,self.n_wp)] = "succeeded"
                    
                    self.follow_wp_ccr = Concurrence(outcomes=['completed','failed'],
                                            input_keys = [],
                                            output_keys = [],
                                            default_outcome = 'completed',
                                            outcome_map = outcome_map)
                    with self.follow_wp_ccr:
                        for self.id in range(1,heritage.N_uav+1):
                            Concurrence.add('{0}_follow_wp_{1}'.format(self.id,self.n_wp),
                                SimpleActionState('/pydag/ANSP_UAV_{}/follow_path_command'.format(self.id),
                                                    FollowPathAction,
                                                    goal_cb=follow_wp_goal_cb,
                                                    result_cb=follow_path_result_cb,
                                                    input_keys=['id','n_wp'],
                                                    output_keys=['id','n_wp'],
                                                    goal_cb_kwargs={'heritage':heritage,'id':self.id,'n_wp':self.n_wp},
                                                    result_cb_kwargs={"heritage":heritage}),
                                remapping={})

                    if self.n_wp != int(heritage.path_length)-1:
                        transitions = {'completed':'follow_wp_{0}_ccr'.format(self.n_wp+1),
                                        'failed': 'failed'}
                    else:
                        transitions = {'completed':'completed',
                                        'failed': 'failed'}

                    StateMachine.add('follow_wp_{0}_ccr'.format(self.n_wp)
                                    ,self.follow_wp_ccr,
                                    transitions = transitions)

            StateMachine.add('follow_paths_sbys_sm',
                            self.follow_paths_sbys_sm,
                            transitions={'completed':'completed',
                                            'failed': 'failed'})

            # ## SM - QUEUE OF FOLLOWERS AD
            # self.queue_of_followers_ad_sm = StateMachine(outcomes=['completed', 'failed'],
            #                                         input_keys=[])

            # with self.queue_of_followers_ad_sm:
            #     outcome_map = {"completed" : {"1_follow_path" : "succeeded"}}
            #     # outcome_map["completed"]["1_follow_path"] = "succeeded"

            #     if heritage.N_uav > 1:
            #         outcome_map["completed"]["2_follow_uav_ad_1"] = "succeeded"

            #     if heritage.N_uav > 2:
            #         outcome_map["completed"]["3_follow_uav_ad_2"] = "succeeded"

            #     self.queue_of_followers_ad_ccr = Concurrence(outcomes=['completed','failed'],
            #                             input_keys= [],
            #                             default_outcome = 'completed',
            #                             outcome_map = outcome_map)
            #     with self.queue_of_followers_ad_ccr:
            #         self.id = 1
            #         Concurrence.add('1_follow_path',
            #             SimpleActionState('/pydag/ANSP_UAV_{}/follow_path_command'.format(self.id),
            #                                 FollowPathAction,
            #                                 goal_cb=follow_path_goal_cb,
            #                                 result_cb=follow_path_result_cb,
            #                                 input_keys=['id'],
            #                                 output_keys=['id'],
            #                                 goal_cb_kwargs={'heritage':heritage,'id':self.id},
            #                                 result_cb_kwargs={"heritage":heritage}),
            #             remapping={'id':'id'})

            #         for self.id in range(2,heritage.N_uav+1):
            #             id_to_follow = self.id - 1
            #             distance = 3
            #             Concurrence.add('{}_follow_uav_ad_{}'.format(self.id,self.id-1),
            #                 SimpleActionState('/pydag/ANSP_UAV_{}/follow_uav_ad_command'.format(self.id),
            #                                     FollowUAVAPAction,
            #                                     goal_cb=follow_uav_ad_goal_cb,
            #                                     result_cb=follow_uav_ad_result_cb,
            #                                     input_keys=['id','id_to_follow'],
            #                                     output_keys=['id','id_to_follow'],
            #                                     goal_cb_kwargs={'heritage':heritage,'target_ID':id_to_follow,'distance':distance},
            #                                     result_cb_kwargs={"heritage":heritage}),
            #                 remapping={'id':'id','id_to_follow':'id_to_follow'})

            #     StateMachine.add('queue_of_followers_ad_ccr',self.queue_of_followers_ad_ccr,
            #                             transitions={'completed':'completed',
            #                                         'failed': 'failed'})
            # StateMachine.add('queue_of_followers_ad_sm',
            #                 self.queue_of_followers_ad_sm,
            #                 transitions={'completed':'all_save_csv_ccr',
            #                                 'failed': 'failed'})

            # ## SM - QUEUE OF FOLLOWERS AP
            # self.queue_of_followers_ap_sm = StateMachine(outcomes=['completed', 'failed'],
            #                                         input_keys=[])

            # with self.queue_of_followers_ap_sm:
            #     outcome_map = {"completed" : {"1_follow_path" : "succeeded"}}
            #     # outcome_map["completed"]["1_follow_path"] = "succeeded"

            #     if heritage.N_uav > 1:
            #         outcome_map["completed"]["2_follow_uav_ap_1"] = "succeeded"

            #     if heritage.N_uav > 2:
            #         outcome_map["completed"]["3_follow_uav_ap_2"] = "succeeded"

            #     self.queue_of_followers_ap_ccr = Concurrence(outcomes=['completed','failed'],
            #                             input_keys= [],
            #                             default_outcome = 'completed',
            #                             outcome_map = outcome_map)
            #     with self.queue_of_followers_ap_ccr:
            #         self.id = 1
            #         Concurrence.add('1_follow_path',
            #             SimpleActionState('/pydag/ANSP_UAV_{}/follow_path_command'.format(self.id),
            #                                 FollowPathAction,
            #                                 goal_cb=follow_path_goal_cb,
            #                                 result_cb=follow_path_result_cb,
            #                                 input_keys=['id'],
            #                                 output_keys=['id'],
            #                                 goal_cb_kwargs={'heritage':heritage,'id':self.id},
            #                                 result_cb_kwargs={"heritage":heritage}),
            #             remapping={'id':'id'})

            #         for self.id in range(2,heritage.N_uav+1):
            #             id_to_follow = self.id - 1
            #             pos = [0,2,0]
            #             Concurrence.add('{}_follow_uav_ap_{}'.format(self.id,self.id-1),
            #                 SimpleActionState('/pydag/ANSP_UAV_{}/follow_uav_ap_command'.format(self.id),
            #                                     FollowUAVAPAction,
            #                                     goal_cb=follow_uav_ap_goal_cb,
            #                                     result_cb=follow_uav_ap_result_cb,
            #                                     input_keys=['id','id_to_follow'],
            #                                     output_keys=['id','id_to_follow'],
            #                                     goal_cb_kwargs={'heritage':heritage,'target_ID':id_to_follow,'pos':pos},
            #                                     result_cb_kwargs={"heritage":heritage}),
            #                 remapping={'id':'id','id_to_follow':'id_to_follow'})

            #     StateMachine.add('queue_of_followers_ap_ccr',self.queue_of_followers_ap_ccr,
            #                             transitions={'completed':'completed',
            #                                         'failed': 'failed'})
            # StateMachine.add('queue_of_followers_ap_sm',
            #                 self.queue_of_followers_ap_sm,
            #                 transitions={'completed':'all_save_csv_ccr',
            #                                 'failed': 'failed'})

            # ### ALL SAVE CSV

            # outcome_map = {"completed" : {}}

            # for self.id in range(1,heritage.N_uav+1):
            #     outcome_map["completed"]["{0}_save_csv".format(self.id)] = "succeeded"
            
            # self.all_save_csv_ccr = Concurrence(outcomes=['completed','failed'],
            #                         input_keys = [],
            #                         output_keys = [],
            #                         default_outcome = 'completed',
            #                         outcome_map = outcome_map)
            # with self.all_save_csv_ccr:
            #     for self.id in range(1,heritage.N_uav+1):
            #          Concurrence.add('{0}_save_csv'.format(self.id),
            #                     SimpleActionState('/pydag/ANSP_UAV_{}/save_csv_command'.format(self.id),
            #                                         LandAction,
            #                                         goal_cb=save_csv_goal_cb,
            #                                         result_cb=save_csv_result_cb,
            #                                         input_keys=['id','n_wp'],
            #                                         output_keys=['id','n_wp'],
            #                                         goal_cb_kwargs={},
            #                                         result_cb_kwargs={}),
            #                     remapping={})

            # StateMachine.add('all_save_csv_ccr'#.format(self.n_wp)
            #                     ,self.all_save_csv_ccr,
            #                     transitions = {'completed':'all_land_ccr',
            #                                     'failed': 'failed'})

            # ### ALL LAND

            # outcome_map = {"completed" : {}}

            # for self.id in range(1,heritage.N_uav+1):
            #     outcome_map["completed"]["{0}_land".format(self.id)] = "succeeded"
            
            # self.all_land_ccr = Concurrence(outcomes=['completed','failed'],
            #                         input_keys = [],
            #                         output_keys = [],
            #                         default_outcome = 'completed',
            #                         outcome_map = outcome_map)
            # with self.all_land_ccr:
            #     for self.id in range(1,heritage.N_uav+1):
            #          Concurrence.add('{0}_land'.format(self.id),
            #                     SimpleActionState('/pydag/ANSP_UAV_{}/land_command'.format(self.id),
            #                                         LandAction,
            #                                         goal_cb=land_goal_cb,
            #                                         result_cb=land_result_cb,
            #                                         input_keys=['id','n_wp'],
            #                                         output_keys=['id','n_wp'],
            #                                         goal_cb_kwargs={},
            #                                         result_cb_kwargs={}),
            #                     remapping={})

            # StateMachine.add('all_land_ccr'.format(self.n_wp)
            #                         ,self.all_land_ccr,
            #                         transitions = {'completed':'completed',
            #                                         'failed': 'failed'})

            if heritage.smach_view == True:
                sis = smach_ros.IntrospectionServer('pydag/ANSP_introspection', self.ansp_sm, '/ANSP')
                sis.start()

    @cb_interface(outcomes=['completed','failed'])
    def new_world_stcb(ud,heritage):
        # Local parameters inizialization
        heritage.CreatingSimulationDataStorage()
        heritage.world = Worlds(1)
        heritage.GettingWorldDefinition()
        heritage.uavs_goal_paths_list = []

        return "completed"

    @cb_interface(outcomes=['completed','failed'])
    def wait_stcb(ud,heritage):
        time.sleep(1)

        return "completed"


    @cb_interface(outcomes=['completed','failed'])
    def spawn_uavs_stcb(ud,heritage):
        for i in range(heritage.N_uav):
            uav_goal_path = heritage.world.PathGenerator()
            heritage.uavs_goal_paths_list.append(uav_goal_path)
            first_pose = uav_goal_path[0]
            heritage.SetUavSpawnFeatures(i+1,heritage.uav_models[i],[first_pose.position.x,first_pose.position.y,0])      
            heritage.UAVSpawner1(i)
            time.sleep(5)
        # time.sleep(20 * heritage.N_uav)

        return "completed"