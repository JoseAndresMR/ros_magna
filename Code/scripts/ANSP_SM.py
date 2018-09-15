from smach import StateMachine, State, CBState, cb_interface,Concurrence
import smach_ros
import time
from smach_ros import SimpleActionState
import numpy as np

from Worlds import *
from pydag.msg import *

class ANSP_SM(object):
    def __init__(self,heritage):
        self.ansp_sm = StateMachine(outcomes=['completed', 'failed'])

        with self.ansp_sm:

            ### CALLBACKS
            def follow_path_goal_cb(ud, goal, heritage):
                goal.goal_path_poses_list = np.array(heritage.uavs_goal_paths_list[ud.id-1])

                return goal

            def follow_wp_goal_cb(ud, goal, heritage,ID,N_WP):
                print(ID,N_WP)
                goal.goal_path_poses_list = [heritage.uavs_goal_paths_list[ID-1][N_WP]]

                return goal

            def follow_path_result_cb(ud, result, heritage):
                return "succeeded"

            def follow_uav_goal_cb(ud, goal, heritage):
                goal.target_ID = ud.id_to_follow
                goal.distance = 3
                goal.time = 10

                return goal

            def follow_uav_result_cb(ud, result, heritage):
                return "succeeded"

            ### TOPOLOGY

            StateMachine.add('new_world', CBState(self.new_world_stcb,cb_kwargs={'heritage':heritage}),
                                    {'completed':'spawn_uavs'})

            self.ansp_sm.userdata.id = 1
            self.ansp_sm.userdata.n_wp = 0
            self.ansp_sm.userdata.id_to_follow = 1

            StateMachine.add('spawn_uavs'.format(self.ansp_sm.userdata.id),
                    CBState(self.spawn_uavs_stcb,
                                        input_keys=['id'],
                                        cb_kwargs={'heritage':heritage}),
                    {'completed': 'wait'},
                    remapping={'id':'id','n_wp':'n_wp'})

            StateMachine.add('wait', CBState(self.wait_stcb,cb_kwargs={'heritage':heritage}),
                                    {'completed':'follow_paths_sbys_sm'})

            outcome_map = {"completed" : {}}

            ### SM - PATHFOLLOWERS, SINGLE WPS
            self.follow_paths_sbys_sm = StateMachine(outcomes=['completed', 'failed'],
                                                    input_keys=['id','n_wp'])

            with self.follow_paths_sbys_sm:

                for self.ansp_sm.userdata.n_wp in range(heritage.path_length):

                    outcome_map = {"completed" : {}}

                    for self.ansp_sm.userdata.id in range(1,heritage.N_uav+1):
                        outcome_map["completed"]["{0}_follow_wp_{1}".format(self.ansp_sm.userdata.id,self.ansp_sm.userdata.n_wp)] = "succeeded"
                    
                    print(outcome_map)

                    self.follow_wp_ccr = Concurrence(outcomes=['completed','failed'],
                                            input_keys = ['id','n_wp'],
                                            output_keys = ['id','n_wp'],
                                            default_outcome = 'completed',
                                            outcome_map = outcome_map)
                    with self.follow_wp_ccr:
                        for self.ansp_sm.userdata.id in range(1,heritage.N_uav+1):
                            Concurrence.add('{0}_follow_wp_{1}'.format(self.ansp_sm.userdata.id,self.ansp_sm.userdata.n_wp),
                                SimpleActionState('/pydag/ANSP_UAV_{}/follow_path_command'.format(self.ansp_sm.userdata.id),
                                                    FollowPathAction,
                                                    goal_cb=follow_wp_goal_cb,
                                                    result_cb=follow_path_result_cb,
                                                    input_keys=['id','n_wp'],
                                                    output_keys=['id','n_wp'],
                                                    goal_cb_kwargs={'heritage':heritage,'ID':self.ansp_sm.userdata.id,'N_WP':self.ansp_sm.userdata.n_wp},
                                                    result_cb_kwargs={"heritage":heritage}),
                                remapping={'id':'id','n_wp':'n_wp'})

                    if self.ansp_sm.userdata.n_wp != int(heritage.path_length)-1:
                        transitions = {'completed':'follow_wp_{0}_ccr'.format(self.ansp_sm.userdata.n_wp+1),
                                        'failed': 'failed'}
                    else:
                        transitions = {'completed':'completed',
                                        'failed': 'failed'}

                    StateMachine.add('follow_wp_{0}_ccr'.format(self.ansp_sm.userdata.n_wp)
                                    ,self.follow_wp_ccr,
                                    transitions = transitions)

            StateMachine.add('follow_paths_sbys_sm',
                            self.follow_paths_sbys_sm,
                            transitions={'completed':'completed',
                                            'failed': 'failed'})

            ### SM - QUEUE OF FOLLOWERS
            # outcome_map["completed"]["1_follow_path"] = "succeeded"

            # if heritage.N_uav > 1:
            #     outcome_map["completed"]["2_follow_uav_1"] = "succeeded"

            # if heritage.N_uav > 2:
            #     outcome_map["completed"]["3_follow_uav_1"] = "succeeded"

            # self.queue_of_followers_ccr = Concurrence(outcomes=['completed','failed'],
            #                           input_keys= ['id','id_to_follow'],
            #                           default_outcome = 'completed',
            #                           outcome_map = outcome_map)
            # with self.queue_of_followers_ccr:
            #     self.ansp_sm.userdata.id = 1
            #     Concurrence.add('1_follow_path',
            #         SimpleActionState('/pydag/ANSP_UAV_{}/follow_path_command'.format(self.ansp_sm.userdata.id),
            #                             FollowPathAction,
            #                             goal_cb=follow_path_goal_cb,
            #                             result_cb=follow_path_result_cb,
            #                             input_keys=['id'],
            #                             output_keys=['id'],
            #                             goal_cb_kwargs={'heritage':heritage},
            #                             result_cb_kwargs={"heritage":heritage}),
            #         remapping={'id':'id'})

            #     if heritage.N_uav > 1:
            #         self.ansp_sm.userdata.id = 2
            #         self.ansp_sm.userdata.id_to_follow = 1
            #         Concurrence.add('2_follow_uav_1',
            #             SimpleActionState('/pydag/ANSP_UAV_{}/follow_uav_command'.format(self.ansp_sm.userdata.id),
            #                                 FollowUAVAction,
            #                                 goal_cb=follow_uav_goal_cb,
            #                                 result_cb=follow_uav_result_cb,
            #                                 input_keys=['id','id_to_follow'],
            #                                 output_keys=['id','id_to_follow'],
            #                                 goal_cb_kwargs={'heritage':heritage},
            #                                 result_cb_kwargs={"heritage":heritage}),
            #             remapping={'id':'id','id_to_follow':'id_to_follow'})

            #     if heritage.N_uav > 2:
            #         self.ansp_sm.userdata.id = 3
            #         self.ansp_sm.userdata.id_to_follow = 2
            #         Concurrence.add('3_follow_uav_2',
            #             SimpleActionState('/pydag/ANSP_UAV_{}/follow_uav_command'.format(self.ansp_sm.userdata.id),
            #                                 FollowUAVAction,
            #                                 goal_cb=follow_uav_goal_cb,
            #                                 result_cb=follow_uav_result_cb,
            #                                 input_keys=['id','id_to_follow'],
            #                                 output_keys=['id','id_to_follow'],
            #                                 goal_cb_kwargs={'heritage':heritage},
            #                                 result_cb_kwargs={"heritage":heritage}),
            #             remapping={'id':'id','id_to_follow':'id_to_follow'})

            # StateMachine.add('queue_of_followers_ccr',self.queue_of_followers_ccr,
            #                         transitions={'completed':'completed',
            #                                      'failed': 'queue_of_followers_ccr'})            

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
        time.sleep(10)

        return "completed"


    @cb_interface(outcomes=['completed','failed'])
    def spawn_uavs_stcb(ud,heritage):
        for i in range(heritage.N_uav):
            uav_goal_path = heritage.world.PathGenerator()
            heritage.uavs_goal_paths_list.append(uav_goal_path)
            first_pose = uav_goal_path[0]
            heritage.SetUavSpawnFeatures(i+1,heritage.uav_models[i],[first_pose.position.x,first_pose.position.y,0])      
            heritage.UAVSpawner1(i)
            time.sleep(10)
        time.sleep(20 * heritage.N_uav)

        return "completed"