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

            self.id = 1
            self.n_wp = 0
            self.target_ID = 1

            heritage.ansp_sm_def.append({"type":"completed"})
            for n_step in range(len(heritage.ansp_sm_def)):
                self.add_sm_from_CSV(heritage,n_step)

            if heritage.smach_view == True:
                sis = smach_ros.IntrospectionServer('pydag/ANSP_introspection', self.ansp_sm, '/ANSP')
                sis.start()

    def add_sm_from_CSV(self,heritage,n_step):

        ### NEW WORLD
        if heritage.ansp_sm_def[n_step]["type"] == "new_world":
            StateMachine.add('new_world', CBState(self.new_world_stcb,cb_kwargs={'heritage':heritage}),
                                {'completed':heritage.ansp_sm_def[n_step+1]["type"]})

        ### SPAWN UAVS
        if heritage.ansp_sm_def[n_step]["type"] == "spawn_uavs":
            StateMachine.add('spawn_uavs'.format(self.id),
                    CBState(self.spawn_uavs_stcb,
                                        input_keys=[],
                                        cb_kwargs={'heritage':heritage}),
                    {'completed': heritage.ansp_sm_def[n_step+1]["type"]},
                    remapping={})

        ### BASIC MOVE
        if heritage.ansp_sm_def[n_step]["type"] == "basic_move_sm":
            self.basic_move_sm = StateMachine(outcomes=['completed', 'failed'],
                                                    input_keys=[])

            with self.basic_move_sm:
                StateMachine.add('basic_move_ansp',
                                    SimpleActionState('/pydag/ANSP_UAV_1/basic_move_command',
                                                        BasicMoveAction,
                                                        goal_cb=self.basic_move_goal_cb,
                                                        result_cb=self.basic_move_result_cb,
                                                        input_keys=[],
                                                        output_keys=[],
                                                        goal_cb_kwargs={},
                                                        result_cb_kwargs={}),
                                    transitions = {'succeeded':'completed',
                                                    'preempted':'completed',
                                                    'aborted':'failed'},
                                    remapping = {})

            StateMachine.add('basic_move_sm',
                    self.basic_move_sm,
                    transitions={'completed':'completed',
                                    'failed': 'failed'})

        ### WAIT
        if heritage.ansp_sm_def[n_step]["type"] == "wait":
            StateMachine.add('wait', CBState(self.wait_stcb,cb_kwargs={'heritage':heritage}),
                                    {'completed':heritage.ansp_sm_def[n_step+1]["type"]})

        ### ALL TAKE OFF
        if heritage.ansp_sm_def[n_step]["type"] == "all_take_off_ccr":
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
                                                    goal_cb=self.take_off_goal_cb,
                                                    result_cb=self.take_off_result_cb,
                                                    input_keys=['id','n_wp'],
                                                    output_keys=['id','n_wp'],
                                                    goal_cb_kwargs={},
                                                    result_cb_kwargs={}),
                                remapping={})

            StateMachine.add('all_take_off_ccr'
                                    ,self.all_take_off_ccr,
                                    transitions = {'completed':heritage.ansp_sm_def[n_step+1]["type"],
                                                    'failed': 'failed'})

        ### FOLLOW PATHS SBYS
        if heritage.ansp_sm_def[n_step]["type"] == "follow_paths_sbys_sm":
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
                                                    goal_cb=self.follow_wp_goal_cb,
                                                    result_cb=self.follow_wp_result_cb,
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
                            transitions={'completed':heritage.ansp_sm_def[n_step+1]["type"],
                                            'failed': 'failed'})


        ### QUEUE OF FOLLOWERS AD
        if heritage.ansp_sm_def[n_step]["type"] == "queue_of_followers_ad_sm":
            self.queue_of_followers_ad_sm = StateMachine(outcomes=['completed', 'failed'],
                                                    input_keys=[])

            with self.queue_of_followers_ad_sm:
                outcome_map = {"completed" : {"1_follow_path" : "succeeded"}}
                # outcome_map["completed"]["1_follow_path"] = "succeeded"

                # if heritage.N_uav > 1:
                #     outcome_map["completed"]["2_follow_uav_ad_1"] = "succeeded"

                # if heritage.N_uav > 2:
                #     outcome_map["completed"]["3_follow_uav_ad_2"] = "succeeded"

                self.queue_of_followers_ad_ccr = Concurrence(outcomes=['completed','failed'],
                                        input_keys= [],
                                        default_outcome = 'completed',
                                        outcome_map = outcome_map)
                with self.queue_of_followers_ad_ccr:
                    self.id = 1
                    Concurrence.add('1_follow_path',
                        SimpleActionState('/pydag/ANSP_UAV_{}/follow_path_command'.format(self.id),
                                            FollowPathAction,
                                            goal_cb=self.follow_path_goal_cb,
                                            result_cb=self.follow_path_result_cb,
                                            input_keys=['id'],
                                            output_keys=['id'],
                                            goal_cb_kwargs={'heritage':heritage,'id':self.id},
                                            result_cb_kwargs={"heritage":heritage}),
                        remapping={'id':'id'})

                    for self.id in range(2,heritage.N_uav+1):
                        target_ID = self.id - 1
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

                StateMachine.add('queue_of_followers_ad_ccr',self.queue_of_followers_ad_ccr,
                                        transitions={'completed':'completed',
                                                    'failed': 'failed'})
            StateMachine.add('queue_of_followers_ad_sm',
                            self.queue_of_followers_ad_sm,
                            transitions={'completed':heritage.ansp_sm_def[n_step+1]["type"],
                                            'failed': 'failed'})

        ### QUEUE OF FOLLOWERS AP
        if heritage.ansp_sm_def[n_step]["type"] == "queue_of_followers_ap_sm":
            self.queue_of_followers_ap_sm = StateMachine(outcomes=['completed', 'failed'],
                                                    input_keys=[])

            with self.queue_of_followers_ap_sm:
                outcome_map = {"completed" : {"follow_path_sm" : "completed"}}
                # outcome_map["completed"]["1_follow_path"] = "succeeded"

                # if heritage.N_uav > 1:
                #     outcome_map["completed"]["2_follow_uav_ap_1"] = "succeeded"

                # if heritage.N_uav > 2:
                #     outcome_map["completed"]["3_follow_uav_ap_2"] = "succeeded"

                self.queue_of_followers_ap_ccr = Concurrence(outcomes=['completed','failed'],
                                        input_keys= [],
                                        default_outcome = 'completed',
                                        outcome_map = outcome_map)

                with self.queue_of_followers_ap_ccr:
                    self.id = 1

                    self.follow_path_sm = StateMachine(outcomes=['completed', 'failed'],
                                                    input_keys=[])

                    with self.follow_path_sm:
                        StateMachine.add('1_follow_path',
                            SimpleActionState('/pydag/ANSP_UAV_{}/follow_path_command'.format(self.id),
                                                FollowPathAction,
                                                goal_cb=self.follow_path_goal_cb,
                                                result_cb=self.follow_path_result_cb,
                                                input_keys=[],
                                                output_keys=[],
                                                goal_cb_kwargs={'heritage':heritage,'id':self.id},
                                                result_cb_kwargs={"heritage":heritage}),
                            remapping={'id':'id','heritage':'heritage','preempt_others':'preempt_others'},
                            transitions = {'succeeded':'completed',
                                           'preempted':'completed',
                                           'aborted':'completed'})

                    Concurrence.add('follow_path_sm',
                        self.follow_path_sm)

                    for self.id in range(2,heritage.N_uav+1):
                        target_ID = self.id - 1
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

                StateMachine.add('queue_of_followers_ap_ccr',self.queue_of_followers_ap_ccr,
                                        transitions={'completed':'completed',
                                                    'failed': 'failed'})
            StateMachine.add('queue_of_followers_ap_sm',
                            self.queue_of_followers_ap_sm,
                            transitions={'completed':heritage.ansp_sm_def[n_step+1]["type"],
                                            'failed': 'failed'})

        ### ALL SAVE CSV
        if heritage.ansp_sm_def[n_step]["type"] == "all_save_csv_ccr":
            outcome_map = {"completed" : {}}

            for self.id in range(1,heritage.N_uav+1):
                outcome_map["completed"]["{0}_save_csv".format(self.id)] = "succeeded"
            
            self.all_save_csv_ccr = Concurrence(outcomes=['completed','failed'],
                                    input_keys = [],
                                    output_keys = [],
                                    default_outcome = 'completed',
                                    outcome_map = outcome_map)
            with self.all_save_csv_ccr:
                for self.id in range(1,heritage.N_uav+1):
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

            StateMachine.add('all_save_csv_ccr'
                                ,self.all_save_csv_ccr,
                                transitions = {'completed':heritage.ansp_sm_def[n_step+1]["type"],
                                                'failed': 'failed'})

        ### ALL LAND
        if heritage.ansp_sm_def[n_step]["type"] == "all_land_ccr":
            outcome_map = {"completed" : {}}

            for self.id in range(1,heritage.N_uav+1):
                outcome_map["completed"]["{0}_land".format(self.id)] = "succeeded"
            
            self.all_land_ccr = Concurrence(outcomes=['completed','failed'],
                                    input_keys = [],
                                    output_keys = [],
                                    default_outcome = 'completed',
                                    outcome_map = outcome_map)
            with self.all_land_ccr:
                for self.id in range(1,heritage.N_uav+1):
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

            StateMachine.add('all_land_ccr'.format(self.n_wp)
                                    ,self.all_land_ccr,
                                    transitions = {'completed':'completed',
                                                    'failed': 'failed'})

    #### STATE CALLBACKS ####

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
    
    #### ACTIONS CALLBACKS ####

    def take_off_goal_cb(self, ud, goal):
        goal.heigth = 5
        return goal

    def take_off_result_cb(self, ud, result, something):
        return "succeeded"

    def basic_move_goal_cb(self, ud, goal):
        goal.move_type = "take_off"
        goal.dynamic = "position"
        goal.direction = "up"
        goal.value = 5.0

        return goal

    def basic_move_result_cb(self, ud, result, something):
        return "succeeded"

    def follow_path_goal_cb(self, ud, goal, heritage, id):
        goal.goal_path_poses_list = np.array(heritage.uavs_goal_paths_list[id-1])

        return goal

    def follow_path_result_cb(self, ud, result, heritage):
        rospy.wait_for_service('/pydag/ANSP/preemption_command_to_2')
        try:
            # print "path for uav {} command".format(ID)
            PreemptCommander = rospy.ServiceProxy('/pydag/ANSP/preemption_command_to_2', StateActualization)
            PreemptCommander(1,"preempt",False)
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in state_actualization"
        return "succeeded"

    def follow_wp_goal_cb(self, ud, goal, heritage, id, n_wp):
        goal.goal_path_poses_list = [heritage.uavs_goal_paths_list[id-1][n_wp]]

        return goal

    def follow_wp_result_cb(self, ud, result, heritage):
        return "succeeded"

    def land_goal_cb(self, ud, goal):
        goal.something = True

        return goal

    def land_result_cb(self, ud, result, something):
        return "succeeded"

    def save_csv_goal_cb(self, ud, goal):
        goal.something = True

        return goal

    def save_csv_result_cb(self, ud, result, something):
        return "succeeded"

    def follow_uav_ad_goal_cb(self, ud, goal, heritage, target_ID, distance):
        goal.target_ID = target_ID
        goal.distance = distance
        goal.time = 10

        return goal

    def follow_uav_ad_result_cb(self, ud, result, heritage):
        return "succeeded"

    def follow_uav_ap_goal_cb(self, ud, goal, heritage, target_ID, pos):
        goal.target_ID = target_ID
        goal.pos = pos
        goal.time = 10

        return goal

    def follow_uav_ap_result_cb(self, ud, result, heritage):
        return "succeeded"