from smach import StateMachine, State, CBState, cb_interface
import smach_ros
import rospy, time
from smach_ros import ActionServerWrapper

from pydag.msg import *

class UAV_Manager_SM(object):
    # At init, the State Machine receives as "heriage" the hole "self" of the Ground Station
    def __init__(self,heritage):
        # Creation of State Machine and definition of its outcomes
        self.uav_sm = StateMachine(outcomes=['completed', 'failed'])

        with self.uav_sm:
            # Initialization of the dictionary containing every Action Service Wrapper
            self.asw_dicc = {}

            ### ACTION SERVER ADVERTISING ###

            # Add a state where drone does nothing but wait to receive a service order
            StateMachine.add('action_server_advertiser',
                            CBState(self.action_server_advertiser_stcb,
                                         cb_kwargs={'heritage':heritage,'asw_dicc':self.asw_dicc}),
                            {'completed':'completed'})

            # Every action service wrapper is similar following a structure defined by SMACH.
            # Each wrapper is defined and added to
            # To understand what they do, see callbacks below

            ### TAKE-OFF STATE MACHINE & WRAPPER ###

            self.take_off_sm = StateMachine(outcomes=['completed', 'failed'],
                                         input_keys=['action_goal'])

            self.asw_dicc['take_off'] = ActionServerWrapper(
                        '/pydag/GS_UAV_{}/take_off_command'.format(heritage.ID),
                        TakeOffAction,
                        self.take_off_sm,
                        ['completed'], ['failed'], ['preempted'],
                        goal_key = 'action_goal',
                        result_key = 'action_result' )

            with self.take_off_sm:

                StateMachine.add('take_off',
                                 CBState(self.take_off_stcb,
                                         input_keys=['action_goal'],
                                         cb_kwargs={'heritage':heritage}),
                                 {'completed':'completed'})

            ### LAND STATE MACHINE & WRAPPER ###

            self.land_sm = StateMachine(outcomes=['completed', 'failed'],
                                         input_keys=['action_goal'])

            self.asw_dicc['land'] = ActionServerWrapper(
                        '/pydag/GS_UAV_{}/land_command'.format(heritage.ID),
                        LandAction,
                        self.land_sm,
                        ['completed'], ['failed'], ['preempted'],
                        goal_key = 'action_goal',
                        result_key = 'action_result' )

            with self.land_sm:

                StateMachine.add('land',
                                 CBState(self.land_stcb,
                                         input_keys=['action_goal'],
                                         cb_kwargs={'heritage':heritage}),
                                 {'completed':'completed'})


            ### BASIC MOVE STATE MACHINE & WRAPPER ###

            self.basic_move_sm = StateMachine(outcomes=['completed', 'failed','collision','low_battery','GS_critical_event'],
                                         input_keys=['action_goal','action_result'])

            self.asw_dicc['basic_move'] = ActionServerWrapper(
                        '/pydag/GS_UAV_{}/basic_move_command'.format(heritage.ID),
                        BasicMoveAction,
                        self.basic_move_sm,
                        ['completed'], ['failed'],['collision','low_battery','GS_critical_event'],
                        goal_key = 'action_goal',
                        result_key = 'action_result' )

            with self.basic_move_sm:

                StateMachine.add('basic_move',
                                 CBState(self.basic_move_stcb,
                                         input_keys=['action_goal','action_result'],
                                         cb_kwargs={'heritage':heritage}),
                                 {'completed':'completed',
                                  'collision':'collision',
                                  'low_battery':'low_battery',
                                  'GS_critical_event':'GS_critical_event'})

            ### SAVE CVS STATE MACHINE & WRAPPER ###

            self.save_csv_sm = StateMachine(outcomes=['completed', 'failed'],
                                         input_keys=['action_goal'])

            self.asw_dicc['save_csv'] = ActionServerWrapper(
                        '/pydag/GS_UAV_{}/save_csv_command'.format(heritage.ID),
                        SaveCSVAction,
                        self.save_csv_sm,
                        ['completed'], ['failed'], ['preempted'],
                        goal_key = 'action_goal',
                        result_key = 'action_result' )

            with self.save_csv_sm:

                StateMachine.add('save_csv',
                                 CBState(self.save_csv_stcb,
                                         input_keys=['action_goal'],
                                         cb_kwargs={'heritage':heritage}),
                                 {'completed':'completed'})

            ### FOLLOW PATH STATE MACHINE & WRAPPER ###
            self.follow_path_sm = StateMachine(outcomes=['completed', 'failed','collision','low_battery','GS_critical_event'],
                                         input_keys=['action_goal','action_result'])

            self.asw_dicc['follow_path'] = ActionServerWrapper(
                        '/pydag/GS_UAV_{}/follow_path_command'.format(heritage.ID),
                        FollowPathAction,
                        self.follow_path_sm,
                        ['completed'], ['failed'], ['collision','low_battery','GS_critical_event'],
                        goal_key = 'action_goal',
                        result_key = 'action_result'
                        )

            with self.follow_path_sm:

                StateMachine.add('follow_path',
                                 CBState(self.follow_path_stcb,
                                         input_keys=['action_goal','action_result','_preempt_requested'],
                                         cb_kwargs={'heritage':heritage}),
                                 {'completed':'completed',
                                  'collision':'collision',
                                  'low_battery':'low_battery',
                                  'GS_critical_event':'GS_critical_event'})

            # StateMachine.add('to_wp', self.follow_path_sm,
            #                         {'completed':'action_server_advertiser'})

            ### FOLLOW UAV AD STATE MACHINE  & WRAPPER###
            self.follow_uav_ad_sm = StateMachine(outcomes=['completed', 'failed','collision','low_battery','GS_critical_event'],
                                         input_keys=['action_goal','action_result'])

            self.asw_dicc['follow_uav_ad'] = ActionServerWrapper(
                        '/pydag/GS_UAV_{}/follow_uav_ad_command'.format(heritage.ID),
                        FollowUAVADAction,
                        self.follow_uav_ad_sm,
                        ['completed'], ['failed'], ['collision','low_battery','GS_critical_event'],
                        goal_key = 'action_goal',
                        result_key = 'action_result' )

            with self.follow_uav_ad_sm:

                StateMachine.add('follow_uav_ad',
                                 CBState(self.follow_uav_ad_stcb,
                                         input_keys=['action_goal','action_result','_preempt_requested'],
                                         cb_kwargs={'heritage':heritage}),
                                 {'completed':'completed',
                                  'collision':'collision',
                                  'low_battery':'low_battery',
                                  'GS_critical_event':'GS_critical_event'})

            ### FOLLOW UAV AP STATE MACHINE  & WRAPPER###
            self.follow_uav_ap_sm = StateMachine(outcomes=['completed', 'failed','collision','low_battery','GS_critical_event'],
                                         input_keys=['action_goal','action_result'])

            self.asw_dicc['follow_uav_ap'] = ActionServerWrapper(
                        '/pydag/GS_UAV_{}/follow_uav_ap_command'.format(heritage.ID),
                        FollowUAVAPAction,
                        self.follow_uav_ap_sm,
                        ['completed'], ['failed'], ['collision','low_battery','GS_critical_event'],
                        goal_key = 'action_goal',
                        result_key = 'action_result' )

            with self.follow_uav_ap_sm:

                StateMachine.add('follow_uav_ap',
                                 CBState(self.follow_uav_ap_stcb,
                                         input_keys=['action_goal','action_result','_preempt_requested'],
                                         cb_kwargs={'heritage':heritage}),
                                 {'completed':'completed',
                                  'collision':'collision',
                                  'low_battery':'low_battery',
                                  'GS_critical_event':'GS_critical_event'})

            if heritage.smach_view == True:
                sis = smach_ros.IntrospectionServer('pydag/UAV_{}_introspection'.format(heritage.ID), self.uav_sm, '/UAV_{}'.format(heritage.ID))
                sis.start()

    #### STATE CALLBACKS ####

    @cb_interface(outcomes=['completed', 'failed'])
    def action_server_advertiser_stcb(self, heritage, asw_dicc):
        heritage.SetVelocityCommand(True)       # Tell GS to hover while no server request is received
        for key in asw_dicc.keys():     # Run every ASW stored
            asw_dicc[key].run_server()

        # Function to inform Ground Station about actual UAV's state
        heritage.state = "waiting for action command"
        heritage.GSStateActualization()

        time.sleep(0.2)

        rospy.spin()

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def take_off_stcb(self,heritage):
        print("TAKE OFF COMMAND")


        heritage.TakeOffCommand(self.action_goal.height,True)     # Tell GS to take off

        # Function to inform Ground Station about actual UAV's state
        heritage.state = "inizializating"
        heritage.GSStateActualization()

        return 'completed'

    @cb_interface(outcomes=['completed','failed','collision','low_battery','GS_critical_event'])
    def follow_path_stcb(self,heritage):
        # print(self._preempt_requested)
        # Copy the goal path to follow into GS's variable
        heritage.smooth_path_mode = self.action_goal.smooth_path_mode
        heritage.goal_path_poses_list = self.action_goal.goal_path_poses_list
        output = heritage.PathFollower(self.action_goal.dynamic)     # Tell the GS to execute that function

        # response = FollowPathActionResponse()
        # response.output = output

        # class Response(object):
        #     def __init__(self):
        #         self.output = ""
        # response = Response()
        # response.output = output

        # return response
        # self.as.set_succeeded = 'collision'

        self.action_result.output = output

        return 'completed'

    @cb_interface(outcomes=['completed','failed','collision','low_battery','GS_critical_event'])
    def follow_uav_ad_stcb(self,heritage):

        # Tell the GS the identity of its new target
        heritage.state = "following UAV {0}".format(self.action_goal.target_ID)

        heritage.GSStateActualization()       # Function to inform Ground Station about actual UAV's state

        # Tell the GS to execute UAVFollowerAD role with at the required distance
        output = heritage.UAVFollowerAtDistance(self.action_goal.target_ID,self.action_goal.distance,self.action_goal.time)

        self.action_result.output = output

        return 'completed'

    @cb_interface(outcomes=['completed','failed','collision','low_battery','GS_critical_event'])
    def follow_uav_ap_stcb(self,heritage):

        # Tell the GS the identity of its new target
        heritage.state = "following UAV {0}".format(self.action_goal.target_ID)

        heritage.GSStateActualization()       # Function to inform Ground Station about actual UAV's state

        # Tell the GS to execute UAVFollowerAP role with the required bias
        output = heritage.UAVFollowerAtPosition(self.action_goal.target_ID,self.action_goal.pos,self.action_goal.time)

        self.action_result.output = output

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def save_csv_stcb(self,heritage):
        heritage.StoreData()        # Tell the GS to store the data

        return "completed"


    @cb_interface(outcomes=['completed','failed'])
    def land_stcb(self,heritage):
        print "landing"
        heritage.LandCommand(True)        # Tell the GS to land

        # Function to inform Ground Station about actual UAV's state
        heritage.state = "landed"
        heritage.GSStateActualization()

        return 'completed'

    @cb_interface(outcomes=['completed','failed','collision','low_battery','GS_critical_event'])
    def basic_move_stcb(self,heritage):

        # Parse received basic move information
        move_type = self.action_goal.move_type
        dynamic = self.action_goal.dynamic
        direction = self.action_goal.direction
        value = self.action_goal.value

        # Tell the GS to execute basic move role with parsed information
        output = heritage.basic_move(move_type,dynamic,direction,value)

        self.action_result.output = output

        return 'completed'
